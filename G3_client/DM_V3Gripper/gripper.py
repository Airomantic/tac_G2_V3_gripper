import threading
import time
from typing import Dict, List, Optional

try:
    import grpc
    from google.protobuf import empty_pb2
    from . import gripper_pb2
    from . import gripper_pb2_grpc
except ImportError as exc:
    grpc = None
    empty_pb2 = None
    gripper_pb2 = None
    gripper_pb2_grpc = None
    _GRPC_IMPORT_ERROR = exc
else:
    _GRPC_IMPORT_ERROR = None


INIT_STATE_TEXT = {
    0: "not initialized",
    1: "initialized",
    2: "initializing",
    0xFFFF: "travel calibration error",
}

GRIP_STATE_TEXT = {
    0: "moving",
    1: "target reached",
    2: "object gripped",
    3: "object dropped",
    0xFFFF: "initialization incomplete",
}

ERROR_TEXT = {
    0: "no error",
    1: "undervoltage, below 15V",
    2: "overvoltage, above 28V",
    3: "overcurrent",
    4: "overtemperature",
    5: "motor disconnected",
    8: "overload",
    11: "overspeed",
    15: "startup error",
    32: "magnetic encoder error",
}


def _clip(value: int, min_value: int, max_value: int) -> int:
    """Clamp an integer value to an inclusive range."""
    return max(min_value, min(max_value, int(value)))


class Gripper:
    def __init__(
        self,
        mode="remote",
        channel="can0",
        motor_id=1,
        remote_ip="192.168.127.10",
        remote_port=50008,
        read_interval_s=0.05,
        read_timeout_s=0.1,
        retry_count=2,
        stale_timeout_s=0.5,
        command_min_interval_s=0.02,
        open_position=1000,
        close_position=0,
        strict_init_check=True,
    ):
        """
        Initialize remote gripper client settings and state cache.

        :param mode: remote only; local CAN-FD access belongs to G3_server
        :param channel: edge-device CAN channel sent to the remote server
        :param motor_id: gripper motor ID on the edge device
        :param remote_ip: edge-server IP address
        :param remote_port: edge-server gRPC port
        :param read_interval_s: polling interval used by wait helpers
        :param read_timeout_s: retained for compatibility with server configuration
        :param retry_count: retained for compatibility with server configuration
        :param stale_timeout_s: mark cached status stale after this interval
        :param command_min_interval_s: minimum interval configured on the server
        :param strict_init_check: retained for API compatibility; enforced server-side
        """
        if mode != "remote":
            raise ValueError("G3_client SDK supports remote mode only; use G3_server for local CAN-FD access")
        if grpc is None or gripper_pb2 is None or gripper_pb2_grpc is None:
            raise ImportError("remote mode requires grpcio, protobuf, and generated gRPC code") from _GRPC_IMPORT_ERROR

        self.mode = "remote"
        self.motor_id = int(motor_id)
        self.channel = channel
        self.read_interval_s = max(0.005, float(read_interval_s))
        self.read_timeout_s = max(0.005, float(read_timeout_s))
        self.retry_count = max(0, int(retry_count))
        self.stale_timeout_s = max(self.read_interval_s, float(stale_timeout_s))
        self.command_min_interval_s = max(0.0, float(command_min_interval_s))
        self.open_position = _clip(open_position, 0, 1000)
        self.close_position = _clip(close_position, 0, 1000)
        self.strict_init_check = bool(strict_init_check)

        self.remote_addr = f"{remote_ip}:{int(remote_port)}"
        self.grpc_channel = grpc.insecure_channel(self.remote_addr)
        self.stub = gripper_pb2_grpc.GripperServiceStub(self.grpc_channel)

        self._connected = False
        self._last_error = ""
        self._last_message = ""
        self._command_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._state_cache = self._empty_state("no gripper status has been read yet")

    # ================= Connection and resource management =================

    def connect(self) -> bool:
        """Connect to the remote edge-device gripper service."""
        if self._connected:
            self._set_command_result(True, "remote gripper is already connected")
            return True

        try:
            self.grpc_channel = grpc.insecure_channel(self.remote_addr)
            self.stub = gripper_pb2_grpc.GripperServiceStub(self.grpc_channel)
            grpc.channel_ready_future(self.grpc_channel).result(timeout=3)
            req = gripper_pb2.ConnectConfig(
                channel=self.channel,
                motor_id=self.motor_id,
                read_interval_s=self.read_interval_s,
                read_timeout_s=self.read_timeout_s,
                retry_count=self.retry_count,
                stale_timeout_s=self.stale_timeout_s,
                command_min_interval_s=self.command_min_interval_s,
            )
            resp = self.stub.Connect(req, timeout=5)
            self._apply_response(resp)
            self._connected = bool(resp.success)
            if resp.success:
                print(f"remote gripper {self.remote_addr} connected.")
            else:
                print(f"remote gripper connection failed: {resp.message}")
            return bool(resp.success)
        except grpc.RpcError as exc:
            return self._fail(f"gRPC connection failed: {exc}")
        except Exception as exc:
            return self._fail(f"remote connection error: {exc}")

    def disconnect(self) -> bool:
        """Disconnect from the remote service and close the gRPC channel."""
        if not self._connected:
            try:
                self.grpc_channel.close()
            except Exception:
                pass
            self._set_command_result(True, "connection disconnected")
            return True

        ok = True
        try:
            resp = self.stub.Disconnect(empty_pb2.Empty(), timeout=3)
            self._apply_response(resp)
            ok = bool(resp.success)
        except grpc.RpcError as exc:
            ok = False
            self._fail(f"remote disconnect failed: {exc}")
        except Exception as exc:
            ok = False
            self._fail(f"remote disconnect error: {exc}")
        finally:
            try:
                self.grpc_channel.close()
            except Exception:
                pass
            self._connected = False
        return ok

    # ================= Status cache and decoding =================

    def _empty_state(self, message: str) -> Dict[str, object]:
        """Create an invalid status snapshot with a diagnostic message."""
        return {
            "valid": False,
            "connected": False,
            "stale": True,
            "message": message,
            "init_state": -1,
            "init_state_text": "unknown",
            "grip_state": -1,
            "grip_state_text": "unknown",
            "position": -1,
            "current": -1,
            "error": -1,
            "error_text": "unknown",
            "timestamp": 0.0,
            "update_age_ms": -1.0,
        }

    def _mark_state_error(self, message: str):
        """Record a status refresh error in the cache and last-error field."""
        with self._state_lock:
            if self._state_cache["timestamp"] == 0.0:
                self._state_cache = self._empty_state(message)
            else:
                self._state_cache["message"] = message
        self._last_error = message

    def _snapshot_state(self) -> Dict[str, object]:
        """Return a copy of cached status with freshness metadata updated."""
        with self._state_lock:
            state = dict(self._state_cache)

        timestamp = float(state.get("timestamp", 0.0))
        if timestamp > 0.0:
            age = time.monotonic() - timestamp
            stale = age > self.stale_timeout_s
            state["update_age_ms"] = age * 1000.0
            state["stale"] = stale
            state["valid"] = bool(state.get("valid")) and not stale
            state["connected"] = self._connected
            if stale:
                state["message"] = f"status has not been updated for {self.stale_timeout_s:.3f}s; gripper may be unresponsive"
        else:
            state["update_age_ms"] = -1.0
            state["stale"] = True
            state["valid"] = False
            state["connected"] = self._connected
        return state

    def refresh_state(self) -> bool:
        """Refresh status once from the remote server."""
        if not self._connected:
            self._mark_state_error("remote gripper is not connected; status cannot be refreshed")
            return False
        try:
            resp = self.stub.GetAllState(empty_pb2.Empty(), timeout=2)
            self._update_state_from_response(resp)
            return bool(resp.valid)
        except grpc.RpcError as exc:
            self._mark_state_error(f"remote status read failed: {exc}")
            return False

    def get_state(self) -> Dict[str, object]:
        """Return the latest gripper status snapshot."""
        self.refresh_state()
        return self._snapshot_state()

    def GetAllState(self) -> dict:
        """Compatibility API for returning all cached status fields."""
        return self.get_state()

    @staticmethod
    def decode_init_state(value: int) -> str:
        """Decode an initialization state code into text."""
        return INIT_STATE_TEXT.get(int(value), f"unknown init state {value}")

    @staticmethod
    def decode_grip_state(value: int) -> str:
        """Decode a grip state code into text."""
        return GRIP_STATE_TEXT.get(int(value), f"unknown grip state {value}")

    @staticmethod
    def decode_error(value: int) -> str:
        """Decode an error code into text."""
        return ERROR_TEXT.get(int(value), f"unknown error code {value}")

    def is_initialized(self) -> bool:
        """Return True when the cached status shows initialization complete."""
        state = self.get_state()
        return bool(state.get("valid")) and state.get("init_state") == 1

    def is_moving(self) -> bool:
        """Return True when the cached status shows active motion."""
        state = self.get_state()
        return bool(state.get("valid")) and state.get("grip_state") == 0

    def has_error(self) -> bool:
        """Return True when the cached status contains an active error code."""
        state = self.get_state()
        return int(state.get("error", -1)) not in (0, -1)

    def wait_until_initialized(self, timeout_s: float = 10.0) -> bool:
        """Wait until the gripper reports successful initialization."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.is_initialized():
                return True
            time.sleep(min(0.05, self.read_interval_s))
        self._fail("waiting for initialization timed out")
        return False

    def wait_until_position(self, target: int, tolerance: int = 10, timeout_s: float = 5.0) -> bool:
        """Wait until the gripper reaches the target position tolerance."""
        target = _clip(target, 0, 1000)
        tolerance = max(0, int(tolerance))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            state = self.get_state()
            pos = int(state.get("position", -1))
            if bool(state.get("valid")) and pos >= 0 and abs(pos - target) <= tolerance:
                self._set_command_result(True, "gripper reached target position")
                return True
            time.sleep(min(0.05, self.read_interval_s))
        self._fail(f"waiting for gripper to reach position {target} timed out")
        return False

    # ================= Public control API =================

    def enable(self) -> bool:
        """Enable the gripper system."""
        return self._remote_bool_call("Enable")

    def disable(self) -> bool:
        """Disable the gripper system."""
        return self._remote_bool_call("Disable")

    def Initialization(self) -> bool:
        """Compatibility API: run full calibration and initialization."""
        return self.calibrate()

    def initialize(self) -> bool:
        """Alias for full calibration and initialization."""
        return self.calibrate()

    def calibrate(self) -> bool:
        """Run full gripper calibration and initialization."""
        return self._remote_bool_call("Initialization")

    def home(self) -> bool:
        """Run one-direction homing."""
        return self._remote_bool_call("Home")

    def SetTargetPosition(self, pos: int) -> bool:
        """Set the target gripper position."""
        return self._remote_value_call("SetTargetPosition", _clip(pos, 0, 1000))

    def SetTargetForce(self, force: int) -> bool:
        """Set the target gripping force."""
        return self._remote_value_call("SetTargetForce", _clip(force, 20, 100))

    def SetTargetSpeed(self, speed: int) -> bool:
        """Set the target gripper speed."""
        return self._remote_value_call("SetTargetSpeed", _clip(speed, 1, 100))

    def move_to(
        self,
        pos: int,
        speed: Optional[int] = None,
        force: Optional[int] = None,
        wait: bool = False,
        timeout_s: float = 5.0,
        tolerance: int = 10,
    ) -> bool:
        """Move to a target position, optionally setting speed and force first."""
        pos = _clip(pos, 0, 1000)
        with self._command_lock:
            if speed is not None and not self.SetTargetSpeed(speed):
                return False
            if force is not None and not self.SetTargetForce(force):
                return False
            if not self.SetTargetPosition(pos):
                return False
        if wait:
            return self.wait_until_position(pos, tolerance=tolerance, timeout_s=timeout_s)
        return True

    def Open(self, wait: bool = False, timeout_s: float = 5.0) -> bool:
        """Move the gripper to the configured open position."""
        ok = self._remote_bool_call("Open")
        if ok and wait:
            return self.wait_until_position(self.open_position, timeout_s=timeout_s)
        return ok

    def Close(self, wait: bool = False, timeout_s: float = 5.0) -> bool:
        """Close the gripper. Use disconnect to release resources."""
        ok = self._remote_bool_call("Close")
        if ok and wait:
            return self.wait_until_position(self.close_position, timeout_s=timeout_s)
        return ok

    def Stop(self) -> bool:
        """Stop the current gripper motion."""
        return self._remote_bool_call("Stop")

    def Resume(self) -> bool:
        """Resume the previous gripper target after a stop."""
        return self._remote_bool_call("Resume")

    def Clear_error(self) -> bool:
        """Clear the active gripper error state."""
        return self._remote_bool_call("ClearError")

    def SetJog(self, direction: int) -> bool:
        """Set jog motion direction: -1, 0, or 1."""
        direction = int(direction)
        if direction not in (-1, 0, 1):
            return self._fail("jog direction must be -1, 0, or 1")
        return self._remote_value_call("SetJog", direction)

    def SetFollowMode(self, enabled: bool) -> bool:
        """Enable or disable position-follow mode."""
        return self._remote_value_call("SetFollowMode", 1 if enabled else 0)

    # ================= Status compatibility API =================

    def GetCurrentPosition(self) -> int:
        """Return the current gripper position."""
        return int(self.get_state().get("position", -1))

    def GetInitState(self) -> int:
        """Return the current initialization state code."""
        return int(self.get_state().get("init_state", -1))

    def GetGripState(self) -> int:
        """Return the current grip state code."""
        return int(self.get_state().get("grip_state", -1))

    def GetCurrentCurrent(self) -> int:
        """Return the current motor current."""
        return int(self.get_state().get("current", -1))

    def GetCurrentForce(self) -> int:
        """Compatibility API: returns motor current because no force feedback register exists."""
        return self.GetCurrentCurrent()

    def GetErrorCode(self) -> int:
        """Return the current gripper error code."""
        return int(self.get_state().get("error", -1))

    def GetLastMessage(self) -> str:
        """Return the last command or status message."""
        return self._last_message

    def GetLastError(self) -> str:
        """Return the last recorded error message."""
        return self._last_error

    def read_registers(self, start_addr: int, count: int = 1):
        """Low-level register access is not available in the remote-only client SDK."""
        self._fail("low-level register access is only available in G3_server")
        return None

    def write_register(self, reg_addr: int, value: int) -> bool:
        """Low-level register access is not available in the remote-only client SDK."""
        return self._fail("low-level register access is only available in G3_server")

    def write_registers(self, start_addr: int, values: List[int]) -> bool:
        """Low-level register access is not available in the remote-only client SDK."""
        return self._fail("low-level register access is only available in G3_server")

    # ================= gRPC helper logic =================

    def _remote_bool_call(self, rpc_name: str) -> bool:
        """Call a no-argument remote gRPC command."""
        if not self._connected:
            return self._fail("remote gripper is not connected")
        try:
            rpc = getattr(self.stub, rpc_name)
            resp = rpc(empty_pb2.Empty(), timeout=5)
            return self._apply_response(resp)
        except grpc.RpcError as exc:
            return self._fail(f"remote call {rpc_name} failed: {exc}")
        except Exception as exc:
            return self._fail(f"remote call {rpc_name} error: {exc}")

    def _remote_value_call(self, rpc_name: str, value: int) -> bool:
        """Call a value-based remote gRPC command."""
        if not self._connected:
            return self._fail("remote gripper is not connected")
        try:
            rpc = getattr(self.stub, rpc_name)
            resp = rpc(gripper_pb2.ValueRequest(value=int(value)), timeout=5)
            return self._apply_response(resp)
        except grpc.RpcError as exc:
            return self._fail(f"remote call {rpc_name} failed: {exc}")
        except Exception as exc:
            return self._fail(f"remote call {rpc_name} error: {exc}")

    def _apply_response(self, resp) -> bool:
        """Apply a gRPC response to command and state caches."""
        message = getattr(resp, "message", "")
        self._set_command_result(bool(resp.success), message)
        state = getattr(resp, "state", None)
        if state is not None:
            self._update_state_from_response(state)
        return bool(resp.success)

    def _update_state_from_response(self, resp):
        """Update the status cache from a gRPC state response."""
        update_age_ms = float(getattr(resp, "update_age_ms", -1.0))
        timestamp = 0.0 if update_age_ms < 0 else time.monotonic() - update_age_ms / 1000.0
        state = {
            "valid": bool(getattr(resp, "valid", False)),
            "connected": bool(getattr(resp, "connected", False)),
            "stale": bool(getattr(resp, "stale", True)),
            "message": getattr(resp, "message", ""),
            "init_state": int(getattr(resp, "init_state", -1)),
            "init_state_text": getattr(resp, "init_state_text", ""),
            "grip_state": int(getattr(resp, "grip_state", -1)),
            "grip_state_text": getattr(resp, "grip_state_text", ""),
            "position": int(getattr(resp, "position", -1)),
            "current": int(getattr(resp, "current", -1)),
            "error": int(getattr(resp, "error", -1)),
            "error_text": getattr(resp, "error_text", ""),
            "timestamp": timestamp,
            "update_age_ms": update_age_ms,
        }
        with self._state_lock:
            self._state_cache = state

    # ================= Result bookkeeping =================

    def _set_command_result(self, success: bool, message: str):
        """Store the latest command result message."""
        self._last_message = message
        if not success:
            self._last_error = message

    def _fail(self, message: str) -> bool:
        """Record and print a failure, then return False."""
        self._set_command_result(False, message)
        print(message)
        return False
