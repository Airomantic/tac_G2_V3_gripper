# DM_V3Gripper API

```python
from DM_V3Gripper import Gripper
```

Remote gRPC client for the DM V3 gripper. Method names are case-sensitive; aliases share one entry below.

## Constructor

`Gripper(...)` creates a client without connecting and returns a `Gripper` instance.

| Parameter | Type / default | Purpose |
| --- | --- | --- |
| `mode` | `str = "remote"` | Only supported mode; other values raise `ValueError`. |
| `channel` | `str = "can0"` | CAN channel on the server. |
| `motor_id` | `int = 1` | Motor ID on the server. |
| `remote_ip` | `str = "192.168.127.10"` | Server IP address. |
| `remote_port` | `int = 50008` | Server gRPC port. |
| `read_interval_s` | `float = 0.05` | Server polling interval and client wait polling setting; minimum `0.005` seconds. |
| `read_timeout_s` | `float = 0.1` | Server read timeout; minimum `0.005` seconds. Does not set RPC deadlines. |
| `retry_count` | `int = 2` | Server read retries; minimum `0`. Does not retry client RPC calls. |
| `stale_timeout_s` | `float = 0.5` | Maximum feedback age in seconds; at least `read_interval_s`. |
| `command_min_interval_s` | `float = 0.02` | Minimum command interval configured on the server, in seconds; minimum `0`. |
| `open_position` | `int = 1000` | Local position expected when waiting for opening; clamped to `0–1000`. |
| `close_position` | `int = 0` | Local position expected when waiting for closing; clamped to `0–1000`. |
| `strict_init_check` | `bool = True` | Compatibility setting; stored but not used or sent to the server. |

The two endpoint settings only affect client waits; they do not change server motion targets.

## Connection and motion

Commands return `bool`: `True` means the server reports success, and `False` means failure. Motion completion is checked only when `wait=True`; a wait timeout also returns `False`. Time values are seconds.

| Function and parameters | Purpose | Return |
| --- | --- | --- |
| `connect()` | Connect and send server configuration. | `bool` |
| `disconnect()` | Disconnect and close the channel; does not explicitly stop or disable the motor. | `bool` |
| `enable()` / `disable()` | Enable / disable the gripper. | `bool` |
| `initialize()` / `Initialization()` / `calibrate()` | Start full calibration and initialization. | `bool`; does not wait for completion |
| `home()` | Start one-direction homing. | `bool` |
| `SetTargetPosition(pos: int)` | Set target position; clamp to `0–1000` (`0` closed, `1000` open). | `bool` |
| `SetTargetSpeed(speed: int)` | Set target speed; clamp to `1–100`. | `bool` |
| `SetTargetForce(force: int)` | Set target force; clamp to `20–100` in device units. | `bool` |
| `move_to(pos: int, speed=None, force=None, wait=False, timeout_s=5.0, tolerance=10)` | Set optional integer speed and force, then position, using the ranges above. `None` leaves a setting unchanged; `wait` checks arrival within `tolerance` position units, for up to `timeout_s`. | `bool` |
| `Open(wait=False, timeout_s=5.0)` / `Close(wait=False, timeout_s=5.0)` | Move to the server's open / closed endpoint. Optional wait checks the corresponding local endpoint with tolerance `10`. | `bool` |
| `Stop()` | Stop current motion. | `bool` |
| `Resume()` | Resume the previous target after stopping; requires server support. | `bool`; `False` if RPC is unimplemented |
| `Clear_error()` | Clear the active device error. | `bool` |
| `SetJog(direction: int)` | Select jog direction `-1` or `1`; `0` stops jogging. | `bool`; `False` for other values |
| `SetFollowMode(enabled: bool)` | Enable (`True`) or disable (`False`) position-follow mode. | `bool` |
| `wait_until_initialized(timeout_s=10.0)` | Poll until valid feedback reports initialization complete. | `bool`: completed before timeout |
| `wait_until_position(target: int, tolerance=10, timeout_s=5.0)` | Poll until valid position is within `tolerance` of `target`. Target is clamped to `0–1000`; tolerance is a nonnegative integer. | `bool`: reached before timeout |

Wait helpers do not initiate motion or stop it on timeout. Speed and force use device scales, not physical units; speed `0` is clamped to `1`. Speed can only be commanded: the client does not provide measured speed or include speed in status snapshots.

## Feedback

Feedback queries below perform a remote status read, except the last-message/error accessors and static decoders. A failed read may leave cached values; check snapshot validity before using feedback. Numeric feedback is initially `-1`.

| Function and parameters | Purpose | Return |
| --- | --- | --- |
| `refresh_state()` | Read server status once and update the cache. | `bool`: server reports valid status; `False` on read failure |
| `get_state()` / `GetAllState()` | Refresh and return a snapshot with local freshness checks. | `dict`, fields below |
| `is_initialized()` | Check valid feedback for initialization code `1`. | `bool` |
| `is_moving()` | Check valid feedback for grip code `0`. | `bool` |
| `has_error()` | Check whether error code is neither `0` nor `-1`; does not check freshness. | `bool` |
| `GetCurrentPosition()` | Read current position. | `int`, device position scale `0–1000` |
| `GetCurrentCurrent()` / `GetCurrentForce()` | Read motor current; the force alias also returns current, not measured force. | `int`, device units |
| `GetInitState()` | Read initialization state code. | `int` |
| `GetGripState()` | Read grip state code. | `int` |
| `GetErrorCode()` | Read device error code; `0` means no error. | `int` |
| `GetLastMessage()` | Read the last command result message without an RPC. | `str` |
| `GetLastError()` | Read the last recorded failure without an RPC; success does not clear it. | `str` |
| `decode_init_state(value: int)` / `decode_grip_state(value: int)` / `decode_error(value: int)` | Decode the corresponding numeric code locally; static methods. | English `str`, including the code when unknown |

Snapshot fields:

| Field | Type | Meaning |
| --- | --- | --- |
| `valid` | `bool` | Server-valid feedback that has not expired locally. |
| `connected` | `bool` | Client connection flag; not a live network probe. |
| `stale` | `bool` | Feedback is too old or no timestamp is available. |
| `message` | `str` | Status diagnostic. |
| `init_state`, `grip_state`, `error` | `int` | Device state/error codes. |
| `init_state_text`, `grip_state_text`, `error_text` | `str` | Corresponding server descriptions. |
| `position`, `current` | `int` | Position and motor current feedback in device units. |
| `timestamp` | `float` | Estimated local monotonic update time in seconds; `0` when unknown. |
| `update_age_ms` | `float` | Feedback age in milliseconds; `-1` when unknown. |

## Register compatibility methods

Direct register access is unsupported in this remote client. These methods log an error and do not send an RPC.

| Function and parameters | Purpose | Return |
| --- | --- | --- |
| `read_registers(start_addr: int, count: int = 1)` | Read `count` registers starting at `start_addr` (unsupported). | `None` |
| `write_register(reg_addr: int, value: int)` / `write_registers(start_addr: int, values: list[int])` | Write one value at `reg_addr` / consecutive values from `start_addr` (unsupported). | `False` |
