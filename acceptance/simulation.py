"""Deterministic checks using the installed client and a simulated RPC service."""

import contextlib
import importlib
import io
from unittest.mock import patch

from .common import CheckFailed, bucket, complete_seconds, report_fps, require
from .gripper_checks import initialized_move


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


@contextlib.contextmanager
def simulated_client():
    import grpc
    from DM_V3Gripper import Gripper, gripper_pb2 as pb
    module = importlib.import_module("DM_V3Gripper.gripper")
    clock = Clock()

    class RpcError(grpc.RpcError):
        def __init__(self, code):
            self.status = code

        def code(self):
            return self.status

        def details(self):
            return "Simulated " + self.status.name

        def __str__(self):
            return self.details()

    class Service:
        def __init__(self):
            self.calls = []
            self.fail = set()
            self.errors = {}
            self.delays = {}
            self.ready = True
            self.init_finishes = True
            self.state = pb.StateResponse(valid=True, connected=True, init_state=0,
                                          grip_state=1, position=500, current=12,
                                          error=0, update_age_ms=0, message="OK")

        def __getattr__(self, name):
            def rpc(request, timeout):
                self.calls.append((name, request, timeout))
                delay = self.delays.get(name, 0)
                clock.sleep(min(delay, timeout))
                if delay > timeout:
                    raise RpcError(grpc.StatusCode.DEADLINE_EXCEEDED)
                if name in self.errors:
                    raise RpcError(self.errors[name])
                if name == "GetAllState":
                    return self.state
                success = name not in self.fail
                if name == "Connect":
                    success = success and request.motor_id == 1
                if success and name == "Initialization":
                    self.state.init_state = 1 if self.init_finishes else 2
                if success and name == "SetTargetPosition":
                    self.state.position = request.value
                if success and name in ("Open", "Close"):
                    self.state.position = 1000 if name == "Open" else 0
                if success and name == "ClearError":
                    self.state.error = 0
                return pb.Response(success=success, message="OK" if success else "Simulated rejection", state=self.state)
            return rpc

    class Channel:
        def close(self):
            pass

    service = Service()

    class Ready:
        def result(self, timeout):
            if not service.ready:
                clock.sleep(timeout)
                raise grpc.FutureTimeoutError()

    with contextlib.ExitStack() as stack:
        stack.enter_context(patch.object(module, "time", clock))
        stack.enter_context(patch.object(module.grpc, "insecure_channel", lambda address: Channel()))
        stack.enter_context(patch.object(module.grpc, "channel_ready_future", lambda channel: Ready()))
        stack.enter_context(patch.object(module.gripper_pb2_grpc, "GripperServiceStub", lambda channel: service))
        client = Gripper()
        try:
            yield client, service, clock
        finally:
            client.disconnect()


def contracts(ctx):
    passed = []

    def check(name, condition):
        require(condition, name)
        passed.append(name)

    with simulated_client() as (client, service, clock):
        check("未连接时的控制指令应失败", client.SetTargetPosition(1) is False)
        check("未连接时不应向服务发送控制指令", not service.calls)
        check("未知的初始反馈应标记为无效", not client.get_state()["valid"])
        check("连接应成功", client.connect() is True)
        check("重复连接不应重复建立连接", client.connect() and sum(n == "Connect" for n, _, _ in service.calls) == 1)
        for name, cases in {
            "SetTargetPosition": ((-1, 0), (0, 0), (500, 500), (1001, 1000)),
            "SetTargetSpeed": ((0, 1), (5, 5), (40, 40), (101, 100)),
            "SetTargetForce": ((0, 20), (20, 20), (101, 100)),
        }.items():
            for supplied, expected in cases:
                check(f"{name}({supplied}) 应截断为 {expected}", getattr(client, name)(supplied) and service.calls[-1][1].value == expected)
        before = len(service.calls)
        check("无效点动方向应返回失败", client.SetJog(2) is False and len(service.calls) == before)
        for direction in (-1, 0, 1):
            check(f"点动方向应为 {direction}", client.SetJog(direction) and service.calls[-1][1].value == direction)
        for enabled in (True, False):
            check(f"跟随模式应为 {enabled}", client.SetFollowMode(enabled) and service.calls[-1][1].value == int(enabled))
        for name in ("initialize", "Initialization", "calibrate"):
            check(f"初始化别名 {name} 应行为一致", getattr(client, name)() and service.calls[-1][0] == "Initialization")
        check("状态接口别名应返回一致结果", client.get_state()["position"] == client.GetAllState()["position"])
        check("电流与力反馈别名均应返回电机电流", client.GetCurrentCurrent() == client.GetCurrentForce() == 12)
        check("客户端不应提供当前速度查询接口", not hasattr(client, "GetCurrentSpeed"))
        check("状态快照不应包含速度反馈", "speed" not in client.get_state() and "speed" not in client.GetAllState())
        for method, field in (("GetCurrentPosition", "position"),
                              ("GetInitState", "init_state"), ("GetGripState", "grip_state"), ("GetErrorCode", "error")):
            check(f"反馈接口 {method} 应返回对应字段", getattr(client, method)() == getattr(service.state, field))
        check("初始化状态判断应正确", client.is_initialized())
        service.state.grip_state = 0
        check("运动状态判断应正确", client.is_moving())
        service.state.error = 32
        check("错误状态判断与清错应正确", client.has_error() and client.Clear_error() and not client.has_error())
        check("本地状态码解码应正确", client.decode_init_state(1) == "initialized" and
              client.decode_grip_state(0) == "moving" and client.decode_error(0) == "no error" and
              "999" in client.decode_error(999))
        before = len(service.calls)
        check("寄存器兼容接口返回值应正确", client.read_registers(0) is None and
              client.write_register(0, 1) is False and client.write_registers(0, [1]) is False)
        check("寄存器兼容接口不应发送 RPC", len(service.calls) == before)
        for name in ("enable", "disable", "home", "Open", "Close", "Stop"):
            check(f"{name} 应返回布尔值", getattr(client, name)() is True)
        check("张开和闭合操作应等待到达对应端点", client.Open(wait=True) and client.Close(wait=True))
        service.fail.add("SetTargetSpeed")
        before = len(service.calls)
        check("速度设置失败后不得发送位置指令", client.move_to(200, speed=10) is False and
              [name for name, _, _ in service.calls[before:]] == ["SetTargetSpeed"])
        message = client.GetLastError()
        service.fail.clear()
        check("设置可选参数后的运动应成功", client.move_to(200, speed=10, force=20, wait=True))
        check("后续操作成功不应清除最近一次错误记录", bool(message) and client.GetLastError() == message and client.GetLastMessage())
        check("等待超时应返回 False", client.wait_until_position(900, timeout_s=0.1) is False)
        check("不得自动调用 Resume", all(name != "Resume" for name, _, _ in service.calls))
        client.disconnect()
        client.motor_id = 999
        check("错误的设备 ID 应被拒绝", client.connect() is False and bool(client.GetLastError()))
        client.motor_id = 1
        check("恢复正确配置后应能连接", client.connect())
    ctx.report.event("simulated_assertions", checks=passed)
    return {"mode": "simulated RPC service; no hardware", "checks": passed}


def faults(ctx):
    import grpc
    passed = []
    with simulated_client() as (client, service, clock):
        service.ready = False
        start = clock.now
        require(not client.connect() and clock.now - start == 3, "服务不可用时，未按连接就绪超时要求返回失败。")
        service.ready = True
        require(client.connect(), "模拟重连失败。")
        passed.append("Unavailable service and reconnect")
        service.fail.add("Initialization")
        before = len(service.calls)
        require(not initialized_move(client, 1000, 0.1), "初始化被拒绝后仍错误地返回成功。")
        require(not any(n == "SetTargetPosition" for n, _, _ in service.calls[before:]), "初始化失败后仍发送了运动指令。")
        service.fail.clear()
        service.init_finishes = False
        before = len(service.calls)
        require(not initialized_move(client, 1000, 0.1), "初始化尚未完成却错误地返回成功。")
        require(not any(n == "SetTargetPosition" for n, _, _ in service.calls[before:]), "初始化超时后仍发送了运动指令。")
        service.init_finishes = True
        require(initialized_move(client, 1000, 0.1), "完成初始化后的运动失败。")
        passed.append("Initialization rejection/timeout blocks movement; recovery succeeds")
        for code in (grpc.StatusCode.UNAVAILABLE, grpc.StatusCode.UNIMPLEMENTED):
            service.errors["Enable"] = code
            require(client.enable() is False and code.name in client.GetLastError(), "RPC 错误未被正确报告。")
        service.errors.clear()
        passed.append("RPC unavailable and unsupported-method failures")
        service.delays["Enable"] = 6
        start = clock.now
        require(client.enable() is False and clock.now - start == 5, "延迟指令未遵守 RPC 超时限制。")
        service.delays.clear()
        require(client.get_state()["valid"], "初始状态无效。")
        service.errors["GetAllState"] = grpc.StatusCode.UNAVAILABLE
        clock.sleep(0.6)
        state = client.get_state()
        require(not state["valid"] and state["stale"], "断流后旧缓存仍被标记为有效。")
        service.errors.clear()
        require(client.get_state()["valid"], "新鲜状态未恢复。")
        passed.append("Delayed RPC deadline, stale cache, and fresh feedback recovery")
        service.delays["GetAllState"] = 3
        start = clock.now
        require(client.refresh_state() is False and clock.now - start == 2, "状态读取 RPC 的超时检查失败。")
        passed.append("Status read has a two-second RPC deadline")
    ctx.report.event("simulated_assertions", checks=passed)
    return {"mode": "virtual time and simulated RPC faults", "checks": passed}


def fps_logic(ctx):
    # Exercise the same timing and reporting helpers as hardware acquisition.
    from .sensor_checks import FrameTracker
    import numpy as np
    start = 100.0
    for now, expected in ((104.9, None), (105, 0), (105.99, 0), (106, 1), (124.99, 19), (125, None)):
        require(bucket(start, now) == expected,
                f"时间 {now} 对应的 FPS 统计区间不正确。")
    require(bucket(0, 106) is None and complete_seconds(0, 106) == 0, "传感器就绪前就开始了 FPS 统计。")
    for now, expected in ((104.9, 0), (105.9, 0), (106, 1), (109.4, 4), (125, 20), (200, 20)):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            reported = report_fps({0: [120] * 20, 1: [120] * 20}, start, now, 0)
        require(reported == expected == complete_seconds(start, now), "错误地计入了不足一秒的区间。")
        require(len(output.getvalue().splitlines()) == expected, "FPS 输出行数不正确。")
    with contextlib.redirect_stdout(io.StringIO()) as output:
        require(report_fps({0: [120] * 20}, start, 109.9, 4) == 4, "未满一秒时统计结果发生变化。")
        require(report_fps({0: [120] * 20}, 0, 200, 0) == 0, "传感器就绪前就输出了 FPS。")
    require(not output.getvalue(), "错误地重复输出了 FPS 或计入了不足一秒的区间。")
    tracker = FrameTracker()
    flow, depth = np.zeros((4, 4, 2), np.float32), np.zeros((4, 4), np.float32)
    require(tracker.observe(1, flow, 1, depth, 105) is not None, "首帧被错误地丢弃。")
    require(tracker.observe(1, flow, 1, depth, 105.1) is None, "重复帧被错误地计数。")
    require(tracker.observe(2, flow, 1, depth, 105.2) is None, "包含重复数据的帧对被错误地计数。")
    require(tracker.observe(2, flow, 2, depth, 105.3) is not None, "新帧被错误地丢弃。")
    require(tracker.observe(3, None, 3, depth, 105.4) is None, "数据缺失的帧被错误地计数。")
    rejected = 0
    for bad_flow, bad_depth in ((np.full_like(flow, np.nan), depth), (flow, np.full_like(depth, np.inf)),
                                (np.zeros((2, 2), np.float32), depth), (flow, np.zeros((3, 3), np.float32))):
        try:
            tracker.observe(3, bad_flow, 3, bad_depth, 105.5)
        except CheckFailed:
            rejected += 1
    require(rejected == 4, "错误地接受了格式异常或包含非有限值的帧。")
    other = FrameTracker()
    require(other.observe(900, flow, 901, depth, 105) is not None,
            "不能把不同传感器或模态的独立帧 ID 当作全局同步依据。")
    return {"mode": "synthetic frames and timestamps", "normal_seconds": 20,
            "early_exit_seconds": 4, "invalid_frame_cases": rejected, "duplicate_pairs": "excluded"}
