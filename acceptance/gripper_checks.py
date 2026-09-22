"""Gripper checks with feedback validation and explicit motion cleanup."""

import contextlib
import time

from .common import CheckFailed, CheckSkipped, confirm, describe_error, distribution, ready, require


SPEEDS = tuple(range(5, 40, 2)) + tuple(range(40, 5, -2))


def initialized_move(client, target, timeout_s):
    """No movement is allowed after rejected or timed-out initialization."""
    return (client.initialize() and client.wait_until_initialized(timeout_s)
            and client.SetTargetPosition(target))


class Session:
    def __init__(self, ctx, **overrides):
        from DM_V3Gripper import Gripper
        self.ctx = ctx
        self.limits = ctx.config["limits"]
        g = ctx.config["gripper"]
        params = dict(remote_ip=g["ip"], remote_port=g["port"], channel=g["channel"],
                      motor_id=g["motor_id"])
        params.update(overrides)
        self.client = Gripper(**params)
        self.armed = False
        self.rpc_times = {}

    def __enter__(self):
        try:
            self.command("connect")
        except BaseException:
            self.invoke("disconnect")
            raise
        return self

    def __exit__(self, exc_type, exc, tb):
        failures = []
        commands = (["Stop", "disable"] if self.armed else []) + ["disconnect"]
        for name in commands:
            try:
                self.command(name)
            except BaseException as error:
                self.ctx.report.event("cleanup_exception", method=name, diagnostic=str(error))
                failures.append(f"{name}：{describe_error(error)}")
        self.armed = False
        self.ctx.report.event("gripper_rpc_summary", methods={name: distribution(values) for name, values in self.rpc_times.items()})
        if failures:
            message = "资源清理失败：" + "; ".join(failures)
            self.ctx.report.event("cleanup_failure", message=message)
            print(message)
            # A cleanup failure must never become a successful or skipped check.
            if exc_type is not KeyboardInterrupt:
                raise CheckFailed(message) from exc

    def invoke(self, name, *args):
        path = self.ctx.report.directory / f"{self.ctx.report.case}.gripper-sdk.log"
        with path.open("a", encoding="utf-8") as log:
            with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                return getattr(self.client, name)(*args)

    def connection_failure(self, diagnostic, elapsed):
        address = self.client.remote_addr
        if diagnostic.strip() == "remote connection error:" and elapsed >= 2.9:
            # Older clients omit the text of grpc.FutureTimeoutError.
            reason = "等待 gRPC 服务连接就绪失败"
            advice = "请检查边缘主机、网络和夹爪服务；若刚重启设备，请等待服务启动完成后重试。"
        elif "DEADLINE_EXCEEDED" in diagnostic:
            reason = "夹爪服务的连接请求响应超时"
            advice = "请检查服务端日志及 CAN 通信状态。"
        elif "has no response" in diagnostic:
            reason = f"服务已响应，但电机 ID {self.client.motor_id} 没有反馈"
            advice = f"请检查电机 ID、夹爪供电及 {self.client.channel} 的 CAN 接线和配置。"
        elif "UNAVAILABLE" in diagnostic:
            reason = "夹爪服务暂时不可用"
            advice = "请确认服务仍在运行，并检查网络连接。"
        else:
            reason = "夹爪服务未能建立连接"
            advice = "请检查服务地址、服务状态及电机配置。"
        log = self.ctx.report.directory / f"{self.ctx.report.case}.gripper-sdk.log"
        return f"连接 {address} 失败（耗时 {elapsed:.2f} 秒）：{reason}。{advice} 详细日志：{log}"

    def command(self, name, *args):
        if name in ("enable", "initialize", "home", "SetTargetPosition", "SetJog", "SetFollowMode"):
            self.armed = True  # Even a timed-out command might have reached the device.
        start = time.monotonic()
        result = self.invoke(name, *args)
        elapsed = time.monotonic() - start
        diagnostic = "" if result else self.client.GetLastError()
        self.rpc_times.setdefault(name, []).append(elapsed)
        self.ctx.report.event("rpc", method=name, args=args, success=bool(result), duration_s=elapsed,
                              message=diagnostic)
        if name == "connect" and result is not True:
            raise CheckFailed(self.connection_failure(diagnostic, elapsed))
        require(result is True, f"调用 {name} 失败，详细诊断请查看事件日志或夹爪 SDK 日志。")
        if name == "connect":
            require(elapsed <= self.limits["connect_timeout_s"], "连接耗时超过设定上限。")
        return start, elapsed

    def state(self, after=None, allow_error=False, allow_pending=False, check_position=False):
        start = time.monotonic()
        state = self.invoke("get_state")
        elapsed = time.monotonic() - start
        self.rpc_times.setdefault("get_state", []).append(elapsed)
        self.ctx.report.event("feedback", duration_s=elapsed, **state)
        if allow_pending and not (state["connected"] and state["valid"] and not state["stale"]):
            return None
        require(state["connected"] and state["valid"] and not state["stale"],
                f"反馈无效或已过期，详细诊断请查看事件日志。")
        # Position bounds apply to motion feedback, not calibration.
        if check_position:
            require(0 <= state["position"] <= 1000, f"位置反馈无效：{state['position']}")
        if not allow_error:
            require(state["error"] == 0, f"设备报告错误，错误码：{state['error']}；详细诊断请查看事件日志。")
        if after is not None and state["timestamp"] < after:
            return None
        return state

    def wait(self, predicate, timeout, after=0, allow_pending=False, check_position=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.state(after=after, allow_pending=allow_pending, check_position=check_position)
            if state is not None and predicate(state):
                return state
            time.sleep(0.05)
        raise CheckFailed(f"等待反馈满足条件超时，时限为 {timeout} 秒。")

    def initialize(self):
        self.command("initialize")
        print("[夹爪] 初始化指令已发送，正在等待完成反馈……", flush=True)
        deadline = time.monotonic() + self.limits["init_timeout_s"]
        # Allow the feedback cache to update, then check valid/init_state.
        time.sleep(0.1)
        next_log = 0.0
        while True:
            state = self.state(allow_pending=True)
            now = time.monotonic()
            if state is not None and state["init_state"] == 1:
                print(f"[夹爪] 初始化完成，当前位置反馈={state['position']}。", flush=True)
                return state
            if now >= deadline:
                raise CheckFailed(f"夹爪初始化超时，等待时间为 {self.limits['init_timeout_s']} 秒。")
            if now >= next_log:
                detail = (f"初始化状态={state['init_state']}，位置反馈={state['position']}"
                          if state is not None else "正在等待有效反馈")
                print(f"[夹爪] 初始化中：{detail}。", flush=True)
                next_log = now + 1
            time.sleep(0.05)

    def prepare(self, speed=None):
        # Calibration belongs to the initialization checks, not every motion check.
        state = self.state()
        require(state["init_state"] == 1,
                f"夹爪尚未完成初始化（状态 {state['init_state']}）；请先执行初始化测试，本项不会自动初始化。")
        self.command("enable")
        self.command("SetTargetForce", self.ctx.config["gripper"]["force"])
        self.command("SetTargetSpeed", self.ctx.config["gripper"]["test_speed"] if speed is None else speed)

    def move(self, target):
        issued, _ = self.command("SetTargetPosition", target)
        state = self.wait(lambda s: abs(s["position"] - target) <= self.limits["position_tolerance"],
                          self.limits["move_timeout_s"], issued, check_position=True)
        result = {"target": target, "position": state["position"],
                  "error": state["position"] - target, "elapsed_s": time.monotonic() - issued}
        self.ctx.report.event("arrival", **result)
        return result

    def stopped(self):
        issued, _ = self.command("Stop")
        time.sleep(self.limits["stop_settle_s"])
        values = []
        until = time.monotonic() + 1
        while time.monotonic() < until:
            state = self.state(after=issued, check_position=True)
            if state:
                values.append(state["position"])
            time.sleep(0.05)
        require(len(values) >= 2, "停止后未获得足够的新鲜反馈。")
        require(max(values) - min(values) <= self.limits["position_tolerance"],
                "停止指令发出并等待稳定后，位置仍在变化。")
        return {"position_span": max(values) - min(values), "samples": len(values)}


class Sweep:
    def __init__(self, session, fixed_speed=None):
        self.session = session
        self.fixed_speed = fixed_speed
        self.start = time.monotonic()
        self.last_tick = 0
        self.last_send = self.start
        self.target = 0
        self.leg_start = self.start
        self.legs = 0
        self.missed = 0
        self.intervals = []
        self.jitter = [0.0]
        speed = 5 if fixed_speed is None else fixed_speed
        self.speeds = {speed}
        self.next_log = self.start
        sent, elapsed = session.command("SetTargetSpeed", speed)
        self.start = self.last_send = self.leg_start = sent
        session.ctx.report.event("speed_change", command_speed=speed, interval_s=None,
                                 schedule_lateness_s=0.0, rpc_s=elapsed)
        session.command("SetTargetPosition", self.target)

    def tick(self):
        now = time.monotonic()
        tick = int((now - self.start) / 0.5)
        speed = SPEEDS[tick % len(SPEEDS)] if self.fixed_speed is None else self.fixed_speed
        if self.fixed_speed is None and tick != self.last_tick:
            self.missed += max(0, tick - self.last_tick - 1)
            sent, elapsed = self.session.command("SetTargetSpeed", speed)
            self.jitter.append(sent - (self.start + tick * 0.5))
            interval = None if self.last_send is None else sent - self.last_send
            if interval is not None:
                self.intervals.append(interval)
            self.session.ctx.report.event("speed_change", command_speed=speed, interval_s=interval,
                                          schedule_lateness_s=self.jitter[-1], rpc_s=elapsed)
            self.last_tick, self.last_send = tick, sent
            self.speeds.add(speed)
        state = self.session.state(check_position=True)
        if abs(state["position"] - self.target) <= self.session.limits["position_tolerance"]:
            self.target = 1000 - self.target
            self.session.command("SetTargetPosition", self.target)
            self.leg_start = time.monotonic()
            self.legs += 1
        require(time.monotonic() - self.leg_start <= self.session.limits["move_timeout_s"],
                "往返运动未在规定时间内到达端点。")
        if now >= self.next_log:
            print(f"[夹爪] 指令速度={speed} "
                  f"位置={state['position']} 目标={self.target}")
            self.next_log = now + 1

    def metrics(self):
        return {"completed_legs": self.legs, "missed_speed_updates": self.missed,
                "speed_values": sorted(self.speeds), "send_intervals_s": distribution(self.intervals),
                "schedule_lateness_s": distribution(self.jitter)}


def connection(ctx):
    for _ in range(3):
        with Session(ctx) as session:
            session.command("connect")
            session.state()
        require(session.invoke("disconnect"), "重复断开连接失败。")
        with session:
            session.state()
    return {"connection_cycles": 6, "repeated_connect_disconnect": True}


def negative_connection(ctx, field):
    g = ctx.config["gripper"]
    names = {"ip": "remote_ip", "port": "remote_port", "motor_id": "motor_id"}
    session = Session(ctx, **{names[field]: g["wrong_" + field]})
    started = time.monotonic()
    try:
        result = session.invoke("connect")
        elapsed = time.monotonic() - started
        message = session.client.GetLastError()
        ctx.report.event("negative_connection", field=field, success=result, duration_s=elapsed, message=message)
        require(not result, f"服务错误地接受了无效的 {field} 配置。")
        require(bool(message.strip()), "连接失败时未返回错误信息。")
        require(elapsed <= ctx.config["limits"]["connect_timeout_s"], "连接失败的返回耗时超过设定上限。")
        return {"duration_s": elapsed, "message": message}
    finally:
        session.invoke("disconnect")


def service_off(ctx):
    ready("请在另一个终端停止夹爪服务，并保持传感器服务运行")
    session = Session(ctx)
    failure = None
    try:
        start = time.monotonic()
        result = session.invoke("connect")
        require(not result, "预期服务已停止，但连接仍然成功。")
        require(bool(session.client.GetLastError()), "未报告连接错误。")
        require(time.monotonic() - start <= ctx.config["limits"]["connect_timeout_s"], "连接超时返回过晚。")
    except CheckFailed as exc:
        failure = str(exc)
    finally:
        session.invoke("disconnect")
    ready("请重新启动夹爪服务")
    with Session(ctx) as session:
        session.state()
    require(failure is None, failure)
    return {"service_restart": "reconnected without motion"}


def initialization(ctx, already_initialized):
    if not already_initialized:
        ready("请按正常上电流程准备未初始化的夹爪，并清空行程区域；若重启了边缘主机，请等待夹爪服务恢复后再选择“就绪”")
    with Session(ctx) as session:
        if already_initialized:
            session.command("enable")
            initial = session.state(allow_pending=True, allow_error=True)
            if initial is None or initial["init_state"] != 1:
                print("[夹爪] 当前尚未完成初始化，先完成一次初始化，再测试重复初始化。", flush=True)
                initial = session.initialize()
        else:
            initial = session.state()
            if initial["init_state"] == 1:
                raise CheckSkipped("夹爪已完成初始化；此项未覆盖，请选择下一项“从已初始化状态再次执行初始化”。")
            require(initial["init_state"] in (0, 2),
                    f"初始化状态码为 {initial['init_state']}；本项要求状态 0（未初始化）或 2（初始化中）。")
            session.command("enable")
        ctx.report.event("initialization_start", initial_state=initial["init_state"], position=initial["position"])
        state = session.initialize()
    confirm("夹爪是否实际完成了完整初始化，且没有异常运动？")
    return {"initial_state": initial["init_state"], "final_state": state["init_state"]}


def positions(ctx):
    results = []
    with Session(ctx) as session:
        session.prepare()
        for target in (1000, 0, 1000):
            result = session.move(target)
            results.append(result)
            print(f"目标位置 {target}：误差={result['error']}，耗时={result['elapsed_s']:.3f} 秒")
    return {"arrivals": results}


def speed(ctx):
    with Session(ctx) as session:
        session.prepare()
        session.move(1000)
        sweep = Sweep(session)
        while time.monotonic() - sweep.start < ctx.config["limits"]["speed_duration_s"]:
            sweep.tick()
            time.sleep(0.02)
        result = sweep.metrics()
    ctx.report.event("sweep_summary", **result)
    require(result["completed_legs"] >= 2, "完成的完整端点运动少于两次。")
    require({5, 40}.issubset(sweep.speeds), "变速过程未覆盖最低和最高速度。")
    require(sweep.missed == 0, "RPC 延迟导致部分速度档位未按计划发送。")
    require(max(sweep.jitter) <= ctx.config["limits"]["maximum_speed_jitter_s"], "速度指令的调度延迟超过设定上限。")
    return result


def stop_control(ctx):
    with Session(ctx) as session:
        session.prepare(speed=20)
        session.move(500)
        issued, _ = session.command("SetTargetPosition", 1000)
        session.wait(lambda s: abs(s["position"] - 500) > session.limits["position_tolerance"], 5, issued, check_position=True)
        result = session.stopped()
        session.command("disable")
    confirm("夹爪是否已停止、未自动恢复运动，且驱动已禁用？")
    return result


def other_control(ctx, control):
    control_label = {"home": "回零", "clear": "清错", "jog": "点动", "follow": "位置跟随"}[control]
    with Session(ctx) as session:
        if control != "clear":
            session.prepare()
        if control == "home":
            issued, _ = session.command("home")
            session.wait(lambda s: s["init_state"] == 1 and s["grip_state"] != 0,
                         session.limits["init_timeout_s"], issued)
        elif control == "clear":
            before = session.state(allow_error=True)["error"]
            session.command("Clear_error")
            require(session.state()["error"] == 0, "设备错误码未清零。")
            return {"error_before": before, "error_after": 0,
                    "scope": "no-error acknowledgement" if before == 0 else "active error cleared"}
        elif control == "jog":
            session.move(500)
            session.command("SetTargetSpeed", 5)
            changes = []
            try:
                for direction in (1, -1):
                    before = session.state(check_position=True)["position"]
                    session.command("SetJog", direction)
                    time.sleep(0.3)
                    session.command("SetJog", 0)
                    time.sleep(0.2)
                    changes.append(session.state(check_position=True)["position"] - before)
            finally:
                session.command("SetJog", 0)
            require(changes[0] * changes[1] < 0, "两个方向的点动指令未产生相反方向的位置变化。")
        elif control == "follow":
            session.move(500)
            try:
                session.command("SetFollowMode", True)
                session.move(450)
                session.move(550)
            finally:
                session.command("SetFollowMode", False)
    confirm(f"实际的{control_label}动作是否与指令一致？")
    return {"control": control, "physical_observation": "pass"}


def feedback_recovery(ctx):
    with Session(ctx) as session:
        session.command("Stop")
        session.command("disable")
        session.state()
        ready("选择“就绪”后，请在故障观察时间内停止夹爪服务")
        deadline = time.monotonic() + ctx.config["limits"]["fault_window_s"]
        detected = False
        while time.monotonic() < deadline:
            state = session.invoke("get_state")
            ctx.report.event("fault_feedback", **state)
            if not state["valid"] and state["stale"]:
                detected = True
                break
            time.sleep(0.05)
        # Closing a failed remote connection may legitimately return False here.
        ready("请恢复夹爪服务；程序将重新连接，但不会启用运动")
        session.invoke("disconnect")
        session.command("connect")
        session.state()
        require(detected, "服务停止后，反馈未按预期标记为过期且无效。")
    return {"stale_feedback_detected": True, "reconnected": True, "automatic_motion": False}
