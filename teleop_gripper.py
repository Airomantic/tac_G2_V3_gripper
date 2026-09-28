#!/usr/bin/env python3
"""Receive Puppeteer trigger positions over ROS 2 and drive a DM V3 gripper."""

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass


class BridgeError(RuntimeError):
    """An input, feedback, or command failure that requires a restart."""


def trigger_to_position(data, hand="right", invert=False, offset=0):
    """The MessageBridge array is [left, right]; 1000 is fully open."""
    if hand not in ("left", "right"):
        raise ValueError("hand 必须为 left 或 right")
    if offset < 0 or len(data) - offset != 2:
        raise ValueError("扳机消息应包含 [左手, 右手] 两个数值")
    values = tuple(float(value) for value in data[offset:])
    if any(not math.isfinite(value) or not 0.0 <= value <= 1.0 for value in values):
        raise ValueError("扳机值必须为 0–1 的有限数值")
    trigger = values[0 if hand == "left" else 1]
    return round(1000 * (trigger if invert else 1.0 - trigger))


@dataclass(frozen=True)
class TriggerSample:
    position: int
    received_at: float


class TriggerInput:
    """One latest sample, shared between the ROS callback and the RPC worker."""

    def __init__(self, hand="right", invert=False, clock=time.monotonic):
        self.hand = hand
        self.invert = invert
        self.clock = clock
        self.lock = threading.Lock()
        self.sample = None
        self.error = None

    def update(self, data, offset=0):
        try:
            position = trigger_to_position(data, self.hand, self.invert, offset)
        except (TypeError, ValueError, OverflowError) as exc:
            # A later valid frame must not erase a fault before the worker sees it.
            with self.lock:
                self.error = str(exc)
            return
        with self.lock:
            self.sample = TriggerSample(position, self.clock())

    def read(self, timeout_s):
        with self.lock:
            sample, error = self.sample, self.error
        if error:
            raise BridgeError(error)
        if sample is not None and self.clock() - sample.received_at > timeout_s:
            raise BridgeError(f"扳机话题超过 {timeout_s:g} 秒未更新")
        return sample


def check_state(state, require_initialized=True):
    if not state.get("connected") or not state.get("valid") or state.get("stale"):
        raise BridgeError(f"夹爪反馈无效或过期：{state.get('message', '')}")
    if state.get("error") != 0:
        raise BridgeError(f"夹爪错误码 {state.get('error')}：{state.get('error_text', '')}")
    if require_initialized:
        if state.get("init_state") != 1:
            raise BridgeError("夹爪未初始化；可使用 --execute --initialize 完成一次初始化")
        if not 0 <= state.get("position", -1) <= 1000:
            raise BridgeError(f"夹爪位置反馈超出 0–1000：{state.get('position')}")


def connection_error(client):
    """Distinguish a server-side CAN rejection from a network/RPC failure."""
    detail = client.GetLastError().strip()
    address = client.remote_addr
    if "has no response" in detail:
        return (f"gRPC 服务 {address} 已响应，但服务端 {client.channel} 上的夹爪 "
                f"ID {client.motor_id} 没有 CAN-FD 回复。请核对夹爪供电、接线、"
                "服务端 CAN 接口配置和实际电机 ID；连接成功前无法标定，"
                f"添加 --initialize 不会解决此错误。服务端原始信息：{detail}")
    if detail == "remote connection error:":
        return f"等待 gRPC 服务 {address} 就绪超时；请检查设备网络和夹爪服务是否运行"
    return f"连接 {address} 失败：{detail}"


class GripperBridge:
    """All gRPC calls run serially in one worker; ROS reception stays responsive."""

    def __init__(self, inputs, client=None, timeout_s=0.5, deadband=5,
                 clock=time.monotonic, cancelled=lambda: False):
        self.inputs = inputs
        self.client = client  # None means dry-run: no connection or hardware writes.
        self.timeout_s = timeout_s
        self.deadband = deadband
        self.clock = clock
        self.cancelled = cancelled
        self.last_position = None
        self.feedback = None
        self.next_feedback_at = 0.0
        self.armed = False

    def command(self, method, *args):
        if self.cancelled() and method not in ("Stop", "disconnect"):
            raise InterruptedError("桥接已停止")
        if getattr(self.client, method)(*args) is not True:
            if method == "connect":
                raise BridgeError(connection_error(self.client))
            raise BridgeError(f"{method} 失败：{self.client.GetLastError()}")

    def prepare(self, speed=10, force=20, initialize=False):
        if self.client is None:
            return
        self.command("connect")
        state = self.client.get_state()
        check_state(state, require_initialized=False)
        if state.get("init_state") != 1 and not initialize:
            check_state(state)
        # Even an RPC that times out may have enabled the motor remotely.
        self.armed = True
        self.command("enable")
        if state.get("init_state") != 1:
            if state.get("init_state") != 2:
                self.command("initialize")
            deadline = self.clock() + 15.0
            while True:
                if self.cancelled():
                    raise InterruptedError("初始化等待已取消")
                state = self.client.get_state()
                if state.get("connected") and state.get("valid") and not state.get("stale"):
                    check_state(state, require_initialized=False)
                    if state.get("init_state") == 1:
                        break
                if self.clock() >= deadline:
                    raise BridgeError("等待夹爪初始化超过 15 秒")
                time.sleep(0.05)
        check_state(self.client.get_state())
        self.command("SetTargetSpeed", speed)
        self.command("SetTargetForce", force)

    def step(self):
        sample = self.inputs.read(self.timeout_s)
        if self.client is not None and self.clock() >= self.next_feedback_at:
            self.feedback = self.client.get_state()
            check_state(self.feedback)
            self.next_feedback_at = self.clock() + 0.2
            # An RPC may take time. Recheck freshness and use the newest target.
            sample = self.inputs.read(self.timeout_s)
        if self.cancelled() or sample is None:
            return None
        target = sample.position
        if self.last_position is not None:
            delta = abs(target - self.last_position)
            if delta == 0 or (delta < self.deadband and target not in (0, 1000)):
                return target
        if self.client is not None:
            self.command("SetTargetPosition", target)
        self.last_position = target
        return target

    def close(self):
        """Attempt Stop even after a command failure, then release the connection."""
        failures = []
        if self.client is None:
            return failures
        commands = (["Stop"] if self.armed else []) + ["disconnect"]
        for method in commands:
            try:
                self.command(method)
            except Exception as exc:
                failures.append(str(exc))
        self.armed = False
        return failures


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/trigger_positions", help="扳机话题")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--invert", action="store_true", help="改为 0=闭合、1=张开")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="连接真机并发送开合指令")
    mode.add_argument("--check-connection", action="store_true",
                      help="仅连接、读状态并断开；不订阅 ROS、不使能或运动")
    parser.add_argument("--initialize", action="store_true", help="尚未初始化时执行标定（会运动）")
    parser.add_argument("--ip", default="192.168.127.10")
    parser.add_argument("--port", type=int, default=50008)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--speed", type=int, default=10, help="指令速度 1–100")
    parser.add_argument("--force", type=int, default=20, help="设备力档位 20–100")
    parser.add_argument("--rate", type=float, default=20.0, help="最大发送频率，Hz，1–50")
    parser.add_argument("--timeout", type=float, default=0.5, help="扳机接收超时，秒")
    parser.add_argument("--deadband", type=int, default=5, help="目标变化小于此值时不重复发送")
    args = parser.parse_args(argv)
    if args.initialize and not args.execute:
        parser.error("--initialize 需要同时指定 --execute")
    for name, low, high in (("speed", 1, 100), ("force", 20, 100),
                            ("rate", 1, 50), ("deadband", 0, 1000),
                            ("port", 1, 65535), ("motor_id", 1, 255)):
        if not low <= getattr(args, name) <= high:
            parser.error(f"--{name.replace('_', '-')} 必须在 {low}–{high} 之间")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout 必须为正的有限秒数")
    return args


def create_client(args):
    try:
        from DM_V3Gripper import Gripper
        return Gripper(remote_ip=args.ip, remote_port=args.port,
                       channel=args.channel, motor_id=args.motor_id)
    except ImportError as exc:
        raise BridgeError("缺少夹爪客户端；请先执行 ./teleop_gripper.sh setup") from exc


def check_connection(args):
    client = create_client(args)
    bridge = GripperBridge(TriggerInput(), client)
    print(f"连接检查：{client.remote_addr}，服务端 CAN={client.channel}，"
          f"电机 ID={client.motor_id}；不使能、不标定、不发送运动指令", flush=True)
    result = 0
    try:
        bridge.command("connect")
        state = client.get_state()
        print(f"反馈：connected={state.get('connected')}，valid={state.get('valid')}，"
              f"stale={state.get('stale')}，init_state={state.get('init_state')}，"
              f"position={state.get('position')}，error={state.get('error')}", flush=True)
        check_state(state, require_initialized=False)
        status = "已初始化" if state.get("init_state") == 1 else "尚未完成初始化"
        print(f"连接检查通过：CAN 反馈有效，{status}。", flush=True)
    except BridgeError as exc:
        print(f"连接检查失败：{exc}", file=sys.stderr)
        result = 1
    finally:
        for error in bridge.close():
            print(f"连接检查清理失败：{error}", file=sys.stderr)
            result = 1
    return result


def run(args):
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64MultiArray
    except ImportError as exc:
        raise BridgeError("无法导入 ROS 2 Jazzy；请通过 ./teleop_gripper.sh 启动") from exc

    inputs = TriggerInput(args.hand, args.invert)
    stop = threading.Event()
    client = create_client(args) if args.execute else None
    bridge = GripperBridge(inputs, client, args.timeout, args.deadband,
                           cancelled=stop.is_set)
    node = None
    worker = None
    failures = []

    def control_loop():
        try:
            bridge.prepare(args.speed, args.force, args.initialize)
            # Samples received during initialization/configuration are not commands.
            with inputs.lock:
                inputs.sample = None
            next_log_at = 0.0
            while not stop.is_set():
                target = bridge.step()
                if time.monotonic() >= next_log_at:
                    if target is None:
                        node.get_logger().info(f"等待 {args.topic} 的扳机数据……")
                    else:
                        mode = "执行" if args.execute else "预览"
                        actual = (f"，实际位置={bridge.feedback['position']}"
                                  if bridge.feedback is not None else "")
                        node.get_logger().info(f"{mode} {args.hand}：目标位置={target}{actual}")
                    next_log_at = time.monotonic() + 1.0
                stop.wait(1.0 / args.rate)
        except InterruptedError:
            pass
        except Exception as exc:
            failures.append(str(exc))
            node.get_logger().error(f"桥接停止：{exc}。处理后重新启动。")
        finally:
            stop.set()
            for error in bridge.close():
                failures.append(error)
                node.get_logger().error(error)

    try:
        rclpy.init(args=[])
        node = Node("v3_gripper_teleop")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(Float64MultiArray, args.topic,
                                 lambda msg: inputs.update(msg.data, msg.layout.data_offset), qos)
        mode = (f"真机 {args.ip}:{args.port}，CAN={args.channel}，电机 ID={args.motor_id}"
                if args.execute else "仅预览")
        node.get_logger().info(
            f"{mode}；订阅 {args.topic} [std_msgs/msg/Float64MultiArray]，"
            f"{args.hand}，invert={args.invert}，最高 {args.rate:g} Hz")
        worker = threading.Thread(target=control_loop, name="gripper-rpc")
        worker.start()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop.set()
        if worker is not None:
            worker.join()
        else:
            failures.extend(bridge.close())
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failures else 0


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.check_connection:
            return check_connection(args)
        return run(args)
    except BridgeError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
