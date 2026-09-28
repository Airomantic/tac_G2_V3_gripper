#!/usr/bin/env python3
"""Verify ROS -> bridge -> real gripper at positions 500 and 1000.

Requires --execute and a sourced ROS 2 Jazzy environment. Uses ROS domain 219
and a unique local topic. Only the bridge sends motor commands; this process
publishes trigger samples and reads existing server feedback without Connect.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--ip", default="192.168.127.10")
    args = parser.parse_args()
    if not args.execute:
        parser.error("Real hardware motion requires --execute")

    os.environ["ROS_DOMAIN_ID"] = "219"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = "/tmp/v3_gripper_hardware_logs"
    import grpc
    from google.protobuf.empty_pb2 import Empty
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray
    from DM_V3Gripper import gripper_pb2_grpc as rpc

    root = Path(__file__).resolve().parents[1]
    rclpy.init(args=[])
    node = Node(f"v3_gripper_hardware_publisher_{os.getpid()}")
    topic = f"/v3_gripper_hardware_{os.getpid()}/trigger_positions"
    publisher = node.create_publisher(Float64MultiArray, topic, qos_profile_sensor_data)
    sample = [0.0, 0.0]
    node.create_timer(0.05, lambda: publisher.publish(Float64MultiArray(data=sample)))

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    channel = grpc.insecure_channel(f"{args.ip}:50008")
    stub = rpc.GripperServiceStub(channel)
    child = None
    success = False
    results = []

    def feedback():
        if child.poll() is not None:
            raise RuntimeError(f"Bridge exited with status {child.returncode}")
        state = stub.GetAllState(Empty(), timeout=1)
        if not state.valid or not state.connected or state.stale or state.error != 0:
            raise RuntimeError(f"Invalid motor feedback: {state}")
        if state.init_state != 1 or not 0 <= state.position <= 1000:
            raise RuntimeError(f"Uninitialized or invalid motor position: {state}")
        return state

    try:
        child = subprocess.Popen(
            [str(root / "teleop_gripper.sh"), "--execute", "--topic", topic,
             "--ip", args.ip, "--hand", "right", "--speed", "10", "--force", "20"],
            cwd=root, env=os.environ.copy(), start_new_session=True)
        deadline = time.monotonic() + 10
        while True:
            if child.poll() is not None:
                raise RuntimeError("Bridge exited during startup")
            state = stub.GetAllState(Empty(), timeout=1)
            if publisher.get_subscription_count() and state.connected and state.valid:
                feedback()
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Bridge did not become ready within 10 s")
            time.sleep(0.1)
        print("ROS_HARDWARE_READY", topic, flush=True)
        for trigger, target in ((0.5, 500), (0.0, 1000)):
            sample = [0.0, trigger]
            started = time.monotonic()
            reached = 0
            while True:
                state = feedback()
                if abs(state.position - target) <= 5 and state.grip_state == 1:
                    reached += 1
                else:
                    reached = 0
                if reached >= 2:
                    result = {"trigger": trigger, "target": target,
                              "actual": state.position, "error": state.error,
                              "elapsed_s": round(time.monotonic() - started, 3)}
                    results.append(result)
                    print("ROS_TARGET_REACHED", json.dumps(result), flush=True)
                    break
                if time.monotonic() - started > 15:
                    raise TimeoutError(f"ROS target {target} was not reached: {state}")
                time.sleep(0.1)
        success = True
    finally:
        if child is not None:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    # Emergency cleanup only, never Connect or Resume.
                    try:
                        stopped = stub.Stop(Empty(), timeout=2)
                        print("CLEANUP_STOP", stopped.success, stopped.message, flush=True)
                    finally:
                        child.kill()
                        child.wait()
                    success = False
            if child.returncode != 0:
                success = False
                print("BRIDGE_EXIT_CODE", child.returncode, flush=True)
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()
        channel.close()
    if success:
        print("ROS_HARDWARE_TEST_PASS", json.dumps(results), flush=True)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
