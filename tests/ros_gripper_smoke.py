"""Local ROS 2 -> bridge -> gRPC smoke test; never connects to real hardware.

Run using .venv-ros/bin/python tests/ros_gripper_smoke.py in a Jazzy environment.
Socket access is required. All RPCs go to a simulated service on 127.0.0.1.
"""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import grpc
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from DM_V3Gripper import gripper_pb2 as pb, gripper_pb2_grpc as rpc


class SimulatedService(rpc.GripperServiceServicer):
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()
        self.position = 500

    def status(self):
        return pb.StateResponse(connected=True, valid=True, stale=False, init_state=1,
                                error=0, position=self.position, update_age_ms=0)

    def GetAllState(self, request, context):
        return self.status()

    def record(self, method, request):
        value = getattr(request, "value", None)
        with self.lock:
            self.calls.append((method, value))
            if method == "SetTargetPosition":
                self.position = value
        return pb.Response(success=True, message="local simulation", state=self.status())

    def recorded(self):
        with self.lock:
            return list(self.calls)


def make_rpc(method):
    return lambda self, request, context: self.record(method, request)


for rpc_name in ("Connect", "Disconnect", "Enable", "SetTargetSpeed",
                 "SetTargetForce", "SetTargetPosition", "Stop"):
    setattr(SimulatedService, rpc_name, make_rpc(rpc_name))


def main():
    os.environ["ROS_DOMAIN_ID"] = "219"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = "/tmp/codex_gripper_smoke_logs"
    root = Path(__file__).resolve().parents[1]
    service = SimulatedService()
    server = grpc.server(ThreadPoolExecutor(max_workers=2))
    rpc.add_GripperServiceServicer_to_server(service, server)
    port = server.add_insecure_port("127.0.0.1:0")
    assert port, "Cannot bind a local gRPC port"
    server.start()
    node = None
    child = None
    output = ""
    try:
        rclpy.init(args=[])
        node = Node(f"gripper_smoke_publisher_{os.getpid()}")
        topic = f"/v3_gripper_smoke_{os.getpid()}/trigger"
        publisher = node.create_publisher(Float64MultiArray, topic, qos_profile_sensor_data)
        child = subprocess.Popen(
            [str(root / "teleop_gripper.sh"), "--execute", "--topic", topic,
             "--ip", "127.0.0.1", "--port", str(port)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=os.environ.copy())
        for data, position in (([0.8, 0.0], 1000), ([0.2, 0.5], 500), ([0.7, 1.0], 0)):
            deadline = time.monotonic() + 10
            while ("SetTargetPosition", position) not in service.recorded():
                assert child.poll() is None, "bridge exited before receiving a target"
                assert time.monotonic() < deadline, f"target {position} was not received"
                publisher.publish(Float64MultiArray(data=data))
                rclpy.spin_once(node, timeout_sec=0.05)
            print(f"ROS {data} -> gRPC SetTargetPosition({position}): PASS", flush=True)
        # Source loss must stop the bridge instead of continuing or resuming motion.
        output, _ = child.communicate(timeout=10)
        assert child.returncode == 1, (child.returncode, output)
        calls = service.recorded()
        assert ("SetTargetSpeed", 10) in calls, calls
        assert ("SetTargetForce", 20) in calls, calls
        assert calls[-2:] == [("Stop", None), ("Disconnect", None)], calls
        assert "未更新" in output, output
        print("Source timeout -> Stop -> Disconnect: PASS", flush=True)
    finally:
        if child is not None:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    output, _ = child.communicate(timeout=10)
                except subprocess.TimeoutExpired:
                    child.kill()
                    output, _ = child.communicate()
            elif not output:
                output, _ = child.communicate()
            print(output)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        server.stop(0).wait()


if __name__ == "__main__":
    main()
