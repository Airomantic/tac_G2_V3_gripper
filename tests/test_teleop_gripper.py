"""No-hardware checks for the trigger bridge and the real SDK's RPC contract."""

import contextlib
import io
import unittest
from unittest.mock import patch

from acceptance.simulation import simulated_client
import teleop_gripper
from teleop_gripper import BridgeError, GripperBridge, TriggerInput, trigger_to_position


class TriggerTests(unittest.TestCase):
    def test_hand_direction_and_endpoints(self):
        for trigger, expected in ((0.0, 1000), (0.25, 750), (0.5, 500), (1.0, 0)):
            with self.subTest(trigger=trigger):
                self.assertEqual(trigger_to_position([1.0, trigger]), expected)
                self.assertEqual(trigger_to_position([trigger, 1.0], "left"), expected)
                self.assertEqual(trigger_to_position([0.0, trigger], invert=True), 1000 - expected)
        self.assertEqual(trigger_to_position([42.0, 0.0, 0.25], offset=1), 750)

    def test_invalid_arrays_and_non_finite_values(self):
        for data in ([], [0.5], [0, 0, 0], [0, -0.01], [0, 1.01],
                     [float("nan"), 0], [0, float("inf")], [0, -float("inf")]):
            with self.subTest(data=data), self.assertRaises(ValueError):
                trigger_to_position(data)
        with self.assertRaises(ValueError):
            trigger_to_position([0, 1], offset=-1)

    def test_invalid_frame_is_latched_even_if_valid_frame_follows(self):
        inputs = TriggerInput()
        inputs.update([0, float("nan")])
        inputs.update([0, 0.5])
        with self.assertRaises(BridgeError):
            inputs.read(0.5)


class ConnectionCheckTests(unittest.TestCase):
    def test_check_reads_uncalibrated_feedback_without_ros_or_motion(self):
        with simulated_client() as (client, service, clock):
            service.state.position = 1200  # Valid before travel calibration.
            with patch.object(teleop_gripper, "create_client", return_value=client), \
                    patch.object(teleop_gripper, "run") as ros_run, \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                result = teleop_gripper.main(["--check-connection"])
            self.assertEqual(result, 0)
            self.assertIn("CAN 反馈有效", output.getvalue())
            self.assertIn("尚未完成初始化", output.getvalue())
            self.assertEqual([name for name, _, _ in service.calls],
                             ["Connect", "GetAllState", "Disconnect"])
            ros_run.assert_not_called()

    def test_rejected_connection_reports_can_failure_and_never_enables(self):
        with simulated_client() as (client, service, clock):
            service.fail.add("Connect")
            diagnostic = "gripper ID 1 has no response; check power, CAN-FD wiring, and interface settings"
            with patch.object(teleop_gripper, "create_client", return_value=client), \
                    patch.object(client, "GetLastError", return_value=diagnostic), \
                    contextlib.redirect_stderr(io.StringIO()) as output:
                result = teleop_gripper.main(["--check-connection"])
            self.assertEqual(result, 1)
            self.assertIn("gRPC 服务 192.168.127.10:50008 已响应", output.getvalue())
            self.assertIn("can0 上的夹爪 ID 1", output.getvalue())
            self.assertIn("连接成功前无法标定", output.getvalue())
            self.assertEqual([name for name, _, _ in service.calls], ["Connect"])

    def test_feedback_or_disconnect_failure_marks_check_failed(self):
        for failure in ("feedback", "disconnect"):
            with self.subTest(failure=failure), simulated_client() as (client, service, clock):
                if failure == "feedback":
                    service.state.valid = False
                else:
                    service.fail.add("Disconnect")
                with patch.object(teleop_gripper, "create_client", return_value=client), \
                        contextlib.redirect_stderr(io.StringIO()):
                    result = teleop_gripper.main(["--check-connection"])
                self.assertEqual(result, 1)
                self.assertEqual([name for name, _, _ in service.calls],
                                 ["Connect", "GetAllState", "Disconnect"])

    def test_check_cannot_be_combined_with_motion_flags(self):
        for flag in ("--execute", "--initialize"):
            with self.subTest(flag=flag), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    teleop_gripper.parse_args(["--check-connection", flag])


class BridgeTests(unittest.TestCase):
    def create_bridge(self, client, clock):
        inputs = TriggerInput(clock=clock.monotonic)
        bridge = GripperBridge(inputs, client, clock=clock.monotonic)
        return inputs, bridge

    def test_dry_run_and_waiting_do_not_require_a_client(self):
        inputs = TriggerInput()
        bridge = GripperBridge(inputs)
        bridge.prepare()
        self.assertIsNone(bridge.step())
        inputs.update([0, 0.25])
        self.assertEqual(bridge.step(), 750)
        self.assertEqual(bridge.close(), [])

    def test_execute_uses_latest_right_target_and_suppresses_noise(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.prepare()
            self.assertIsNone(bridge.step())
            inputs.update([0.0, 0.75])
            inputs.update([1.0, 0.5])
            self.assertEqual(bridge.step(), 500)
            inputs.update([0.0, 0.501])
            bridge.step()
            inputs.update([0.0, 0.6])
            bridge.step()
            positions = [req.value for name, req, _ in service.calls if name == "SetTargetPosition"]
            self.assertEqual(positions, [500, 400])
            names = [name for name, _, _ in service.calls]
            self.assertNotIn("Initialization", names)
            self.assertNotIn("Resume", names)
            self.assertEqual(bridge.close(), [])
            self.assertEqual([name for name, _, _ in service.calls[-2:]], ["Stop", "Disconnect"])

    def test_endpoints_are_not_lost_in_deadband(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.prepare()
            for trigger in (0.003, 0.0, 0.997, 1.0):
                inputs.update([0, trigger])
                bridge.step()
            positions = [req.value for name, req, _ in service.calls if name == "SetTargetPosition"]
            self.assertEqual(positions, [997, 1000, 3, 0])
            bridge.close()

    def test_uninitialized_device_requires_explicit_initialization(self):
        with simulated_client() as (client, service, clock):
            _, bridge = self.create_bridge(client, clock)
            with self.assertRaisesRegex(BridgeError, "未初始化"):
                bridge.prepare()
            bridge.close()
            self.assertEqual([name for name, _, _ in service.calls],
                             ["Connect", "GetAllState", "Disconnect"])

    def test_failed_connection_prevents_even_requested_initialization(self):
        with simulated_client() as (client, service, clock):
            service.fail.add("Connect")
            _, bridge = self.create_bridge(client, clock)
            with self.assertRaisesRegex(BridgeError, "连接.*失败"):
                bridge.prepare(initialize=True)
            bridge.close()
            self.assertEqual([name for name, _, _ in service.calls], ["Connect"])

    def test_requested_initialization_runs_only_when_needed(self):
        for initial in (0, 1):
            with self.subTest(initial=initial), simulated_client() as (client, service, clock):
                service.state.init_state = initial
                _, bridge = self.create_bridge(client, clock)
                bridge.prepare(initialize=True)
                names = [name for name, _, _ in service.calls]
                self.assertEqual(names.count("Initialization"), 1 if initial == 0 else 0)
                bridge.close()

    def test_timeout_stops_without_resuming_old_target(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.prepare()
            inputs.update([0, 0.5])
            bridge.step()
            clock.sleep(0.6)
            with self.assertRaisesRegex(BridgeError, "未更新"):
                bridge.step()
            bridge.close()
            names = [name for name, _, _ in service.calls]
            self.assertEqual(names.count("SetTargetPosition"), 1)
            self.assertNotIn("Resume", names)
            self.assertEqual(names[-2:], ["Stop", "Disconnect"])

    def test_slow_feedback_rpc_cannot_send_expired_target(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.prepare()
            inputs.update([0, 0.5])
            service.delays["GetAllState"] = 0.6
            with self.assertRaisesRegex(BridgeError, "未更新"):
                bridge.step()
            self.assertNotIn("SetTargetPosition", [name for name, _, _ in service.calls])
            bridge.close()

    def test_feedback_rpc_uses_newest_sample_after_return(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.prepare()
            inputs.update([0, 0.1])
            original = client.get_state

            def receive_during_rpc():
                state = original()
                inputs.update([0, 0.7])
                return state

            with patch.object(client, "get_state", receive_during_rpc):
                self.assertEqual(bridge.step(), 300)
            self.assertEqual(service.calls[-1][0], "SetTargetPosition")
            self.assertEqual(service.calls[-1][1].value, 300)
            bridge.close()

    def test_invalid_feedback_prevents_position_command(self):
        for changes in ({"valid": False}, {"update_age_ms": 600}, {"error": 8},
                        {"init_state": 0}, {"position": 1001}):
            with self.subTest(changes=changes), simulated_client() as (client, service, clock):
                service.state.init_state = 1
                inputs, bridge = self.create_bridge(client, clock)
                bridge.prepare()
                inputs.update([0, 1])
                for key, value in changes.items():
                    setattr(service.state, key, value)
                with self.assertRaises(BridgeError):
                    bridge.step()
                self.assertNotIn("SetTargetPosition", [name for name, _, _ in service.calls])
                bridge.close()

    def test_command_failure_and_stop_failure_still_disconnect(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.prepare()
            inputs.update([0, 1])
            service.fail.update(["SetTargetPosition", "Stop"])
            with self.assertRaisesRegex(BridgeError, "SetTargetPosition"):
                bridge.step()
            failures = bridge.close()
            self.assertEqual(len(failures), 1)
            self.assertIn("Stop", failures[0])
            self.assertEqual(service.calls[-1][0], "Disconnect")

    def test_cancellation_prevents_startup_motion(self):
        with simulated_client() as (client, service, clock):
            service.state.init_state = 1
            inputs, bridge = self.create_bridge(client, clock)
            bridge.cancelled = lambda: True
            with self.assertRaises(InterruptedError):
                bridge.prepare()
            self.assertEqual(service.calls, [])
            bridge.close()

    def test_cancellation_during_initialization_stops_without_position_command(self):
        with simulated_client() as (client, service, clock):
            service.init_finishes = False
            _, bridge = self.create_bridge(client, clock)
            original = client.get_state

            def cancel_when_initializing():
                state = original()
                if state["init_state"] == 2:
                    bridge.cancelled = lambda: True
                return state

            with patch.object(client, "get_state", cancel_when_initializing):
                with self.assertRaises(InterruptedError):
                    bridge.prepare(initialize=True)
            bridge.close()
            names = [name for name, _, _ in service.calls]
            self.assertNotIn("SetTargetPosition", names)
            self.assertEqual(names[-2:], ["Stop", "Disconnect"])

    def test_initialization_timeout_stops_without_position_command(self):
        with simulated_client() as (client, service, clock):
            service.init_finishes = False
            _, bridge = self.create_bridge(client, clock)
            with patch("teleop_gripper.time.sleep", clock.sleep):
                with self.assertRaisesRegex(BridgeError, "超过 15 秒"):
                    bridge.prepare(initialize=True)
            bridge.close()
            names = [name for name, _, _ in service.calls]
            self.assertNotIn("SetTargetPosition", names)
            self.assertEqual(names[-2:], ["Stop", "Disconnect"])


if __name__ == "__main__":
    unittest.main()
