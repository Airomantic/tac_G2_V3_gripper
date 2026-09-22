"""No-hardware regression checks for the acceptance runner and acquisition lifecycle."""

import contextlib
import csv
import io
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import acceptance_test as app
from acceptance import common, gripper_checks, simulation
from acceptance.common import CheckFailed, CheckSkipped, Report, load_config
from acceptance.sensor_checks import SensorRun
from acceptance import sensor_checks


ROOT = Path(__file__).resolve().parents[1]


class Base(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="v3-acceptance-regression-")
        self.addCleanup(self.temp.cleanup)
        self.config = load_config(ROOT / "acceptance_config.json")
        self.report = Report(Path(self.temp.name) / "report", self.config, "test")
        self.ctx = app.Context(self.config, self.report, False)
        self.addCleanup(setattr, common, "PROMPT_POLL", None)
        self.addCleanup(setattr, common, "PROMPT_RECORD", None)


class GripperTests(Base):
    def test_speed_is_set_only_even_with_legacy_server_feedback(self):
        import grpc
        with simulation.simulated_client() as (client, service, clock):
            self.assertFalse(hasattr(client, "GetCurrentSpeed"))
            self.assertNotIn("speed", client.get_state())
            service.state.speed = 65381  # Older servers still populate the wire field.
            self.assertTrue(client.connect())
            self.assertTrue(client.SetTargetSpeed(20))
            self.assertEqual(service.calls[-1][0], "SetTargetSpeed")
            self.assertEqual(service.calls[-1][1].value, 20)
            for snapshot in (client.get_state(), client.GetAllState()):
                self.assertNotIn("speed", snapshot)
                self.assertNotIn("command_speed", snapshot)
                self.assertEqual(snapshot["position"], 500)
            service.errors["GetAllState"] = grpc.StatusCode.UNAVAILABLE
            clock.sleep(1)
            snapshot = client.get_state()
            self.assertNotIn("speed", snapshot)
            self.assertFalse(snapshot["valid"])

    def test_connections_accept_recorded_uncalibrated_positions(self):
        for init_state, position in ((0, 1057), (2, 1139)):
            with self.subTest(init_state=init_state), simulation.simulated_client() as (_, service, clock):
                service.state.init_state, service.state.position = init_state, position
                with patch.object(gripper_checks, "time", clock):
                    result = gripper_checks.connection(self.ctx)
                self.assertEqual(result["connection_cycles"], 6)
                names = [name for name, _, _ in service.calls]
                self.assertNotIn("Initialization", names)
                self.assertNotIn("SetTargetPosition", names)

    def test_motion_preparation_reuses_initialization_across_connections(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            with patch.object(gripper_checks, "time", clock):
                for target in (250, 750):
                    with gripper_checks.Session(self.ctx) as session:
                        session.prepare()
                        self.assertEqual(session.move(target)["position"], target)
            names = [name for name, _, _ in service.calls]
            self.assertEqual(names.count("Enable"), 2)
            self.assertEqual(names.count("Disable"), 2)
            self.assertNotIn("Initialization", names)
            self.assertNotIn("Resume", names)

    def test_unfinished_motion_preparation_fails_without_initializing(self):
        for init_state in (0, 2):
            with self.subTest(init_state=init_state), simulation.simulated_client() as (_, service, clock):
                service.state.init_state, service.state.position = init_state, 1139
                with patch.object(gripper_checks, "time", clock), self.assertRaisesRegex(CheckFailed, "请先执行初始化测试"):
                    with gripper_checks.Session(self.ctx) as session:
                        session.prepare()
                        session.move(250)
                self.assertEqual([name for name, _, _ in service.calls], ["Connect", "GetAllState", "Disconnect"])

    def test_initialization_polls_valid_state_without_position_or_timestamp_gate(self):
        with simulation.simulated_client() as (_, service, clock):
            feedback = iter(((2, 1139, True, 0), (2, 1139, False, 0), (1, 500, True, 300)))

            def read(request, timeout):
                service.calls.append(("GetAllState", request, timeout))
                init_state, position, valid, age = next(feedback)
                service.state.init_state, service.state.position = init_state, position
                service.state.valid, service.state.update_age_ms = valid, age
                return service.state

            service.GetAllState = read
            with patch.object(gripper_checks, "time", clock):
                with gripper_checks.Session(self.ctx) as session:
                    session.command("enable")
                    issued = clock.now
                    state = session.initialize()
                    self.assertEqual(state["init_state"], 1)
                    self.assertTrue(state["valid"])
                    self.assertLess(state["timestamp"], issued)
                    self.assertAlmostEqual(clock.now - issued, 0.2)

    def test_unfinished_initialization_case_accepts_states_zero_and_two(self):
        for init_state in (0, 2):
            with self.subTest(init_state=init_state), simulation.simulated_client() as (_, service, clock):
                service.state.init_state, service.state.position = init_state, 1139
                with patch.object(gripper_checks, "time", clock), patch.object(gripper_checks, "ready"), \
                        patch.object(gripper_checks, "confirm"):
                    result = gripper_checks.initialization(self.ctx, False)
                self.assertEqual(result, {"initial_state": init_state, "final_state": 1})
                names = [name for name, _, _ in service.calls]
                self.assertEqual(names.count("Initialization"), 1)
                self.assertNotIn("SetTargetPosition", names)

    def test_already_initialized_case_recalibrates_without_position_gate(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state, service.state.position = 1, 1139
            with patch.object(gripper_checks, "time", clock), patch.object(gripper_checks, "confirm"):
                result = gripper_checks.initialization(self.ctx, True)
            self.assertEqual(result, {"initial_state": 1, "final_state": 1})
            self.assertEqual(sum(name == "Initialization" for name, _, _ in service.calls), 1)

    def test_initialized_device_skips_unfinished_case_without_moving(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            with patch.object(gripper_checks, "time", clock), patch.object(gripper_checks, "ready"), \
                    self.assertRaises(CheckSkipped):
                gripper_checks.initialization(self.ctx, False)
            self.assertEqual([name for name, _, _ in service.calls], ["Connect", "GetAllState", "Disconnect"])

    def test_position_range_is_still_checked_after_motion_command(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            with patch.object(gripper_checks, "time", clock):
                with gripper_checks.Session(self.ctx) as session:
                    session.prepare()

                    def read(request, timeout):
                        service.state.position = 1139
                        return service.state

                    service.GetAllState = read
                    with self.assertRaisesRegex(CheckFailed, "位置反馈无效：1139"):
                        session.move(250)

    def test_initialization_device_error_still_prevents_motion(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.position, service.state.error = 1139, 32
            with patch.object(gripper_checks, "time", clock), self.assertRaisesRegex(CheckFailed, "错误码：32"):
                with gripper_checks.Session(self.ctx) as session:
                    session.command("enable")
                    session.initialize()
                    session.move(250)
            names = [name for name, _, _ in service.calls]
            self.assertIn("Initialization", names)
            self.assertNotIn("SetTargetPosition", names)

    def test_connection_timeout_reports_endpoint_without_initializing(self):
        with simulation.simulated_client() as (_, service, clock):
            service.ready = False
            with patch.object(gripper_checks, "time", clock), self.assertRaises(CheckFailed) as error:
                with gripper_checks.Session(self.ctx):
                    self.fail("Unavailable service became connected")
            message = str(error.exception)
            self.assertIn("192.168.127.10:50008", message)
            self.assertIn("3.00", message)
            self.assertIn("等待 gRPC 服务连接就绪失败", message)
            self.assertIn(".gripper-sdk.log", message)
            self.assertFalse(service.calls)

    def test_speed_commands_follow_actual_half_second_schedule(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            with patch.object(gripper_checks, "time", clock):
                with gripper_checks.Session(self.ctx) as session:
                    session.prepare()
                    session.move(1000)
                    before = len(service.calls)
                    sweep = gripper_checks.Sweep(session)
                    sweep.tick()
                    clock.sleep(0.5)
                    sweep.tick()
                    clock.sleep(0.5)
                    sweep.tick()
                    speed_values = [request.value for name, request, _ in service.calls[before:] if name == "SetTargetSpeed"]
                    self.assertEqual(speed_values, [5, 7, 9])
                    self.assertEqual(sweep.intervals, [0.5, 0.5])
                    self.assertEqual(sweep.missed, 0)
                    clock.sleep(1)
                    sweep.tick()
                    self.assertEqual(sweep.missed, 1)
            events = [json.loads(line) for line in (self.report.directory / "events.jsonl").read_text().splitlines()]
            changes = [row for row in events if row["kind"] == "speed_change"]
            self.assertEqual([row["command_speed"] for row in changes], [5, 7, 9, 13])
            self.assertTrue(all("speed" not in row for row in events))
            with (self.report.directory / "gripper.csv").open(newline="") as f:
                reader = csv.DictReader(f)
                self.assertNotIn("speed", reader.fieldnames)
                self.assertIn("command_speed", reader.fieldnames)
                self.assertEqual([row["command_speed"] for row in reader if row["kind"] == "speed_change"],
                                 ["5", "7", "9", "13"])

    def test_initialization_failure_prevents_real_runner_movement(self):
        for finish in (False, True):
            with self.subTest(finish=finish), simulation.simulated_client() as (_, service, clock):
                if finish:
                    service.fail.add("Initialization")
                else:
                    service.init_finishes = False
                with patch.object(gripper_checks, "time", clock), self.assertRaises(CheckFailed):
                    with gripper_checks.Session(self.ctx) as session:
                        session.command("enable")
                        session.initialize()
                        session.move(1000)
                names = [name for name, _, _ in service.calls]
                self.assertNotIn("SetTargetPosition", names)
                self.assertNotIn("Resume", names)
                self.assertIn("Stop", names)
                self.assertIn("Disable", names)

    def test_positions_performs_one_round_trip_without_initializing(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            with patch.object(gripper_checks, "time", clock):
                result = gripper_checks.positions(self.ctx)
            self.assertEqual([row["target"] for row in result["arrivals"]], [1000, 0, 1000])
            targets = [request.value for name, request, _ in service.calls if name == "SetTargetPosition"]
            self.assertEqual(targets, [1000, 0, 1000])
            self.assertNotIn("Initialization", [name for name, _, _ in service.calls])

    def test_stop_uses_speed_twenty_without_initializing_or_resuming(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            with patch.object(gripper_checks, "time", clock), patch.object(gripper_checks, "confirm"):
                result = gripper_checks.stop_control(self.ctx)
            self.assertGreaterEqual(result["samples"], 2)
            self.assertEqual([req.value for name, req, _ in service.calls if name == "SetTargetSpeed"], [20])
            names = [name for name, _, _ in service.calls]
            self.assertIn("Stop", names)
            self.assertIn("Disable", names)
            self.assertNotIn("Initialization", names)
            self.assertNotIn("Resume", names)

    def test_combined_keeps_speed_twenty_throughout_acquisition(self):
        with simulation.simulated_client() as (_, service, clock):
            service.state.init_state = 1
            run = MagicMock()
            run.__enter__.return_value = run
            run.__exit__.return_value = False
            run.summary.return_value = {str(i): {"complete_seconds": 20, "fps": [120] * 20,
                                               "max_gap_s": 0.01} for i in (0, 1)}

            def observe(seconds, tick, print_fps):
                self.assertEqual(seconds, 25)
                for _ in range(seconds * 2):
                    clock.sleep(0.5)
                    tick()

            run.observe.side_effect = observe
            with patch.object(gripper_checks, "time", clock), patch.object(sensor_checks, "time", clock), \
                    patch.object(sensor_checks, "resources", return_value={"cpu_seconds": 0, "rss_bytes": 0}), \
                    patch.object(sensor_checks, "SensorRun", return_value=run):
                sensor_checks.performance(self.ctx, (0, 1), moving=True)
            speeds = [req.value for name, req, _ in service.calls if name == "SetTargetSpeed"]
            self.assertTrue(speeds)
            self.assertEqual(set(speeds), {20})
            targets = [req.value for name, req, _ in service.calls if name == "SetTargetPosition"]
            self.assertEqual(targets[:5], [1000, 0, 1000, 0, 1000])
            self.assertNotIn("Initialization", [name for name, _, _ in service.calls])

    def test_jog_and_follow_each_run_once_without_initializing(self):
        for control in ("jog", "follow"):
            with self.subTest(control=control), simulation.simulated_client() as (_, service, clock):
                service.state.init_state = 1
                original_jog = service.SetJog

                def jog(request, timeout):
                    response = original_jog(request, timeout)
                    service.state.position += request.value * 10
                    return response

                service.SetJog = jog
                with patch.object(gripper_checks, "time", clock), patch.object(gripper_checks, "confirm"):
                    gripper_checks.other_control(self.ctx, control)
                names = [name for name, _, _ in service.calls]
                self.assertNotIn("Initialization", names)
                if control == "jog":
                    self.assertEqual([req.value for name, req, _ in service.calls
                                      if name == "SetJog" and req.value != 0], [1, -1])
                else:
                    self.assertEqual([req.value for name, req, _ in service.calls if name == "SetFollowMode"], [1, 0])
                    self.assertEqual([req.value for name, req, _ in service.calls if name == "SetTargetPosition"],
                                     [500, 450, 550])

    def test_cleanup_failure_cannot_be_skipped(self):
        with simulation.simulated_client() as (_, service, clock):
            service.fail.add("Stop")
            with patch.object(gripper_checks, "time", clock), self.assertRaisesRegex(CheckFailed, "资源清理失败"):
                with gripper_checks.Session(self.ctx) as session:
                    session.command("enable")
                    raise CheckSkipped("Skipped observation")

    def test_offline_checks(self):
        with contextlib.redirect_stdout(io.StringIO()):
            for function in (simulation.contracts, simulation.faults, simulation.fps_logic):
                self.assertTrue(function(self.ctx))


class RunnerTests(Base):
    def test_plan_ends_at_combined_as_step_twenty_two(self):
        plan = app.steps()
        self.assertEqual(len(plan), 22)
        self.assertEqual(plan[-1].id, "combined")
        self.assertEqual([s.id for s in plan[10:17]],
                         ["positions", "speed", "stop", "home", "clear_error", "jog", "follow"])

    def test_tactile_records_one_press_and_slide_per_sensor(self):
        self.ctx.display = True
        run = MagicMock()
        run.__enter__.return_value = run
        run.__exit__.return_value = False
        run.summary.return_value = {str(i): {"stages": {"baseline": {
            "flow_rms": {"mean": 0.0}, "depth_mean": {"mean": 0.0}}}} for i in (0, 1)}
        with simulation.simulated_client() as (_, service, clock), \
                patch.object(gripper_checks, "time", clock), \
                patch.object(sensor_checks, "SensorRun", return_value=run), \
                patch.object(sensor_checks, "ready"), patch.object(sensor_checks, "confirm") as confirm:
            sensor_checks.tactile(self.ctx)
            self.assertNotIn("Initialization", [name for name, _, _ in service.calls])
        self.assertEqual([call.args[0] for call in run.capture.call_args_list],
                         ["baseline", "sensor_0_press", "sensor_0_slide", "sensor_1_press", "sensor_1_slide"])
        self.assertEqual(confirm.call_count, 4)
        self.assertFalse(any("释放" in call.args[0] for call in confirm.call_args_list))

    def test_retry_retains_failure_and_attempt_history(self):
        outcomes = iter((False, True))

        def execute(ctx):
            if not next(outcomes):
                raise CheckFailed("First attempt failed")
            return {"observed": True}

        plan = [app.Step("sample", "Sample", "Synthetic interaction", execute)]
        output = Path(self.temp.name) / "retry"
        with patch.object(app, "steps", return_value=plan), patch("sys.stdin.isatty", return_value=True), \
                patch("builtins.input", side_effect=["就绪", "开始", "重试", "开始"]), \
                contextlib.redirect_stdout(io.StringIO()):
            code = app.main(["--output", str(output)])
        self.assertEqual(code, 1)
        report = json.loads((output / "report.json").read_text())
        self.assertEqual([r["status"] for r in report["results"]], ["FAIL", "PASS"])
        self.assertEqual([r["attempt"] for r in report["results"]], [1, 2])

    def test_abort_marks_remaining_checks_unrun(self):
        plan = [app.Step("one", "One", "", lambda c: {}), app.Step("two", "Two", "", lambda c: {})]
        output = Path(self.temp.name) / "abort"
        with patch.object(app, "steps", return_value=plan), patch("sys.stdin.isatty", return_value=True), \
                patch("builtins.input", side_effect=["1", "3"]), contextlib.redirect_stdout(io.StringIO()):
            code = app.main(["--output", str(output)])
        self.assertEqual(code, 130)
        results = json.loads((output / "report.json").read_text())["results"]
        self.assertEqual([r["status"] for r in results], ["ABORT", "SKIP"])

    def test_skip_is_not_pass(self):
        self.report.add("sample", "SKIP", 0, "Not observed")
        self.assertEqual(self.report.exit_code(), 2)
        self.assertTrue((self.report.directory / "summary.csv").exists())

    def test_invalid_config_rejected_before_connections(self):
        self.config["limits"]["measurement_seconds"] = 21
        path = Path(self.temp.name) / "bad.json"
        path.write_text(json.dumps(self.config))
        with self.assertRaises(CheckFailed):
            load_config(path)

    def test_sensor_recovery_is_attempted_after_cleanup_failure(self):
        faulty, recovered = MagicMock(), MagicMock()
        faulty.__enter__.return_value = faulty
        faulty.check.side_effect = CheckFailed("传感器 0 未产生新的帧对。")
        faulty.__exit__.side_effect = CheckFailed("传感器 0 的采集进程需要强制终止。")
        recovered.__enter__.return_value = recovered
        recovered.__exit__.return_value = False
        with patch.object(sensor_checks, "SensorRun", side_effect=[faulty, recovered]), \
                patch.object(sensor_checks, "ready") as prompt, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(CheckFailed, "强制终止"):
                sensor_checks.sensor_fault(self.ctx, "stream")
        recovered.observe.assert_called_once_with(3)
        self.assertEqual(prompt.call_count, 2)


class AcquisitionTests(Base):
    def setUp(self):
        super().setUp()
        # Only child processes import the SDK, from this isolated test fixture.
        sys.path.insert(0, str(ROOT / "tests" / "fixtures"))
        self.addCleanup(sys.path.remove, str(ROOT / "tests" / "fixtures"))
        self.config["limits"].update(warmup_seconds=0.05, measurement_seconds=1,
                                      startup_timeout_s=3, stream_timeout_s=0.4, shutdown_timeout_s=0.3)

    def assert_reaped(self, run):
        self.assertTrue(all(not w["process"].is_alive() for w in run.workers.values()))

    def test_dual_acquisition_counts_and_resource_cleanup(self):
        with SensorRun(self.ctx, (0, 1)) as run:
            run.capture("baseline", 1.1)
        self.assert_reaped(run)
        result = run.summary()
        self.assertEqual(set(result), {"0", "1"})
        for row in result.values():
            self.assertEqual(row["complete_seconds"], 1)
            self.assertGreater(row["fps"][0], 0)
            self.assertGreater(row["stages"]["baseline"]["samples"], 0)
            self.assertEqual(row["id_mismatch_pairs"], 0)
        self.assertIsNone(common.PROMPT_POLL)

    def test_early_exit_does_not_count_shutdown_as_measurement(self):
        with SensorRun(self.ctx, (0,)) as run:
            run.observe(0.1)
        self.assertEqual(run.summary()["0"]["complete_seconds"], 0)
        self.assert_reaped(run)

    def test_invalid_frames_fail_and_workers_exit(self):
        self.config["sensors"][0]["dev_id"] = "bad"
        run = SensorRun(self.ctx, (0,))
        with self.assertRaises(CheckFailed):
            with run:
                run.observe(1)
        self.assert_reaped(run)

    def test_single_stream_stall_detected(self):
        self.config["sensors"][0]["dev_id"] = "stall"
        run = SensorRun(self.ctx, (0, 1))
        with self.assertRaisesRegex(CheckFailed, "未产生新的帧对"):
            with run:
                run.observe(1)
        self.assert_reaped(run)

    def test_process_exit_is_detected_and_other_worker_reaped(self):
        with SensorRun(self.ctx, (0, 1)) as run:
            run.expected_terminated.add(0)
            run.workers[0]["process"].terminate()
            run.workers[0]["process"].join(2)
            with self.assertRaisesRegex(CheckFailed, "采集进程已退出"):
                run.check()
            run.expected_fault = True
        self.assert_reaped(run)
        with SensorRun(self.ctx, (0, 1)) as recovered:
            recovered.observe(0.1)
        self.assert_reaped(recovered)

    def test_disconnect_failure_is_not_pass(self):
        self.config["sensors"][0]["dev_id"] = "disconnect_error"
        run = SensorRun(self.ctx, (0,))
        with self.assertRaisesRegex(CheckFailed, "断开连接失败"):
            with run:
                run.observe(0.1)
        self.assert_reaped(run)

    def test_startup_hang_has_bounded_cleanup(self):
        self.config["sensors"][0]["dev_id"] = "hang_start"
        self.config["limits"]["startup_timeout_s"] = 0.6
        run = SensorRun(self.ctx, (0,))
        start = time.monotonic()
        with self.assertRaisesRegex(CheckFailed, "启动超时"):
            with run:
                self.fail("Hanging constructor became ready")
        self.assertLess(time.monotonic() - start, 5)
        self.assert_reaped(run)


if __name__ == "__main__":
    unittest.main()
