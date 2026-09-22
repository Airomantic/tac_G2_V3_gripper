"""Isolated sensor acquisition, frame validation, and guided contact tests."""

import csv
import multiprocessing as mp
import os
import queue
import statistics
import time
import traceback
from pathlib import Path

from . import common
from .common import CheckFailed, CheckSkipped, bucket, complete_seconds, confirm, describe_error, ready, report_fps, require
from .gripper_checks import Session, Sweep


class FrameTracker:
    def __init__(self):
        self.ids = (-1, -1)
        self.schema = None
        self.last_time = None
        self.max_gap = 0.0

    def observe(self, flow_id, flow, depth_id, depth, now):
        import numpy as np
        if flow is None or depth is None:
            return None
        ids = int(flow_id), int(depth_id)
        require(min(ids) >= 0, "帧 ID 不能为负数。")
        require(all(new >= old for new, old in zip(ids, self.ids)), "帧 ID 出现倒退，请重新连接传感器。")
        if any(new == old for new, old in zip(ids, self.ids)):
            return None
        require(isinstance(flow, np.ndarray) and isinstance(depth, np.ndarray), "帧数据不是 NumPy 数组。")
        require(flow.ndim == 3 and flow.shape[-1] == 2 and flow.size > 0, "形变数据应为非空的 H × W × 2 数组。")
        require(depth.ndim in (2, 3) and depth.size > 0 and
                (depth.ndim == 2 or depth.shape[-1] == 1), "深度数据应为 H × W 或 H × W × 1 数组。")
        require(np.issubdtype(flow.dtype, np.number) and np.issubdtype(depth.dtype, np.number), "帧数据类型不是数值类型。")
        require(np.isrealobj(flow) and np.isrealobj(depth), "不支持复数帧数据。")
        schema = (flow.shape, str(flow.dtype), depth.shape, str(depth.dtype))
        require(self.schema is None or schema == self.schema, "采集过程中帧的尺寸或数据类型发生变化。")
        require(np.isfinite(flow).all() and np.isfinite(depth).all(), "帧数据包含 NaN 或 Inf。")
        if self.last_time is not None:
            self.max_gap = max(self.max_gap, now - self.last_time)
        self.ids, self.schema, self.last_time = ids, schema, now
        return {"flow_id": ids[0], "depth_id": ids[1], "same_id": ids[0] == ids[1],
                "flow_rms": float(np.sqrt(np.mean(np.square(flow, dtype=np.float64)))),
                "depth_mean": float(np.mean(depth)), "depth_std": float(np.std(depth))}


def resources():
    """Sample CPU time and current RSS without an optional monitoring dependency."""
    rss = None
    try:
        rss = int(Path("/proc/self/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE")
    except (OSError, ValueError, IndexError, AttributeError):
        pass
    return {"cpu_seconds": time.process_time(), "rss_bytes": rss}


def sensor_process(config, limits, display, output, stop, ready_event, start, counts,
                   heartbeat, last_frame, stage, messages, index):
    sensor = None
    cv2 = None
    try:
        # Native SDK output goes to a per-device log instead of interrupting prompts.
        with open(output + ".sdk.log", "a", buffering=1) as log:
            os.dup2(log.fileno(), 1)
            os.dup2(log.fileno(), 2)
        import numpy as np
        from dmrobotics import Mode, Sensor, SensorOptions
        sensor = Sensor(SensorOptions(**config, mode=Mode.STANDARD, max_fps=120, show_fps=False,
                                      enable_deformation=True, enable_depth=True))
        if display:
            import cv2
            from dmrobotics.utils import put_arrows_on_image
            windows = (f"传感器 {config['dev_id']} - 形变", f"传感器 {config['dev_id']} - 深度")
            for name in windows:
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                cv2.imshow(name, np.zeros((240, 320, 3), dtype=np.uint8))
        tracker = FrameTracker()
        next_check = next_stats = 0.0
        status = None
        fields = ("receipt_time", "stage", "flow_id", "depth_id", "same_id", "read_gap_ms",
                  "flow_rms", "depth_mean", "depth_std", "fps_bucket")
        with open(output + ".csv", "w", newline="", buffering=1) as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            while not stop.is_set():
                heartbeat.value = time.monotonic()
                if display:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27) or any(cv2.getWindowProperty(w, cv2.WND_PROP_VISIBLE) < 1 for w in windows):
                        messages.put((index, "closed", "测试人员关闭了传感器窗口。"))
                        break
                if time.monotonic() >= next_check:
                    before = time.monotonic()
                    if config["backend"].lower() == "flux":
                        sensor.getEvents()
                    status = int(sensor.getDevStatus())
                    messages.put((index, "status", {"value": status, "duration_s": time.monotonic() - before}))
                    next_check = time.monotonic() + 5.0
                    if ready_event.is_set():
                        require(status == 0, f"传感器状态异常，状态码为 {status}。")
                if status != 0:
                    stop.wait(0.02)
                    continue
                if not sensor.wait_for_new(tracker.ids[1], timeout_ms=20):
                    continue
                flow_id, flow = sensor.getDeformation2D()
                flow_received = time.monotonic()
                depth_id, depth = sensor.getDepth()
                now = time.monotonic()
                row = tracker.observe(flow_id, flow, depth_id, depth, now)
                if row is None:
                    stop.wait(0.001)
                    continue
                if display:
                    canvas = np.zeros(flow.shape[:2] + (3,), dtype=np.uint8)
                    cv2.imshow(windows[0], put_arrows_on_image(canvas, flow, step=16, scale=20.0))
                    depth_image = depth.squeeze(-1) if depth.ndim == 3 else depth
                    cv2.imshow(windows[1], (depth_image * 100).clip(0, 255).astype(np.uint8))
                last_frame.value = now
                second = bucket(start.value, now, limits["warmup_seconds"], limits["measurement_seconds"])
                if second is not None:
                    with counts.get_lock():
                        counts[second] += 1
                writer.writerow({**row, "receipt_time": now, "stage": stage.value,
                                 "read_gap_ms": (now - flow_received) * 1000, "fps_bucket": second})
                if not ready_event.is_set():
                    messages.put((index, "schema", tracker.schema))
                    ready_event.set()
                if now >= next_stats:
                    messages.put((index, "resources", {**resources(), "time": now, "max_gap_s": tracker.max_gap}))
                    next_stats = now + 1
    except BaseException as exc:
        traceback.print_exc()
        messages.put((index, "error", describe_error(exc)))
    finally:
        try:
            if sensor is not None:
                sensor.disconnect()
        except BaseException as exc:
            traceback.print_exc()
            messages.put((index, "cleanup_error", describe_error(exc)))
        finally:
            if cv2 is not None:
                cv2.destroyAllWindows()
            messages.put((index, "finished", None))


class SensorRun:
    def __init__(self, ctx, indices):
        self.ctx, self.indices = ctx, list(indices)
        self.limits = ctx.config["limits"]
        self.mp = mp.get_context("spawn")
        self.stop = self.mp.Event()
        self.start = self.mp.Value("d", 0)
        self.stage = self.mp.Value("i", -1)
        self.messages = self.mp.Queue()
        self.workers = {}
        self.events = []
        self.stages = {}
        self.cutoff = None
        self.expected_terminated = set()
        self.expected_fault = False
        self.previous_prompt_poll = common.PROMPT_POLL

    def __enter__(self):
        try:
            for index in self.indices:
                cfg = self.ctx.config["sensors"][index]
                prefix = self.ctx.report.directory / f"{self.ctx.report.case}-{time.monotonic_ns()}-sensor-{index}"
                item = {"ready": self.mp.Event(), "counts": self.mp.Array("L", self.limits["measurement_seconds"]),
                        "heartbeat": self.mp.Value("d", time.monotonic()), "last": self.mp.Value("d", 0),
                        "prefix": str(prefix)}
                process = self.mp.Process(target=sensor_process, args=(
                    cfg, self.limits, self.ctx.display, str(prefix), self.stop, item["ready"], self.start,
                    item["counts"], item["heartbeat"], item["last"], self.stage, self.messages, index),
                    name=f"acceptance-sensor-{index}")
                item["process"] = process
                self.workers[index] = item
                process.start()
            deadline = time.monotonic() + self.limits["startup_timeout_s"]
            while not all(w["ready"].is_set() for w in self.workers.values()):
                self.check(startup=True)
                require(time.monotonic() < deadline, "传感器启动超时，请查看对应设备的 SDK 日志。")
                time.sleep(0.02)
            self.start.value = time.monotonic()
            self.check()
            common.PROMPT_POLL = self.check
            return self
        except BaseException:
            self.close()
            raise

    def drain(self):
        while True:
            try:
                event = self.messages.get_nowait()
            except queue.Empty:
                break
            self.events.append(event)
            self.ctx.report.event("sensor_event", sensor=event[0], event=event[1], data=event[2])

    def check(self, startup=False):
        self.drain()
        for index, kind, value in self.events:
            if kind == "closed":
                raise KeyboardInterrupt
            if kind in ("error", "cleanup_error"):
                raise CheckFailed(f"传感器 {index}： {value}")
        now = time.monotonic()
        for index, item in self.workers.items():
            require(item["process"].is_alive(), f"传感器 {index} 的采集进程已退出。")
            if not startup:
                require(now - item["last"].value <= self.limits["stream_timeout_s"], f"传感器 {index} 未产生新的帧对。")
                require(now - item["heartbeat"].value <= self.limits["stream_timeout_s"], f"传感器 {index} 的 SDK 调用发生阻塞。")

    def observe(self, seconds, tick=None, print_fps=False):
        until = time.monotonic() + seconds
        reported = 0
        counts = {index: worker["counts"] for index, worker in self.workers.items()}
        while time.monotonic() < until:
            self.check()
            if tick:
                tick()
            if print_fps:
                reported = report_fps(counts, self.start.value, time.monotonic(), reported,
                                      self.limits["warmup_seconds"], self.limits["measurement_seconds"])
            time.sleep(0.01)
        self.check()
        if print_fps:
            report_fps(counts, self.start.value, time.monotonic(), reported,
                       self.limits["warmup_seconds"], self.limits["measurement_seconds"])

    def capture(self, label, seconds, display_label=None):
        display_label = display_label or ("无接触基线" if label == "baseline" else label)
        number = len(self.stages)
        self.stages[number] = label
        self.stage.value = number
        print(f"正在记录{display_label}，持续 {seconds} 秒……")
        try:
            self.observe(seconds)
        finally:
            self.stage.value = -1

    def close(self):
        if self.cutoff is not None:
            return []
        self.cutoff = time.monotonic()
        common.PROMPT_POLL = self.previous_prompt_poll
        self.stop.set()
        failures = []
        deadline = time.monotonic() + self.limits["shutdown_timeout_s"]
        for index, item in self.workers.items():
            process = item["process"]
            if process.pid is None:
                continue
            process.join(max(0, deadline - time.monotonic()))
            if process.is_alive():
                process.terminate()
                process.join(2)
                if process.is_alive():
                    process.kill()
                    process.join(2)
                failures.append(f"传感器 {index} 的采集进程需要强制终止。")
            elif process.exitcode != 0 and index not in self.expected_terminated:
                failures.append(f"传感器 {index} 的采集进程已退出，退出码为 {process.exitcode}。")
        self.drain()
        if not self.expected_fault:
            failures.extend(f"传感器 {i}： {value}" for i, kind, value in self.events if kind == "error")
        failures.extend(f"传感器 {i} 断开连接失败，详细诊断请查看事件日志。" for i, kind, value in self.events if kind == "cleanup_error")
        self.messages.close()
        self.messages.join_thread()
        self.ctx.report.event("sensor_cleanup", failures=failures, stages=self.stages)
        return failures

    def __exit__(self, exc_type, exc, tb):
        failures = self.close()
        if failures and exc_type is not KeyboardInterrupt:
            raise CheckFailed("; ".join(failures))

    def summary(self):
        seconds = complete_seconds(self.start.value, self.cutoff or time.monotonic(),
                                   self.limits["warmup_seconds"], self.limits["measurement_seconds"])
        result = {}
        for index, item in self.workers.items():
            with open(item["prefix"] + ".csv", newline="") as f:
                rows = list(csv.DictReader(f))
            readings = [float(row["receipt_time"]) for row in rows
                        if self.start.value + self.limits["warmup_seconds"] <= float(row["receipt_time"]) < self.start.value + self.limits["warmup_seconds"] + seconds]
            gaps = [b - a for a, b in zip(readings, readings[1:])]
            if seconds:
                begin = self.start.value + self.limits["warmup_seconds"]
                gaps.extend((readings[0] - begin, begin + seconds - readings[-1]) if readings else (seconds,))
            fps = list(item["counts"][:seconds])
            stats = [value for i, kind, value in self.events if i == index and kind == "resources"]
            rss_samples = [sample["rss_bytes"] for sample in stats if sample["rss_bytes"] is not None]
            cpu = None
            if len(stats) > 1:
                cpu = 100 * (stats[-1]["cpu_seconds"] - stats[0]["cpu_seconds"]) / (stats[-1]["time"] - stats[0]["time"])
            stage_values = {}
            for number, label in self.stages.items():
                selected = [row for row in rows if int(row["stage"]) == number]
                require(selected, f"传感器 {index} 在阶段 {label} 没有采集到样本。")
                stage_values[label] = {"samples": len(selected)}
                for field in ("flow_rms", "depth_mean", "depth_std"):
                    values = [float(row[field]) for row in selected]
                    stage_values[label][field] = {"mean": statistics.mean(values), "std": statistics.pstdev(values)}
            result[str(index)] = {"device_id": self.ctx.config["sensors"][index]["dev_id"],
                                  "complete_seconds": seconds, "fps": fps,
                                  "mean_fps": statistics.mean(fps) if fps else None,
                                  "max_gap_s": max(gaps, default=0), "received_pairs": len(rows),
                                  "id_mismatch_pairs": sum(row["same_id"] == "False" for row in rows),
                                  "max_read_gap_ms": max((float(row["read_gap_ms"]) for row in rows), default=0),
                                  "worker_cpu_percent": cpu,
                                  "worker_peak_rss_bytes": max(rss_samples) if rss_samples else None,
                                  "csv": Path(item["prefix"] + ".csv").name, "stages": stage_values}
        return result


def performance(ctx, indices, moving=False):
    seconds = ctx.config["limits"]["warmup_seconds"] + ctx.config["limits"]["measurement_seconds"]
    parent_before = resources()
    started = time.monotonic()
    if moving:
        with Session(ctx) as session:
            session.prepare(speed=20)
            session.move(1000)
            with SensorRun(ctx, indices) as run:
                sweep = Sweep(session, fixed_speed=20)
                run.observe(seconds, sweep.tick, print_fps=True)
            ctx.report.event("combined_sweep", **sweep.metrics())
    else:
        with SensorRun(ctx, indices) as run:
            run.observe(seconds, print_fps=True)
    results = run.summary()
    parent_after = resources()
    ctx.report.event("parent_resources", cpu_percent=100 * (parent_after["cpu_seconds"] - parent_before["cpu_seconds"]) / (time.monotonic() - started),
                     rss_bytes=parent_after["rss_bytes"])
    ctx.report.event("sensor_summary", results=results)
    ctx.benchmarks[ctx.report.case] = results
    for index, row in results.items():
        require(row["complete_seconds"] == ctx.config["limits"]["measurement_seconds"], "FPS 统计未完成。")
        require(min(row["fps"]) >= ctx.config["limits"]["minimum_fps"], f"传感器 {index} 的帧率低于设定下限。")
        require(row["max_gap_s"] <= ctx.config["limits"]["maximum_frame_gap_s"], f"传感器 {index} 的帧间隔超过设定上限。")
    return results


def performance_comparison(ctx):
    required = ("sensor_0", "sensor_1", "dual", "combined")
    if any(key not in ctx.benchmarks for key in required):
        raise CheckSkipped("请先在本轮中完成两个单传感器、双传感器及夹爪联动测试。")
    rows = []
    for index in ("0", "1"):
        baseline = ctx.benchmarks["sensor_" + index][index]["mean_fps"]
        require(baseline is not None and baseline > 0, "单传感器基准测试没有完整的 FPS 样本。")
        for mode in ("dual", "combined"):
            fps = ctx.benchmarks[mode][index]["mean_fps"]
            drop = max(0, 1 - fps / baseline)
            rows.append({"sensor": index, "mode": mode, "baseline_fps": baseline, "fps": fps, "drop": drop})
    ctx.report.event("performance_comparison", rows=rows)
    require(all(row["drop"] <= ctx.config["limits"]["maximum_fps_drop"] for row in rows), "同时运行时的帧率下降比例超过设定上限。")
    return {"comparison": rows}


def tactile(ctx):
    if not ctx.display:
        raise CheckSkipped("触觉接触与方向观察需要图形显示窗口。")
    with Session(ctx) as session:
        session.command("Stop")
        session.command("disable")
        session.state()
        ready("请确认驱动已禁用，准备软头探针，并保持两个触觉面均无接触")
        with SensorRun(ctx, (0, 1)) as run:
            duration = ctx.config["limits"]["contact_seconds"]
            run.capture("baseline", duration)
            for index in (0, 1):
                device = ctx.config["sensors"][index]["dev_id"]
                for action, action_label in (("press", "按压"), ("slide", "横向滑动")):
                    ready(f"请对传感器 {device} 执行{action_label}，不要接触另一个传感器。滑动测试中请在采集期间持续移动探针")
                    run.capture(f"sensor_{index}_{action}", duration,
                                f"传感器 {device} {action_label}")
                    confirm(f"传感器 {device} 的{action_label}响应位置和方向是否正确，且另一个窗口没有变化？")
    result = run.summary()
    for index, row in result.items():
        baseline = row["stages"]["baseline"]
        for label, stage in row["stages"].items():
            stage["depth_delta_from_baseline"] = stage["depth_mean"]["mean"] - baseline["depth_mean"]["mean"]
            stage["flow_delta_from_baseline"] = stage["flow_rms"]["mean"] - baseline["flow_rms"]["mean"]
        print(f"传感器 {index}：基线形变均方根={baseline['flow_rms']['mean']:.6g}，深度均值={baseline['depth_mean']['mean']:.6g}")
    ctx.report.event("contact_summary", results=result)
    return result


def sensor_fault(ctx, fault):
    injected = False
    detected = None
    failures = []
    try:
        with SensorRun(ctx, (0, 1)) as run:
            run.observe(1)
            if fault == "process":
                injected = True
                run.expected_terminated.add(0)
                run.workers[0]["process"].terminate()
                run.workers[0]["process"].join(2)
            else:
                instruction = ("仅停止第一个传感器的远程服务" if fault == "service" else
                               "仅中断第一个传感器的帧数据传输，并保持状态服务可用")
                ready(f"选择“就绪”后，请在 {ctx.config['limits']['fault_window_s']} 秒内{instruction}")
                injected = True
            start = time.monotonic()
            while time.monotonic() - start < ctx.config["limits"]["fault_window_s"]:
                try:
                    run.check()
                except CheckFailed as exc:
                    detected = str(exc)
                    break
                time.sleep(0.02)
            require(detected is not None, "未检测到本项要求的传感器故障。")
            require("传感器 0" in detected, f"故障来源与预期不符：{detected}")
            run.expected_fault = True
            print("已检测到故障：" + detected)
            ctx.report.event("expected_sensor_fault", fault=fault, detail=detected,
                             observation_elapsed_s=time.monotonic() - start)
    except CheckFailed as exc:
        if not injected:
            raise
        failures.append(str(exc))
        print(f"故障检查失败：{exc}。仍将继续尝试恢复设备。")
    if fault != "process":
        ready("请恢复第一个传感器的服务及数据传输，程序将重新创建两个客户端")
    with SensorRun(ctx, (0, 1)) as recovered:
        recovered.observe(3)
    result = {"detected": detected, "reconnected": True, "automatic_gripper_motion": False,
              "failures": failures,
              "fault_timing": "observation starts before operator injection; not exact injection latency"}
    ctx.report.event("sensor_recovery", **result)
    require(not failures, "; ".join(failures))
    return result
