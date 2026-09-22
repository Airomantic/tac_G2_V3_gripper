"""Configuration, operator prompts, persistent reports, and timing helpers."""

import argparse
import csv
import importlib.metadata
import json
import math
import platform
import select
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


PROMPT_POLL = None
PROMPT_RECORD = None
CHOICE_LABELS = {"run": "开始", "skip": "跳过", "quit": "退出", "ready": "就绪",
                 "pass": "通过", "fail": "不通过", "continue": "继续", "retry": "重试"}
STATUS_LABELS = {"PASS": "通过", "FAIL": "失败", "SKIP": "跳过", "ABORT": "中止"}


class CheckFailed(Exception):
    pass


class CheckSkipped(Exception):
    pass


def describe_error(exc):
    """Keep operator messages Chinese; detailed exceptions remain in diagnostic logs."""
    if isinstance(exc, (CheckFailed, CheckSkipped)) and any("\u4e00" <= c <= "\u9fff" for c in str(exc)):
        return str(exc)
    if isinstance(exc, ModuleNotFoundError):
        return f"缺少 Python 模块 {exc.name}，请激活已完成安装的环境，或执行 ./install.sh。"
    if isinstance(exc, FileNotFoundError):
        return f"文件不存在：{exc.filename}"
    if isinstance(exc, FileExistsError):
        return f"输出目录已存在，请选择新的报告目录：{exc.filename}"
    if isinstance(exc, json.JSONDecodeError):
        return f"JSON 格式错误，请检查第 {exc.lineno} 行、第 {exc.colno} 列。"
    if isinstance(exc, KeyError):
        return f"缺少必需的配置或数据字段：{exc.args[0]}"
    return f"操作发生异常（{type(exc).__name__}），请检查配置并查看事件日志或对应的 SDK 日志。"


class ChineseHelpFormatter(argparse.HelpFormatter):
    def _format_usage(self, usage, actions, groups, prefix):
        return super()._format_usage(usage, actions, groups, prefix or "用法：")


class ChineseArgumentParser(argparse.ArgumentParser):
    def __init__(self, **kwargs):
        super().__init__(add_help=False, formatter_class=ChineseHelpFormatter, **kwargs)
        self._positionals.title = "位置参数"
        self._optionals.title = "选项"
        self.add_argument("-h", "--help", action="help", help="显示帮助信息并退出")

    def error(self, message):
        if not any("\u4e00" <= c <= "\u9fff" for c in message):
            if message.startswith("unrecognized arguments:"):
                message = "无法识别的参数：" + message.split(":", 1)[1].strip()
            elif message.endswith("expected one argument"):
                message = "参数缺少取值：" + message.split(":", 1)[0].removeprefix("argument ")
            else:
                message = "命令行参数无效，请使用 --help 查看用法。"
        self.print_usage(sys.stderr)
        self.exit(2, f"{self.prog}：错误：{message}\n")


def show_config(config):
    g = config["gripper"]
    print(f"夹爪地址：{g['ip']}:{g['port']}；CAN 通道：{g['channel']}；电机 ID：{g['motor_id']}")
    print(f"错误配置测试：IP={g['wrong_ip']}，端口={g['wrong_port']}，电机 ID={g['wrong_motor_id']}")
    print(f"测试速度：{g['test_speed']}；夹持力指令：{g['force']}（设备单位）")
    print("停止及联动测试固定速度：20；定位、点动、跟随和触觉测试各执行一轮。")
    for index, sensor in enumerate(config["sensors"]):
        print(f"传感器 {index}：设备 ID={sensor['dev_id']}，后端={sensor['backend']}，"
              f"服务地址={sensor['remote_addr']}，本机接收地址={sensor['pc_host']}:{sensor['pc_port']}")
    labels = {
        "connect_timeout_s": "连接超时（秒）", "init_timeout_s": "初始化超时（秒）",
        "move_timeout_s": "运动超时（秒）", "position_tolerance": "位置容差（设备单位）",
        "speed_duration_s": "变速测试时长（秒）",
        "maximum_speed_jitter_s": "最大速度调度延迟（秒）", "stop_settle_s": "停止稳定等待时间（秒）",
        "startup_timeout_s": "传感器启动超时（秒）", "shutdown_timeout_s": "传感器退出等待时间（秒）",
        "stream_timeout_s": "无新帧超时（秒）", "fault_window_s": "故障观察时间（秒）",
        "warmup_seconds": "预热时间（秒）", "measurement_seconds": "FPS 统计时间（秒）",
        "minimum_fps": "最低单秒帧率", "maximum_fps_drop": "最大帧率下降比例",
        "maximum_frame_gap_s": "最大帧间隔（秒）", "contact_seconds": "每段接触采集时间（秒）",
    }
    print("验收阈值：")
    for key, value in config["limits"].items():
        print(f"  {labels.get(key, key)}：{value}")


def require(condition, message):
    if not condition:
        raise CheckFailed(message)


def ask(message, choices=("run", "skip", "quit")):
    labels = [CHOICE_LABELS[choice] for choice in choices]
    aliases = {label: choice for label, choice in zip(labels, choices)}
    aliases.update({str(i): choice for i, choice in enumerate(choices, 1)})
    while True:
        try:
            options = " / ".join(f"{i}.{label}" for i, label in enumerate(labels, 1))
            prompt = f"{message} [{options}]（输入序号或中文选项）："
            if PROMPT_POLL is not None and sys.stdin.isatty() and sys.platform != "win32":
                print(prompt, end="", flush=True)
                while not select.select([sys.stdin], [], [], 0.1)[0]:
                    PROMPT_POLL()
                PROMPT_POLL()
                line = sys.stdin.readline()
                if not line:
                    raise EOFError
                answer = line.strip().lower()
            else:
                answer = input(prompt).strip().lower()
        except EOFError:
            raise KeyboardInterrupt from None
        answer = aliases.get(answer, answer)
        if answer in choices:
            if PROMPT_RECORD is not None:
                PROMPT_RECORD(message, answer)
            if answer == "quit":
                raise KeyboardInterrupt
            return answer
        print("输入无效，请输入对应序号或以下选项：" + "、".join(labels))


def ready(message):
    if ask(message, ("ready", "skip", "quit")) == "skip":
        raise CheckSkipped("测试人员跳过了必要的准备步骤。")


def confirm(message):
    answer = ask(message, ("pass", "fail", "skip", "quit"))
    if answer == "skip":
        raise CheckSkipped("未完成人工观察。")
    require(answer == "pass", "人工判定未通过：" + message)


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "mean": statistics.mean(values),
            "min": values[0], "max": values[-1],
            "p95": values[max(0, math.ceil(len(values) * 0.95) - 1)]}


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    require(isinstance(cfg, dict), "配置内容必须是 JSON 对象。")
    for section in ("gripper", "sensors", "limits"):
        require(section in cfg, f"缺少配置项：{section}")
    g, limits = cfg["gripper"], cfg["limits"]
    required_limits = ("connect_timeout_s", "init_timeout_s", "move_timeout_s", "position_tolerance",
                       "speed_duration_s", "maximum_speed_jitter_s", "stop_settle_s",
                       "startup_timeout_s", "shutdown_timeout_s", "stream_timeout_s", "fault_window_s", "warmup_seconds",
                       "measurement_seconds", "minimum_fps", "maximum_fps_drop", "maximum_frame_gap_s",
                       "contact_seconds")
    require(all(key in limits for key in required_limits), "配置文件缺少必需的测试阈值。")
    for key in ("ip", "channel", "wrong_ip"):
        require(isinstance(g.get(key), str) and g[key], f"夹爪配置 gripper.{key} 无效")
    for key in ("port", "wrong_port"):
        require(type(g.get(key)) is int and 1 <= g[key] <= 65535, f"夹爪配置 gripper.{key} 无效")
    for key in ("motor_id", "wrong_motor_id"):
        require(type(g.get(key)) is int, f"夹爪配置 gripper.{key} 无效")
    require(g["wrong_motor_id"] != g["motor_id"], "用于错误配置测试的电机 ID 必须与正确 ID 不同。")
    require(g["wrong_ip"] != g["ip"] and g["wrong_port"] != g["port"],
            "用于错误配置测试的 IP 和端口必须分别与正确配置不同。")
    require(1 <= g["test_speed"] <= 40 and 20 <= g["force"] <= 100,
            "test_speed 必须在 1～40 范围内，force 必须在 20～100 范围内。")
    require(len(cfg["sensors"]) == 2, "必须配置两个传感器。")
    require(len({str(s["dev_id"]) for s in cfg["sensors"]}) == 2, "两个传感器的 ID 必须不同。")
    require(len({s["pc_port"] for s in cfg["sensors"]}) == 2, "两个传感器的接收端口必须不同。")
    for sensor in cfg["sensors"]:
        require(sensor["backend"].lower() in ("cpu", "cuda", "flux"), "传感器后端配置无效。")
        require(1 <= sensor["pc_port"] <= 65535, "传感器接收端口无效。")
        require(bool(sensor["remote_addr"]) and bool(sensor["pc_host"]), "缺少传感器地址。")
    for key, value in limits.items():
        require(type(value) in (int, float) and math.isfinite(value) and value > 0,
                f"limits.{key} 必须是有限的正数。")
    require(type(limits["measurement_seconds"]) is int, "limits.measurement_seconds 必须是整数。")
    require(limits["measurement_seconds"] <= 20, "FPS 最多统计 20 个完整秒。")
    require(limits["warmup_seconds"] == 5, "预热时间必须为 5 秒。")
    require(limits["maximum_fps_drop"] < 1, "maximum_fps_drop 必须是小于 1 的正数比例。")
    require(limits["speed_duration_s"] >= 20, "变速测试至少需要 20 秒，以覆盖一个完整的速度变化周期。")
    return cfg


class Report:
    def __init__(self, directory, config, mode):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        packages = {}
        for name in ("DM_V3Gripper", "dmrobotics", "numpy", "grpcio"):
            try:
                packages[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                packages[name] = "not installed"
        self.data = {"started_utc": datetime.now(timezone.utc).isoformat(),
                     "python": sys.executable, "platform": platform.platform(),
                     "packages": packages, "mode": mode, "config": config, "results": []}
        self.case = "setup"
        self.save()

    def save(self):
        target = self.directory / "report.json"
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.data, indent=2, ensure_ascii=False, allow_nan=False),
                             encoding="utf-8")
        temporary.replace(target)
        with (self.directory / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=("id", "attempt", "status", "duration_s", "detail"))
            writer.writeheader()
            writer.writerows({k: row[k] for k in writer.fieldnames} for row in self.data["results"])

    def add(self, case, status, elapsed, detail, metrics=None):
        attempt = 1 + sum(row["id"] == case for row in self.data["results"])
        self.data["results"].append({"id": case, "attempt": attempt, "status": status,
                                     "duration_s": round(elapsed, 3), "detail": detail,
                                     "metrics": metrics or {}})
        self.save()

    def event(self, kind, **values):
        row = {"time": time.monotonic(), "case": self.case, "kind": kind, **values}
        with (self.directory / "events.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        if kind in ("rpc", "feedback", "arrival", "speed_change", "fault_feedback"):
            path = self.directory / "gripper.csv"
            fields = ("time", "case", "kind", "method", "args", "success", "duration_s", "target",
                      "position", "command_speed", "current", "valid", "stale", "update_age_ms", "error",
                      "init_state", "grip_state", "interval_s", "schedule_lateness_s", "elapsed_s", "message")
            exists = path.exists()
            with path.open("a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
                if not exists:
                    writer.writeheader()
                if "args" in row:
                    row["args"] = json.dumps(row["args"])
                writer.writerow(row)

    def exit_code(self):
        statuses = [row["status"] for row in self.data["results"]]
        if "ABORT" in statuses:
            return 130
        if "FAIL" in statuses:
            return 1
        if "SKIP" in statuses:
            return 2
        return 0


def bucket(start, now, warmup=5, seconds=20):
    elapsed = now - start - warmup
    return int(elapsed) if start > 0 and 0 <= elapsed < seconds else None


def complete_seconds(start, now, warmup=5, seconds=20):
    return min(seconds, max(0, int(now - start - warmup))) if start > 0 else 0


def report_fps(counts, start, now, reported, warmup=5, seconds=20):
    """Print only complete measurement seconds, once per interval."""
    completed = complete_seconds(start, now, warmup, seconds)
    for second in range(reported, completed):
        values = " | ".join(f"传感器 {index}：{values[second]} 帧/秒" for index, values in counts.items())
        print(f"[FPS {second + 1:02d}] {values}", flush=True)
    return completed
