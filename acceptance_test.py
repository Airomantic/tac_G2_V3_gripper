#!/usr/bin/env python3
"""逐步引导实机验收测试，或运行独立的离线模拟测试。"""

import contextlib
import sys
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from acceptance import gripper_checks as grip
from acceptance import sensor_checks as sensors
from acceptance import simulation
from acceptance import common
from acceptance.common import (CheckFailed, CheckSkipped, Report, ask, load_config, ready,
                               ChineseArgumentParser, STATUS_LABELS, describe_error, show_config)


@dataclass
class Context:
    config: dict
    report: Report
    display: bool = True
    benchmarks: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Step:
    id: str
    title: str
    instructions: str
    run: object
    offline: bool = False


def steps():
    return [
        Step("contracts", "API 参数与返回值", "使用模拟 RPC 服务，不连接实机。", simulation.contracts, True),
        Step("simulated_faults", "初始化失败、RPC 延迟与缓存恢复", "使用模拟故障和虚拟时间，不连接实机。", simulation.faults, True),
        Step("fps_logic", "FPS 统计与帧数据校验", "使用模拟帧验证测试程序的计时逻辑，不启动采集进程。", simulation.fps_logic, True),
        Step("connection", "正确配置与重复连接", "请启动夹爪服务，并保持夹爪静止。", grip.connection),
        Step("wrong_ip", "错误 IP 地址", "请确认 wrong_ip 指向未使用的测试地址。", lambda c: grip.negative_connection(c, "ip")),
        Step("wrong_port", "错误端口", "请确认夹爪服务器上的 wrong_port 端口未被使用。", lambda c: grip.negative_connection(c, "port")),
        Step("wrong_id", "错误电机 ID", "请确认 wrong_motor_id 不对应其他实际电机。", lambda c: grip.negative_connection(c, "motor_id")),
        Step("service_off", "服务停机与重启", "请按提示在另一个终端中手动停止、重启夹爪服务。", grip.service_off),
        Step("init_uninitialized", "从未完成初始化状态执行初始化", "【运动测试】允许状态 0（未初始化）或 2（初始化中）。请清空夹爪整个行程区域。", lambda c: grip.initialization(c, False)),
        Step("init_initialized", "从已初始化状态再次执行初始化", "【运动测试】将再次执行完整初始化。", lambda c: grip.initialization(c, True)),
        Step("positions", "单次往返定位确认", "【运动测试】低速执行一次 1000 → 0 → 1000 往返；请保持夹爪空载。", grip.positions),
        Step("speed", "端点往返变速运动", "【运动测试】速度按 5 → 40 → 5 变化，每 0.5 秒更新一次，并记录实际指令间隔。", grip.speed),
        Step("stop", "停止、使能与禁用", "【运动测试】速度设为 20，在运动中停止并检查位置是否稳定。停止后不会调用 Resume。", grip.stop_control),
        Step("home", "单方向回零", "【运动测试】请观察夹爪是否正确执行单方向回零。", lambda c: grip.other_control(c, "home")),
        Step("clear_error", "清除设备错误", "清除当前错误；若原本无错误，将单独注明仅验证指令应答。", lambda c: grip.other_control(c, "clear")),
        Step("jog", "双方向点动", "【运动测试】仅做一轮，两个方向各短时点动一次。若不交付此接口，请选择跳过。", lambda c: grip.other_control(c, "jog")),
        Step("follow", "位置跟随模式", "【运动测试】仅做一轮，启用跟随模式后依次到达 450 和 550。若不交付此接口，请选择跳过。", lambda c: grip.other_control(c, "follow")),
        Step("sensor_0", "第一个传感器的数据与基准性能", "请启动传感器服务，检查配置列表中第一个传感器的配置及接收地址。", lambda c: sensors.performance(c, (0,))),
        Step("sensor_1", "第二个传感器的数据与基准性能", "请检查配置列表中第二个传感器的配置及接收地址。", lambda c: sensors.performance(c, (1,))),
        Step("tactile", "触觉按压、滑动与设备对应关系", "程序会先停止并禁用驱动。两个传感器各做一次按压和滑动。请准备软头探针，并使用图形显示窗口。", sensors.tactile),
        Step("dual", "双传感器同时运行", "性能对比期间，请保持两个触觉面均无接触。", lambda c: sensors.performance(c, (0, 1))),
        Step("combined", "双传感器与夹爪运动联动", "【运动测试】夹爪以固定速度 20 往返，双传感器持续采集；完成本项后结束整套测试。", lambda c: sensors.performance(c, (0, 1), True)),
    ]


def main(argv=None):
    root = Path(__file__).resolve().parent
    parser = ChineseArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "acceptance_config.json", help="设备配置及验收阈值的 JSON 文件")
    parser.add_argument("--output", type=Path, help="新建报告目录，不覆盖已有目录")
    parser.add_argument("--offline", action="store_true", help="仅运行自动模拟测试，不连接实机，不需要人工操作")
    parser.add_argument("--only", help="用英文逗号分隔步骤 ID；仍按标准顺序执行")
    parser.add_argument("--list", action="store_true", help="列出步骤 ID，不加载硬件软件包")
    parser.add_argument("--no-display", action="store_true", help="不打开图形窗口；跳过触觉观察测试")
    args = parser.parse_args(argv)
    plan = steps()
    if args.list:
        for step in plan:
            print(f"{step.id:26} {step.title}" + (" [离线模拟]" if step.offline else ""))
        return 0
    if args.only:
        selected = set(args.only.split(","))
        unknown = selected - {step.id for step in plan}
        if unknown:
            parser.error("未知的步骤 ID：" + ", ".join(sorted(unknown)))
        if args.offline and any(step.id in selected and not step.offline for step in plan):
            parser.error("--offline 模式不能选择实机测试步骤")
        plan = [step for step in plan if step.id in selected]
    if args.offline:
        plan = [step for step in plan if step.offline]
    if not args.offline and not sys.stdin.isatty():
        parser.error("实机测试必须在交互式终端运行。无人值守模拟测试请使用 --offline。")
    try:
        config = load_config(args.config)
        output = args.output or root / "test_results" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        report = Report(output, config, "offline" if args.offline else "interactive hardware")
    except (OSError, ValueError, KeyError, TypeError, CheckFailed) as exc:
        parser.error(describe_error(exc))
    ctx = Context(config, report, not args.no_display)
    common.PROMPT_RECORD = lambda prompt, answer: report.event("operator_input", prompt=prompt, answer=answer)
    report.data["planned_steps"] = [step.id for step in plan]
    report.data["display"] = ctx.display
    report.save()
    print("V3 触觉夹爪验收测试")
    print(f"报告目录：{report.directory}")
    print("测试结果分为：通过、失败、跳过、中止。跳过项不视为通过。")
    try:
        if not args.offline:
            show_config(config)
            ready("请核对以上设备地址及测试阈值，并关闭其他客户端。运动测试要求夹爪空载，且行程区域内无障碍物")
        for number, step in enumerate(plan, 1):
            while True:
                report.case = step.id
                print(f"\n[{number}/{len(plan)}] {step.id}: {step.title}\n{step.instructions}")
                started = time.monotonic()
                metrics = {}
                try:
                    if not args.offline and ask("是否开始此项测试？") == "skip":
                        raise CheckSkipped("测试人员跳过了此项测试。")
                    if step.offline:
                        # Expected RPC errors should not obscure the operator's result.
                        with (report.directory / f"{step.id}.log").open("a", encoding="utf-8") as log:
                            with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                                metrics = step.run(ctx)
                    else:
                        metrics = step.run(ctx)
                    status, detail = "PASS", "所有必需的自动检查与人工观察均已通过。"
                except CheckSkipped as exc:
                    status, detail = "SKIP", describe_error(exc)
                except KeyboardInterrupt:
                    report.add(step.id, "ABORT", time.monotonic() - started, "测试人员中止了此项测试。")
                    raise
                except Exception as exc:
                    status, detail = "FAIL", describe_error(exc)
                    report.event("exception", traceback=traceback.format_exc())
                report.add(step.id, status, time.monotonic() - started, detail, metrics)
                print(f"[{STATUS_LABELS[status]}] {detail}")
                if status == "FAIL" and not args.offline:
                    if ask("请选择后续操作", ("continue", "retry", "quit")) == "retry":
                        continue
                break
    except (KeyboardInterrupt, CheckSkipped):
        if not any(r["status"] == "ABORT" for r in report.data["results"]):
            report.add("session", "ABORT", 0, "测试人员结束了本轮测试。")
        print("\n本轮测试已结束，已完成的测量数据已保存。")
    finally:
        recorded = {r["id"] for r in report.data["results"]}
        for step in plan:
            if step.id not in recorded:
                report.add(step.id, "SKIP", 0, "本轮测试结束前未执行到此步骤。")
        report.data["finished_utc"] = datetime.now(timezone.utc).isoformat()
        report.save()
        common.PROMPT_RECORD = None
    counts = {s: sum(r["status"] == s for r in report.data["results"]) for s in ("PASS", "FAIL", "SKIP", "ABORT")}
    print("\n" + " | ".join(f"{STATUS_LABELS[name]}：{count}" for name, count in counts.items()))
    print(f"报告文件：{report.directory / 'report.json'}")
    print("即使重试通过，退出码仍会保留此前失败的记录。")
    return report.exit_code()


if __name__ == "__main__":
    raise SystemExit(main())
