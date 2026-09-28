#!/usr/bin/env python3
"""Explicit, bounded real-hardware check; finishes open and attempts Stop."""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from DM_V3Gripper import Gripper
from teleop_gripper import GripperBridge, check_state


def move_and_wait(bridge, target):
    state = bridge.client.get_state()
    check_state(state)
    positions = [state["position"]]
    closing = target < state["position"]
    requested_at = time.monotonic()
    bridge.command("SetTargetPosition", target)
    started = time.monotonic()
    next_log = 0.0
    reached_samples = 0
    last_sample_time = -1.0
    while True:
        state = bridge.client.get_state()
        check_state(state)
        now = time.monotonic()
        positions.append(state["position"])
        if now >= next_log:
            print(json.dumps({"target": target, "position": state["position"],
                              "grip_state": state["grip_state"],
                              "error": state["error"],
                              "elapsed_s": round(now - started, 3)}), flush=True)
            next_log = now + 0.5
        near_target = abs(state["position"] - target) <= 10
        # Status is cached. A previous close may still report "object gripped"
        # just after requesting open. Only premature contact during a new close
        # is a reason to abort; opening must be able to release the gripper.
        fresh_for_request = state["timestamp"] >= requested_at
        if (closing and fresh_for_request and state["grip_state"] == 2
                and not near_target):
            raise RuntimeError("Object contact detected; stopping hardware test")
        settled = state["grip_state"] == 1 or (closing and state["grip_state"] == 2)
        reached = (near_target and settled and fresh_for_request
                   and now - started >= 0.15)
        if reached and state["timestamp"] > last_sample_time + 0.01:
            reached_samples += 1
            last_sample_time = state["timestamp"]
        elif not reached:
            reached_samples = 0
        if reached_samples >= 2:
            result = {"target": target, "actual": state["position"],
                      "elapsed_s": round(now - started, 3),
                      "min_position": min(positions), "max_position": max(positions),
                      "error": state["error"]}
            print("TARGET_REACHED", json.dumps(result), flush=True)
            return result
        if now - started > 20:
            raise TimeoutError(f"Target {target} not reached within 20 s")
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Allow real gripper motion")
    parser.add_argument("--initialize", action="store_true", help="Calibrate if needed (moves gripper)")
    parser.add_argument("--open-only", action="store_true", help="Only verify opening to position 1000")
    parser.add_argument("--ip", default="192.168.127.10")
    args = parser.parse_args()
    if not args.execute:
        parser.error("Real hardware motion requires --execute")

    client = Gripper(remote_ip=args.ip)
    # prepare/command/close do not use the ROS input buffer.
    bridge = GripperBridge(inputs=None, client=client)
    exit_code = 0
    try:
        print(f"Real hardware test: {args.ip}:50008, speed=10, force=20", flush=True)
        bridge.prepare(speed=10, force=20, initialize=args.initialize)
        print("INITIALIZED_STATE", json.dumps(client.get_state()), flush=True)
        results = []
        for target in ((1000,) if args.open_only else (1000, 0, 1000)):
            results.append(move_and_wait(bridge, target))
            time.sleep(0.5)
        label = "OPEN_TEST_PASS" if args.open_only else "OPEN_CLOSE_TEST_PASS"
        print(label, json.dumps(results), flush=True)
    except KeyboardInterrupt:
        print("Hardware test interrupted", file=sys.stderr, flush=True)
        exit_code = 130
    except Exception as exc:
        print(f"Hardware test failed: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        failures = bridge.close()
        if failures:
            print("CLEANUP_FAILED", json.dumps(failures, ensure_ascii=False),
                  file=sys.stderr, flush=True)
            exit_code = 1
        else:
            print("STOP_AND_DISCONNECT_OK", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
