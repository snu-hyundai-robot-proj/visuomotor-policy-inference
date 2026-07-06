#!/usr/bin/env python3
"""Interactive RH56 j6 target test through the normal ROS command path only.

Path used:
    /inspire/right/target -> inspire_driver_node -> retarget_fingers() -> move_fingers()

This tool does not touch raw serial registers directly and does not call any driver
calibration/mode/clear functions. It only publishes Float64MultiArray commands to the
normal target topic, while logging target echo and joint feedback.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import select
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


TARGET_TOPIC = "/inspire/right/target"
STATE_TOPIC = "/inspire/joint_states"
PUBLISH_HZ = 20.0
DISPLAY_HZ = 10.0
ALLOWED_EXTERNAL_PUBLISHER_NAMES = set()
CONFLICT_HINTS = (
    "inspire_bridge",
    "drive_arm",
    "drive_hand",
    "run_robot_loop",
    "lerobot",
    "home",
    "teleop_gui",
    "system_player",
    "test",
)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


def norm_to_expected_j6_deg(data5: float) -> float:
    return clamp((1900.0 - 950.0 * float(data5)) / 10.0, 60.0, 180.0)


def tj_deg_to_norm(tj_deg: float, idx: int) -> float:
    # tj_deg is the displayed value tjN / 10 from /inspire/joint_states.
    actuator = float(tj_deg) * 10.0
    if idx < 4:
        return clamp((actuator - 750.0) / 1100.0, 0.0, 1.0)
    if idx == 4:
        return clamp((actuator - 1100.0) / 400.0, 0.0, 1.0)
    return clamp((1900.0 - actuator) / 950.0, 0.0, 1.0)


def fmt6(xs: Iterable[float | None]) -> str:
    return "[" + ", ".join("nan" if x is None else f"{float(x):.4f}" for x in xs) + "]"


@dataclass
class Feedback:
    actual_deg: list[float]
    tj_deg: list[float]
    stamp_mono: float


class J6LiveTargetNode(Node):
    def __init__(self, csv_path: Path):
        super().__init__("rh56_j6_live_target_test")
        self.csv_path = csv_path
        self.requested = [0.5] * 6
        self.start_target = None
        self.observed_target = None
        self.feedback: Feedback | None = None
        self._lock = threading.Lock()
        self._stop = False

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Float64MultiArray, TARGET_TOPIC, qos)
        self.create_subscription(Float64MultiArray, TARGET_TOPIC, self._on_target, qos)
        self.create_subscription(JointState, STATE_TOPIC, self._on_joint_state, qos)

        self.csv_file = self.csv_path.open("w", newline="")
        fields = (
            ["timestamp_monotonic"]
            + [f"requested_target_{i}" for i in range(6)]
            + [f"observed_target_{i}" for i in range(6)]
            + [f"j{i}_actual" for i in range(1, 7)]
            + [f"tj{i}_deg" for i in range(1, 7)]
            + ["expected_j6_deg", "j6_error_deg", "joint_states_age_sec"]
        )
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.writer.writeheader()

        self.publish_timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_current)
        self.log_timer = self.create_timer(1.0 / DISPLAY_HZ, self._log_row)

    def close(self) -> None:
        self.csv_file.flush()
        self.csv_file.close()

    def _on_target(self, msg: Float64MultiArray) -> None:
        data = [float(x) for x in msg.data[:6]]
        if len(data) < 6:
            data += [math.nan] * (6 - len(data))
        with self._lock:
            self.observed_target = data

    def _on_joint_state(self, msg: JointState) -> None:
        by_name = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        actual = [by_name.get(f"j{i}", math.nan) for i in range(1, 7)]
        tj_raw = [by_name.get(f"tj{i}", math.nan) for i in range(1, 7)]
        tj_deg = [x / 10.0 if math.isfinite(x) else math.nan for x in tj_raw]
        with self._lock:
            self.feedback = Feedback(actual_deg=actual, tj_deg=tj_deg, stamp_mono=time.monotonic())

    def wait_for_initial_feedback(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self.feedback is not None:
                    fb = self.feedback
                    start = [tj_deg_to_norm(fb.tj_deg[i], i) for i in range(6)]
                    self.start_target = start.copy()
                    self.requested = start.copy()
                    return True
        return False

    def set_j6(self, data5: float) -> None:
        data5 = clamp(data5, 0.0, 1.0)
        with self._lock:
            self.requested[5] = data5
        print(f"\nrequested data[5]={data5:.4f} -> expected_j6_deg={norm_to_expected_j6_deg(data5):.2f}")

    def restore_start_j6(self, duration_sec: float = 1.0) -> None:
        with self._lock:
            if self.start_target is None:
                return
            self.requested = self.start_target.copy()
            data5 = self.requested[5]
        print(f"\nRestoring start target: data[5]={data5:.4f}, expected_j6_deg={norm_to_expected_j6_deg(data5):.2f}")
        end = time.monotonic() + duration_sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_current(self) -> None:
        with self._lock:
            msg = Float64MultiArray(data=[float(x) for x in self.requested])
        self.pub.publish(msg)

    def _snapshot(self):
        with self._lock:
            req = self.requested.copy()
            obs = None if self.observed_target is None else self.observed_target.copy()
            fb = self.feedback
        return req, obs, fb

    def _log_row(self) -> None:
        now = time.monotonic()
        req, obs, fb = self._snapshot()
        obs = obs if obs is not None else [math.nan] * 6
        actual = fb.actual_deg if fb is not None else [math.nan] * 6
        tj = fb.tj_deg if fb is not None else [math.nan] * 6
        age = math.nan if fb is None else now - fb.stamp_mono
        expected_j6 = norm_to_expected_j6_deg(req[5])
        j6_error = tj[5] - actual[5] if math.isfinite(tj[5]) and math.isfinite(actual[5]) else math.nan

        row = {"timestamp_monotonic": now}
        row.update({f"requested_target_{i}": req[i] for i in range(6)})
        row.update({f"observed_target_{i}": obs[i] for i in range(6)})
        row.update({f"j{i+1}_actual": actual[i] for i in range(6)})
        row.update({f"tj{i+1}_deg": tj[i] for i in range(6)})
        row.update(
            {
                "expected_j6_deg": expected_j6,
                "j6_error_deg": j6_error,
                "joint_states_age_sec": age,
            }
        )
        self.writer.writerow(row)
        self.csv_file.flush()

        sys.stdout.write(
            "\r"
            f"requested_data5={req[5]:.4f}  "
            f"observed_target_data5={obs[5]:.4f}  "
            f"expected_tj6_deg={expected_j6:7.2f}  "
            f"observed_tj6_deg={tj[5]:7.2f}  "
            f"observed_j6_actual_deg={actual[5]:7.2f}  "
            f"j6_error_deg={j6_error:7.2f}  "
            f"joint_states_age_ms={age * 1000.0:7.1f}"
        )
        sys.stdout.flush()


def print_safety_banner() -> None:
    print(
        f"""
RH56 j6 live target test - NORMAL ROS TOPIC PATH ONLY

This tool publishes only:
  {TARGET_TOPIC} (std_msgs/msg/Float64MultiArray)

It subscribes to:
  {TARGET_TOPIC}
  {STATE_TOPIC}

Before running, verify manually with these read-only commands:

  ros2 node list
  ros2 topic info {TARGET_TOPIC} -v
  ros2 topic echo {STATE_TOPIC} --once

Safety checklist:
  - 손 주변 물체를 치울 것
  - 엄지 회전축을 육안으로 볼 것
  - emergency stop 또는 손 전원 접근 가능 상태일 것
  - inspire_bridge_node, Manus bridge, replay, model loop, home, test script, GUI, player 등
    다른 {TARGET_TOPIC} publisher를 모두 종료할 것
  - publisher conflict가 있으면 실행하지 말 것
  - 이 도구는 기존 publisher가 하나라도 있으면 종료함

Recommended j6 normalized sweep:
  0.11 -> 0.15 -> 0.20 -> 0.25 -> 0.30 -> 0.35 -> 0.20 -> 0.11
"""
    )


def check_graph_for_conflicts(node: Node) -> bool:
    publishers = node.get_publishers_info_by_topic(TARGET_TOPIC)
    subscribers = node.get_subscriptions_info_by_topic(TARGET_TOPIC)

    print("Read-only graph check before creating command publisher:")
    print(f"  topic: {TARGET_TOPIC}")
    print(f"  publisher count before this tool: {len(publishers)}")
    for info in publishers:
        node_name = getattr(info, "node_name", "") or "_NODE_NAME_UNKNOWN_"
        node_ns = getattr(info, "node_namespace", "") or "/"
        print(f"    publisher: {node_ns.rstrip('/')}/{node_name}")
    print(f"  subscriber count: {len(subscribers)}")
    for info in subscribers:
        node_name = getattr(info, "node_name", "") or "_NODE_NAME_UNKNOWN_"
        node_ns = getattr(info, "node_namespace", "") or "/"
        print(f"    subscriber: {node_ns.rstrip('/')}/{node_name}")

    if len(publishers) == 0:
        print("  OK: no external publisher detected before this tool starts publishing.")
        return True

    suspicious = []
    unknown = []
    for info in publishers:
        name = (getattr(info, "node_name", "") or "").lower()
        if not name or "unknown" in name:
            unknown.append(info)
        if any(hint in name for hint in CONFLICT_HINTS):
            suspicious.append(name)

    print("\nERROR: /inspire/right/target already has publisher(s).")
    if suspicious:
        print(f"Likely conflicting publisher(s): {sorted(set(suspicious))}")
    if unknown:
        print("At least one publisher has an unknown node name.")
    print(
        "Strong warning: stop external publishers first: inspire_bridge_node, replay/model loop, "
        "home/test scripts, teleop GUI, system_player, lerobot_system_right."
    )
    return False


def input_worker(out: queue.Queue[str], stop: threading.Event) -> None:
    while not stop.is_set():
        sys.stdout.write("\nEnter j6 normalized [0.0..1.0], or q: ")
        sys.stdout.flush()
        while not stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.2)
            if ready:
                break
        if stop.is_set():
            return
        line = sys.stdin.readline()
        if line == "":
            out.put("q")
            return
        out.put(line.strip())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive RH56 j6 test through /inspire/right/target only."
    )
    parser.add_argument("--initial-wait", type=float, default=8.0, help="seconds to wait for /inspire/joint_states")
    parser.add_argument("--stale-age", type=float, default=0.5, help="joint_states age threshold in seconds")
    parser.add_argument("--stale-duration", type=float, default=0.5, help="seconds stale must persist before fail-safe exit")
    parser.add_argument("--csv", default="", help="optional CSV output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_safety_banner()

    rclpy.init()
    checker = Node("rh56_j6_live_target_preflight")
    try:
        discovery_deadline = time.monotonic() + 1.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(checker, timeout_sec=0.05)
        if not check_graph_for_conflicts(checker):
            return 2
    finally:
        checker.destroy_node()

    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.csv or f"/tmp/rh56_j6_direct_test_{ts}.csv")
    node = J6LiveTargetNode(csv_path)
    stop_event = threading.Event()
    input_q: queue.Queue[str] = queue.Queue()

    def handle_signal(signum, frame):  # noqa: ARG001
        stop_event.set()
        input_q.put("q")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        print(f"Waiting for {STATE_TOPIC} for up to {args.initial_wait:.1f}s ...")
        if not node.wait_for_initial_feedback(args.initial_wait):
            print(f"ERROR: no live {STATE_TOPIC}; is inspire_driver_node connected?")
            return 3

        assert node.start_target is not None
        print(f"Initial target inferred from tj1..tj6: {fmt6(node.start_target)}")
        print(f"Initial j6 data[5]={node.start_target[5]:.4f}, expected_j6_deg={norm_to_expected_j6_deg(node.start_target[5]):.2f}")
        print(f"CSV log: {csv_path}")

        thread = threading.Thread(target=input_worker, args=(input_q, stop_event), daemon=True)
        thread.start()
        stale_since = None

        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
            _, _, fb = node._snapshot()
            now = time.monotonic()
            age = math.inf if fb is None else now - fb.stamp_mono
            if age > args.stale_age:
                if stale_since is None:
                    stale_since = now
                elif now - stale_since >= args.stale_duration:
                    print(
                        f"\nFAIL-SAFE: {STATE_TOPIC} stale for {now - stale_since:.2f}s "
                        f"(age={age:.2f}s > {args.stale_age:.2f}s). Restoring start target and exiting."
                    )
                    break
            else:
                stale_since = None
            try:
                line = input_q.get_nowait()
            except queue.Empty:
                continue
            if line.lower() in {"q", "quit", "exit"}:
                break
            try:
                value = float(line)
            except ValueError:
                print("Invalid input. Enter a number in [0.0, 1.0], or q.")
                continue
            if not 0.0 <= value <= 1.0:
                print("Out of range. Enter a number in [0.0, 1.0], or q.")
                continue
            node.set_j6(value)

        node.restore_start_j6(duration_sec=1.0)
        print(f"\nDone. CSV log saved to {csv_path}")
        return 0
    finally:
        stop_event.set()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
