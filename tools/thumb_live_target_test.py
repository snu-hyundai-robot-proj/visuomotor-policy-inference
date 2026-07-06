#!/usr/bin/env python3
"""Interactive RH56 thumb target test through the normal ROS command path only.

Path used:
    /inspire/right/target -> inspire_driver_node -> retarget_fingers() -> move_fingers()

This tool does not touch raw serial registers directly and does not call any
driver calibration/mode/clear functions. It only publishes Float64MultiArray
commands to the normal target topic, while logging target echo and joint
feedback.

README:
    - j1~j4 are held at the target inferred from tj1~tj4 at startup.
    - j5 is held at the target inferred from tj5 at startup until changed.
    - j6 is held at a safe start target inferred from j6 actual, not tj6,
      because target and actual may differ materially at startup.
    - j5 thumb bend and j6 thumb rotation are controlled interactively.
    - j5 runtime target is clamped to 110~135 deg, so data[4] around 0.625
      or higher may saturate to the same 135 deg target.
    - j6 direction is reversed: increasing data[5] decreases target degree.
    - No command is published before live joint feedback is received and the
      user explicitly types "arm".
"""

from __future__ import annotations

import argparse
import csv
import math
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
CONFLICT_HINTS = (
    "inspire_bridge",
    "drive_arm",
    "drive_hand",
    "run_robot_loop",
    "lerobot",
    "home",
    "teleop",
    "model",
    "replay",
    "player",
    "gui",
    "test",
)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


def norm_to_expected_j5_deg(data4: float) -> float:
    return clamp((1100.0 + 400.0 * float(data4)) / 10.0, 110.0, 135.0)


def norm_to_expected_j6_deg(data5: float) -> float:
    return clamp((1900.0 - 950.0 * float(data5)) / 10.0, 60.0, 180.0)


def tj_deg_to_norm(tj_deg: float, idx: int) -> float:
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


class ThumbLiveTargetNode(Node):
    def __init__(self, csv_path: Path):
        super().__init__("rh56_thumb_live_target_test")
        self.csv_path = csv_path
        self.requested = [0.5] * 6
        self.start_target: list[float] | None = None
        self.publish_enabled = False
        self.observed_target: list[float] | None = None
        self.feedback: Feedback | None = None
        self._lock = threading.Lock()

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Float64MultiArray, TARGET_TOPIC, qos)
        self.create_subscription(Float64MultiArray, TARGET_TOPIC, self._on_target, qos)
        self.create_subscription(JointState, STATE_TOPIC, self._on_joint_state, qos)

        self.csv_file = self.csv_path.open("w", newline="")
        fields = (
            ["timestamp_monotonic"]
            + [f"requested_target_{i}" for i in range(6)]
            + [f"observed_target_{i}" for i in range(6)]
            + [f"j{i}_actual_deg" for i in range(1, 7)]
            + [f"tj{i}_deg" for i in range(1, 7)]
            + [
                "expected_j5_deg",
                "expected_j6_deg",
                "j5_error_deg",
                "j6_error_deg",
                "joint_states_age_sec",
            ]
        )
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.writer.writeheader()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_current)
        self.create_timer(1.0 / DISPLAY_HZ, self._log_and_display)

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
                    start = [tj_deg_to_norm(self.feedback.tj_deg[i], i) for i in range(6)]
                    j6_actual = self.feedback.actual_deg[5]
                    if not math.isfinite(j6_actual):
                        return False
                    start[5] = clamp((1900.0 - j6_actual * 10.0) / 950.0, 0.0, 1.0)
                    self.start_target = start.copy()
                    self.requested = start.copy()
                    return True
        return False

    def enable_publish(self) -> None:
        with self._lock:
            if self.start_target is None:
                raise RuntimeError("cannot publish before initial feedback")
            self.requested = self.start_target.copy()
            self.publish_enabled = True

    def disable_publish(self) -> None:
        with self._lock:
            self.publish_enabled = False

    def set_thumb_bend(self, data4: float) -> None:
        data4 = clamp(data4, 0.0, 1.0)
        with self._lock:
            self.requested[4] = data4
        print(f"\nrequested data[4]={data4:.4f} -> expected_j5_deg={norm_to_expected_j5_deg(data4):.2f}")

    def set_thumb_rotation(self, data5: float) -> None:
        data5 = clamp(data5, 0.0, 1.0)
        with self._lock:
            self.requested[5] = data5
        print(f"\nrequested data[5]={data5:.4f} -> expected_j6_deg={norm_to_expected_j6_deg(data5):.2f}")

    def set_thumb_both(self, data4: float, data5: float) -> None:
        data4 = clamp(data4, 0.0, 1.0)
        data5 = clamp(data5, 0.0, 1.0)
        with self._lock:
            self.requested[4] = data4
            self.requested[5] = data5
        print(
            f"\nrequested data[4]={data4:.4f} -> expected_j5_deg={norm_to_expected_j5_deg(data4):.2f}; "
            f"data[5]={data5:.4f} -> expected_j6_deg={norm_to_expected_j6_deg(data5):.2f}"
        )

    def restore_start_target(self, duration_sec: float = 1.0) -> None:
        with self._lock:
            if self.start_target is None:
                return
            self.requested = self.start_target.copy()
            self.publish_enabled = True
            data4 = self.requested[4]
            data5 = self.requested[5]
        print(
            f"\nRestoring start target: data[4]={data4:.4f}, expected_j5_deg={norm_to_expected_j5_deg(data4):.2f}; "
            f"data[5]={data5:.4f}, expected_j6_deg={norm_to_expected_j6_deg(data5):.2f}"
        )
        end = time.monotonic() + duration_sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_current(self) -> None:
        with self._lock:
            if not self.publish_enabled:
                return
            msg = Float64MultiArray(data=[float(x) for x in self.requested])
        self.pub.publish(msg)

    def snapshot(self):
        with self._lock:
            req = self.requested.copy()
            obs = None if self.observed_target is None else self.observed_target.copy()
            fb = self.feedback
        return req, obs, fb

    def print_status(self) -> None:
        req, obs, fb = self.snapshot()
        obs = obs if obs is not None else [math.nan] * 6
        actual = fb.actual_deg if fb is not None else [math.nan] * 6
        tj = fb.tj_deg if fb is not None else [math.nan] * 6
        age = math.nan if fb is None else time.monotonic() - fb.stamp_mono
        j5_error = tj[4] - actual[4] if math.isfinite(tj[4]) and math.isfinite(actual[4]) else math.nan
        j6_error = tj[5] - actual[5] if math.isfinite(tj[5]) and math.isfinite(actual[5]) else math.nan
        print(
            "\n"
            f"requested target: {fmt6(req)}\n"
            f"observed  target: {fmt6(obs)}\n"
            f"j5 requested={req[4]:.4f} observed={obs[4]:.4f} "
            f"tj5={tj[4]:.2f} actual={actual[4]:.2f} error={j5_error:.2f}\n"
            f"j6 requested={req[5]:.4f} observed={obs[5]:.4f} "
            f"tj6={tj[5]:.2f} actual={actual[5]:.2f} error={j6_error:.2f}\n"
            f"joint_states_age_ms={age * 1000.0:.1f}"
        )

    def _log_and_display(self) -> None:
        now = time.monotonic()
        req, obs, fb = self.snapshot()
        obs = obs if obs is not None else [math.nan] * 6
        actual = fb.actual_deg if fb is not None else [math.nan] * 6
        tj = fb.tj_deg if fb is not None else [math.nan] * 6
        age = math.nan if fb is None else now - fb.stamp_mono
        expected_j5 = norm_to_expected_j5_deg(req[4])
        expected_j6 = norm_to_expected_j6_deg(req[5])
        j5_error = tj[4] - actual[4] if math.isfinite(tj[4]) and math.isfinite(actual[4]) else math.nan
        j6_error = tj[5] - actual[5] if math.isfinite(tj[5]) and math.isfinite(actual[5]) else math.nan

        row = {"timestamp_monotonic": now}
        row.update({f"requested_target_{i}": req[i] for i in range(6)})
        row.update({f"observed_target_{i}": obs[i] for i in range(6)})
        row.update({f"j{i+1}_actual_deg": actual[i] for i in range(6)})
        row.update({f"tj{i+1}_deg": tj[i] for i in range(6)})
        row.update(
            {
                "expected_j5_deg": expected_j5,
                "expected_j6_deg": expected_j6,
                "j5_error_deg": j5_error,
                "j6_error_deg": j6_error,
                "joint_states_age_sec": age,
            }
        )
        self.writer.writerow(row)
        self.csv_file.flush()

        sys.stdout.write(
            "\r"
            f"j5 req={req[4]:.4f} obs={obs[4]:.4f} tj={tj[4]:7.2f} act={actual[4]:7.2f} err={j5_error:7.2f} | "
            f"j6 req={req[5]:.4f} obs={obs[5]:.4f} tj={tj[5]:7.2f} act={actual[5]:7.2f} err={j6_error:7.2f} | "
            f"age_ms={age * 1000.0:7.1f}"
        )
        sys.stdout.flush()


def print_readme_banner() -> None:
    print(
        f"""
RH56 thumb live target test - NORMAL ROS TOPIC PATH ONLY

This tool publishes only:
  {TARGET_TOPIC} (std_msgs/msg/Float64MultiArray)

It subscribes to:
  {TARGET_TOPIC}
  {STATE_TOPIC}

README / safety notes:
  - j1~j4는 시작 당시 tj1~tj4 target 기준으로 유지합니다.
  - j5는 시작 당시 tj5 target 기준으로 유지합니다.
  - j6는 시작 당시 tj6 target이 아니라 j6 actual feedback 기준 safe-start로 유지합니다.
  - j5 thumb bend와 j6 thumb rotation만 interactive하게 조작합니다.
  - j5는 runtime target이 110~135°로 clamp되므로,
    data[4] 약 0.625 이상은 같은 135° target으로 포화될 수 있습니다.
  - j6는 data[5]가 증가할수록 target degree가 감소합니다.
  - /joint_states 수신 전에는 publish하지 않습니다.
  - initial feedback 후에도 사용자가 arm을 입력하기 전에는 publish하지 않습니다.
  - raw serial write, mode, forceClb, clearErr, calibration write를 하지 않습니다.
  - replay/model/bridge/home/test/GUI/player publisher가 있으면 실행하지 않습니다.

Read-only checks to run before use:
  ros2 node list
  ros2 topic info {TARGET_TOPIC} -v
  ros2 topic echo {STATE_TOPIC} --once

Arm step:
  arm               # begin publishing the preserved safe start target
  q                 # exit without publishing

Commands after arm:
  b <0~1>          # j5 thumb bend
  r <0~1>          # j6 thumb rotation
  br <0~1> <0~1>  # j5 and j6 together
  status
  q
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
        "Stop external publishers first: inspire_bridge_node, Manus/teleop bridge, replay/model loop, "
        "home/test scripts, GUI, player, or any other /inspire/right/target publisher."
    )
    return False


def input_worker(out: queue.Queue[str], stop: threading.Event) -> None:
    while not stop.is_set():
        sys.stdout.write("\nEnter command [arm/b/r/br/status/q]: ")
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


def parse_value(token: str) -> float:
    value = float(token)
    if not 0.0 <= value <= 1.0:
        raise ValueError("value out of range [0, 1]")
    return value


def handle_command(node: ThumbLiveTargetNode, line: str) -> bool:
    parts = line.split()
    if not parts:
        return True
    cmd = parts[0].lower()
    if cmd in {"q", "quit", "exit"}:
        return False
    if cmd == "status":
        node.print_status()
        return True
    try:
        if cmd == "b" and len(parts) == 2:
            node.set_thumb_bend(parse_value(parts[1]))
            return True
        if cmd == "r" and len(parts) == 2:
            node.set_thumb_rotation(parse_value(parts[1]))
            return True
        if cmd == "br" and len(parts) == 3:
            node.set_thumb_both(parse_value(parts[1]), parse_value(parts[2]))
            return True
    except ValueError as exc:
        print(f"Invalid value: {exc}")
        return True
    print("Invalid command. Use: b <0~1>, r <0~1>, br <0~1> <0~1>, status, q")
    return True


def wait_for_arm_or_quit(input_q: queue.Queue[str], stop_event: threading.Event) -> bool:
    print("\nType `arm` to begin publishing the preserved start target.")
    print("Type `q` to exit without publishing.")
    while not stop_event.is_set():
        try:
            line = input_q.get(timeout=0.1)
        except queue.Empty:
            continue
        cmd = line.strip().lower()
        if cmd in {"q", "quit", "exit"}:
            return False
        if cmd == "arm":
            return True
        if cmd:
            print("Not armed yet. Type `arm` to publish the safe start target, or `q` to exit.")
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive RH56 thumb bend/rotation test through /inspire/right/target only."
    )
    parser.add_argument("--initial-wait", type=float, default=8.0, help="seconds to wait for /inspire/joint_states")
    parser.add_argument("--stale-age", type=float, default=0.5, help="joint_states age threshold in seconds")
    parser.add_argument("--stale-duration", type=float, default=0.5, help="seconds stale must persist before fail-safe exit")
    parser.add_argument("--csv", default="", help="optional CSV output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_readme_banner()

    rclpy.init()
    checker = Node("rh56_thumb_live_target_preflight")
    try:
        discovery_deadline = time.monotonic() + 1.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(checker, timeout_sec=0.05)
        if not check_graph_for_conflicts(checker):
            return 2
    finally:
        checker.destroy_node()

    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.csv or f"/tmp/rh56_thumb_live_target_test_{ts}.csv")
    node = ThumbLiveTargetNode(csv_path)
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
        _, _, fb = node.snapshot()
        assert fb is not None
        initial_tj6_deg = fb.tj_deg[5]
        initial_j6_actual_deg = fb.actual_deg[5]
        safe_j6_norm = node.start_target[5]
        j6_mismatch = initial_tj6_deg - initial_j6_actual_deg
        print(f"Initial safe target: {fmt6(node.start_target)}")
        print(
            f"Initial j5 data[4]={node.start_target[4]:.4f}, "
            f"expected_j5_deg={norm_to_expected_j5_deg(node.start_target[4]):.2f}"
        )
        print(
            f"Initial tj6 target deg={initial_tj6_deg:.2f}\n"
            f"Initial j6 actual deg={initial_j6_actual_deg:.2f}\n"
            f"Safe j6 norm from actual={safe_j6_norm:.4f}\n"
            f"j6 target/actual mismatch deg={j6_mismatch:.2f}"
        )
        if abs(j6_mismatch) >= 5.0:
            print(
                "WARNING: j6 target and actual differ materially.\n"
                "The tool will hold j6 at actual-derived safe start, not tj6-derived target."
            )
        print(f"CSV log: {csv_path}")

        thread = threading.Thread(target=input_worker, args=(input_q, stop_event), daemon=True)
        thread.start()
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
            if wait_for_arm_or_quit(input_q, stop_event):
                node.enable_publish()
                print("Armed: publishing preserved safe start target at 20 Hz.")
                break
            print("Exited before arm. No /inspire/right/target messages were published by this tool.")
            return 0

        stale_since = None

        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
            _, _, fb = node.snapshot()
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
            if not handle_command(node, line):
                break

        node.restore_start_target(duration_sec=1.0)
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
