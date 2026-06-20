#!/usr/bin/env python3
"""Replay a recorded episode through the policy and drive the REAL HDR35 arm (left or right).

Pick the side with --side (default left). It resolves the arm IP (left=192.168.4.152,
right=192.168.4.151), the episode (sample_episodes/<side>), and joint targets. Make sure
the inference server is serving the MATCHING side's model.

Pipeline (ARM ONLY — the gripper is NOT touched here, for safety):
  recorded episode (state + front/wrist images)  -> /predict (left model)  -> action[26]
  arm = action[0:6] (rad) -> deg -> HDR35 OpenStream TCP (192.168.4.152:49000)

Reuses the PROVEN hdr_stream OpenStream client (handshake / monitor / joint_traject_*),
so it talks to the robot exactly like the production node does. Safety:
  * DRY-RUN by default  (computes everything, reads the live pose, prints the plan,
    sends NO motion). Pass --engage to actually move the arm.
  * SOFT-START: ramps from the live current pose to the episode's first pose at a slow,
    speed-limited rate before replaying (kills the first-tick jump of absolute targets).
  * SPEED CLAMP every 200 Hz tick during replay.
  * api.stop('control') on exit / Ctrl-C / any error.

Feeds the RECORDED state (not live) so the policy reproduces the recorded action
(validated open-loop L1 ~0.007 rad) -> the arm follows the recorded trajectory.

    # serve the matching side's model first (VPI_MODEL_ID=...<side>-flowmatch, :8000)
    python examples/drive_arm_replay.py --side left  --steps 120           # DRY-RUN (no motion)
    python examples/drive_arm_replay.py --side left  --steps 120 --engage  # actually move the arm
    python examples/drive_arm_replay.py --side right --steps 0  --engage   # right arm, full episode
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import requests

HDR_UTILS = "/home/bi/Tesollo/1.Project/1.hyundae/seoul_uiwang/system_Teleop/src/Robot_/src/hdr_stream"
sys.path.insert(0, HDR_UTILS)
from hdr_stream.utils.net import NetClient          # noqa: E402
from hdr_stream.utils.api import OpenStreamAPI       # noqa: E402
from hdr_stream.utils.parser import NDJSONParser     # noqa: E402
from hdr_stream.utils.dispatcher import Dispatcher   # noqa: E402

SAMPLES = Path(__file__).resolve().parent / "sample_episodes"
ARM_IP = {"left": "192.168.4.152", "right": "192.168.4.151"}   # HDR35 OpenStream, per side
SEND_DT = 0.005          # 200 Hz stream (matches hdr_stream)
LOOK_AHEAD = 0.2
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
GET_JOINTS = "/project/robot/joints/joint_states"


def b64(p: Path) -> str:
    return base64.b64encode(p.read_bytes()).decode("ascii")


def compute_arm_targets_deg(side, url, steps):
    """Run the episode through /predict; return arm targets in DEGREES (N, 6)."""
    root = SAMPLES / side
    meta = json.loads((root / "episode.json").read_text())
    frames = meta["frames"][:steps] if steps else meta["frames"]
    sess = requests.Session()
    sess.post(f"{url}/reset", timeout=10).raise_for_status()
    out = []
    for fr in frames:
        payload = {
            "front_rgb": b64(root / "front_rgb" / f"{fr['frame']}.jpg"),
            "wrist_rgb": b64(root / "wrist_rgb" / f"{fr['frame']}.jpg"),
            "state": fr["state"],
        }
        a = np.asarray(sess.post(f"{url}/predict", json=payload, timeout=30).json()["action"], dtype=np.float64)
        out.append(a[:6] * 180.0 / math.pi)
    return np.asarray(out), meta.get("fps", 30)


class ArmClient:
    """Minimal OpenStream client: connect, handshake, monitor joints, stream targets."""

    def __init__(self, ip, port):
        self.net = NetClient(ip, int(port))
        self.api = OpenStreamAPI(self.net)
        self.parser = NDJSONParser()
        self.disp = Dispatcher()
        self.handshake_ok = threading.Event()
        self.latest_q = None
        self.disp.on_type["handshake_ack"] = lambda m: self.handshake_ok.set() if m.get("ok") else None
        self.disp.on_type["data"] = self._on_data
        self.disp.on_error = lambda e: print(f"[robot error] {e}")
        self._tfs = 0.0

    def _on_data(self, m):
        res = m.get("result")
        if isinstance(res, dict) and res.get("_type") == "JObject":
            pos = res.get("position")
            if pos:
                self.latest_q = [float(x) for x in pos]

    def connect_and_init(self, timeout=5.0):
        self.net.connect()
        self.net.start_recv_loop(lambda b: self.parser.feed(b, self.disp.dispatch))
        self.api.handshake(major=1)
        if not self.handshake_ok.wait(timeout):
            raise RuntimeError("handshake timeout")
        self.api.monitor(url=GET_JOINTS, period_ms=4, args={})
        self.api.joint_traject_init()

    def wait_for_pose(self, timeout=5.0):
        t0 = time.time()
        while self.latest_q is None:
            if time.time() - t0 > timeout:
                raise RuntimeError("no joint state received from robot")
            time.sleep(0.02)
        return list(self.latest_q)

    def insert(self, point_deg):
        self.api.joint_traject_insert_point({
            "interval": SEND_DT, "time_from_start": self._tfs,
            "look_ahead_time": LOOK_AHEAD, "point": [float(x) for x in point_deg],
        })
        self._tfs += SEND_DT

    def stop(self):
        try:
            self.api.stop(target="control")
        except Exception:
            pass

    def close(self):
        try:
            self.net.close()
        except Exception:
            pass


def step_toward(cmd, desired, max_delta):
    cmd = np.asarray(cmd); desired = np.asarray(desired)
    return cmd + np.clip(desired - cmd, -max_delta, max_delta)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--robot-ip", default="", help="override; else resolved from --side")
    ap.add_argument("--port", type=int, default=49000)
    ap.add_argument("--steps", type=int, default=120, help="episode frames to replay (safety: short first)")
    ap.add_argument("--soft-speed", type=float, default=8.0, help="deg/s during soft-start ramp")
    ap.add_argument("--replay-speed", type=float, default=60.0, help="deg/s max joint speed during replay")
    ap.add_argument("--max-start-gap", type=float, default=90.0, help="abort if start gap exceeds this (deg)")
    ap.add_argument("--engage", action="store_true", help="ACTUALLY move the arm (default: dry-run, no motion)")
    args = ap.parse_args()

    print(f"[1/3] computing arm targets via {args.url} (left model), {args.steps} frames ...")
    targets, fps = compute_arm_targets_deg(args.side, args.url, args.steps)
    N = len(targets)
    print(f"      got {N} targets @ {fps}Hz")
    per_frame_delta = np.abs(np.diff(targets, axis=0)).max() if N > 1 else 0.0
    print(f"      target range/joint (deg): "
          + ", ".join(f"[{targets[:,j].min():.1f},{targets[:,j].max():.1f}]" for j in range(6)))
    print(f"      max frame-to-frame step: {per_frame_delta:.2f} deg")

    robot_ip = args.robot_ip or ARM_IP[args.side]
    print(f"[2/3] connecting to {args.side} arm {robot_ip}:{args.port} ...")
    arm = ArmClient(robot_ip, args.port)
    try:
        arm.connect_and_init()
        q0 = arm.wait_for_pose()
        q0 = np.asarray(q0[:6])
        gap = targets[0] - q0
        max_gap = float(np.abs(gap).max())
        print(f"      current pose q0 (deg): {np.round(q0,2).tolist()}")
        print(f"      episode start    (deg): {np.round(targets[0],2).tolist()}")
        print(f"      start gap/joint  (deg): {np.round(gap,2).tolist()}  (max {max_gap:.2f})")
        soft_secs = max_gap / max(args.soft_speed, 1e-6)
        print(f"      soft-start est: ~{soft_secs:.1f}s @ {args.soft_speed} deg/s")

        if max_gap > args.max_start_gap:
            print(f"[ABORT] start gap {max_gap:.1f} > {args.max_start_gap} deg — too far; move robot closer or raise --max-start-gap")
            return

        if not args.engage:
            print("[3/3] DRY-RUN — no motion sent. Re-run with --engage to move the arm.")
            return

        print("[3/3] ENGAGE — soft-start ramp ...")
        cmd = q0.copy()
        soft_step = args.soft_speed * SEND_DT
        while np.abs(targets[0] - cmd).max() > 0.5:
            cmd = step_toward(cmd, targets[0], soft_step)
            arm.insert(cmd)
            time.sleep(SEND_DT)
        print("      at episode start. replaying ...")

        replay_step = args.replay_speed * SEND_DT
        frame_period = 1.0 / fps
        t_next_frame = time.monotonic()
        frame = 0
        while frame < N:
            desired = targets[frame]
            cmd = step_toward(cmd, desired, replay_step)
            arm.insert(cmd)
            time.sleep(SEND_DT)
            if time.monotonic() >= t_next_frame:
                t_next_frame += frame_period
                frame += 1
        # settle on the last target
        for _ in range(40):
            cmd = step_toward(cmd, targets[-1], replay_step)
            arm.insert(cmd)
            time.sleep(SEND_DT)
        print("      replay complete.")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] stopping arm.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        arm.stop()
        arm.close()
        print("stopped + disconnected.")


if __name__ == "__main__":
    main()
