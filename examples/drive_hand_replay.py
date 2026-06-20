#!/usr/bin/env python3
"""Drive ONLY the hand from the policy output — arm stays still. Side-aware.

  recorded episode (state + imgs) -> /predict -> action[26]
  hand = action[hand_slice]  (per action_indices.json)
    LEFT  (DG-5F-M, 20 DOF): action[6:26] -> MultiDOFCommand -> /dg5f_left/lj_dg_pospid/reference
                             (slots 10,14,18 are constant spread joints — sent as-is, fine)
    RIGHT (Inspire RH56, 6): action[6:12] -> Float64MultiArray -> /inspire/right/target

Action layout (from the HF model repos' action_indices.json):
  left  : 23 active = arm6 + DG-5F-M active hand (26 output, drop constant [10,14,18])
  right : 12 active = arm6 + INSPIRE 6 (26 output, drop [12..25])

Run inside the teleop container (rclpy + control_msgs + the hand driver running):
  python3 drive_hand_replay.py --side left  --steps 120             # DRY-RUN
  python3 drive_hand_replay.py --side left  --steps 120 --engage    # move LEFT hand
  python3 drive_hand_replay.py --side right --steps 120 --engage    # move RIGHT (Inspire)

SAFETY: DRY-RUN default; soft-start ramps from the live current hand pose; per-tick clamp.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import time
from pathlib import Path

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# /dg5f_*/joint_states is RELIABLE + TRANSIENT_LOCAL (latched); inspire is default/streaming.
JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Inspire (RIGHT) units: the model state/action are joint angles in RADIANS (~1.4-3.3),
# but /inspire/right/target expects a normalized ~[0,1] vector (retarget_fingers maps it
# to actuator units 880-1740). Verified mapping: actuator = rad * 1800/pi, and
# received_angle (joint_states) ≈ actuator/10 ≈ degrees.  So rad<->[0,1]:
INSPIRE_K = 1800.0 / math.pi   # radian -> Inspire actuator units


def inspire_rad_to_norm(rad6):
    """model action (rad) -> /inspire/right/target normalized command (retarget re-clamps)."""
    out = []
    for i, r in enumerate(rad6):
        a = float(r) * INSPIRE_K
        if i < 4:    x = (a - 750.0) / 1100.0   # fingers
        elif i == 4: x = (a - 1100.0) / 400.0   # thumb
        else:        x = (1900.0 - a) / 950.0    # thumb rotation
        out.append(x)
    return out


def inspire_angle_to_rad(ang6):
    """/inspire/joint_states received_angle (deg-ish) -> radians (model state units)."""
    return [float(a) * math.pi / 180.0 for a in ang6]

LEFT_DELTO_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]

# hand slice of action[26] + how to command it, per side
HAND = {
    "left":  {"slice": (6, 26), "type": "delto",   "ndof": 20,
              "ref": "/dg5f_left/lj_dg_pospid/reference", "state": "/dg5f_left/joint_states"},
    "right": {"slice": (6, 12), "type": "inspire",  "ndof": 6,
              "ref": "/inspire/right/target",            "state": "/inspire/joint_states"},
}
# canonical hand init pose (hand_init_pose.json — mean of episode-start state)
HAND_INIT = {
    "left": [-0.174693, 0.088572, -0.171882, -0.064431, -0.344696, 0.039543,
             0.383013, 0.300303, -0.401026, 0.188575, 0.13543, 0.108224,
             -0.372288, 0.453359, 0.029937, 0.037518, 0.000573, -0.427912,
             0.54842, 0.033721],
    "right": [2.89855, 2.878596, 2.811947, 2.807127, 2.324232, 2.490377],
}
SEND_DT = 0.02   # 50 Hz publish


def b64(p): return base64.b64encode(p.read_bytes()).decode("ascii")
def step_toward(cmd, des, mx): cmd = np.asarray(cmd); return cmd + np.clip(np.asarray(des) - cmd, -mx, mx)


def compute_hand(root: Path, url: str, steps, sl, source="model"):
    meta = json.loads((root / "episode.json").read_text())
    frames = meta["frames"][:steps] if steps else meta["frames"]
    out = []
    if source == "recorded":
        # ground-truth recorded action straight from the dataset (no model)
        for fr in frames:
            out.append(np.asarray(fr["action"], dtype=np.float64)[sl[0]:sl[1]])
    else:
        s = requests.Session(); s.post(f"{url}/reset", timeout=10).raise_for_status()
        for fr in frames:
            a = np.asarray(s.post(f"{url}/predict", json={
                "front_rgb": b64(root / "front_rgb" / f"{fr['frame']}.jpg"),
                "wrist_rgb": b64(root / "wrist_rgb" / f"{fr['frame']}.jpg"),
                "state": fr["state"]}, timeout=30).json()["action"], dtype=np.float64)
            out.append(a[sl[0]:sl[1]])
    return np.asarray(out), meta.get("fps", 30)


class HandIO(Node):
    def __init__(self, spec):
        super().__init__("drive_hand_replay")
        self.spec = spec; self.ndof = spec["ndof"]; self.cur = None
        if spec["type"] == "delto":
            self.pub = self.create_publisher(MultiDOFCommand, spec["ref"], 1)
            js_qos = JS_QOS                     # dg5f joint_states is latched
        else:
            self.pub = self.create_publisher(Float64MultiArray, spec["ref"], 1)
            js_qos = 1                          # inspire streams with default QoS
        self.create_subscription(JointState, spec["state"], self._on_js, js_qos)

    def _on_js(self, m):
        raw = np.asarray(m.position, dtype=np.float64)[: self.ndof]
        # inspire joint_states are in received_angle (deg-ish); convert to rad so the
        # ramp/clamp works in the same units as the model action.
        self.cur = np.asarray(inspire_angle_to_rad(raw)) if self.spec["type"] == "inspire" else raw

    def wait_state(self, timeout=6.0):
        t0 = time.time()
        while self.cur is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return None if self.cur is None else self.cur.copy()

    def publish(self, vals):
        if self.spec["type"] == "delto":
            m = MultiDOFCommand()
            m.dof_names = LEFT_DELTO_JOINT_NAMES[: len(vals)]
            m.values = [float(x) for x in vals]
            m.values_dot = [0.0] * len(vals)
        else:
            # inspire: model cmd is radians -> convert to normalized /inspire/right/target
            m = Float64MultiArray(data=[float(x) for x in inspire_rad_to_norm(vals)])
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--episode-dir", default="")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--source", choices=["model", "recorded"], default="model")
    ap.add_argument("--steps", type=int, default=120)
    ap.add_argument("--soft", type=float, default=0.2, help="rad/s ramp to start pose")
    ap.add_argument("--speed", type=float, default=2.0, help="rad/s max joint speed during replay")
    ap.add_argument("--engage", action="store_true")
    args = ap.parse_args()

    spec = HAND[args.side]
    root = Path(args.episode_dir or f"/tmp/sample_episodes/{args.side}")
    print(f"[1/3] side={args.side} hand={spec['type']} ndof={spec['ndof']} "
          f"slice=action[{spec['slice'][0]}:{spec['slice'][1]}]")
    hand_t, fps = compute_hand(root, args.url, args.steps, spec["slice"], args.source)
    N = len(hand_t)
    print(f"      {N} frames @ {fps}Hz | per-joint range(rad):")
    for j in range(hand_t.shape[1]):
        nm = LEFT_DELTO_JOINT_NAMES[j] if spec["type"] == "delto" else f"insp_{j}"
        print(f"        {nm:10s} [{hand_t[:,j].min():+.2f}, {hand_t[:,j].max():+.2f}]")
    print(f"      max frame step: {np.abs(np.diff(hand_t,axis=0)).max():.3f} rad")

    rclpy.init()
    io = HandIO(spec)
    try:
        print(f"[2/3] reading current hand pose ({spec['state']}) ...")
        q0 = io.wait_state()
        if q0 is None:
            print(f"      [warn] no live {spec['state']} (latched/static) — using hand_init as ramp start")
            q0 = np.asarray(HAND_INIT[args.side], dtype=np.float64)
        print(f"      current(rad): {np.round(q0,2).tolist()}")
        print(f"      ep start(rad): {np.round(hand_t[0],2).tolist()}")
        print(f"      start gap max: {np.abs(hand_t[0]-q0).max():.2f} rad")

        if not args.engage:
            print("[3/3] DRY-RUN — no motion. Re-run with --engage to move the hand.")
            return

        cmd = q0.copy(); soft = args.soft * SEND_DT
        TOL = 0.02   # rad convergence tolerance for the soft ramps
        def ramp_to(cmd, target):
            target = np.asarray(target, dtype=np.float64)
            while np.abs(target - cmd).max() > TOL:
                cmd = step_toward(cmd, target, soft); io.publish(cmd); time.sleep(SEND_DT)
            return cmd
        # PHASE A: home to the canonical hand_init pose
        init = np.asarray(HAND_INIT[args.side], dtype=np.float64)
        print(f"[3/3] ENGAGE — HOME to hand_init (gap {np.abs(init-cmd).max():.2f} rad) ...")
        cmd = ramp_to(cmd, init)
        for _ in range(15):     # settle at init
            io.publish(cmd); time.sleep(SEND_DT)
        print("      at hand_init. now ramping to episode start ...")
        # PHASE B: ramp init -> episode first frame, then replay
        cmd = ramp_to(cmd, hand_t[0])
        print("      replaying hand (model output) ...")
        spd = args.speed * SEND_DT
        period = 1.0 / fps; t_next = time.monotonic(); frame = 0
        while frame < N:
            cmd = step_toward(cmd, hand_t[frame], spd); io.publish(cmd); time.sleep(SEND_DT)
            if time.monotonic() >= t_next:
                t_next += period; frame += 1
        for _ in range(25):
            cmd = step_toward(cmd, hand_t[-1], spd); io.publish(cmd); time.sleep(SEND_DT)
        print("      replay complete.")
    except KeyboardInterrupt:
        print("\n[INTERRUPT]")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        rclpy.shutdown()
        print("done.")


if __name__ == "__main__":
    main()
