#!/usr/bin/env python3
"""Replay a recorded episode through the policy and drive the REAL arm + hand. Side-aware.

  recorded episode (state + front/wrist imgs) -> /predict -> action[26]
  arm  = action[0:6] (rad) -> deg -> HDR35 OpenStream TCP
           left=192.168.4.152  right=192.168.4.151  (:49000)
  hand = action[hand_slice] (rad):
    LEFT  (DG-5F-M, 20): action[6:26] -> MultiDOFCommand -> /dg5f_left/lj_dg_pospid/reference
    RIGHT (Inspire, 6):  action[6:12] -> rad->[0,1] -> Float64MultiArray -> /inspire/right/target

Run INSIDE the teleop container (rclpy + control_msgs + reaches the arm/predict). The hand
driver for that side must be running (dg5f pospid controller / inspire_driver_node).

  python3 drive_arm_hand_replay.py --side right --steps 0            # DRY-RUN (no motion)
  python3 drive_arm_hand_replay.py --side right --steps 0 --engage   # move right arm + Inspire

SAFETY: DRY-RUN default; homes to mean init first; soft-start + per-tick speed clamp on both
arm and hand; api.stop on exit. Feeds RECORDED state so the policy reproduces the real trajectory.
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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# /dg5f_*/joint_states is RELIABLE + TRANSIENT_LOCAL (latched); inspire streams (default QoS).
JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

HDR_UTILS = "/workspace/src/Robot_/src/hdr_stream"
sys.path.insert(0, HDR_UTILS)
from hdr_stream.utils.net import NetClient          # noqa: E402
from hdr_stream.utils.api import OpenStreamAPI       # noqa: E402
from hdr_stream.utils.parser import NDJSONParser     # noqa: E402
from hdr_stream.utils.dispatcher import Dispatcher   # noqa: E402

SEND_DT = 0.005
# arm trajectory look-ahead. Larger = smoother but the arm physically lags its command
# (and so trails the hand). 0.1 keeps some smoothing while cutting the arm-vs-hand lag. (was 0.2)
LOOK_AHEAD = 0.1
GET_JOINTS = "/project/robot/joints/joint_states"
LEFT_DELTO_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4", "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4", "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]
ARM_IP = {"left": "192.168.4.152", "right": "192.168.4.151"}
ARM_INIT_RAD = {
    "left":  [-1.734299, 1.679489, -0.113224, -0.657517, -1.625038, -0.202079],
    "right": [1.737826, 1.765172, -0.192581, 0.688188, -1.419986, 0.320249],
}
HAND = {
    "left":  {"type": "delto", "ndof": 20, "slice": (6, 26), "ref": "/dg5f_left/lj_dg_pospid/reference",
              "state": "/dg5f_left/joint_states",
              "init": [-0.174693, 0.088572, -0.171882, -0.064431, -0.344696, 0.039543, 0.383013, 0.300303,
                       -0.401026, 0.188575, 0.13543, 0.108224, -0.372288, 0.453359, 0.029937, 0.037518,
                       0.000573, -0.427912, 0.54842, 0.033721]},
    "right": {"type": "inspire", "ndof": 6, "slice": (6, 12), "ref": "/inspire/right/target",
              "state": "/inspire/joint_states",
              "init": [2.89855, 2.878596, 2.811947, 2.807127, 2.324232, 2.490377]},
}

# Inspire (RIGHT): model action/state are joint angles in RADIANS (~1.4-3.3); /inspire/right/target
# wants normalized ~[0,1] (retarget_fingers -> actuator 880-1740). Mapping: actuator = rad*1800/pi,
# received_angle (joint_states) ≈ actuator/10 ≈ degrees.
INSPIRE_K = 1800.0 / math.pi


def inspire_rad_to_norm(rad6):
    out = []
    for i, r in enumerate(rad6):
        a = float(r) * INSPIRE_K
        if i < 4:    x = (a - 750.0) / 1100.0
        elif i == 4: x = (a - 1100.0) / 400.0
        else:        x = (1900.0 - a) / 950.0
        out.append(x)
    return out


def inspire_angle_to_rad(ang6):
    return [float(a) * math.pi / 180.0 for a in ang6]


def b64(p): return base64.b64encode(p.read_bytes()).decode("ascii")
def step_toward(cmd, des, mx): cmd = np.asarray(cmd); return cmd + np.clip(np.asarray(des) - cmd, -mx, mx)


def synced_step(arm_cur, arm_tgt, hand_cur, hand_tgt, a_max, h_max):
    """ONE time-synced step toward the targets: arm (deg) and hand are scaled by the SAME
    factor so neither exceeds its per-tick cap (a_max deg / h_max rad) AND they stay aligned
    (no joint leads the other). If both deltas already fit the caps the step is exact (s=1).
    This replaces per-joint independent clamping, which let the hand lag the arm (or vice
    versa) whenever one side's recorded/commanded step exceeded its own cap."""
    arm_cur = np.asarray(arm_cur, float); hand_cur = np.asarray(hand_cur, float)
    da = np.asarray(arm_tgt, float) - arm_cur
    dh = np.asarray(hand_tgt, float) - hand_cur
    need = max(np.abs(da).max() / a_max if a_max > 0 else 0.0,
               np.abs(dh).max() / h_max if h_max > 0 else 0.0, 1.0)
    s = 1.0 / need
    return arm_cur + da * s, hand_cur + dh * s


def sync_ramp(arm, a0, a_tgt, hand, h0, h_tgt, arm_dps, hand_rps):
    """Move arm (deg) and hand to their targets TOGETHER — both finish at the same tick,
    so neither leads. n = the larger of the two step counts at each one's own speed limit
    (the faster one is slowed to stay in sync). Returns the final (arm_cmd, hand_cmd)."""
    a0 = np.asarray(a0, float); a_tgt = np.asarray(a_tgt, float)
    h0 = np.asarray(h0, float); h_tgt = np.asarray(h_tgt, float)
    an = int(np.ceil(np.abs(a_tgt - a0).max() / max(arm_dps * SEND_DT, 1e-9)))
    hn = int(np.ceil(np.abs(h_tgt - h0).max() / max(hand_rps * SEND_DT, 1e-9)))
    n = max(an, hn, 1)
    ac, hc = a0.copy(), h0.copy()
    for i in range(1, n + 1):
        f = i / n
        ac = a0 + (a_tgt - a0) * f
        hc = h0 + (h_tgt - h0) * f
        arm.insert(ac); hand.publish(hc)
        time.sleep(SEND_DT)
    return ac, hc


def compute_actions(root: Path, url: str, steps, sl, source="model"):
    meta = json.loads((root / "episode.json").read_text())
    frames = meta["frames"][:steps] if steps else meta["frames"]
    arm_deg, hand_rad = [], []
    if source == "recorded":
        for fr in frames:
            a = np.asarray(fr["action"], dtype=np.float64)
            arm_deg.append(a[:6] * 180.0 / math.pi); hand_rad.append(a[sl[0]:sl[1]])
    else:
        s = requests.Session(); s.post(f"{url}/reset", timeout=10).raise_for_status()
        for fr in frames:
            a = np.asarray(s.post(f"{url}/predict", json={
                "front_rgb": b64(root / "front_rgb" / f"{fr['frame']}.jpg"),
                "wrist_rgb": b64(root / "wrist_rgb" / f"{fr['frame']}.jpg"),
                "state": fr["state"]}, timeout=30).json()["action"], dtype=np.float64)
            arm_deg.append(a[:6] * 180.0 / math.pi); hand_rad.append(a[sl[0]:sl[1]])
    return np.asarray(arm_deg), np.asarray(hand_rad), meta.get("fps", 30)


class ArmClient:
    def __init__(self, ip, port):
        self.net = NetClient(ip, int(port)); self.api = OpenStreamAPI(self.net)
        self.parser = NDJSONParser(); self.disp = Dispatcher()
        self.handshake_ok = threading.Event(); self.latest_q = None; self._tfs = 0.0
        self.disp.on_type["handshake_ack"] = lambda m: self.handshake_ok.set() if m.get("ok") else None
        self.disp.on_type["data"] = self._on_data
        self.disp.on_error = lambda e: print(f"[robot err] {e}")

    def _on_data(self, m):
        r = m.get("result")
        if isinstance(r, dict) and r.get("_type") == "JObject" and r.get("position"):
            self.latest_q = [float(x) for x in r["position"]]

    def connect_and_init(self, timeout=5.0):
        self.net.connect(); self.net.start_recv_loop(lambda b: self.parser.feed(b, self.disp.dispatch))
        self.api.handshake(major=1)
        if not self.handshake_ok.wait(timeout): raise RuntimeError("arm handshake timeout")
        self.api.monitor(url=GET_JOINTS, period_ms=4, args={}); self.api.joint_traject_init()

    def wait_pose(self, timeout=5.0):
        t0 = time.time()
        while self.latest_q is None:
            if time.time() - t0 > timeout: raise RuntimeError("no arm joint state")
            time.sleep(0.02)
        return list(self.latest_q)

    def insert(self, deg):
        self.api.joint_traject_insert_point({"interval": SEND_DT, "time_from_start": self._tfs,
                                             "look_ahead_time": LOOK_AHEAD, "point": [float(x) for x in deg]})
        self._tfs += SEND_DT

    def stop(self):
        try: self.api.stop(target="control")
        except Exception: pass
    def close(self):
        try: self.net.close()
        except Exception: pass


class HandIO(Node):
    """read current hand joints (rad), publish hand reference. delto or inspire."""
    def __init__(self, spec):
        super().__init__("drive_arm_hand_io")
        self.spec = spec; self.ndof = spec["ndof"]; self.cur = None
        if spec["type"] == "delto":
            self.pub = self.create_publisher(MultiDOFCommand, spec["ref"], 1); qos = JS_QOS
        else:
            self.pub = self.create_publisher(Float64MultiArray, spec["ref"], 1); qos = 1
        self.create_subscription(JointState, spec["state"], self._on_js, qos)

    def _on_js(self, msg):
        raw = np.asarray(msg.position, dtype=np.float64)[: self.ndof]
        # inspire joint_states are received_angle (deg-ish) -> rad to match the model units
        self.cur = np.asarray(inspire_angle_to_rad(raw)) if self.spec["type"] == "inspire" else raw

    def wait_state(self, timeout=6.0):
        t0 = time.time()
        while self.cur is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return None if self.cur is None else self.cur.copy()

    def publish(self, vals):
        if self.spec["type"] == "delto":
            m = MultiDOFCommand(); m.dof_names = LEFT_DELTO_JOINT_NAMES[: len(vals)]
            m.values = [float(x) for x in vals]; m.values_dot = [0.0] * len(vals)
        else:
            m = Float64MultiArray(data=[float(x) for x in inspire_rad_to_norm(vals)])
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--episode-dir", default="")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--port", type=int, default=49000)
    ap.add_argument("--source", choices=["model", "recorded"], default="model")
    ap.add_argument("--steps", type=int, default=120)
    ap.add_argument("--arm-soft", type=float, default=8.0)      # deg/s (home/ramp cap)
    ap.add_argument("--arm-speed", type=float, default=100.0)   # deg/s (replay cap)
    ap.add_argument("--hand-soft", type=float, default=0.6)     # rad/s home/ramp cap (synced w/ arm)
    ap.add_argument("--hand-speed", type=float, default=8.0)    # rad/s replay cap — must cover the
    #   recorded hand step rate (inspire ~7 rad/s @30fps) or the hand lags the arm; synced_step
    #   keeps arm+hand aligned even when it does bind
    ap.add_argument("--engage", action="store_true")
    args = ap.parse_args()

    spec = HAND[args.side]
    root = Path(args.episode_dir or f"/tmp/sample_episodes/{args.side}")
    arm_init_deg = np.asarray(ARM_INIT_RAD[args.side]) * 180.0 / math.pi
    hand_init = np.asarray(spec["init"], dtype=np.float64)

    print(f"[1/3] side={args.side} arm={ARM_IP[args.side]} hand={spec['type']} | "
          f"computing actions ({args.steps or 'all'} frames) via {args.url} ...")
    arm_t, hand_t, fps = compute_actions(root, args.url, args.steps, spec["slice"], args.source)
    N = len(arm_t)
    print(f"      {N} frames @ {fps}Hz")
    print(f"      arm range/joint(deg): " + ", ".join(f"[{arm_t[:,j].min():.0f},{arm_t[:,j].max():.0f}]" for j in range(6)))
    print(f"      hand range overall(rad): [{hand_t.min():.2f},{hand_t.max():.2f}], max step {np.abs(np.diff(hand_t,axis=0)).max():.3f}")

    rclpy.init()
    hand = HandIO(spec)
    arm = ArmClient(ARM_IP[args.side], args.port)
    try:
        print(f"[2/3] connecting arm {ARM_IP[args.side]} + reading hand state {spec['state']} ...")
        arm.connect_and_init(); arm_q0 = np.asarray(arm.wait_pose()[:6])
        hand_q0 = hand.wait_state(timeout=6.0)
        if hand_q0 is None:
            print(f"      [warn] no live {spec['state']} (latched/static) — using init as ramp start")
            hand_q0 = hand_init.copy()
        print(f"      arm  q0(deg): {np.round(arm_q0,1).tolist()}  start gap max {np.abs(arm_t[0]-arm_q0).max():.1f}")
        print(f"      hand q0(rad): {np.round(hand_q0,2).tolist()}")
        print(f"      hand start gap max {np.abs(hand_t[0]-hand_q0).max():.2f} rad")

        if not args.engage:
            print("[3/3] DRY-RUN — no motion. Re-run with --engage."); return

        arm_cmd = arm_q0.copy(); hand_cmd = hand_q0.copy()
        # PHASE A: home to the mean init pose — arm + hand TIME-SYNCED (start & finish together)
        print(f"[3/3] ENGAGE — HOME to mean init (arm gap {np.abs(arm_init_deg-arm_cmd).max():.1f}deg, "
              f"hand gap {np.abs(hand_init-hand_cmd).max():.2f}rad) ...")
        arm_cmd, hand_cmd = sync_ramp(arm, arm_cmd, arm_init_deg, hand, hand_cmd, hand_init,
                                      args.arm_soft, args.hand_soft)
        for _ in range(40):
            arm.insert(arm_cmd); hand.publish(hand_cmd); time.sleep(SEND_DT)
        print("      at mean init. ramping to episode start ...")
        # PHASE B: init -> episode first frame (also synced), then replay
        arm_cmd, hand_cmd = sync_ramp(arm, arm_cmd, arm_t[0], hand, hand_cmd, hand_t[0],
                                      args.arm_soft, args.hand_soft)
        print("      replaying arm+hand ...")
        a_spd, h_spd = args.arm_speed*SEND_DT, args.hand_speed*SEND_DT
        period = 1.0/fps; t_next = time.monotonic(); frame = 0
        while frame < N:
            arm_cmd, hand_cmd = synced_step(arm_cmd, arm_t[frame], hand_cmd, hand_t[frame], a_spd, h_spd)
            arm.insert(arm_cmd); hand.publish(hand_cmd)
            time.sleep(SEND_DT)
            if time.monotonic() >= t_next:
                t_next += period; frame += 1
        for _ in range(40):
            arm_cmd, hand_cmd = synced_step(arm_cmd, arm_t[-1], hand_cmd, hand_t[-1], a_spd, h_spd)
            arm.insert(arm_cmd); hand.publish(hand_cmd); time.sleep(SEND_DT)
        print("      replay complete.")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] stop.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        arm.stop(); arm.close()
        rclpy.shutdown()
        print("stopped.")


if __name__ == "__main__":
    main()
