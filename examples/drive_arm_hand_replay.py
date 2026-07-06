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
import csv
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


def inspire_norm_to_driver_deg(norm6):
    """Normalized /inspire/right/target command -> driver-side received_angle degrees."""
    out = []
    for i, x in enumerate(norm6):
        x = float(x)
        if i < 4:
            a = x * 1100.0 + 750.0
        elif i == 4:
            a = x * 400.0 + 1100.0
        else:
            a = 1900.0 - x * 950.0
        out.append(a / 10.0)
    return out


def clamp01(x):
    return max(0.0, min(1.0, float(x)))


class TraceWriter:
    def __init__(self, trace_dir, metadata):
        self.dir = Path(trace_dir) if trace_dir else None
        self.f = None
        self.frame_rows = {}
        if self.dir is not None:
            self.dir.mkdir(parents=True, exist_ok=True)
            (self.dir / "trace_metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
            self.f = (self.dir / "trace_ticks.jsonl").open("w")

    def enabled(self):
        return self.f is not None

    def write_tick(self, row):
        if self.f is None:
            return
        self.f.write(json.dumps(row, sort_keys=True) + "\n")
        phase = row.get("phase")
        frame = row.get("episode_frame")
        if phase == "replay" and frame is not None and frame >= 0:
            self.frame_rows[int(frame)] = row

    def close(self):
        if self.f is not None:
            self.f.close()

    def write_frames_csv(self):
        if self.dir is None:
            return
        path = self.dir / "trace_frames.csv"
        fields = ["episode_frame", "episode_time_sec", "phase", "j6_hold_enabled",
                  "j6_episode_action_deg", "j6_effective_target_deg", "j6_actual_deg"]
        for prefix in ["arm_action_deg", "arm_cmd_frame_end_deg", "arm_actual_frame_end_deg",
                       "hand_action_deg", "hand_target_deg", "hand_actual_frame_end_deg"]:
            fields += [f"{prefix}_j{i}" for i in range(1, 7)]
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for frame in sorted(self.frame_rows):
                r = self.frame_rows[frame]
                out = {
                    "episode_frame": frame,
                    "episode_time_sec": r.get("episode_timestamp_sec"),
                    "phase": r.get("phase"),
                    "j6_hold_enabled": r.get("j6_hold_enabled"),
                    "j6_episode_action_deg": r.get("j6_episode_action_deg"),
                    "j6_effective_target_deg": (r.get("hand_target_driver_deg") or [None] * 6)[5],
                    "j6_actual_deg": (r.get("hand_actual_deg_raw") or [None] * 6)[5],
                }
                mapping = {
                    "arm_action_deg": r.get("arm_action_episode_deg"),
                    "arm_cmd_frame_end_deg": r.get("arm_cmd_sent_deg"),
                    "arm_actual_frame_end_deg": r.get("arm_actual_deg"),
                    "hand_action_deg": r.get("hand_action_episode_deg"),
                    "hand_target_deg": r.get("hand_target_driver_deg"),
                    "hand_actual_frame_end_deg": r.get("hand_actual_deg_raw"),
                }
                for prefix, vals in mapping.items():
                    vals = vals or [None] * 6
                    for i in range(6):
                        out[f"{prefix}_j{i+1}"] = vals[i] if i < len(vals) else None
                w.writerow(out)


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


def sync_ramp(arm, a0, a_tgt, hand, h0, h_tgt, arm_dps, hand_rps, publish_hand=None, tick_cb=None):
    """Move arm (deg) and hand to their targets TOGETHER — both finish at the same tick,
    so neither leads. n = the larger of the two step counts at each one's own speed limit
    (the faster one is slowed to stay in sync). Returns the final (arm_cmd, hand_cmd)."""
    a0 = np.asarray(a0, float); a_tgt = np.asarray(a_tgt, float)
    h0 = np.asarray(h0, float); h_tgt = np.asarray(h_tgt, float)
    an = np.abs(a_tgt - a0).max() / max(arm_dps * SEND_DT, 1e-9)
    hn = np.abs(h_tgt - h0).max() / max(hand_rps * SEND_DT, 1e-9)
    # smoothstep velocity profile: eases IN from 0 and OUT to 0 instead of jumping straight to
    # cruise speed — required for an acceleration-limited robot so the FIRST move starts slowly.
    # Peak velocity is 1.5x the average, so use 1.5x the steps to keep the peak under the cap.
    n = max(int(np.ceil(1.5 * max(an, hn))), 1)
    ac, hc = a0.copy(), h0.copy()
    for i in range(1, n + 1):
        t = i / n
        f = t * t * (3.0 - 2.0 * t)          # smoothstep: zero velocity at both ends
        ac = a0 + (a_tgt - a0) * f
        hc = h0 + (h_tgt - h0) * f
        tfs = arm.insert(ac)
        if publish_hand is not None:
            hand_info = publish_hand(hc)
        else:
            hand.publish(hc); hand_info = None
        if tick_cb is not None:
            tick_cb(ac, hc, tfs, hand_info)
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
        self.handshake_ok = threading.Event(); self.latest_q = None; self._tfs = 0.0; self._t0 = None
        self.latest_actual_deg = None; self.latest_actual_rx_monotonic = None
        self.disp.on_type["handshake_ack"] = lambda m: self.handshake_ok.set() if m.get("ok") else None
        self.disp.on_type["data"] = self._on_data
        self.disp.on_error = lambda e: print(f"[ARM ERR] code={e.get('error')} "
                                             f"msg={e.get('message')} hint={e.get('hint')}")

    def _on_data(self, m):
        r = m.get("result")
        if isinstance(r, dict) and r.get("_type") == "JObject" and r.get("position"):
            self.latest_q = [float(x) for x in r["position"]]
            self.latest_actual_deg = list(self.latest_q[:6])
            self.latest_actual_rx_monotonic = time.monotonic()

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
        # time_from_start = REAL elapsed since the first point (not a fixed SEND_DT increment), so
        # any gap (e.g. a predict() call) doesn't make the arm's scheduled trajectory drift behind
        # the immediately-applied hand.
        now = time.perf_counter()
        if self._t0 is None: self._t0 = now
        self._tfs = now - self._t0
        self.api.joint_traject_insert_point({"interval": SEND_DT, "time_from_start": self._tfs,
                                             "look_ahead_time": LOOK_AHEAD, "point": [float(x) for x in deg]})
        return self._tfs

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
        self.hand_actual_deg_raw = None; self.hand_target_01deg_raw = None; self.hand_actual_rx_monotonic = None
        if spec["type"] == "delto":
            self.pub = self.create_publisher(MultiDOFCommand, spec["ref"], 1); qos = JS_QOS
        else:
            self.pub = self.create_publisher(Float64MultiArray, spec["ref"], 1); qos = 1
        self.create_subscription(JointState, spec["state"], self._on_js, qos)

    def _on_js(self, msg):
        raw = np.asarray(msg.position, dtype=np.float64)[: self.ndof]
        by_name = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        self.hand_actual_deg_raw = np.asarray([by_name.get(f"j{i+1}", np.nan) for i in range(self.ndof)], dtype=np.float64)
        self.hand_target_01deg_raw = np.asarray([by_name.get(f"tj{i+1}", np.nan) for i in range(self.ndof)], dtype=np.float64)
        self.hand_actual_rx_monotonic = time.monotonic()
        # inspire joint_states are received_angle (deg-ish) -> rad to match the model units
        self.cur = np.asarray(inspire_angle_to_rad(raw)) if self.spec["type"] == "inspire" else raw

    def wait_state(self, timeout=6.0):
        t0 = time.time()
        while self.cur is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return None if self.cur is None else self.cur.copy()

    def publish(self, vals, effective_norm=None):
        if self.spec["type"] == "delto":
            m = MultiDOFCommand(); m.dof_names = LEFT_DELTO_JOINT_NAMES[: len(vals)]
            m.values = [float(x) for x in vals]; m.values_dot = [0.0] * len(vals)
        else:
            data = effective_norm if effective_norm is not None else inspire_rad_to_norm(vals)
            m = Float64MultiArray(data=[float(x) for x in data])
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
    ap.add_argument("--trace-dir", default="", help="write trace_metadata.json, trace_ticks.jsonl, trace_frames.csv")
    ap.add_argument("--hold-j6", action="store_true", help="RIGHT Inspire only: hold published j6 at current actual pose")
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
    episode_meta = json.loads((root / "episode.json").read_text())
    episode_frames = (episode_meta["frames"][: args.steps] if args.steps else episode_meta["frames"])
    trace = TraceWriter(args.trace_dir, {
        "source": args.source,
        "side": args.side,
        "episode_dir": str(root),
        "episode_frames": N,
        "episode_fps": fps,
        "send_dt_sec": SEND_DT,
        "arm_units": "deg for arm_action_episode_deg, arm_cmd_sent_deg, arm_actual_deg",
        "hand_units": {
            "hand_action_episode_rad": "episode action[6:12] original radians",
            "hand_action_episode_deg": "episode action[6:12] converted rad*180/pi",
            "hand_target_norm_requested": "normalized command from inspire_rad_to_norm(original hand command)",
            "hand_target_norm_effective": "actual normalized command published; j6 may be held when --hold-j6",
            "hand_target_driver_deg": "effective normalized command converted to driver received_angle degrees",
            "hand_actual_deg_raw": "/inspire/joint_states j1..j6 raw feedback degrees-ish",
            "hand_tj_deg": "/inspire/joint_states tj1..tj6 divided by 10; command-side value, not actual feedback",
        },
        "j6_hold_requested": bool(args.hold_j6),
        "j6_hold_formula": "clamp((1900 - j6_actual_deg * 10) / 950, 0, 1)",
        "arm_monitor_endpoint": GET_JOINTS,
        "arm_look_ahead_time": LOOK_AHEAD,
    })
    j6_hold_norm = None
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
        if args.hold_j6 and args.side == "right":
            if hand.hand_actual_deg_raw is not None and np.isfinite(hand.hand_actual_deg_raw[5]):
                j6_actual_deg = float(hand.hand_actual_deg_raw[5])
            else:
                j6_actual_deg = float(hand_q0[5] * 180.0 / math.pi)
            j6_hold_norm = clamp01((1900.0 - j6_actual_deg * 10.0) / 950.0)
            print(f"      --hold-j6: j6_actual={j6_actual_deg:.2f}deg -> hold_norm={j6_hold_norm:.4f}")
        if arm.latest_actual_rx_monotonic is None or time.monotonic() - arm.latest_actual_rx_monotonic > 0.2:
            print("      [warn] arm actual feedback is stale/missing before replay")

        if not args.engage:
            print("[3/3] DRY-RUN — no motion. Re-run with --engage."); return

        arm_cmd = arm_q0.copy(); hand_cmd = hand_q0.copy()

        def publish_hand_trace(vals):
            requested = np.asarray(inspire_rad_to_norm(vals), dtype=float)
            effective = requested.copy()
            if args.hold_j6 and args.side == "right" and j6_hold_norm is not None and len(effective) >= 6:
                effective[5] = j6_hold_norm
            hand.publish(vals, effective_norm=effective)
            return {
                "requested_norm": requested.tolist(),
                "effective_norm": effective.tolist(),
                "driver_deg": inspire_norm_to_driver_deg(effective),
            }

        def write_tick(phase, frame_idx, frame_advanced, arm_action_deg, hand_action_rad,
                       arm_cmd_sent_deg, hand_cmd_rad, arm_time_from_start, hand_info):
            rclpy.spin_once(hand, timeout_sec=0.0)
            now = time.monotonic()
            arm_actual_age = None if arm.latest_actual_rx_monotonic is None else now - arm.latest_actual_rx_monotonic
            hand_actual_age = None if hand.hand_actual_rx_monotonic is None else now - hand.hand_actual_rx_monotonic
            hand_action_deg = (np.asarray(hand_action_rad) * 180.0 / math.pi).tolist()
            row = {
                "timestamp_monotonic": now,
                "phase": phase,
                "episode_frame": int(frame_idx) if frame_idx is not None else -1,
                "episode_timestamp_sec": float(episode_frames[frame_idx]["timestamp"]) if frame_idx is not None and 0 <= frame_idx < len(episode_frames) else None,
                "frame_advanced": bool(frame_advanced),
                "arm_action_episode_deg": np.asarray(arm_action_deg, dtype=float).tolist() if arm_action_deg is not None else None,
                "arm_cmd_sent_deg": np.asarray(arm_cmd_sent_deg, dtype=float).tolist(),
                "arm_actual_deg": None if arm.latest_actual_deg is None else list(arm.latest_actual_deg),
                "arm_actual_age_sec": arm_actual_age,
                "arm_actual_stale": arm_actual_age is None or arm_actual_age > 0.2,
                "arm_time_from_start": arm_time_from_start,
                "arm_look_ahead_time": LOOK_AHEAD,
                "actual_send_dt_sec": SEND_DT,
                "hand_action_episode_rad": np.asarray(hand_action_rad, dtype=float).tolist() if hand_action_rad is not None else None,
                "hand_action_episode_deg": hand_action_deg if hand_action_rad is not None else None,
                "hand_target_norm_requested": None if hand_info is None else hand_info["requested_norm"],
                "hand_target_norm_effective": None if hand_info is None else hand_info["effective_norm"],
                "hand_target_driver_deg": None if hand_info is None else hand_info["driver_deg"],
                "hand_actual_deg_raw": None if hand.hand_actual_deg_raw is None else hand.hand_actual_deg_raw.tolist(),
                "hand_tj_deg": None if hand.hand_target_01deg_raw is None else (hand.hand_target_01deg_raw / 10.0).tolist(),
                "hand_actual_age_sec": hand_actual_age,
                "hand_actual_stale": hand_actual_age is None or hand_actual_age > 0.2,
                "j6_hold_enabled": bool(args.hold_j6 and args.side == "right"),
                "j6_episode_action_rad": float(hand_action_rad[5]) if hand_action_rad is not None and len(hand_action_rad) > 5 else None,
                "j6_episode_action_deg": float(hand_action_rad[5] * 180.0 / math.pi) if hand_action_rad is not None and len(hand_action_rad) > 5 else None,
                "j6_effective_target_norm": None if hand_info is None or len(hand_info["effective_norm"]) < 6 else hand_info["effective_norm"][5],
                "j6_hold_norm": j6_hold_norm,
            }
            trace.write_tick(row)

        # PHASE A: home to the mean init pose — arm + hand TIME-SYNCED (start & finish together)
        print(f"[3/3] ENGAGE — HOME to mean init (arm gap {np.abs(arm_init_deg-arm_cmd).max():.1f}deg, "
              f"hand gap {np.abs(hand_init-hand_cmd).max():.2f}rad) ...")
        arm_cmd, hand_cmd = sync_ramp(arm, arm_cmd, arm_init_deg, hand, hand_cmd, hand_init,
                                      args.arm_soft, args.hand_soft,
                                      publish_hand=publish_hand_trace,
                                      tick_cb=lambda ac, hc, tfs, hi: write_tick("home", -1, False, arm_init_deg, hand_init, ac, hc, tfs, hi))
        for _ in range(40):
            tfs = arm.insert(arm_cmd); hi = publish_hand_trace(hand_cmd)
            write_tick("home", -1, False, arm_init_deg, hand_init, arm_cmd, hand_cmd, tfs, hi)
            time.sleep(SEND_DT)
        print("      at mean init. ramping to episode start ...")
        # PHASE B: init -> episode first frame (also synced), then replay
        arm_cmd, hand_cmd = sync_ramp(arm, arm_cmd, arm_t[0], hand, hand_cmd, hand_t[0],
                                      args.arm_soft, args.hand_soft,
                                      publish_hand=publish_hand_trace,
                                      tick_cb=lambda ac, hc, tfs, hi: write_tick("ramp_to_episode", 0, False, arm_t[0], hand_t[0], ac, hc, tfs, hi))
        print("      replaying arm+hand ...")
        a_spd, h_spd = args.arm_speed*SEND_DT, args.hand_speed*SEND_DT
        period = 1.0/fps; t_next = time.monotonic(); frame = 0
        while frame < N:
            cur_frame = frame
            arm_cmd, hand_cmd = synced_step(arm_cmd, arm_t[cur_frame], hand_cmd, hand_t[cur_frame], a_spd, h_spd)
            tfs = arm.insert(arm_cmd); hi = publish_hand_trace(hand_cmd)
            time.sleep(SEND_DT)
            if time.monotonic() >= t_next:
                t_next += period; frame += 1
                advanced = True
            else:
                advanced = False
            write_tick("replay", cur_frame, advanced, arm_t[cur_frame], hand_t[cur_frame],
                       arm_cmd, hand_cmd, tfs, hi)
        for _ in range(40):
            arm_cmd, hand_cmd = synced_step(arm_cmd, arm_t[-1], hand_cmd, hand_t[-1], a_spd, h_spd)
            tfs = arm.insert(arm_cmd); hi = publish_hand_trace(hand_cmd)
            write_tick("settle", N - 1, False, arm_t[-1], hand_t[-1], arm_cmd, hand_cmd, tfs, hi)
            time.sleep(SEND_DT)
        print("      replay complete.")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] stop.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        trace.write_frames_csv()
        trace.close()
        arm.stop(); arm.close()
        rclpy.shutdown()
        print("stopped.")


if __name__ == "__main__":
    main()
