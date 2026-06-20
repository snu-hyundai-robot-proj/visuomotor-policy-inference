#!/usr/bin/env python3
"""Closed loop: LIVE cameras + state -> policy -> drive the REAL arm + hand, repeat. Side-aware.

  every tick:
    front = /system_<side>/zivid_rgb, wrist = /system_<side>/d405_rgb
    state[26] = arm6(rad) + hand20(rad)   ->  POST /predict
      LEFT  hand20 = DG-5F-M canonical (lj_dg_1_1..5_4)
      RIGHT hand20 = Inspire6(rad) + 14 zeros (dataset pad)
    action[26] -> arm = action[0:6] (rad->deg) -> HDR35 OpenStream TCP (left .152 / right .151)
                  hand = action[hand_slice] (rad):
                    LEFT  -> MultiDOFCommand -> /dg5f_left/lj_dg_pospid/reference
                    RIGHT -> rad->[0,1]      -> Float64MultiArray -> /inspire/right/target

Run INSIDE the teleop container. The side's hand driver + vision node must be up
(docker compose --profile <side> up -d).

  python3 run_robot_loop.py --side right --hand-only            # DRY-RUN, arm off
  python3 run_robot_loop.py --side right --engage               # move ARM + hand (arm powered)

SAFETY: DRY-RUN default; homes to mean init first; per-tick velocity clamp; api.stop on exit;
--secs caps the run. Static scene -> model holds near init; put a task object in view for action.
"""
from __future__ import annotations

import argparse
import base64
import io
import math
import sys
import threading
import time

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image
from PIL import Image as PILImage

HDR_UTILS = "/workspace/src/Robot_/src/hdr_stream"
sys.path.insert(0, HDR_UTILS)
from hdr_stream.utils.net import NetClient          # noqa: E402
from hdr_stream.utils.api import OpenStreamAPI       # noqa: E402
from hdr_stream.utils.parser import NDJSONParser     # noqa: E402
from hdr_stream.utils.dispatcher import Dispatcher   # noqa: E402

SEND_DT = 0.005
# arm trajectory look-ahead. Larger = smoother but the arm lags its command (trailing the
# hand). 0.1 keeps some smoothing while cutting the arm-vs-hand lag. (was 0.2)
LOOK_AHEAD = 0.1
GET_JOINTS = "/project/robot/joints/joint_states"
JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
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


def step_toward(c, d, m): c = np.asarray(c); return c + np.clip(np.asarray(d) - c, -m, m)


def synced_step(arm_cur, arm_tgt, hand_cur, hand_tgt, a_max, h_max):
    """ONE time-synced step toward the targets: arm (deg) and hand are scaled by the SAME
    factor so neither exceeds its per-tick cap (a_max deg / h_max rad) AND they stay aligned.
    If a model action jumps, both arm and hand slow by the same factor — safe AND in sync,
    instead of per-joint clamps letting one side lag the other. s=1 when both deltas fit."""
    arm_cur = np.asarray(arm_cur, float); hand_cur = np.asarray(hand_cur, float)
    da = np.asarray(arm_tgt, float) - arm_cur
    dh = np.asarray(hand_tgt, float) - hand_cur
    need = max(np.abs(da).max() / a_max if a_max > 0 else 0.0,
               np.abs(dh).max() / h_max if h_max > 0 else 0.0, 1.0)
    s = 1.0 / need
    return arm_cur + da * s, hand_cur + dh * s


def sync_ramp(arm, a0, a_tgt, publish, h0, h_tgt, arm_dps, hand_rps, do_hand):
    """Ramp arm (deg) and hand to their targets TOGETHER — both finish on the same tick so
    neither leads. n = larger step count at each one's own speed cap. Returns (arm_cmd, hand_cmd)."""
    a0 = np.asarray(a0, float); a_tgt = np.asarray(a_tgt, float)
    h0 = np.asarray(h0, float); h_tgt = np.asarray(h_tgt, float)
    an = int(np.ceil(np.abs(a_tgt - a0).max() / max(arm_dps * SEND_DT, 1e-9))) if arm is not None else 0
    hn = int(np.ceil(np.abs(h_tgt - h0).max() / max(hand_rps * SEND_DT, 1e-9))) if do_hand else 0
    n = max(an, hn, 1)
    ac, hc = a0.copy(), h0.copy()
    for i in range(1, n + 1):
        f = i / n
        ac = a0 + (a_tgt - a0) * f
        hc = h0 + (h_tgt - h0) * f
        if arm is not None: arm.insert(ac)
        if do_hand: publish(hc)
        time.sleep(SEND_DT)
    return ac, hc


def decode_rgb(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    a = np.frombuffer(msg.data, dtype=np.uint8)
    step = msg.step or w * 3
    a = a.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding == "bgr8":
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a)


def jpeg_b64(rgb):
    b = io.BytesIO(); PILImage.fromarray(rgb).save(b, "JPEG", quality=90)
    return base64.b64encode(b.getvalue()).decode()


def hand_obs20(msg: JointState, hand_type: str) -> np.ndarray:
    """build the 20-d hand part of the model state from joint_states."""
    v = np.zeros(20, dtype=np.float64)
    if hand_type == "delto":
        for name, pos in zip(msg.name, msg.position):
            t = name.split("_")
            if len(t) >= 4:
                idx = 4 * (int(t[2]) - 1) + (int(t[3]) - 1)
                if 0 <= idx < 20:
                    v[idx] = pos
    else:  # inspire: 6 received_angle (deg-ish) -> rad, rest stay 0 (dataset pad)
        for i in range(min(6, len(msg.position))):
            v[i] = float(msg.position[i]) * math.pi / 180.0
    return v


class ArmClient:
    def __init__(self, ip):
        self.net = NetClient(ip, 49000); self.api = OpenStreamAPI(self.net)
        self.parser = NDJSONParser(); self.disp = Dispatcher()
        self.ok = threading.Event(); self.q = None; self._tfs = 0.0
        self.disp.on_type["handshake_ack"] = lambda m: self.ok.set() if m.get("ok") else None
        self.disp.on_type["data"] = self._d; self.disp.on_error = lambda e: None

    def _d(self, m):
        r = m.get("result")
        if isinstance(r, dict) and r.get("_type") == "JObject" and r.get("position"):
            self.q = [float(x) for x in r["position"]]

    def connect(self, t=5.0):
        self.net.connect(); self.net.start_recv_loop(lambda b: self.parser.feed(b, self.disp.dispatch))
        self.api.handshake(major=1)
        if not self.ok.wait(t): raise RuntimeError("arm handshake timeout")
        self.api.monitor(url=GET_JOINTS, period_ms=4, args={}); self.api.joint_traject_init()

    def wait(self, t=5.0):
        t0 = time.time()
        while self.q is None:
            if time.time() - t0 > t: raise RuntimeError("no arm state")
            time.sleep(0.02)
        return list(self.q)

    def state_rad(self):
        return None if self.q is None else np.asarray(self.q[:6]) * math.pi / 180.0

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


class LoopIO(Node):
    def __init__(self, side, spec, url):
        super().__init__("run_robot_loop")
        self.url = url.rstrip("/"); self.spec = spec
        self.front = self.wrist = self.hand = None
        self.front_b64 = self.wrist_b64 = None
        self.finfo = self.winfo = "?"
        self.create_subscription(Image, f"/system_{side}/zivid_rgb", self._f, 1)
        self.create_subscription(Image, f"/system_{side}/d405_rgb", self._w, 1)
        js_qos = JS_QOS if spec["type"] == "delto" else 1
        self.create_subscription(JointState, spec["state"],
                                 lambda m: setattr(self, "hand", hand_obs20(m, spec["type"])), js_qos)
        if spec["type"] == "delto":
            self.pub = self.create_publisher(MultiDOFCommand, spec["ref"], 1)
        else:
            self.pub = self.create_publisher(Float64MultiArray, spec["ref"], 1)
        self.info = requests.get(f"{self.url}/info", timeout=10).json()
        requests.post(f"{self.url}/reset", timeout=10)

    # encode each NEW frame to JPEG once (not once per /predict) — payload bytes are identical
    def _f(self, m): self.front = decode_rgb(m); self.front_b64 = jpeg_b64(self.front); self.finfo = f"{m.width}x{m.height}"
    def _w(self, m): self.wrist = decode_rgb(m); self.wrist_b64 = jpeg_b64(self.wrist); self.winfo = f"{m.width}x{m.height}"

    def publish_hand(self, vals):
        if self.spec["type"] == "delto":
            m = MultiDOFCommand(); m.dof_names = LEFT_DELTO_JOINT_NAMES[: len(vals)]
            m.values = [float(x) for x in vals]; m.values_dot = [0.0] * len(vals)
        else:
            m = Float64MultiArray(data=[float(x) for x in inspire_rad_to_norm(vals)])
        self.pub.publish(m)

    def predict(self, arm6):
        state = np.concatenate([arm6, self.hand]).astype(np.float32)   # [arm6 + hand20] = 26
        payload = {"front_rgb": self.front_b64, "wrist_rgb": self.wrist_b64, "state": state.tolist()}
        t = time.perf_counter()
        a = np.asarray(requests.post(f"{self.url}/predict", json=payload, timeout=30).json()["action"])
        return a, (time.perf_counter() - t) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--hand-only", action="store_true", help="arm off: arm-6 state = fixed init")
    ap.add_argument("--no-hand", action="store_true", help="don't drive/read the hand (hand off): "
                    "use fixed init for the hand-20 state, drive the arm only")
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--arm-speed", type=float, default=60.0, help="deg/s per-tick clamp")
    ap.add_argument("--hand-speed", type=float, default=2.0, help="rad/s per-tick clamp")
    ap.add_argument("--home-arm-speed", type=float, default=8.0)
    ap.add_argument("--home-hand-speed", type=float, default=0.5)
    ap.add_argument("--substeps", type=int, default=8)
    ap.add_argument("--engage", action="store_true")
    args = ap.parse_args()

    spec = HAND[args.side]; sl = spec["slice"]; ndof = spec["ndof"]
    arm_init_deg = np.asarray(ARM_INIT_RAD[args.side]) * 180.0 / math.pi
    arm_init_rad = np.asarray(ARM_INIT_RAD[args.side])
    hand_init = np.asarray(spec["init"], dtype=np.float64)

    hand_obs_init = np.zeros(20, dtype=np.float64)
    hand_obs_init[: len(spec["init"])] = spec["init"]   # 20-d hand state for --no-hand

    rclpy.init()
    node = LoopIO(args.side, spec, args.url)
    if args.no_hand:
        node.hand = hand_obs_init.copy()
    arm = None if args.hand_only else ArmClient(ARM_IP[args.side])
    a_clamp, h_clamp = args.arm_speed * SEND_DT, args.hand_speed * SEND_DT
    try:
        print(f"[1/3] side={args.side} | server={node.info['model_id']} | hand={spec['type']} "
              f"| mode={'HAND-ONLY' if args.hand_only else 'ARM+HAND'}")
        if arm is not None:
            print(f"      connecting arm {ARM_IP[args.side]} ..."); arm.connect(); aq = np.asarray(arm.wait()[:6])
        else:
            aq = arm_init_deg.copy()
        t0 = time.time()
        while (node.front is None or node.wrist is None or node.hand is None) and time.time() - t0 < 12:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.front is None or node.wrist is None:
            print(f"[ABORT] cameras not publishing — is system_vision_{args.side} running?"); return
        if node.hand is None:
            print("      [warn] no live hand state — using init"); node.hand = hand_obs_init.copy()
        print(f"      cameras front[{node.finfo}] wrist[{node.winfo}] | hand state OK")

        if not args.engage:
            print("[2/3] DRY-RUN — predicting 5x, target deltas (no motion):")
            arm6 = arm_init_rad if arm is None else arm.state_rad()
            for k in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
                if arm is not None: arm6 = arm.state_rad()
                act, ms = node.predict(arm6)
                d_arm = (act[:6] * 180.0 / math.pi) - aq
                print(f"  [{k}] {ms:.0f}ms arm dtarget(deg)={np.round(d_arm,1).tolist()} "
                      f"hand_tgt={np.round(act[sl[0]:sl[1]],2).tolist()}")
                time.sleep(0.3)
            print("[3/3] DRY-RUN done. Re-run with --engage."); return

        # PHASE A: home to mean init — arm + hand TIME-SYNCED (start & finish together)
        print("[2/3] ENGAGE — homing to mean init ...")
        arm_cmd = aq.copy(); hand_cmd = node.hand[:ndof].copy()
        def hold(n=40):   # flush the latest command for n ticks so the robot settles
            for _ in range(n):
                if arm is not None: arm.insert(arm_cmd)
                if not args.no_hand: node.publish_hand(hand_cmd)
                time.sleep(SEND_DT)
        arm_cmd, hand_cmd = sync_ramp(arm, arm_cmd, arm_init_deg, node.publish_hand,
                                      hand_cmd, hand_init, args.home_arm_speed,
                                      args.home_hand_speed, not args.no_hand)
        hold()

        # PHASE B: closed loop
        print(f"[3/3] CLOSED LOOP — live -> model -> robot ({args.secs:.0f}s, "
              f"clamp arm {args.arm_speed}deg/s hand {args.hand_speed}rad/s) ...")
        t_start = time.time(); k = 0
        while time.time() - t_start < args.secs and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            arm6 = arm_init_rad if arm is None else arm.state_rad()
            if node.front is None or node.wrist is None or arm6 is None:
                time.sleep(0.02); continue
            act, ms = node.predict(arm6)
            if not np.all(np.isfinite(act)):
                print(f"  [{k}] non-finite — skip"); continue
            arm_tgt = act[:6] * 180.0 / math.pi
            hand_tgt = act[sl[0]:sl[1]]
            for _ in range(args.substeps):
                if arm is not None and not args.no_hand:        # arm+hand: step in lockstep
                    arm_cmd, hand_cmd = synced_step(arm_cmd, arm_tgt, hand_cmd, hand_tgt, a_clamp, h_clamp)
                    arm.insert(arm_cmd); node.publish_hand(hand_cmd)
                elif arm is not None:                           # arm only (--no-hand)
                    arm_cmd = step_toward(arm_cmd, arm_tgt, a_clamp); arm.insert(arm_cmd)
                elif not args.no_hand:                          # hand only (--hand-only)
                    hand_cmd = step_toward(hand_cmd, hand_tgt, h_clamp); node.publish_hand(hand_cmd)
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(SEND_DT)
            if k % 10 == 0:
                ht = "off" if args.no_hand else np.round(hand_tgt, 2).tolist()
                print(f"  [{k}] {ms:.0f}ms arm_tgt(deg)={np.round(arm_tgt,1).tolist()} hand_tgt={ht}")
            k += 1
        hold()
        print(f"      closed loop done ({k} predictions).")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] stopping.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if arm is not None:
            arm.stop(); arm.close()
        rclpy.shutdown(); print("stopped.")


if __name__ == "__main__":
    main()
