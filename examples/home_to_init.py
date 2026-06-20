#!/usr/bin/env python3
"""Move the robot (arm + hand) to the canonical mean INIT pose, then stop. No replay.

init pose = mean of episode-start state (from the HF repo's hand_init_pose.json):
  arm  -> HDR35 OpenStream TCP (deg)
  hand -> LEFT: DG-5F-M pospid (MultiDOFCommand, 20) / RIGHT: Inspire (Float64MultiArray, 6)

Run inside the teleop container (rclpy + control_msgs + reaches the arm; hand driver running):
  python3 home_to_init.py --side left            # DRY-RUN (no motion)
  python3 home_to_init.py --side left --engage   # move to init
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# joint_state_broadcaster publishes RELIABLE + TRANSIENT_LOCAL (latched); match it
# so we receive the last value immediately even when the hand is static.
JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

HDR_UTILS = "/workspace/src/Robot_/src/hdr_stream"
sys.path.insert(0, HDR_UTILS)
from hdr_stream.utils.net import NetClient          # noqa: E402
from hdr_stream.utils.api import OpenStreamAPI       # noqa: E402
from hdr_stream.utils.parser import NDJSONParser     # noqa: E402
from hdr_stream.utils.dispatcher import Dispatcher   # noqa: E402

SEND_DT = 0.005
LOOK_AHEAD = 0.2
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
HAND_INIT = {
    "left":  [-0.174693, 0.088572, -0.171882, -0.064431, -0.344696, 0.039543, 0.383013, 0.300303,
              -0.401026, 0.188575, 0.13543, 0.108224, -0.372288, 0.453359, 0.029937, 0.037518,
              0.000573, -0.427912, 0.54842, 0.033721],
    "right": [2.89855, 2.878596, 2.811947, 2.807127, 2.324232, 2.490377],
}
HAND = {
    "left":  {"type": "delto",   "ndof": 20, "ref": "/dg5f_left/lj_dg_pospid/reference", "state": "/dg5f_left/joint_states"},
    "right": {"type": "inspire", "ndof": 6,  "ref": "/inspire/right/target",            "state": "/inspire/joint_states"},
}


def step_toward(c, d, m): c = np.asarray(c); return c + np.clip(np.asarray(d) - c, -m, m)


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
    def __init__(self, spec):
        super().__init__("home_to_init"); self.spec = spec; self.cur = None
        cls = MultiDOFCommand if spec["type"] == "delto" else Float64MultiArray
        self.pub = self.create_publisher(cls, spec["ref"], 1)
        self.create_subscription(JointState, spec["state"],
                                 lambda m: setattr(self, "cur", np.asarray(m.position)[: spec["ndof"]]), JS_QOS)

    def wait(self, t=6.0):
        t0 = time.time()
        while self.cur is None and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return None if self.cur is None else self.cur.copy()

    def publish(self, v):
        if self.spec["type"] == "delto":
            m = MultiDOFCommand(); m.dof_names = LEFT_DELTO_JOINT_NAMES[: len(v)]
            m.values = [float(x) for x in v]; m.values_dot = [0.0] * len(v)
        else:
            m = Float64MultiArray(data=[float(x) for x in v])
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--arm-speed", type=float, default=8.0, help="deg/s")
    ap.add_argument("--hand-speed", type=float, default=0.4, help="rad/s")
    ap.add_argument("--hand-only", action="store_true", help="skip the arm entirely (arm powered off)")
    ap.add_argument("--settle", type=float, default=6.0, help="seconds to hold the init command so the pospid converges")
    ap.add_argument("--engage", action="store_true")
    args = ap.parse_args()

    arm_init = np.asarray(ARM_INIT_RAD[args.side]) * 180.0 / math.pi   # deg
    hand_init = np.asarray(HAND_INIT[args.side])
    spec = HAND[args.side]

    rclpy.init()
    hand = HandIO(spec)
    arm = None if args.hand_only else ArmClient(ARM_IP[args.side])
    try:
        tgt = "hand only" if args.hand_only else f"arm {ARM_IP[args.side]} + hand"
        print(f"[1/2] side={args.side} — connecting {tgt} ({spec['state']}) ...")
        if arm is not None:
            arm.connect(); aq = np.asarray(arm.wait()[:6])
            print(f"      arm  now(deg)={np.round(aq,1).tolist()}  init={np.round(arm_init,1).tolist()}  gap {np.abs(arm_init-aq).max():.1f}")
        hq = hand.wait()
        if hq is None:
            print(f"[ABORT] no {spec['state']} — is the {spec['type']} hand driver running?"); return
        print(f"      hand gap to init: {np.abs(hand_init-hq).max():.2f} rad")
        if not args.engage:
            print("[2/2] DRY-RUN — no motion. Re-run with --engage to home to init."); return
        print(f"[2/2] ENGAGE — homing to mean init ({tgt}) ...")
        hc = hq.copy(); h_s = args.hand_speed * SEND_DT
        ac = None if arm is None else aq.copy(); a_s = args.arm_speed * SEND_DT
        def arm_ok():  return arm is None or np.abs(arm_init - ac).max() <= 0.5
        # ramp the COMMAND toward init
        while not arm_ok() or np.abs(hand_init - hc).max() > 0.02:
            if arm is not None:
                ac = step_toward(ac, arm_init, a_s); arm.insert(ac)
            hc = step_toward(hc, hand_init, h_s); hand.publish(hc)
            time.sleep(SEND_DT)
        # HOLD init command and let the pospid converge — the hand is slow (P-only),
        # 0.2s is not enough; keep publishing init for --settle s while watching actual.
        print(f"      command at init; holding {args.settle:.1f}s to settle (pospid converges) ...")
        t0 = time.time()
        while time.time() - t0 < args.settle:
            if arm is not None: arm.insert(ac)
            hand.publish(hc)
            rclpy.spin_once(hand, timeout_sec=0.0)   # refresh hand.cur
            time.sleep(SEND_DT)
        act = hand.cur
        if act is not None:
            err = np.abs(hand_init - act[: len(hand_init)])
            print(f"      hand ACTUAL gap to init: max {err.max():.3f} rad @ j{int(err.argmax())} | mean {err.mean():.3f}")
            far = [int(i) for i in np.where(err > 0.1)[0]]
            print("      reached init within 0.1 rad on all joints." if not far
                  else f"      joints still >0.1 rad off: {far}  (P-only pospid steady-state error)")
        print("      done.")
    except KeyboardInterrupt:
        print("\n[INTERRUPT]")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if arm is not None:
            arm.stop(); arm.close()
        rclpy.shutdown(); print("stopped.")


if __name__ == "__main__":
    main()
