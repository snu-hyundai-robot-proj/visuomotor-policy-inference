#!/usr/bin/env python3
"""Live ROS cameras -> our model -> action. (inference only, no robot motion)

Builds the model observation from LIVE data:
  front_rgb = /system_<side>/zivid_rgb   (Zivid, sensor_msgs/Image rgb8)
  wrist_rgb = /system_<side>/d405_rgb    (RealSense, rgb8)
  state[26] = arm joints(6, via HDR35 OpenStream TCP) + hand joints(20, /dg5f_left/joint_states)
-> POST /predict -> action[26], printed with arm(6)/hand(20) split + sanity.

Run inside the teleop container (rclpy + requests + reaches arm/predict):
  python3 live_infer.py --side left --n 10
"""
from __future__ import annotations

import argparse
import base64
import io
import sys
import threading
import time

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Image
from PIL import Image as PILImage

# /dg5f_*/joint_states is published RELIABLE + TRANSIENT_LOCAL (latched) and in a
# SCRAMBLED name order. Match the QoS, and reindex by name to the canonical model
# order idx = 4*(finger-1)+(joint-1)  (lj_dg_1_1, 1_2, ... 5_4) — same order the
# dataset recorder (system_left.cpp gripperCallback) used.
JS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)


def hand_state_canonical(msg: JointState, ndof: int = 20) -> np.ndarray:
    v = np.zeros(ndof, dtype=np.float64)
    for name, pos in zip(msg.name, msg.position):
        t = name.split("_")            # lj_dg_<finger>_<joint>
        if len(t) >= 4:
            idx = 4 * (int(t[2]) - 1) + (int(t[3]) - 1)
            if 0 <= idx < ndof:
                v[idx] = pos
    return v


def inspire_hand20(msg: JointState) -> np.ndarray:
    """RIGHT: /inspire/joint_states position[0:6] = received_angle (deg-ish) -> rad, pad to 20.
    The model's right state is arm6 + inspire6(rad) + 14 zeros (dataset pad)."""
    v = np.zeros(20, dtype=np.float64)
    for i in range(min(6, len(msg.position))):
        v[i] = float(msg.position[i]) * np.pi / 180.0
    return v

HDR_UTILS = "/workspace/src/Robot_/src/hdr_stream"
sys.path.insert(0, HDR_UTILS)
from hdr_stream.utils.net import NetClient          # noqa: E402
from hdr_stream.utils.api import OpenStreamAPI       # noqa: E402
from hdr_stream.utils.parser import NDJSONParser     # noqa: E402
from hdr_stream.utils.dispatcher import Dispatcher   # noqa: E402

ARM_IP = {"left": "192.168.4.152", "right": "192.168.4.151"}
GET_JOINTS = "/project/robot/joints/joint_states"


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


class ArmReader:
    """read-only OpenStream client: current arm joints in radians."""
    def __init__(self, ip):
        self.net = NetClient(ip, 49000); self.api = OpenStreamAPI(self.net)
        self.parser = NDJSONParser(); self.disp = Dispatcher()
        self.ok = threading.Event(); self.q_deg = None
        self.disp.on_type["handshake_ack"] = lambda m: self.ok.set() if m.get("ok") else None
        self.disp.on_type["data"] = self._d
        self.disp.on_error = lambda e: None

    def _d(self, m):
        r = m.get("result")
        if isinstance(r, dict) and r.get("_type") == "JObject" and r.get("position"):
            self.q_deg = [float(x) for x in r["position"]]

    def connect(self, timeout=5.0):
        self.net.connect(); self.net.start_recv_loop(lambda b: self.parser.feed(b, self.disp.dispatch))
        self.api.handshake(major=1)
        if not self.ok.wait(timeout): raise RuntimeError("arm handshake timeout")
        self.api.monitor(url=GET_JOINTS, period_ms=20, args={})

    def state_rad(self):
        return None if self.q_deg is None else np.asarray(self.q_deg[:6]) * np.pi / 180.0

    def close(self):
        try: self.net.close()
        except Exception: pass


class LiveInfer(Node):
    def __init__(self, side, url):
        super().__init__("live_infer")
        self.url = url.rstrip("/")
        self.front = self.wrist = self.hand = None
        self.finfo = self.winfo = "?"
        self.create_subscription(Image, f"/system_{side}/zivid_rgb", self._f, 1)
        self.create_subscription(Image, f"/system_{side}/d405_rgb", self._w, 1)
        if side == "right":   # Inspire hand: /inspire/joint_states streams (default QoS)
            self.create_subscription(JointState, "/inspire/joint_states",
                                     lambda m: setattr(self, "hand", inspire_hand20(m)), 1)
        else:                 # DG5F: latched joint_states, canonical reorder
            self.create_subscription(JointState, f"/dg5f_{side}/joint_states",
                                     lambda m: setattr(self, "hand", hand_state_canonical(m)), JS_QOS)
        self.info = requests.get(f"{self.url}/info", timeout=10).json()
        self.get_logger().info(f"server: {self.info['model_id']} state_dim={self.info['state_dim']}")
        requests.post(f"{self.url}/reset", timeout=10)

    def _f(self, m): self.front = decode_rgb(m); self.finfo = f"{m.width}x{m.height} {m.encoding}"
    def _w(self, m): self.wrist = decode_rgb(m); self.winfo = f"{m.width}x{m.height} {m.encoding}"

    def predict(self, arm6):
        state = np.concatenate([arm6, self.hand]).astype(np.float32)  # [arm6 + hand20] = 26
        payload = {"front_rgb": jpeg_b64(self.front), "wrist_rgb": jpeg_b64(self.wrist),
                   "state": state.tolist()}
        t = time.perf_counter()
        a = np.asarray(requests.post(f"{self.url}/predict", json=payload, timeout=30).json()["action"])
        return a, (time.perf_counter() - t) * 1000, state


# mean arm-6 init pose (rad), used for the state when the arm is powered off (--no-arm)
ARM_INIT_RAD = {
    "left":  [-1.734299, 1.679489, -0.113224, -0.657517, -1.625038, -0.202079],
    "right": [1.737826, 1.765172, -0.192581, 0.688188, -1.419986, 0.320249],
}
# mean hand init pose (canonical order lj_dg_1_1..5_4), used with --no-hand so the
# state is the in-distribution init and only the IMAGES are live.
HAND_INIT = {
    "left":  [-0.174693, 0.088572, -0.171882, -0.064431, -0.344696, 0.039543, 0.383013, 0.300303,
              -0.401026, 0.188575, 0.13543, 0.108224, -0.372288, 0.453359, 0.029937, 0.037518,
              0.000573, -0.427912, 0.54842, 0.033721],
    "right": [2.89855, 2.878596, 2.811947, 2.807127, 2.324232, 2.490377,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="left")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--no-arm", action="store_true",
                    help="arm powered off: use the fixed init pose for the arm-6 state")
    ap.add_argument("--no-hand", action="store_true",
                    help="don't read the live hand: use the fixed hand init for the hand-20 state")
    args = ap.parse_args()

    rclpy.init()
    node = LiveInfer(args.side, args.url)
    if args.no_hand:
        node.hand = np.asarray(HAND_INIT[args.side])
        node.get_logger().info("--no-hand: using fixed init hand state (hand not read)")
    arm = None
    arm_fixed = np.asarray(ARM_INIT_RAD[args.side])
    if not args.no_arm:
        arm = ArmReader(ARM_IP[args.side]); arm.connect()
    else:
        node.get_logger().info("--no-arm: using fixed init arm-6 state (arm not read)")
    try:
        done = 0
        while rclpy.ok() and done < args.n:
            rclpy.spin_once(node, timeout_sec=0.2)
            a6 = arm_fixed if arm is None else arm.state_rad()
            if node.front is None or node.wrist is None or node.hand is None or a6 is None:
                node.get_logger().warn(
                    f"waiting (front={node.front is not None} wrist={node.wrist is not None} "
                    f"hand={node.hand is not None} arm={a6 is not None})", throttle_duration_sec=2.0)
                time.sleep(0.2); continue
            act, ms, st = node.predict(a6)
            finite = bool(np.all(np.isfinite(act)))
            print(f"[{done}] LIVE front[{node.finfo}] wrist[{node.winfo}] {ms:.0f}ms | "
                  f"finite={finite} action[{len(act)}] range=[{act.min():.2f},{act.max():.2f}]")
            print(f"    state arm(6)={np.round(st[:6],2).tolist()}")
            print(f"    action arm(6)={np.round(act[:6],2).tolist()}  hand[:6]={np.round(act[6:12],2).tolist()}")
            done += 1
            time.sleep(0.4)
        print(f"==== DONE {done} live inferences ====")
    finally:
        if arm is not None:
            arm.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
