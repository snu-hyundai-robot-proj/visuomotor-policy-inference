#!/usr/bin/env python3
"""Live camera -> policy inference sanity test.

Subscribes to the real camera topics published by the system_Teleop vision node
(RealSense wrist + Zivid front), sends a synchronized pair to the /predict server,
and prints the returned 26-d action with sanity stats. Verifies the end-to-end
"real cameras -> model inference -> action" path produces sane values.

NOTE: this is an INFERENCE sanity check, not robot control. The robot STATE is filled
with a constant (no robot needed); action magnitudes are what we check (finite, in a
sane radian range), not task correctness.

Config via env vars:
  FRONT_TOPIC  (default /system_right/zivid_rgb)   # Zivid scene/front
  WRIST_TOPIC  (default /system_right/d405_rgb)     # RealSense wrist
  VPI_URL      (default http://localhost:8000)
  N_SAMPLES    (default 5)        STATE_FILL (default 0.0)

Run (inside a ROS2 env on the same host/domain as the vision node):
  python3 examples/camera_infer_test.py
"""
from __future__ import annotations

import base64
import io
import os
import time

import numpy as np
import rclpy
import requests
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image

# SIDE is the single knob: left/right swaps both camera topics. Explicit
# FRONT_TOPIC/WRIST_TOPIC still override if set.
SIDE = os.environ.get("SIDE", "right")
FRONT_TOPIC = os.environ.get("FRONT_TOPIC", f"/system_{SIDE}/zivid_rgb")   # Zivid front
WRIST_TOPIC = os.environ.get("WRIST_TOPIC", f"/system_{SIDE}/d405_rgb")    # RealSense wrist
VPI_URL = os.environ.get("VPI_URL", "http://localhost:8000").rstrip("/")
N_SAMPLES = int(os.environ.get("N_SAMPLES", "5"))
STATE_FILL = float(os.environ.get("STATE_FILL", "0.0"))


def decode_image(msg: Image) -> np.ndarray:
    """sensor_msgs/Image (rgb8 or bgr8) -> RGB uint8 (H, W, 3), no cv_bridge needed."""
    h, w = msg.height, msg.width
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = msg.step or w * 3
    arr = arr.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding == "bgr8":
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def jpeg_b64(rgb: np.ndarray) -> str:
    buf = io.BytesIO()
    PILImage.fromarray(rgb).save(buf, format="JPEG", quality=90)
    return base64.b64encode(buf.getvalue()).decode("ascii")


class CameraInferTest(Node):
    def __init__(self):
        super().__init__("camera_infer_test")
        self.front = None
        self.wrist = None
        self.front_info = self.wrist_info = ""
        self.create_subscription(Image, FRONT_TOPIC, self._on_front, 1)
        self.create_subscription(Image, WRIST_TOPIC, self._on_wrist, 1)
        self.info = requests.get(f"{VPI_URL}/info", timeout=10).json()
        self.get_logger().info(f"server: {self.info}")
        requests.post(f"{VPI_URL}/reset", timeout=10)
        self.state_dim = int(self.info["state_dim"])
        self.n_done = 0
        self.errs = 0
        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"front={FRONT_TOPIC}  wrist={WRIST_TOPIC}  url={VPI_URL}  state=fill({STATE_FILL})x{self.state_dim}"
        )

    def _on_front(self, msg):
        self.front = decode_image(msg)
        self.front_info = f"{msg.width}x{msg.height} {msg.encoding}"

    def _on_wrist(self, msg):
        self.wrist = decode_image(msg)
        self.wrist_info = f"{msg.width}x{msg.height} {msg.encoding}"

    def _tick(self):
        if self.front is None or self.wrist is None:
            self.get_logger().warn(
                f"waiting for frames (front={self.front is not None}, wrist={self.wrist is not None})",
                throttle_duration_sec=2.0,
            )
            return
        front, wrist = self.front, self.wrist
        state = np.full(self.state_dim, STATE_FILL, dtype=np.float32)
        payload = {"front_rgb": jpeg_b64(front), "wrist_rgb": jpeg_b64(wrist), "state": state.tolist()}
        try:
            t0 = time.perf_counter()
            r = requests.post(f"{VPI_URL}/predict", json=payload, timeout=30)
            dt = (time.perf_counter() - t0) * 1000
            r.raise_for_status()
            a = np.asarray(r.json()["action"], dtype=np.float64)
        except Exception as e:
            self.errs += 1
            self.get_logger().error(f"predict failed: {e}")
            return

        finite = bool(np.all(np.isfinite(a)))
        arm, hand = a[:6], a[6:]
        self.get_logger().info(
            f"[{self.n_done}] front[{self.front_info}] wrist[{self.wrist_info}] -> "
            f"action[{len(a)}] {dt:.0f}ms | finite={finite} "
            f"range=[{a.min():.3f},{a.max():.3f}] |a|mean={np.abs(a).mean():.3f}"
        )
        self.get_logger().info(f"     arm(6)={np.round(arm,3).tolist()}")
        self.get_logger().info(f"     hand[:6]={np.round(hand[:6],3).tolist()}")
        self.n_done += 1
        if self.n_done >= N_SAMPLES:
            ok = self.errs == 0
            self.get_logger().info(
                f"==== DONE: {self.n_done} inferences, {self.errs} errors -> "
                f"{'ACTIONS OK' if ok else 'HAD ERRORS'} ===="
            )
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraInferTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
