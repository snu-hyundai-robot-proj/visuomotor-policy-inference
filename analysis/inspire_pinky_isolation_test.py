#!/usr/bin/env python3
"""Safe Inspire RH56 right-hand channel isolation test (no arm/vision/inference)."""
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# From inspire_driver.py retarget_fingers + inspire_comm.py SRBL_INSPIRE_FINGER_LIST order
CODE_LABELS = [
    "pinky (finger_little)",
    "ring (finger_ring)",
    "middle (finger_middle)",
    "index (finger_index)",
    "thumb flex (finger_thumb_bending)",
    "thumb rotation (finger_thumb_rotation)",
]


def tj_to_norm(tj: list[float]) -> list[float]:
    out = []
    for i in range(4):
        out.append((tj[i] - 750.0) / 1100.0)
    out.append((tj[4] - 1100.0) / 400.0)
    out.append((1900.0 - tj[5]) / 950.0)
    return [max(0.0, min(1.0, x)) for x in out]


def norm_to_tj(n: list[float]) -> list[float]:
    return [
        n[0] * 1100 + 750,
        n[1] * 1100 + 750,
        n[2] * 1100 + 750,
        n[3] * 1100 + 750,
        n[4] * 400 + 1100,
        -n[5] * 950 + 1900,
    ]


class InspireProbe(Node):
    def __init__(self):
        super().__init__("inspire_pinky_probe")
        self.latest: JointState | None = None
        self.pub = self.create_publisher(Float64MultiArray, "/inspire/right/target", 1)
        self.create_subscription(JointState, "/inspire/joint_states", self._cb, 1)

    def _cb(self, msg: JointState):
        self.latest = msg

    def wait_js(self, timeout=5.0):
        t0 = time.time()
        while self.latest is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError("no /inspire/joint_states")
        return self.latest

    def publish(self, norm6: list[float], hold=2.0):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in norm6]
        self.pub.publish(msg)
        time.sleep(hold)
        rclpy.spin_once(self, timeout_sec=0.05)
        return self.snapshot()

    def snapshot(self):
        js = self.latest
        names = list(js.name)
        pos = [float(x) for x in js.position]
        actual = {names[i]: pos[i] for i in range(min(6, len(names)))}
        target = {names[i]: pos[i] for i in range(6, min(12, len(names)))}
        return {
            "names": names,
            "position": pos,
            "actual_by_name": actual,
            "target_by_name": target,
            "actual_j": pos[:6],
            "target_tj": pos[6:12] if len(pos) >= 12 else [],
        }


def delta(a: list[float], b: list[float]) -> list[float]:
    return [round(b[i] - a[i], 3) for i in range(min(len(a), len(b)))]


def main():
    rclpy.init()
    node = InspireProbe()
    out = {"code_mapping_labels": CODE_LABELS, "steps": []}

    try:
        base_js = node.wait_js()
        base_tj = node.snapshot()["target_tj"]
        if len(base_tj) < 6:
            base_tj = norm_to_tj([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        base_norm = tj_to_norm(base_tj)
        safe_open = [0.55, 0.55, 0.55, 0.55, 0.55, 0.55]

        # settle to current-derived neutral first
        s0 = node.publish(base_norm, hold=2.0)
        out["steps"].append({"phase": "baseline_current", "target_norm": base_norm, "state": s0})

        pinky_norm = base_norm.copy()
        pinky_bump = min(1.0, base_norm[0] + 0.15)
        pinky_norm[0] = pinky_bump

        s1 = node.publish(pinky_norm, hold=2.5)
        out["steps"].append(
            {
                "phase": "pinky_channel_only_plus_0p15",
                "changed_index": 0,
                "target_norm": pinky_norm,
                "state": s1,
                "actual_delta_j": delta(s0["actual_j"], s1["actual_j"]),
                "target_delta_tj": delta(s0["target_tj"], s1["target_tj"]),
            }
        )

        s2 = node.publish(base_norm, hold=2.5)
        out["steps"].append(
            {
                "phase": "pinky_restore_baseline",
                "target_norm": base_norm,
                "state": s2,
                "actual_delta_j": delta(s1["actual_j"], s2["actual_j"]),
                "target_delta_tj": delta(s1["target_tj"], s2["target_tj"]),
            }
        )

        # per-channel mapping probe: bump each index +0.10 from baseline
        mapping = []
        for i in range(6):
            n = base_norm.copy()
            n[i] = min(1.0, base_norm[i] + 0.10)
            sb = node.snapshot()
            si = node.publish(n, hold=2.0)
            node.publish(base_norm, hold=1.5)
            mapping.append(
                {
                    "target_index": i,
                    "code_label": CODE_LABELS[i],
                    "target_norm": n,
                    "actual_delta_j": delta(sb["actual_j"], si["actual_j"]),
                    "target_delta_tj": delta(sb["target_tj"], si["target_tj"]),
                    "dominant_actual_index": int(
                        max(range(6), key=lambda k: abs(delta(sb["actual_j"], si["actual_j"])[k]))
                    )
                    if si["actual_j"]
                    else None,
                }
            )
        out["channel_mapping_probe"] = mapping

        # thumb rotation small test
        tr = base_norm.copy()
        tr[5] = max(0.0, min(1.0, base_norm[5] - 0.15))
        st0 = node.snapshot()
        st1 = node.publish(tr, hold=2.5)
        node.publish(base_norm, hold=1.5)
        out["thumb_rotation_probe"] = {
            "target_norm": tr,
            "actual_delta_j": delta(st0["actual_j"], st1["actual_j"]),
            "target_delta_tj": delta(st0["target_tj"], st1["target_tj"]),
        }

        # safe restore
        sf = node.publish(safe_open, hold=2.5)
        out["final_safe_open"] = {"target_norm": safe_open, "state": sf}

        print(json.dumps(out, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
