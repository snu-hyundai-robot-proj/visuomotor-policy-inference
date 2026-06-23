# utils/motion.py
import json
import math
import os
import time
from typing import List, Tuple
from typing import Optional

def rad_to_deg(rad_list: List[float]) -> List[float]:
    return [r * 180.0 / math.pi for r in rad_list]

def generate_sine_trajectory(
    base_deg: List[float],
    *,
    cycle_sec: float = 1.0,
    amplitude_deg: float = 5.0,
    dt_sec: float = 0.02,
    total_sec: float = 1.0,
    active_joint_count: Optional[int] = 6
) -> List[List[float]]:
    if active_joint_count is None:
        active_joint_count = len(base_deg)

    omega = 2.0 * math.pi / cycle_sec
    steps = int(total_sec / dt_sec) + 1

    traj = []
    for k in range(steps):
        t = k * dt_sec
        point = []
        for i, base in enumerate(base_deg):
            # if i < active_joint_count:
            if i == 5:
                offset = amplitude_deg * math.sin(omega * t)
                point.append(base + offset)
            else:
                point.append(base)
        traj.append(point)

    return traj

def save_trajectory(
    points_deg: List[List[float]],
    dt_sec: float,
    *,
    base_dir: str = "data",
) -> str:
    os.makedirs(base_dir, exist_ok=True)
    ts = time.strftime("%m%d%H%M%S")
    path = os.path.join(base_dir, f"trajectory_{ts}.json")

    with open(path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "dt_sec": dt_sec,
                "points_deg": points_deg,
            },
            f,
            indent=2,
        )

    return os.path.abspath(path)

def load_trajectory(path: str) -> Tuple[float, List[List[float]]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    return data["dt_sec"], data["points_deg"]