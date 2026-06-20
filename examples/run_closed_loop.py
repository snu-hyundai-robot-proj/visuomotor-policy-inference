#!/usr/bin/env python3
"""Continuous closed-loop runner: stream frames in real time -> /predict -> robot command.

This is the *live* counterpart of the offline L1 eval. It paces frames at the model's
control rate (so "images arrive continuously"), calls the policy each tick, splits the
action into arm(0:6)+hand(6:26), and emits the EXACT command that would go to the robot
(arm rad->deg for /robot/joint_target_deg, hand rad for the dg5f pospid reference).

Sinks:
  --sink dryrun  : print/record the commands; DOES NOT move a robot (default, safe, no ROS)
  --sink ros     : not implemented here on purpose — the real ROS robot sink is the
                   `vpi_robot_client` node (ros2_robot_client), which subscribes to live
                   camera topics + state and publishes these same commands. Use that on
                   the actual robot. This script is for verifying the loop without hardware.

Image source:
  --source replay : stream a recorded sample episode in real time (default)
                    (a real deployment's source is the live camera topics, handled by
                     vpi_robot_client)

    # start the server, then:
    python examples/run_closed_loop.py --side left --fps 30
    python examples/run_closed_loop.py --side left --steps 90 --record /tmp/cmds.jsonl
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

SAMPLES = Path(__file__).resolve().parent / "sample_episodes"
ARM, HAND = slice(0, 6), slice(6, 26)


def b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--fps", type=float, default=None, help="control rate (default: episode fps)")
    ap.add_argument("--steps", type=int, default=None)
    ap.add_argument("--source", choices=["replay"], default="replay")
    ap.add_argument("--sink", choices=["dryrun"], default="dryrun")
    ap.add_argument("--record", default="", help="optional .jsonl path to log commands")
    ap.add_argument("--print-every", type=int, default=30)
    args = ap.parse_args()

    root = SAMPLES / args.side
    meta = json.loads((root / "episode.json").read_text())
    frames = meta["frames"][: args.steps] if args.steps else meta["frames"]
    fps = args.fps or meta["fps"]
    period = 1.0 / fps
    print(f"closed-loop {args.side}: {len(frames)} frames @ {fps}Hz | sink={args.sink} "
          f"(dryrun = NOT moving a robot)")

    sess = requests.Session()
    sess.post(f"{args.url}/reset", timeout=30).raise_for_status()

    rec = open(args.record, "w") if args.record else None
    lat, late = [], 0
    next_t = time.perf_counter()
    t_start = next_t
    for i, fr in enumerate(frames):
        # --- pace to the control rate: this is the "continuous image stream" ---
        next_t += period
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            late += 1
            next_t = time.perf_counter()

        # --- in a real run these frames come from the live cameras ---
        payload = {
            "front_rgb": b64(root / "front_rgb" / f"{fr['frame']}.jpg"),
            "wrist_rgb": b64(root / "wrist_rgb" / f"{fr['frame']}.jpg"),
            "state": fr["state"],
        }
        t0 = time.perf_counter()
        r = sess.post(f"{args.url}/predict", json=payload, timeout=30)
        r.raise_for_status()
        lat.append((time.perf_counter() - t0) * 1000)
        action = np.asarray(r.json()["action"], dtype=np.float64)

        # --- build the exact robot commands (what the sink would send) ---
        arm_deg = (action[ARM] * 180.0 / math.pi).tolist()        # /robot/joint_target_deg
        hand_rad = action[HAND].tolist()                          # dg5f pospid reference (rad)

        if rec:
            rec.write(json.dumps({"frame": fr["frame"], "arm_deg": [round(x, 4) for x in arm_deg],
                                  "hand_rad": [round(x, 4) for x in hand_rad]}) + "\n")
        if i % args.print_every == 0 or i == len(frames) - 1:
            print(f"  [{i:3d}] -> ROBOT  arm_deg={np.round(arm_deg, 2).tolist()}  "
                  f"hand_rad[:4]={np.round(hand_rad[:4], 3).tolist()}")

    if rec:
        rec.close()
    wall = time.perf_counter() - t_start
    lat = np.asarray(lat)
    print(f"\ndone: {len(frames)} ticks in {wall:.1f}s -> {len(frames)/wall:.1f} Hz achieved "
          f"(target {fps}Hz, {late} late ticks)")
    print(f"  /predict latency: mean {lat.mean():.1f}ms p95 {np.percentile(lat,95):.1f}ms")
    if args.record:
        print(f"  commands logged: {args.record}")
    print("NOTE: dry-run only. For the real robot use the vpi_robot_client ROS2 node "
          "(it sends these same arm_deg / hand_rad commands to the drivers).")


if __name__ == "__main__":
    main()
