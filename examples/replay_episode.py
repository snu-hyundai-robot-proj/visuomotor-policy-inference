#!/usr/bin/env python3
"""Replay a recorded sample episode against the inference server.

Feeds the real frames + states from `examples/sample_episodes/<side>/` to the
running server's /predict endpoint (instead of synthetic noise), and optionally
reports the error vs. the recorded action.

    # start the server first (docker compose up, or uvicorn), then:
    python examples/replay_episode.py --side left
    python examples/replay_episode.py --side right --url http://localhost:8000 --compare
"""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path

import numpy as np
import requests

SAMPLES = Path(__file__).resolve().parent / "sample_episodes"


def b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--steps", type=int, default=None, help="limit number of frames (default: whole episode)")
    ap.add_argument("--compare", action="store_true", help="report MAE vs the recorded action")
    args = ap.parse_args()

    root = SAMPLES / args.side
    meta = json.loads((root / "episode.json").read_text())
    frames = meta["frames"][: args.steps] if args.steps else meta["frames"]
    print(f"replaying {args.side} episode (source ep#{meta['source_episode_index']}, "
          f"{len(frames)}/{meta['num_frames']} frames @ {meta['fps']} Hz)")

    requests.post(f"{args.url}/reset", timeout=30).raise_for_status()

    errs = []
    for i, fr in enumerate(frames):
        payload = {
            "front_rgb": b64(root / "front_rgb" / f"{fr['frame']}.jpg"),
            "wrist_rgb": b64(root / "wrist_rgb" / f"{fr['frame']}.jpg"),
            "state": fr["state"],
        }
        r = requests.post(f"{args.url}/predict", json=payload, timeout=60)
        r.raise_for_status()
        action = np.asarray(r.json()["action"], dtype=np.float32)
        if args.compare:
            errs.append(float(np.abs(action - np.asarray(fr["action"], dtype=np.float32)).mean()))
        if i < 3 or i == len(frames) - 1:
            print(f"  frame {fr['frame']}: action[:6]={np.round(action[:6], 3).tolist()}")

    if args.compare and errs:
        print(f"mean |pred - recorded| over {len(errs)} frames: {np.mean(errs):.4f}")
    print("done.")


if __name__ == "__main__":
    main()
