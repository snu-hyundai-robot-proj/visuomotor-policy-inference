#!/usr/bin/env python3
"""Minimal client for the visuomotor policy inference server.

Encodes two camera frames + a state vector, calls /predict, prints the action.
Run the server first (see README), then:

    python examples/client_example.py --url http://localhost:8000
"""

from __future__ import annotations

import argparse
import base64
import io

import numpy as np
import requests
from PIL import Image


def encode_image(arr: np.ndarray) -> str:
    """RGB uint8 (H, W, 3) -> base64 PNG."""
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--steps", type=int, default=3)
    args = ap.parse_args()

    info = requests.get(f"{args.url}/info", timeout=30).json()
    print("server info:", info)
    state_dim = info["state_dim"]

    requests.post(f"{args.url}/reset", timeout=30).raise_for_status()

    rng = np.random.default_rng(0)
    for t in range(args.steps):
        # Replace these synthetic frames/state with your robot's real data.
        front = rng.integers(0, 256, size=(480, 640, 3), dtype=np.uint8)
        wrist = rng.integers(0, 256, size=(480, 640, 3), dtype=np.uint8)
        state = rng.standard_normal(state_dim).astype(np.float32)

        payload = {
            "front_rgb": encode_image(front),
            "wrist_rgb": encode_image(wrist),
            "state": state.tolist(),
        }
        r = requests.post(f"{args.url}/predict", json=payload, timeout=60)
        r.raise_for_status()
        action = r.json()["action"]
        print(f"step {t}: action[{len(action)}] = {[round(a, 3) for a in action]}")


if __name__ == "__main__":
    main()
