#!/usr/bin/env python3
"""Open-loop L1 evaluation of the policy against a recorded sample episode.

Feeds each frame's real (front_rgb, wrist_rgb, state) to the running /predict server,
in order (policy reset once at the start so its temporal queue is fresh), and compares
the predicted action to the recorded ground-truth action with the **L1 error**
(mean absolute error). Reports overall / arm(0:6) / hand(6:26) / per-dimension, plus a
naive baseline (action = current state) for interpretability.

    # start the server first (uvicorn / docker), then:
    python scripts/eval_l1.py --side left --url http://localhost:8000 --out /tmp/l1_report.json
"""
from __future__ import annotations

import argparse
import base64
import json
import time
from pathlib import Path

import numpy as np
import requests

SAMPLES = Path(__file__).resolve().parents[1] / "examples" / "sample_episodes"


def b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--steps", type=int, default=None, help="limit frames (default: whole episode)")
    ap.add_argument("--out", default="/tmp/l1_report.json")
    args = ap.parse_args()

    root = SAMPLES / args.side
    meta = json.loads((root / "episode.json").read_text())
    frames = meta["frames"][: args.steps] if args.steps else meta["frames"]
    print(f"episode: {args.side} ep#{meta['source_episode_index']} | "
          f"{len(frames)}/{meta['num_frames']} frames @ {meta['fps']}Hz | "
          f"state_dim={meta['state_dim']} action_dim={meta['action_dim']}")

    sess = requests.Session()
    sess.post(f"{args.url}/reset", timeout=30).raise_for_status()

    preds, gts, states = [], [], []
    t0 = time.perf_counter()
    for i, fr in enumerate(frames):
        payload = {
            "front_rgb": b64(root / "front_rgb" / f"{fr['frame']}.jpg"),
            "wrist_rgb": b64(root / "wrist_rgb" / f"{fr['frame']}.jpg"),
            "state": fr["state"],
        }
        r = sess.post(f"{args.url}/predict", json=payload, timeout=60)
        r.raise_for_status()
        preds.append(np.asarray(r.json()["action"], dtype=np.float64))
        gts.append(np.asarray(fr["action"], dtype=np.float64))
        states.append(np.asarray(fr["state"], dtype=np.float64))
        if (i + 1) % 100 == 0:
            print(f"  ...{i+1}/{len(frames)}")
    wall = time.perf_counter() - t0

    pred = np.stack(preds)      # (N, 26)
    gt = np.stack(gts)
    state = np.stack(states)
    N, D = pred.shape

    abs_err = np.abs(pred - gt)                 # (N, 26)
    base_err = np.abs(state - gt)               # baseline: "don't move" (action == current state)

    arm = slice(0, 6)
    hand = slice(6, D)
    per_dim = abs_err.mean(axis=0)              # (26,)
    per_frame = abs_err.mean(axis=1)            # (N,)

    def m(x):
        return float(np.mean(x))

    report = {
        "side": args.side,
        "source_episode_index": meta["source_episode_index"],
        "frames": N,
        "fps": meta["fps"],
        "wall_sec": round(wall, 2),
        "l1_model": {
            "overall_mean": round(m(abs_err), 5),
            "arm_mean": round(m(abs_err[:, arm]), 5),
            "hand_mean": round(m(abs_err[:, hand]), 5),
            "total_sum": round(float(abs_err.sum()), 3),
        },
        "l1_baseline_action_eq_state": {
            "overall_mean": round(m(base_err), 5),
            "arm_mean": round(m(base_err[:, arm]), 5),
            "hand_mean": round(m(base_err[:, hand]), 5),
        },
        "per_dimension_mean_l1": [round(x, 5) for x in per_dim.tolist()],
        "worst_dims": sorted(
            [{"dim": int(i), "mean_l1": round(float(per_dim[i]), 5)} for i in range(D)],
            key=lambda d: -d["mean_l1"])[:5],
        "per_frame_mean_l1_first10": [round(x, 5) for x in per_frame[:10].tolist()],
        "per_frame_mean_l1_last10": [round(x, 5) for x in per_frame[-10:].tolist()],
    }
    Path(args.out).write_text(json.dumps(report, indent=2))

    lm, lb = report["l1_model"], report["l1_baseline_action_eq_state"]
    print("\n================ L1 (mean |pred - recorded action|) ================")
    print(f" frames evaluated : {N}  (wall {wall:.1f}s, {N/wall:.1f} fps)")
    print(f" MODEL    overall : {lm['overall_mean']:.4f}   arm {lm['arm_mean']:.4f}   hand {lm['hand_mean']:.4f}")
    print(f" BASELINE overall : {lb['overall_mean']:.4f}   arm {lb['arm_mean']:.4f}   hand {lb['hand_mean']:.4f}   (action==state)")
    imp = (1 - lm['overall_mean'] / lb['overall_mean']) * 100 if lb['overall_mean'] else 0
    print(f" model vs baseline: {imp:+.1f}%  (lower L1 = better)")
    print(f" worst dims       : {[ (d['dim'], d['mean_l1']) for d in report['worst_dims'] ]}")
    print(f" report saved     : {args.out}")
    print("====================================================================")


if __name__ == "__main__":
    main()
