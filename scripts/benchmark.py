#!/usr/bin/env python3
"""Latency + bandwidth benchmark for the Hyundai Uiwang FlowMatch Diffusion Policy.

Measures the NATIVE (in-process, GPU) inference path so the numbers reflect the
model compute itself, separated into:
  * heavy steps  — the diffusion re-plan (runs once per n_action_steps)
  * cheap steps  — action-queue pop (the other ticks)
and the amortized per-control-step cost (what actually gates your control rate).

Also reports payload / bandwidth: how many bytes go in (2 RGB frames + state) and
out (action) per tick and per second at the model's 30 Hz target, for raw, PNG and
JPEG transports (the HTTP server base64-encodes PNG).

Run inside the `vpi` conda env:
    python scripts/benchmark.py --device cuda --steps 240 --hw 480 640
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import statistics
import time

import numpy as np
import torch
from PIL import Image

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

FRONT_KEY = "observation.images.front_rgb"
WRIST_KEY = "observation.images.wrist_rgb"
STATE_KEY = "observation.state"
MODEL_ID = "Ngseo/hyundai-uiwang-left-flowmatch"


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * p / 100.0
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def summarize(xs):
    return {
        "n": len(xs),
        "mean_ms": round(statistics.mean(xs), 3) if xs else None,
        "median_ms": round(statistics.median(xs), 3) if xs else None,
        "p95_ms": round(pct(xs, 95), 3) if xs else None,
        "p99_ms": round(pct(xs, 99), 3) if xs else None,
        "max_ms": round(max(xs), 3) if xs else None,
        "min_ms": round(min(xs), 3) if xs else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-id", default=MODEL_ID)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--steps", type=int, default=240)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--hw", type=int, nargs=2, default=[480, 640], help="camera H W for bandwidth")
    ap.add_argument("--jpeg-quality", type=int, default=90)
    args = ap.parse_args()

    device = args.device if (args.device != "cuda" or torch.cuda.is_available()) else "cpu"
    gpu_name = torch.cuda.get_device_name(0) if device == "cuda" else "CPU"

    policy = DiffusionPolicy.from_pretrained(args.model_id)
    policy.config.device = device
    policy.to(device).eval()
    policy.reset()
    preprocess, postprocess = make_pre_post_processors(
        policy.config, args.model_id, preprocessor_overrides={"device_processor": {"device": device}}
    )

    n_action_steps = int(getattr(policy.config, "n_action_steps", 8))
    H, W = args.hw
    rng = np.random.default_rng(0)

    def make_obs():
        front = rng.integers(0, 256, size=(H, W, 3), dtype=np.uint8)
        wrist = rng.integers(0, 256, size=(H, W, 3), dtype=np.uint8)
        state = rng.standard_normal(26).astype(np.float32)
        return front, wrist, state

    @torch.no_grad()
    def step(front, wrist, state):
        obs = {
            FRONT_KEY: torch.from_numpy(front).float().div(255).permute(2, 0, 1)[None].to(device),
            WRIST_KEY: torch.from_numpy(wrist).float().div(255).permute(2, 0, 1)[None].to(device),
            STATE_KEY: torch.from_numpy(state).float()[None].to(device),
            "task": "", "robot_type": "",
        }
        a = postprocess(policy.select_action(preprocess(obs)))
        return a.squeeze(0).float().cpu().numpy()

    # A re-plan (diffusion forward) happens exactly when the internal action queue
    # is empty at the start of select_action. Inspect it to label each step precisely.
    def queue_empty():
        try:
            return len(policy._queues["action"]) == 0
        except Exception:
            return True

    # warmup
    for _ in range(args.warmup):
        step(*make_obs())
    if device == "cuda":
        torch.cuda.synchronize()

    # timed loop
    per_step = []
    is_replan = []
    for _ in range(args.steps):
        f, w, s = make_obs()
        replan = queue_empty()
        t0 = time.perf_counter()
        step(f, w, s)
        if device == "cuda":
            torch.cuda.synchronize()
        per_step.append((time.perf_counter() - t0) * 1000.0)
        is_replan.append(replan)

    # classify by the actual queue state, not a fixed period
    heavy = [t for t, r in zip(per_step, is_replan) if r]
    cheap = [t for t, r in zip(per_step, is_replan) if not r]

    amortized_ms = statistics.mean(per_step)
    eff_rate = 1000.0 / amortized_ms

    # ---- bandwidth ----
    f, w, _ = make_obs()
    raw_per_img = H * W * 3
    png = len(_encode(f, "PNG"))
    jpg = len(_encode(f, "JPEG", quality=args.jpeg_quality))
    b64 = lambda n: (n + 2) // 3 * 4  # base64 expands ~4/3
    state_bytes = 26 * 4
    action_bytes = 26 * 4

    def per_sec(b, hz=30.0):
        return b * hz

    transports = {}
    for name, img_bytes in [("raw", raw_per_img), ("png", png), ("jpeg", jpg)]:
        # HTTP server sends base64 of the encoded image; raw shown for reference only
        in_imgs = 2 * img_bytes
        in_imgs_b64 = 2 * b64(img_bytes) if name != "raw" else 2 * img_bytes
        in_total = in_imgs_b64 + state_bytes
        transports[name] = {
            "per_img_bytes": img_bytes,
            "per_img_b64_bytes": b64(img_bytes) if name != "raw" else img_bytes,
            "in_per_tick_bytes": in_total,
            "out_per_tick_bytes": action_bytes,
            "in_MBps_30hz": round(per_sec(in_total) / 1e6, 3),
            "in_Mbps_30hz": round(per_sec(in_total) * 8 / 1e6, 2),
            "out_KBps_30hz": round(per_sec(action_bytes) / 1e3, 3),
        }

    report = {
        "gpu": gpu_name,
        "device": device,
        "torch": torch.__version__,
        "model_id": args.model_id,
        "n_action_steps_chunk": n_action_steps,
        "n_obs_steps": int(getattr(policy.config, "n_obs_steps", "?")),
        "horizon": int(getattr(policy.config, "horizon", "?")),
        "scheduler": getattr(policy.config, "noise_scheduler_type", "?"),
        "num_inference_steps": getattr(policy.config, "num_inference_steps", "?"),
        "camera_hw": [H, W],
        "latency": {
            "heavy_replan_step": summarize(heavy),
            "cheap_queue_step": summarize(cheap),
            "amortized_per_control_step_ms": round(amortized_ms, 3),
            "effective_max_control_hz": round(eff_rate, 1),
            "meets_30hz_amortized": amortized_ms <= 1000.0 / 30.0,
            "heavy_meets_30hz": (summarize(heavy)["p95_ms"] or 1e9) <= 1000.0 / 30.0,
        },
        "bandwidth": transports,
    }
    print(json.dumps(report, indent=2))


def _encode(arr, fmt, **kw):
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format=fmt, **kw)
    return buf.getvalue()


if __name__ == "__main__":
    main()
