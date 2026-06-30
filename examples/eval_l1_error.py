"""Offline L1 inference error against the training data, through the serving server.

Streams a training episode's recorded observations (front+wrist+state+gripper+ft) to the
running /predict server (open-loop / teacher-forced) and reports the L1 error between the
predicted actions and the recorded ground-truth actions.

    python examples/eval_l1_error.py --url http://localhost:8000 --episode 0
"""
import argparse
import base64
import io

import numpy as np
import requests
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset

DEFAULT_ROOT = "/home/ngseo/remove_hook/lerobot/data/lerobot/hyundai_uiwang_right"
ARM = slice(0, 6)
HAND_ACTIVE = slice(6, 12)   # RH56: arm 6 + 6 active hand dims; 12:26 are constant padding


def img_b64(t) -> str:
    a = t.detach().cpu().numpy() if hasattr(t, "detach") else np.asarray(t)
    if a.ndim == 3 and a.shape[0] in (1, 3):          # CHW -> HWC
        a = np.transpose(a, (1, 2, 0))
    if a.dtype != np.uint8:
        a = (np.clip(a, 0.0, 1.0) * 255).astype(np.uint8)
    if a.shape[-1] == 1:
        a = np.repeat(a, 3, axis=-1)
    buf = io.BytesIO()
    Image.fromarray(a).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--repo_id", default="local/hyundai_uiwang_right")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--max_frames", type=int, default=0, help="0 = whole episode")
    args = ap.parse_args()

    info = requests.get(f"{args.url}/info", timeout=30).json()
    print(f"server model: {info.get('model_id')} | device={info.get('device')} | "
          f"rtc={info.get('rtc')} | ruckig={info.get('ruckig')}")

    ds = LeRobotDataset(args.repo_id, root=args.root)
    a = int(ds.meta.episodes["dataset_from_index"][args.episode])
    b = int(ds.meta.episodes["dataset_to_index"][args.episode])
    if args.max_frames:
        b = min(b, a + args.max_frames)
    print(f"episode {args.episode}: frames {a}..{b} ({b - a})")

    requests.post(f"{args.url}/reset", timeout=30).raise_for_status()
    sess = requests.Session()
    gt, pred = [], []
    for n, i in enumerate(range(a, b)):
        fr = ds[i]
        payload = {
            "front_rgb": img_b64(fr["observation.images.front_rgb"]),
            "wrist_rgb": img_b64(fr["observation.images.wrist_rgb"]),
            "state": fr["observation.state"].tolist(),
            "gripper_sensor": fr["observation.gripper_sensor"].tolist(),
            "wrist_ft_sensor": fr["observation.wrist_ft_sensor"].tolist(),
        }
        r = sess.post(f"{args.url}/predict", json=payload, timeout=120)
        r.raise_for_status()
        pred.append(r.json()["action"])
        gt.append(fr["action"].tolist())
        if (n + 1) % 100 == 0:
            print(f"  {n + 1} frames...")

    gt = np.asarray(gt, dtype=np.float32)
    pred = np.asarray(pred, dtype=np.float32)
    keep = ~np.all(gt[:, ARM] == 0, axis=1)            # drop frame-0 zero-config artifact
    gt, pred = gt[keep], pred[keep]
    l1 = np.abs(pred - gt)

    print(f"\n=== L1 inference error ({len(gt)} frames) ===")
    print(f"  overall (26 dims) : {l1.mean():.5f}")
    print(f"  arm (0:6)         : {l1[:, ARM].mean():.5f}")
    print(f"  hand active (6:12): {l1[:, HAND_ACTIVE].mean():.5f}")
    print(f"  per-arm-joint MAE : {np.array2string(l1[:, ARM].mean(0), precision=4)}")
    print(f"  max |err|         : {l1.max():.4f}")


if __name__ == "__main__":
    main()
