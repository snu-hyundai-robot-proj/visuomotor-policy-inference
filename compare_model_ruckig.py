"""
Compare the trained right-hand Diffusion Policy output against the training data,
with and without Ruckig (jerk-limited) post-processing.

For one episode of the training dataset:
  1. feed the recorded observations into the policy (open-loop / teacher-forced)
     -> model action sequence  (26-d, 30 Hz)
  2. apply Ruckig to the model output (arm 6 + active hand 6) -> jerk-limited
  3. compare GT (training action) vs model-raw vs model+ruckig:
       - per-joint time series
       - EEF (tool0) 3D trajectory via FK
       - MSE-to-GT and peak velocity/accel/jerk

Run:  python compare_model_ruckig.py --episode 0 --device cuda
"""
import argparse, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from viz_eef_trajectory import parse_joints, chain_to, fk_positions, ARM_JOINTS, EEF_LINK, BASE_LINK, URDF_XACRO

CKPT = ("/home/ngseo/remove_hook/lerobot/outputs/train/uiwang_right_flowmatch_full/"
        "checkpoints/200000/pretrained_model")
DS_REPO = "local/hyundai_uiwang_right"
DS_ROOT = "/home/ngseo/remove_hook/lerobot/data/lerobot/hyundai_uiwang_right"
FPS = 30

# kinematic limits — arm (URDF vel; cuRobo acc/jerk) + active hand (RH56, reasonable)
ARM_V = np.array([3.141, 3.141, 3.316, 5.410, 5.410, 7.330]); ARM_A = np.full(6, 12.0); ARM_J = np.full(6, 500.0)
HAND_V = np.full(6, 3.0); HAND_A = np.full(6, 12.0); HAND_J = np.full(6, 500.0)
ACTIVE = list(range(6)) + list(range(6, 12))          # arm 0-5 + hand 6-11 (rest are const padding)
VMAX = np.concatenate([ARM_V, HAND_V]); AMAX = np.concatenate([ARM_A, HAND_A]); JMAX = np.concatenate([ARM_J, HAND_J])
CTRL_DT = 0.01  # 100 Hz ruckig control, then resampled to the 30 Hz frame grid


def ruckig_smooth(targets, policy_dt, vmax, amax, jmax):
    """Stream `targets` (T, D) through Ruckig @100Hz; return executed traj resampled to T rows."""
    from ruckig import Ruckig, InputParameter, OutputParameter, Result
    D = targets.shape[1]
    otg = Ruckig(D, CTRL_DT)
    inp, out = InputParameter(D), OutputParameter(D)
    inp.current_position = targets[0].tolist()
    inp.current_velocity = [0.0] * D; inp.current_acceleration = [0.0] * D
    inp.max_velocity = vmax.tolist(); inp.max_acceleration = amax.tolist(); inp.max_jerk = jmax.tolist()
    tv = np.zeros_like(targets); tv[1:-1] = (targets[2:] - targets[:-2]) / (2 * policy_dt)
    tv = np.clip(tv, -vmax, vmax)
    ticks = max(1, round(policy_dt / CTRL_DT))
    rows = []  # one executed sample per policy step (end of its window) for 1:1 GT compare
    P = []
    for i in range(len(targets)):
        inp.target_position = targets[i].tolist()
        inp.target_velocity = (tv[i] if i < len(targets) - 1 else np.zeros(D)).tolist()
        inp.target_acceleration = [0.0] * D
        for _ in range(ticks):
            otg.update(inp, out); P.append(list(out.new_position)); out.pass_to_input(inp)
        rows.append(P[-1])
    return np.array(rows), np.array(P), ticks


def vaj(x, dt):
    v = np.gradient(x, dt, axis=0); a = np.gradient(v, dt, axis=0); j = np.gradient(a, dt, axis=0)
    return v, a, j


def run_policy_on_episode(episode, device, cache=True):
    cache_path = f".cache_ep{episode}_gt_pred.npz"
    if cache and os.path.exists(cache_path):
        z = np.load(cache_path); print(f"[cache] loaded {cache_path}")
        return z["gt"], z["pred"]
    import torch
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    device = device if (device != "cuda" or torch.cuda.is_available()) else "cpu"
    print(f"[load] policy from {CKPT}")
    policy = DiffusionPolicy.from_pretrained(CKPT)
    policy.config.device = device; policy.to(device); policy.eval(); policy.reset()
    preprocess, postprocess = make_pre_post_processors(
        policy.config, CKPT, preprocessor_overrides={"device_processor": {"device": device}})

    in_keys = [k for k in policy.config.input_features]      # exactly what THIS model needs
    print(f"[model] input_features = {in_keys}")
    print(f"[data]  loading {DS_REPO} @ {DS_ROOT}")
    ds = LeRobotDataset(DS_REPO, root=DS_ROOT)
    ep_from = int(ds.meta.episodes["dataset_from_index"][episode])   # lerobot v0.5.1 (v3.0 dataset) API
    ep_to = int(ds.meta.episodes["dataset_to_index"][episode])
    print(f"[data]  episode {episode}: frames {ep_from}..{ep_to} ({ep_to-ep_from} steps)")

    gt, pred = [], []
    with torch.no_grad():
        for idx in range(ep_from, ep_to):
            fr = ds[idx]
            obs = {}
            for k in in_keys:
                v = fr[k]
                if not torch.is_tensor(v):
                    v = torch.as_tensor(np.asarray(v))
                obs[k] = v.unsqueeze(0).to(device)
            obs["task"] = ""; obs["robot_type"] = ""
            a = policy.select_action(preprocess(obs))
            a = postprocess(a)
            pred.append(a.squeeze(0).float().cpu().numpy())
            gt.append(fr["action"].float().cpu().numpy())
    gt, pred = np.asarray(gt), np.asarray(pred)
    if cache:
        np.savez(cache_path, gt=gt, pred=pred); print(f"[cache] saved {cache_path}")
    return gt, pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    args = ap.parse_args()
    dt = 1.0 / FPS

    gt, pred = run_policy_on_episode(args.episode, args.device)
    print(f"[done] GT {gt.shape}  model {pred.shape}")
    # drop leading all-zero arm frames (frame-0 pre-start artifact) consistently
    keep = ~np.all(gt[:, 0:6] == 0, axis=1)
    if (~keep).sum():
        print(f"[clean] dropped {(~keep).sum()} zero-arm frame(s)")
    gt, pred = gt[keep], pred[keep]

    # ruckig on active dims of the MODEL output
    pred_active = pred[:, ACTIVE]
    rk_rows, rk_full, ticks = ruckig_smooth(pred_active, dt, VMAX, AMAX, JMAX)
    pred_rk = pred.copy(); pred_rk[:, ACTIVE] = rk_rows   # padding dims untouched

    # ---- metrics ----
    def mse(a, b): return float(np.mean((a - b) ** 2))
    print("\n=== action MSE vs training data (GT) ===")
    print(f"  model raw    : {mse(pred[:,ACTIVE], gt[:,ACTIVE]):.5f}")
    print(f"  model+ruckig : {mse(pred_rk[:,ACTIVE], gt[:,ACTIVE]):.5f}")
    # peak jerk on the 100Hz executed stream
    _, _, gj = vaj(gt[:, ACTIVE], dt)
    _, _, pj = vaj(pred[:, ACTIVE], dt)
    _, _, rj = vaj(rk_full, CTRL_DT)
    print("\n=== peak |jerk| (rad/s^3), active dims ===")
    print(f"  GT (30Hz)        : {np.abs(gj).max():10.0f}")
    print(f"  model raw (30Hz) : {np.abs(pj).max():10.0f}")
    print(f"  model+ruckig     : {np.abs(rj).max():10.0f}   (limit {JMAX[ACTIVE].max():.0f})")

    # ---- FK EEF trajectories ----
    joints = parse_joints(URDF_XACRO); chain = chain_to(joints, EEF_LINK, BASE_LINK)
    def eef(arr):
        q = {n: arr[:, i] for i, n in enumerate(ARM_JOINTS)}
        return fk_positions(chain, q)
    eef_gt, eef_pred, eef_rk = eef(gt), eef(pred), eef(pred_rk)

    # ================= plot 1: per-joint time series (arm 6) =================
    t = np.arange(len(gt)) * dt
    fig, ax = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    for j in range(6):
        a = ax[j // 2, j % 2]
        a.plot(t, gt[:, j], "k-", lw=2, label="GT (training)")
        a.plot(t, pred[:, j], color="tab:red", lw=1.2, alpha=.8, label="model raw")
        a.plot(t, pred_rk[:, j], color="tab:blue", lw=1.4, alpha=.9, label="model+ruckig")
        a.set_title(f"arm joint j{j+1}"); a.set_ylabel("rad"); a.grid(alpha=.3)
        if j == 0: a.legend(fontsize=8)
    for k in range(2): ax[2, k].set_xlabel("time (s)")
    fig.suptitle(f"Right-arm action: GT vs Diffusion model vs model+Ruckig — episode {args.episode}", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97]); fig.savefig("compare_model_ruckig_joints.png", dpi=140)
    print("\nsaved: compare_model_ruckig_joints.png")

    # ================= plot 2: EEF 3D + jerk =================
    fig = plt.figure(figsize=(16, 7))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    for p, c, l in [(eef_gt, "k", "GT"), (eef_pred, "tab:red", "model raw"), (eef_rk, "tab:blue", "model+ruckig")]:
        ax3d.plot(p[:, 0], p[:, 1], p[:, 2], color=c, lw=1.6, label=l, alpha=.9)
        ax3d.scatter(*p[0], color=c, marker="o", s=30); ax3d.scatter(*p[-1], color=c, marker="x", s=40)
    ax3d.set_xlabel("X"); ax3d.set_ylabel("Y"); ax3d.set_zlabel("Z")
    ax3d.set_title("EEF (tool0) 3D trajectory"); ax3d.legend(fontsize=9)
    # peak (velocity/accel/jerk) as ratio to limit, over arm dims — bars per method
    axb = fig.add_subplot(1, 2, 2)
    gv, ga, gjk = vaj(gt[:, :6], dt)
    pv, pa, pjk = vaj(pred[:, :6], dt)
    rkv_all, rka_all, rkj_all = vaj(rk_full, CTRL_DT)          # ruckig executed @100Hz (active dims)
    rkv, rka, rkj = rkv_all[:, :6], rka_all[:, :6], rkj_all[:, :6]  # arm
    def ratio(v, a, jk):
        return [np.max(np.abs(v) / ARM_V), np.max(np.abs(a) / ARM_A), np.max(np.abs(jk) / ARM_J)]
    data = {"GT (training)": ratio(gv, ga, gjk),
            "model raw": ratio(pv, pa, pjk),
            "model+ruckig": ratio(rkv, rka, rkj)}
    x = np.arange(3); w = 0.26
    for i, (lbl, vals) in enumerate(data.items()):
        bars = axb.bar(x + (i - 1) * w, vals, w, label=lbl,
                       color=["k", "tab:red", "tab:blue"][i], alpha=.85)
        for b, v in zip(bars, vals):
            axb.text(b.get_x() + w/2, v * 1.02, f"{v:.1f}×", ha="center", va="bottom", fontsize=7)
    axb.axhline(1.0, color="g", ls="--", lw=1.5, label="kinematic limit (=1.0)")
    axb.set_yscale("log"); axb.set_xticks(x); axb.set_xticklabels(["velocity", "accel", "jerk"])
    axb.set_ylabel("peak |·| ÷ limit  (log)"); axb.set_title("kinematic feasibility (arm) — lower is safer")
    axb.legend(fontsize=8); axb.grid(axis="y", alpha=.3)
    fig.suptitle(f"Model output + Ruckig vs training data — episode {args.episode}", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig("compare_model_ruckig_eef.png", dpi=140)
    print("saved: compare_model_ruckig_eef.png")


if __name__ == "__main__":
    main()
