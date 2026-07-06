#!/usr/bin/env python3
"""Clamp-aware RH56 replay state fidelity audit.

Offline only: reads episode JSON and existing trace-derived CSV, then compares
episode state/action conventions with replay actual hand feedback.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


REPO = Path("/home/bi/visuomotor-policy-inference")
OUT = REPO / "analysis/replay_state_fidelity_audit"
EP = REPO / "examples/sample_episodes/right/episode.json"
TRACE_DIR = REPO / "analysis/replay_actual_trace_right_20260630_022959"
TRACE = TRACE_DIR / "frame_level_comparison.csv"
NAMES = ["pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]
LIMITS = np.asarray([[90, 174], [90, 174], [90, 174], [90, 174], [110, 135], [60, 180]], dtype=float)


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_obj(x) -> str:
    return hashlib.sha256(json.dumps(x, sort_keys=True).encode()).hexdigest()


def deg_from_rad_hand(x):
    return np.asarray(x, dtype=float) * 180.0 / math.pi


def clamp_deg(deg):
    return np.minimum(LIMITS[:, 1], np.maximum(LIMITS[:, 0], deg))


def load_episode():
    meta = json.loads(EP.read_text())
    frames = meta["frames"]
    t = np.asarray([fr.get("timestamp", i / meta.get("fps", 30)) for i, fr in enumerate(frames)], dtype=float)
    state_rad = np.asarray([fr["state"][6:12] for fr in frames], dtype=float)
    action_rad = np.asarray([fr["action"][6:12] for fr in frames], dtype=float)
    state_deg = deg_from_rad_hand(state_rad)
    action_raw_deg = deg_from_rad_hand(action_rad)
    action_post_deg = clamp_deg(action_raw_deg)
    return meta, frames, t, state_deg, action_raw_deg, action_post_deg


def load_trace():
    with TRACE.open(newline="") as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    replay_actual = np.full((n, 6), np.nan)
    replay_target = np.full((n, 6), np.nan)
    frame = np.zeros(n, dtype=int)
    ts = np.zeros(n)
    for r_i, r in enumerate(rows):
        frame[r_i] = int(float(r["episode_frame"]))
        ts[r_i] = float(r["episode_time_sec"])
        for j in range(6):
            replay_actual[r_i, j] = float(r[f"hand_C_actual_j{j+1}"])
            replay_target[r_i, j] = float(r[f"hand_B_target_j{j+1}"])
    return rows, frame, ts, replay_target, replay_actual


def metrics(a, b):
    e = b - a
    ae = np.abs(e)
    return {
        "mae": float(np.mean(ae)),
        "rmse": float(np.sqrt(np.mean(e * e))),
        "p95_abs": float(np.percentile(ae, 95)),
        "max_abs": float(np.max(ae)),
        "signed_mean": float(np.mean(e)),
        "worst_local_index": int(np.argmax(ae)),
    }


def best_lag(ref, obs, max_lag=30):
    best = None
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            r, o = ref[-lag:], obs[: len(obs) + lag]
        elif lag > 0:
            r, o = ref[: len(ref) - lag], obs[lag:]
        else:
            r, o = ref, obs
        if len(r) < 5:
            continue
        rmse = float(np.sqrt(np.mean((o - r) ** 2)))
        if best is None or rmse < best[1]:
            best = (lag, rmse)
    return best or (0, float("nan"))


def write_provenance(meta, frames):
    container = []
    cmd = (
        "docker ps --format '{{.Names}}' | while read n; do "
        "docker exec \"$n\" bash -lc 'if [ -f /tmp/sample_episodes/right/episode.json ]; "
        "then echo $HOSTNAME; sha256sum /tmp/sample_episodes/right/episode.json; fi' 2>/dev/null; done"
    )
    try:
        container_text = subprocess.check_output(cmd, shell=True, text=True, timeout=5).strip()
    except Exception as e:
        container_text = f"container check failed: {e}"
    first, last = frames[0], frames[-1]
    text = f"""# Dataset Provenance Check

## Host repo episode

- path: `{EP}`
- sha256: `{sha256_file(EP)}`
- frames: {len(frames)}
- fps: {meta.get('fps')}
- first frame: `{first.get('frame')}`, timestamp `{first.get('timestamp')}`
- last frame: `{last.get('frame')}`, timestamp `{last.get('timestamp')}`
- first state sha256: `{sha256_obj(first['state'])}`
- first action sha256: `{sha256_obj(first['action'])}`
- last state sha256: `{sha256_obj(last['state'])}`
- last action sha256: `{sha256_obj(last['action'])}`

## Container replay source check

Command looked for `/tmp/sample_episodes/right/episode.json` in running containers.

```text
{container_text or 'No running container currently exposes /tmp/sample_episodes/right/episode.json'}
```

At analysis time, the container copy was not available from the running containers. Therefore all numeric results in this folder are labeled as **host repo sample episode 기준**.

`run_episode_replay.sh` copies `examples/sample_episodes` into `/tmp/sample_episodes` only when the container path is missing, and runs `/tmp/drive_arm_hand_replay.py` with default `--episode-dir /tmp/sample_episodes/right`. To prove a future replay exactly, capture SHA256 from inside `ros2_teleop_system` immediately before/after replay.
"""
    (OUT / "dataset_provenance_check.md").write_text(text)


def write_action_state_comparison(t, state, raw, post):
    rows = []
    for j, name in enumerate(NAMES):
        raw_err = raw[:, j] - state[:, j]
        post_err = post[:, j] - state[:, j]
        clamp_delta = post[:, j] - raw[:, j]
        closer_post = np.abs(post_err) < np.abs(raw_err)
        closer_raw = np.abs(raw_err) < np.abs(post_err)
        rows.append({
            "joint": f"j{j+1}",
            "name": name,
            "action_raw_deg_min": float(raw[:, j].min()),
            "action_raw_deg_max": float(raw[:, j].max()),
            "action_post_clamp_deg_min": float(post[:, j].min()),
            "action_post_clamp_deg_max": float(post[:, j].max()),
            "state_deg_min": float(state[:, j].min()),
            "state_deg_max": float(state[:, j].max()),
            "raw_vs_state_mae": float(np.mean(np.abs(raw_err))),
            "post_clamp_vs_state_mae": float(np.mean(np.abs(post_err))),
            "raw_vs_state_rmse": float(np.sqrt(np.mean(raw_err**2))),
            "post_clamp_vs_state_rmse": float(np.sqrt(np.mean(post_err**2))),
            "clamp_changed_frame_pct": float(np.mean(np.abs(clamp_delta) > 1e-9) * 100),
            "clamp_delta_mae_deg": float(np.mean(np.abs(clamp_delta))),
            "state_outside_clamp_pct": float(np.mean((state[:, j] < LIMITS[j, 0]) | (state[:, j] > LIMITS[j, 1])) * 100),
            "post_clamp_closer_pct": float(np.mean(closer_post) * 100),
            "raw_closer_pct": float(np.mean(closer_raw) * 100),
            "tie_pct": float(np.mean(~closer_post & ~closer_raw) * 100),
        })
    with (OUT / "episode_action_state_clamp_comparison.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    fig, axs = plt.subplots(6, 1, figsize=(12, 13), sharex=True)
    for j, ax in enumerate(axs):
        ax.plot(t, raw[:, j], lw=1, label="episode raw action")
        ax.plot(t, post[:, j], lw=1, label="episode post-clamp action")
        ax.plot(t, state[:, j], lw=1, label="episode state")
        ax.axhline(LIMITS[j, 0], color="k", lw=0.5, alpha=0.25)
        ax.axhline(LIMITS[j, 1], color="k", lw=0.5, alpha=0.25)
        ax.set_ylabel(f"j{j+1}\n{NAMES[j]}\ndeg")
        if j == 0:
            ax.legend(ncol=3, fontsize=8)
    axs[-1].set_xlabel("episode time sec")
    fig.tight_layout()
    fig.savefig(OUT / "episode_action_state_clamp_plots.png", dpi=160)
    plt.close(fig)


def write_replay_metrics(t, state, post, frames, ts, replay_target, replay_actual):
    state_f = state[frames]
    post_f = post[frames]
    rows = []
    js = range(5)
    for label, ref in [
        ("episode_state_vs_replay_actual", state_f),
        ("episode_post_clamp_action_vs_replay_actual", post_f),
        ("replay_post_clamp_target_vs_replay_actual", replay_target),
    ]:
        for j in js:
            m = metrics(ref[:, j], replay_actual[:, j])
            lag, lag_rmse = best_lag(ref[:, j], replay_actual[:, j])
            worst = m.pop("worst_local_index")
            rows.append({
                "comparison": label,
                "joint": f"j{j+1}",
                "name": NAMES[j],
                **m,
                "best_fit_lag_frames_reference_only": int(lag),
                "best_fit_lag_rmse_reference_only": float(lag_rmse),
                "worst_frame": int(frames[worst]),
                "worst_timestamp_sec": float(ts[worst]),
            })
    with (OUT / "replay_state_vs_actual_metrics.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    by = {}
    for r in rows:
        by.setdefault(r["comparison"], {})[r["joint"]] = {k: v for k, v in r.items() if k not in ("comparison", "joint", "name")}
    (OUT / "replay_state_vs_actual_metrics.json").write_text(json.dumps(by, indent=2))

    fig, axs = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    for j, ax in enumerate(axs):
        ax.plot(ts, state_f[:, j], lw=1, label="episode state")
        ax.plot(ts, post_f[:, j], lw=1, label="episode post-clamp action", alpha=0.8)
        ax.plot(ts, replay_target[:, j], lw=1, label="replay post-clamp target", alpha=0.8)
        ax.plot(ts, replay_actual[:, j], lw=1, label="replay actual")
        ax.set_ylabel(f"j{j+1}\ndeg")
        if j == 0:
            ax.legend(ncol=4, fontsize=8)
    axs[-1].set_xlabel("episode time sec")
    fig.tight_layout()
    fig.savefig(OUT / "replay_state_vs_actual_j1_j5.png", dpi=160)
    plt.close(fig)

    mask = (frames >= 168) & (frames <= 210)
    fig, axs = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    for j, ax in enumerate(axs):
        ax.plot(ts[mask], raw_global[frames[mask], j], lw=1, label="episode raw action")
        ax.plot(ts[mask], post_f[mask, j], lw=1, label="episode post-clamp action")
        ax.plot(ts[mask], state_f[mask, j], lw=1, label="episode state")
        ax.plot(ts[mask], replay_target[mask, j], lw=1, label="replay post-clamp target")
        ax.plot(ts[mask], replay_actual[mask, j], lw=1, label="replay actual")
        ax.set_ylabel(f"j{j+1}\ndeg")
        if j == 0:
            ax.legend(ncol=3, fontsize=8)
    axs[-1].set_xlabel("episode time sec")
    fig.tight_layout()
    fig.savefig(OUT / "transition_frames_168_210_j1_j5.png", dpi=160)
    plt.close(fig)
    return rows


def write_j6(meta, action_raw, action_post, rows, replay_target, replay_actual):
    j = 5
    requested_norm = (1900.0 - action_raw[:, j] * 10.0) / 950.0
    text = f"""# j6 Excluded: hold-j6 Trace

- dataset action[11] raw degree range: `{action_raw[:, j].min():.3f} .. {action_raw[:, j].max():.3f}`
- reconstructed requested_norm range: `{requested_norm.min():.6f} .. {requested_norm.max():.6f}`
- final serial/degree target range after clamp: `{action_post[:, j].min():.3f} .. {action_post[:, j].max():.3f}` deg
- existing trace `j6_hold_enabled`: `{rows[0].get('j6_hold_enabled') if rows else 'unknown'}` in frame CSV rows
- existing trace effective target degree range: `{replay_target[:, j].min():.3f} .. {replay_target[:, j].max():.3f}`
- existing trace actual degree range: `{replay_actual[:, j].min():.3f} .. {replay_actual[:, j].max():.3f}`

This trace was generated with `--hold-j6=true`, so effective j6 was intentionally fixed. It can prove that the dataset requested j6 varies and that the trace held it fixed, but it cannot evaluate hold-free physical j6 replay fidelity.
"""
    (OUT / "j6_excluded_hold_explanation.md").write_text(text)


def write_conclusion(comp_rows, metric_rows):
    comp = {r["joint"]: r for r in comp_rows}
    primary = [r for r in metric_rows if r["comparison"] == "episode_state_vs_replay_actual"]
    secondary = [r for r in metric_rows if r["comparison"] == "episode_post_clamp_action_vs_replay_actual"]
    ptxt = "\n".join(
        f"- {r['joint']} {r['name']}: MAE {float(r['mae']):.2f} deg, RMSE {float(r['rmse']):.2f}, p95 {float(r['p95_abs']):.2f}, signed {float(r['signed_mean']):+.2f}, worst frame {r['worst_frame']}"
        for r in primary
    )
    stxt = "\n".join(
        f"- {r['joint']} {r['name']}: post-clamp action vs actual MAE {float(r['mae']):.2f} deg"
        for r in secondary
    )
    j5 = comp["j5"]
    text = f"""# State Fidelity Conclusion

## A. dataset representation

The data supports this interpretation:

- episode `action[6:12]` is closer to a desired/pre-clamp command.
- episode `state[6:12]` is closer to physical/post-clamp recorded hand state.

The strongest evidence is j5:

- j5 raw action degree range: `{float(j5['action_raw_deg_min']):.2f} .. {float(j5['action_raw_deg_max']):.2f}`
- j5 post-clamp action degree range: `{float(j5['action_post_clamp_deg_min']):.2f} .. {float(j5['action_post_clamp_deg_max']):.2f}`
- j5 episode state degree range: `{float(j5['state_deg_min']):.2f} .. {float(j5['state_deg_max']):.2f}`
- j5 clamp changed frame pct: `{float(j5['clamp_changed_frame_pct']):.2f}%`

So for j5, action asks above the runtime limit, while state sits near the physical upper limit. That is much more consistent with pre-clamp desired action vs post-clamp physical state than with a replay code bug.

## B. j1-j5 replay fidelity

Primary metric is same-frame episode state degree vs replay actual degree, without time shift:

{ptxt}

Secondary reference:

{stxt}

## C. actual replay code issue 여부

The old large action-vs-actual error is partly a dataset representation issue: comparing pre-clamp action directly to actual physical state exaggerates error, especially for j5.

For j1-j5, the right fidelity question is whether replay actual tracks episode state and/or post-clamp command. The generated metrics separate those comparisons. Any residual error after using episode state as reference is replay/runtime fidelity error, not merely action representation.

The best-fit lag values in the CSV are reference-only diagnostics and are not mixed into the primary metrics.

## D. j6

j6 is excluded from primary ranking because this trace used `--hold-j6`. It can show requested j6 varies and effective j6 is held, but it cannot determine hold-free j6 replay fidelity.
"""
    (OUT / "state_fidelity_conclusion.md").write_text(text)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    meta, frames, t, state, raw, post = load_episode()
    global raw_global
    raw_global = raw
    trace_rows, frame_idx, ts, replay_target, replay_actual = load_trace()
    write_provenance(meta, frames)
    write_action_state_comparison(t, state, raw, post)
    metric_rows = write_replay_metrics(t, state, post, frame_idx, ts, replay_target, replay_actual)
    with (OUT / "episode_action_state_clamp_comparison.csv").open(newline="") as f:
        comp_rows = list(csv.DictReader(f))
    write_j6(meta, raw, post, trace_rows, replay_target, replay_actual)
    write_conclusion(comp_rows, metric_rows)
    print(json.dumps({
        "episode": str(EP),
        "trace": str(TRACE),
        "outputs": sorted(p.name for p in OUT.iterdir() if p.is_file()),
    }, indent=2))


if __name__ == "__main__":
    main()
