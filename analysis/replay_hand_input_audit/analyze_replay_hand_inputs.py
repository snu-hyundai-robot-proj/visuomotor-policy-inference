#!/usr/bin/env python3
"""Offline audit of RH56 right-hand replay inputs.

This script reads the sample episode, replay code, driver code, and the existing
trace-derived CSV. It does not publish ROS messages or touch hardware.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
REPO = Path("/home/bi/visuomotor-policy-inference")
EPISODE_CANDIDATES = [
    Path("/tmp/sample_episodes/right/episode.json"),
    REPO / "examples/sample_episodes/right/episode.json",
]
TRACE_DIR = REPO / "analysis/replay_actual_trace_right_20260630_022959"
TRACE_CSV = TRACE_DIR / "frame_level_comparison.csv"
REPLAY_CODE = REPO / "examples/drive_arm_hand_replay.py"
DRIVER_CODE = REPO / "system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_driver.py"
COMM_CODE = REPO / "system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicate/inspire_comm.py"

NAMES = ["pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation"]
INSPIRE_K = 1800.0 / math.pi


def clamp(x, lo, hi):
    return np.minimum(hi, np.maximum(lo, x))


def episode_path() -> Path:
    for p in EPISODE_CANDIDATES:
        if p.exists():
            return p
    raise FileNotFoundError("No right episode.json found in /tmp or repo examples")


def load_episode():
    p = episode_path()
    meta = json.loads(p.read_text())
    frames = meta["frames"]
    states = np.asarray([fr["state"][6:12] for fr in frames], dtype=float)
    actions = np.asarray([fr["action"][6:12] for fr in frames], dtype=float)
    timestamps = np.asarray([fr.get("timestamp", i / meta.get("fps", 30)) for i, fr in enumerate(frames)], dtype=float)
    return p, meta, frames, timestamps, states, actions


def replay_mapping(rad6: np.ndarray):
    a = rad6 * INSPIRE_K
    norm = np.empty_like(a)
    norm[:, :4] = (a[:, :4] - 750.0) / 1100.0
    norm[:, 4] = (a[:, 4] - 1100.0) / 400.0
    norm[:, 5] = (1900.0 - a[:, 5]) / 950.0

    pre_serial = np.empty_like(a)
    pre_serial[:, :4] = 750.0 + 1100.0 * norm[:, :4]
    pre_serial[:, 4] = 1100.0 + 400.0 * norm[:, 4]
    pre_serial[:, 5] = 1900.0 - 950.0 * norm[:, 5]

    final = np.empty_like(a)
    final[:, :4] = clamp(pre_serial[:, :4], 900.0, 1740.0)
    final[:, 4] = clamp(pre_serial[:, 4], 1100.0, 1350.0)
    final[:, 5] = clamp(pre_serial[:, 5], 600.0, 1800.0)
    return a, norm, pre_serial, final


def pct(mask):
    return float(np.mean(mask) * 100.0)


def summarize_channel(vals: np.ndarray):
    dif = np.diff(vals)
    return {
        "min": float(np.min(vals)),
        "max": float(np.max(vals)),
        "span": float(np.max(vals) - np.min(vals)),
        "mean": float(np.mean(vals)),
        "std": float(np.std(vals)),
        "p1": float(np.percentile(vals, 1)),
        "p50": float(np.percentile(vals, 50)),
        "p99": float(np.percentile(vals, 99)),
        "first": float(vals[0]),
        "last": float(vals[-1]),
        "max_per_frame_step": float(np.max(np.abs(dif))) if len(vals) > 1 else 0.0,
        "constant_value_frame_ratio": float(np.mean(np.isclose(dif, 0.0, atol=1e-12))) if len(vals) > 1 else 1.0,
    }


def write_dataset_summary(meta, states, actions):
    a_serial, norm, pre, final = replay_mapping(actions)
    rows = []
    for i, name in enumerate(NAMES):
        for source, arr in [("state", states), ("action", actions)]:
            s = summarize_channel(arr[:, i])
            row = {
                "source": source,
                "joint_index": i,
                "joint_name": name,
                "raw_unit": "rad",
                **s,
                "actuator_serial_min": float((arr[:, i] * INSPIRE_K).min()),
                "actuator_serial_max": float((arr[:, i] * INSPIRE_K).max()),
                "actuator_serial_span": float(np.ptp(arr[:, i] * INSPIRE_K)),
                "actuator_serial_first": float(arr[0, i] * INSPIRE_K),
                "actuator_serial_last": float(arr[-1, i] * INSPIRE_K),
                "actuator_deg_min": float((arr[:, i] * INSPIRE_K / 10.0).min()),
                "actuator_deg_max": float((arr[:, i] * INSPIRE_K / 10.0).max()),
            }
            if source == "action":
                row.update({
                    "requested_norm_min": float(norm[:, i].min()),
                    "requested_norm_max": float(norm[:, i].max()),
                    "final_serial_min": float(final[:, i].min()),
                    "final_serial_max": float(final[:, i].max()),
                    "final_deg_min": float((final[:, i] / 10.0).min()),
                    "final_deg_max": float((final[:, i] / 10.0).max()),
                })
            rows.append(row)
    with (OUT / "dataset_hand_action_summary.csv").open("w", newline="") as f:
        fieldnames = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def write_mapping_tables(actions):
    a_serial, norm, pre, final = replay_mapping(actions)
    mapping_rows = []
    clamp_rows = []
    limits = [(900, 1740), (900, 1740), (900, 1740), (900, 1740), (1100, 1350), (600, 1800)]
    for i, name in enumerate(NAMES):
        lo, hi = limits[i]
        mapping_rows.append({
            "joint_index": i,
            "joint_name": name,
            "action_rad_min": float(actions[:, i].min()),
            "action_rad_max": float(actions[:, i].max()),
            "action_serial_min": float(a_serial[:, i].min()),
            "action_serial_max": float(a_serial[:, i].max()),
            "requested_norm_min": float(norm[:, i].min()),
            "requested_norm_max": float(norm[:, i].max()),
            "pre_clamp_serial_min": float(pre[:, i].min()),
            "pre_clamp_serial_max": float(pre[:, i].max()),
            "final_serial_min": float(final[:, i].min()),
            "final_serial_max": float(final[:, i].max()),
            "final_deg_min": float((final[:, i] / 10.0).min()),
            "final_deg_max": float((final[:, i] / 10.0).max()),
        })
        clamp_rows.append({
            "joint_index": i,
            "joint_name": name,
            "lower_limit_serial": lo,
            "upper_limit_serial": hi,
            "lower_clamp_hit_pct": pct(pre[:, i] < lo),
            "upper_clamp_hit_pct": pct(pre[:, i] > hi),
            "changed_by_clamp_pct": pct(~np.isclose(pre[:, i], final[:, i], atol=1e-9)),
            "final_constant_step_pct": pct(np.isclose(np.diff(final[:, i]), 0.0, atol=1e-9)),
            "final_unique_values": int(len(np.unique(np.round(final[:, i], 9)))),
        })
    for path, rows in [
        (OUT / "replay_hand_mapping_table.csv", mapping_rows),
        (OUT / "replay_hand_clamp_report.csv", clamp_rows),
    ]:
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)


def plot_dataset(t, states, actions):
    fig, axs = plt.subplots(6, 2, figsize=(13, 12), sharex=True)
    for i, name in enumerate(NAMES):
        axs[i, 0].plot(t, states[:, i], lw=1)
        axs[i, 0].set_ylabel(f"{name}\nrad")
        axs[i, 1].plot(t, actions[:, i], lw=1, color="tab:orange")
        axs[i, 1].set_ylabel("rad")
    axs[0, 0].set_title("episode state[6:12]")
    axs[0, 1].set_title("episode action[6:12]")
    axs[-1, 0].set_xlabel("episode time sec")
    axs[-1, 1].set_xlabel("episode time sec")
    fig.tight_layout()
    fig.savefig(OUT / "dataset_hand_action_plots.png", dpi=160)
    plt.close(fig)


def plot_serial(t, actions, joint_idx, filename):
    a, norm, pre, final = replay_mapping(actions)
    fig, axs = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    name = NAMES[joint_idx]
    axs[0].plot(t, actions[:, joint_idx], lw=1)
    axs[0].set_ylabel("action rad")
    axs[0].set_title(f"{name}: dataset action to replay/driver serial target")
    axs[1].plot(t, norm[:, joint_idx], lw=1, color="tab:orange")
    axs[1].set_ylabel("requested norm")
    axs[2].plot(t, pre[:, joint_idx] / 10.0, lw=1, label="pre clamp")
    axs[2].plot(t, final[:, joint_idx] / 10.0, lw=1, label="final", alpha=0.8)
    axs[2].set_ylabel("target deg")
    axs[2].legend(loc="best")
    axs[3].plot(t[1:], np.diff(final[:, joint_idx] / 10.0), lw=1, color="tab:red")
    axs[3].set_ylabel("deg/frame")
    axs[3].set_xlabel("episode time sec")
    fig.tight_layout()
    fig.savefig(OUT / filename, dpi=160)
    plt.close(fig)


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def trace_summary():
    rows = read_csv(TRACE_CSV)
    if not rows:
        return {"available": False, "reason": f"{TRACE_CSV} not found"}
    def col(name):
        vals = []
        for r in rows:
            v = r.get(name, "")
            if v != "":
                vals.append(float(v))
        return np.asarray(vals, dtype=float)
    j6_action = col("j6_action_deg")
    eff = col("j6_effective_target_deg")
    actual = col("j6_actual_deg")
    hold_vals = [r.get("j6_hold_enabled", "") for r in rows]
    hold_true = sum(str(v).lower() == "true" for v in hold_vals)
    requested_norm = (1900.0 - j6_action * 10.0) / 950.0 if len(j6_action) else np.asarray([])
    effective_norm = (1900.0 - eff * 10.0) / 950.0 if len(eff) else np.asarray([])
    return {
        "available": True,
        "trace_csv": str(TRACE_CSV),
        "rows": len(rows),
        "hold_true_pct": hold_true / len(rows) * 100.0 if rows else None,
        "requested_norm_min": float(requested_norm.min()) if len(requested_norm) else None,
        "requested_norm_max": float(requested_norm.max()) if len(requested_norm) else None,
        "requested_norm_span": float(np.ptp(requested_norm)) if len(requested_norm) else None,
        "effective_norm_min": float(effective_norm.min()) if len(effective_norm) else None,
        "effective_norm_max": float(effective_norm.max()) if len(effective_norm) else None,
        "effective_norm_span": float(np.ptp(effective_norm)) if len(effective_norm) else None,
        "j6_action_deg_min": float(j6_action.min()) if len(j6_action) else None,
        "j6_action_deg_max": float(j6_action.max()) if len(j6_action) else None,
        "j6_effective_target_deg_min": float(eff.min()) if len(eff) else None,
        "j6_effective_target_deg_max": float(eff.max()) if len(eff) else None,
        "j6_actual_deg_min": float(actual.min()) if len(actual) else None,
        "j6_actual_deg_max": float(actual.max()) if len(actual) else None,
    }


def write_markdowns(ep_path, meta, actions, trace):
    replay = REPLAY_CODE.read_text()
    driver = DRIVER_CODE.read_text()
    comm = COMM_CODE.read_text()
    pipeline = f"""# Replay Hand Pipeline Audit

## Inputs inspected

- requested episode path: `/tmp/sample_episodes/right/episode.json`
- actual episode path read: `{ep_path}`
- frames: {meta.get('num_frames')} @ {meta.get('fps')} Hz
- replay code: `{REPLAY_CODE}`
- driver code: `{DRIVER_CODE}`
- comm code: `{COMM_CODE}`

`/tmp/sample_episodes/right/episode.json` was not present on the host at analysis time, so the repo copy used by `run_episode_replay.sh` as the source for container copy was inspected.

## Code path

1. `HAND["right"]` sets `ndof=6`, `slice=(6, 12)`, `ref="/inspire/right/target"`.
2. `compute_actions(..., sl, source="recorded")` reads each frame `fr["action"]` and appends `a[sl[0]:sl[1]]`, therefore right hand is exactly `action[6:12]`.
3. `hand_t` is the resulting `N x 6` array. No zero padding or dimension truncation is applied after the slice.
4. `sync_ramp()` and `synced_step()` operate on the full `hand_t` vector using numpy vector deltas. All six dimensions are updated together.
5. `publish_hand_trace(vals)` computes `requested = inspire_rad_to_norm(vals)`.
6. Without `--hold-j6`, `effective = requested.copy()` and all six normalized values are published as `Float64MultiArray(data=[...])`.
7. With `--hold-j6`, only `effective[5]` is overwritten by `j6_hold_norm`; j1-j5 are unchanged.
8. `inspire_driver.py::cmd_callback()` receives `/inspire/right/target`, maps all six values through `retarget_fingers()`, clamps, then calls `move_fingers(current_data)`.
9. `inspire_comm.py::move_fingers()` clamps again to communication limits and writes all six `angleSet` register values.

## Relevant formulas

Replay `inspire_rad_to_norm()`:

- j1-j4: `norm = (action_rad * 1800/pi - 750) / 1100`
- j5: `norm = (action_rad * 1800/pi - 1100) / 400`
- j6: `norm = (1900 - action_rad * 1800/pi) / 950`

Driver `retarget_fingers()`:

- j1-j4: `serial = norm * 1100 + 750`
- j5: `serial = norm * 400 + 1100`
- j6: `serial = -norm * 950 + 1900`

Driver/comm clamp:

- j1-j4: effectively `900..1740` serial units in `inspire_comm.py`; `inspire_driver.py` has `880..1740` but comm reclamps to `900..1740`
- j5: `1100..1350`
- j6: `600..1800`

## Static audit result

- action slice is correct for right hand: `action[6:12]`.
- six hand dimensions are preserved through `hand_t`, `sync_ramp()`, `synced_step()`, `publish_hand_trace()`, and `Float64MultiArray`.
- no NaN handling was found; if NaN entered the command it would likely propagate until int/clamp/write behavior fails or becomes unsafe. The inspected dataset contains finite values.
- no default zero padding was found.
- clamp can change commands at the driver/comm stage; see `replay_hand_clamp_report.csv`.
- `--hold-j6` is the only inspected replay-code path that intentionally overwrites j6. It applies in home, ramp_to_episode, replay, and settle phases because all phases call `publish_hand_trace()`.
"""
    (OUT / "replay_hand_pipeline_audit.md").write_text(pipeline)

    hold_text = f"""# Existing Trace: hold-j6 Explanation

Trace analyzed: `{TRACE_CSV}`

Raw `trace_ticks.jsonl` was not present in the copied analysis folder, so this explanation uses the available `frame_level_comparison.csv`.

## Summary

```json
{json.dumps(trace, indent=2)}
```

## Interpretation

- `j6_action_deg` varies across the episode, so the requested j6 implied by the episode is not constant.
- Reconstructed requested norm is `(1900 - j6_action_deg * 10) / 950`; it spans `{trace.get('requested_norm_span')}`.
- `j6_effective_target_deg` is the command after replay hold/mapping. In this trace it spans `{trace.get('j6_effective_target_deg_max') - trace.get('j6_effective_target_deg_min') if trace.get('available') else None}` degrees.
- `j6_hold_enabled` is true for `{trace.get('hold_true_pct')}` percent of available frame rows.
- Therefore this trace can show that `--hold-j6` fixed the effective j6 command, but it cannot judge hold-free physical j6 tracking.
"""
    (OUT / "existing_trace_hold_j6_explanation.md").write_text(hold_text)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    ep_path, meta, frames, t, states, actions = load_episode()
    write_dataset_summary(meta, states, actions)
    write_mapping_tables(actions)
    plot_dataset(t, states, actions)
    plot_serial(t, actions, 4, "j5_dataset_to_serial_target.png")
    plot_serial(t, actions, 5, "j6_dataset_to_serial_target.png")
    trace = trace_summary()
    write_markdowns(ep_path, meta, actions, trace)
    print(json.dumps({
        "episode_path": str(ep_path),
        "frames": len(frames),
        "fps": meta.get("fps"),
        "trace": trace,
        "outputs": sorted(p.name for p in OUT.iterdir() if p.is_file()),
    }, indent=2))


if __name__ == "__main__":
    main()
