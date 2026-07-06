#!/usr/bin/env python3
"""Offline analysis for RH56 j6 directional behavior.

Reads the thumb live target CSV and produces command-segment summaries and a
timeline plot. No ROS, serial, docker, or hardware access is used.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path("/home/bi/visuomotor-policy-inference/analysis/j6_directional_diagnostic")
CSV_CANDIDATES = [
    Path("/tmp/rh56_thumb_live_target_test_20260630_171919.csv"),
    Path("/home/bi/visuomotor-policy-inference/tmp/rh56_thumb_live_target_test_20260630_171919.csv"),
]
NORM_TO_DEG = lambda x: max(60.0, min(180.0, (1900.0 - 950.0 * float(x)) / 10.0))


def find_csv() -> Path:
    for p in CSV_CANDIDATES:
        if p.exists():
            return p
    raise FileNotFoundError("rh56_thumb_live_target_test_20260630_171919.csv not found on host")


def f(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else np.nan
    except Exception:
        return np.nan


def load_rows(path: Path):
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def split_segments(rows):
    segments = []
    cur = []
    key = None
    for row in rows:
        k = (round(f(row, "requested_target_4"), 4), round(f(row, "requested_target_5"), 4))
        if key is None or k == key:
            cur.append(row)
            key = k
        else:
            segments.append((key, cur))
            cur = [row]
            key = k
    if cur:
        segments.append((key, cur))
    return segments


def median(rows, key):
    vals = [f(r, key) for r in rows]
    vals = [v for v in vals if math.isfinite(v)]
    return float(np.median(vals)) if vals else np.nan


def last_change_time(rows, key="j6_actual_deg", eps=0.05):
    vals = np.asarray([f(r, key) for r in rows], dtype=float)
    ts = np.asarray([f(r, "timestamp_monotonic") for r in rows], dtype=float)
    good = np.isfinite(vals) & np.isfinite(ts)
    vals, ts = vals[good], ts[good]
    if len(vals) < 2:
        return np.nan
    diffs = np.abs(np.diff(vals))
    idxs = np.where(diffs > eps)[0]
    if len(idxs) == 0:
        return np.nan
    return float(ts[idxs[-1] + 1])


def classify(expected_delta, actual_delta, final_error):
    if abs(expected_delta) < 0.2:
        return "tracks" if abs(final_error) <= 0.5 else "feedback_ambiguous"
    if abs(actual_delta) >= 0.5 and np.sign(actual_delta) == np.sign(expected_delta) and abs(final_error) <= 0.75:
        return "tracks"
    if abs(actual_delta) < 0.5:
        return "does_not_track"
    if np.sign(actual_delta) == np.sign(expected_delta):
        return "feedback_ambiguous"
    return "does_not_track"


def analyze(path: Path):
    rows = load_rows(path)
    segments = split_segments(rows)
    out_rows = []
    prev_settled_actual = None
    prev_expected = None
    for i, (key, seg) in enumerate(segments):
        req4, req5 = key
        n = len(seg)
        cut_pre = max(1, int(n * 0.2))
        cut_settle = max(0, int(n * 0.5))
        early = seg[:cut_pre]
        settled = seg[cut_settle:] or seg
        expected = NORM_TO_DEG(req5)
        obs5 = median(settled, "observed_target_5")
        tj6 = median(settled, "tj6_deg")
        actual_before = prev_settled_actual if prev_settled_actual is not None else median(early, "j6_actual_deg")
        actual_after = median(settled, "j6_actual_deg")
        actual_delta = actual_after - actual_before
        expected_delta = 0.0 if prev_expected is None else expected - prev_expected
        err = tj6 - actual_after
        start_t = f(seg[0], "timestamp_monotonic")
        end_t = f(seg[-1], "timestamp_monotonic")
        last_t = last_change_time(seg)
        out_rows.append(
            {
                "segment": i,
                "command": "initial" if i == 0 else f"set r {req5:.4f}",
                "requested_data5": req5,
                "observed_target_data5": obs5,
                "expected_tj6_deg": expected,
                "observed_tj6_deg": tj6,
                "actual_before_deg": actual_before,
                "actual_after_deg": actual_after,
                "actual_delta_deg": actual_delta,
                "expected_delta_deg": expected_delta,
                "tj6_minus_actual_error_deg": err,
                "segment_start_monotonic": start_t,
                "segment_end_monotonic": end_t,
                "segment_duration_sec": end_t - start_t if np.isfinite(start_t) and np.isfinite(end_t) else np.nan,
                "actual_last_change_timestamp_monotonic": last_t,
                "direction_result": classify(expected_delta, actual_delta, err),
            }
        )
        prev_settled_actual = actual_after
        prev_expected = expected
    return rows, out_rows


def write_csv(rows):
    path = OUT / "j6_thumb_test_csv_analysis.csv"
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def plot_timeline(rows, summary):
    t0 = f(rows[0], "timestamp_monotonic")
    t = np.asarray([f(r, "timestamp_monotonic") - t0 for r in rows])
    req5 = np.asarray([f(r, "requested_target_5") for r in rows])
    obs5 = np.asarray([f(r, "observed_target_5") for r in rows])
    tj6 = np.asarray([f(r, "tj6_deg") for r in rows])
    actual = np.asarray([f(r, "j6_actual_deg") for r in rows])
    j5actual = np.asarray([f(r, "j5_actual_deg") for r in rows])
    fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    axs[0].plot(t, req5, label="requested data[5]", lw=1)
    axs[0].plot(t, obs5, label="observed target.data[5]", lw=1, alpha=0.7)
    axs[0].set_ylabel("norm")
    axs[0].legend(fontsize=8)
    axs[1].plot(t, tj6, label="tj6 target deg", lw=1)
    axs[1].plot(t, actual, label="j6 actual deg", lw=1)
    axs[1].set_ylabel("j6 deg")
    axs[1].legend(fontsize=8)
    axs[2].plot(t, tj6 - actual, label="tj6 - actual", color="tab:red", lw=1)
    axs[2].axhline(0, color="k", lw=0.5)
    axs[2].set_ylabel("error deg")
    axs[2].legend(fontsize=8)
    axs[3].plot(t, j5actual, label="j5 actual sanity", color="tab:green", lw=1)
    axs[3].set_ylabel("j5 deg")
    axs[3].set_xlabel("time since CSV start sec")
    axs[3].legend(fontsize=8)
    for r in summary:
        x = r["segment_start_monotonic"] - t0
        for ax in axs:
            ax.axvline(x, color="k", alpha=0.15, lw=0.7)
        axs[0].text(x, axs[0].get_ylim()[1], str(r["segment"]), fontsize=7, va="top")
    fig.tight_layout()
    fig.savefig(OUT / "j6_thumb_test_timeline.png", dpi=160)
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    path = find_csv()
    rows, summary = analyze(path)
    write_csv(summary)
    plot_timeline(rows, summary)
    print(json.dumps({"csv": str(path), "rows": len(rows), "segments": len(summary)}, indent=2))


if __name__ == "__main__":
    main()
