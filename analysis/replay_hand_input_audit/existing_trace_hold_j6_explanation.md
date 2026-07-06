# Existing Trace: hold-j6 Explanation

Trace analyzed: `/home/bi/visuomotor-policy-inference/analysis/replay_actual_trace_right_20260630_022959/frame_level_comparison.csv`

Raw `trace_ticks.jsonl` was not present in the copied analysis folder, so this explanation uses the available `frame_level_comparison.csv`.

## Summary

```json
{
  "available": true,
  "trace_csv": "/home/bi/visuomotor-policy-inference/analysis/replay_actual_trace_right_20260630_022959/frame_level_comparison.csv",
  "rows": 364,
  "hold_true_pct": 100.0,
  "requested_norm_min": 0.1743707057648812,
  "requested_norm_max": 0.9999998729810544,
  "requested_norm_span": 0.8256291672161732,
  "effective_norm_min": 0.10736842105263159,
  "effective_norm_max": 0.10736842105263159,
  "effective_norm_span": 0.0,
  "j6_action_deg_min": 95.00001206679983,
  "j6_action_deg_max": 173.4347829523363,
  "j6_effective_target_deg_min": 179.8,
  "j6_effective_target_deg_max": 179.8,
  "j6_actual_deg_min": 179.7,
  "j6_actual_deg_max": 179.8
}
```

## Interpretation

- `j6_action_deg` varies across the episode, so the requested j6 implied by the episode is not constant.
- Reconstructed requested norm is `(1900 - j6_action_deg * 10) / 950`; it spans `0.8256291672161732`.
- `j6_effective_target_deg` is the command after replay hold/mapping. In this trace it spans `0.0` degrees.
- `j6_hold_enabled` is true for `100.0` percent of available frame rows.
- Therefore this trace can show that `--hold-j6` fixed the effective j6 command, but it cannot judge hold-free physical j6 tracking.
