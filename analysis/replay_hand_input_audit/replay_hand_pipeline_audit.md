# Replay Hand Pipeline Audit

## Inputs inspected

- requested episode path: `/tmp/sample_episodes/right/episode.json`
- actual episode path read: `/home/bi/visuomotor-policy-inference/examples/sample_episodes/right/episode.json`
- frames: 364 @ 30 Hz
- replay code: `/home/bi/visuomotor-policy-inference/examples/drive_arm_hand_replay.py`
- driver code: `/home/bi/visuomotor-policy-inference/system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_driver.py`
- comm code: `/home/bi/visuomotor-policy-inference/system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicate/inspire_comm.py`

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
