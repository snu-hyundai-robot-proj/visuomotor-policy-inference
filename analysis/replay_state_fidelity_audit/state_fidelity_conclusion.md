# State Fidelity Conclusion

## A. dataset representation

The data supports this interpretation:

- episode `action[6:12]` is closer to a desired/pre-clamp command.
- episode `state[6:12]` is closer to physical/post-clamp recorded hand state.

The strongest evidence is j5:

- j5 raw action degree range: `135.45 .. 146.11`
- j5 post-clamp action degree range: `135.00 .. 135.00`
- j5 episode state degree range: `134.90 .. 134.90`
- j5 clamp changed frame pct: `100.00%`

So for j5, action asks above the runtime limit, while state sits near the physical upper limit. That is much more consistent with pre-clamp desired action vs post-clamp physical state than with a replay code bug.

## B. j1-j5 replay fidelity

Primary metric is same-frame episode state degree vs replay actual degree, without time shift:

- j1 pinky: MAE 0.63 deg, RMSE 1.71, p95 3.29, signed +0.14, worst frame 202
- j2 ring: MAE 0.57 deg, RMSE 1.61, p95 2.87, signed -0.02, worst frame 186
- j3 middle: MAE 3.77 deg, RMSE 10.72, p95 27.37, signed +2.79, worst frame 187
- j4 index: MAE 0.75 deg, RMSE 1.73, p95 2.60, signed -0.15, worst frame 181
- j5 thumb_bend: MAE 0.00 deg, RMSE 0.00, p95 0.00, signed +0.00, worst frame 0

Secondary reference:

- j1 pinky: post-clamp action vs actual MAE 2.87 deg
- j2 ring: post-clamp action vs actual MAE 3.26 deg
- j3 middle: post-clamp action vs actual MAE 6.40 deg
- j4 index: post-clamp action vs actual MAE 3.55 deg
- j5 thumb_bend: post-clamp action vs actual MAE 0.10 deg

## C. actual replay code issue 여부

The old large action-vs-actual error is partly a dataset representation issue: comparing pre-clamp action directly to actual physical state exaggerates error, especially for j5.

For j1-j5, the right fidelity question is whether replay actual tracks episode state and/or post-clamp command. The generated metrics separate those comparisons. Any residual error after using episode state as reference is replay/runtime fidelity error, not merely action representation.

Transition frames 168..210 make the split clear:

- original recording post-clamp action -> episode state already has large lag/error for j1-j4: roughly 17..21 deg MAE, best-fit lag about 11..13 frames.
- replay episode state -> replay actual is much smaller for j1, j2, j4: about 2.2..3.2 deg MAE in that window.
- replay j3 is the exception: episode state -> replay actual is about 25.4 deg MAE in that window, so j3 has an additional replay/runtime fidelity issue in this trace.
- j5 is not a replay fidelity problem here: episode state and replay actual are essentially identical, while target/action comparisons look bad because j5 is clamp-saturated.

The best-fit lag values in the CSV are reference-only diagnostics and are not mixed into the primary metrics.

## D. j6

j6 is excluded from primary ranking because this trace used `--hold-j6`. It can show requested j6 varies and effective j6 is held, but it cannot determine hold-free j6 replay fidelity.
