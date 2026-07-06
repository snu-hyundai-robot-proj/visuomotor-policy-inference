# j6 Excluded: hold-j6 Trace

- dataset action[11] raw degree range: `95.000 .. 173.435`
- reconstructed requested_norm range: `0.174371 .. 1.000000`
- final serial/degree target range after clamp: `95.000 .. 173.435` deg
- existing trace `j6_hold_enabled`: `True` in frame CSV rows
- existing trace effective target degree range: `179.800 .. 179.800`
- existing trace actual degree range: `179.700 .. 179.800`

This trace was generated with `--hold-j6=true`, so effective j6 was intentionally fixed. It can prove that the dataset requested j6 varies and that the trace held it fixed, but it cannot evaluate hold-free physical j6 replay fidelity.
