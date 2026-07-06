# j6 Directional Diagnostic Conclusion

## A. command generation

`data[5] -> final serial target` appears normal.

Evidence:

- The live CSV shows observed `target.data[5]` matches requested values.
- `tj6` follows the expected formula:
  - `data[5]=0.1400 -> tj6=176.70 deg`
  - `data[5]=0.1663 -> tj6=174.20 deg`
  - `data[5]=0.2000 -> tj6=171.00 deg`
- Static driver audit shows j6 uses `cur_6d_vec[5]`, maps to `thumb_rot = -data[5] * 950 + 1900`, is clamped, and is serialized as the sixth `angleSet` pair.

This argues against root cause 1, "j6 driver command/serial packet is wrongly generated or omitted", based on currently inspected code and logs.

## B. feedback semantics

- `j6 actual` is hardware `angleAct` feedback parsed by `get_position_values()` and published from `self.ser.received_angle[5]`.
- `tj6` is driver command echo, not hand hardware ACK.
- Therefore `tj6=171.0` with `actual=176.7` means the driver intended/echoed a lower target, but the actual angle feedback did not follow.

## C. available driver diagnostics

Currently safe, already-available values:

- angleAct actual feedback
- final command target echo

Not currently available without extra register reads:

- current
- errCode
- statusCode
- forceAct
- speedAct

An opt-in diagnostics publisher was added with default OFF:

```text
publish_driver_diagnostics:=false
/inspire/right/driver_diagnostics
```

It publishes only in-memory `angleAct` and final target echo. Unavailable fields are `null`, not guessed.

## D. directional symptom

The directional symptom cannot be explained by replay mapping, clamp range, j6 hold policy, or obvious driver index/sign bugs.

CSV command result:

| command | requested data[5] | expected/observed tj6 | actual before | actual after | delta | tj6-actual | result |
|---|---:|---:|---:|---:|---:|---:|---|
| initial | 0.1663 | 174.20 | 174.20 | 174.20 | 0.00 | 0.00 | tracks |
| r 0.14 | 0.1400 | 176.70 | 174.20 | 176.70 | +2.50 | 0.00 | tracks |
| r 0.1663 | 0.1663 | 174.20 | 176.70 | 176.70 | 0.00 | -2.50 | does_not_track |
| r 0.20 | 0.2000 | 171.00 | 176.70 | 176.70 | 0.00 | -5.70 | does_not_track |
| restore | 0.1663 | 174.20 | 176.70 | 176.70 | 0.00 | -2.50 | does_not_track |

This pattern is most consistent with target delivery plus a physical/internal-controller/feedback issue in one direction.

## E. next action

Recommended next action: **1. diagnostics-enabled driver를 한 번 실행해 status/current/error를 관측.**

Reason:

- The software command path is normal enough that changing mapping/clamp/sign would be premature.
- The current safe diagnostics patch can confirm target echo vs angleAct in the running driver.
- However, current/error/status are still unavailable unless a future carefully designed safe read path is added. If diagnostics-enabled run confirms target echo keeps changing while angleAct sticks, then the next handoff should be actuator/mechanism/internal controller inspection rather than replay code changes.

Do not change `data[5]` sign, replay mapping, clamp range, or j6 hold policy based on this trace.
