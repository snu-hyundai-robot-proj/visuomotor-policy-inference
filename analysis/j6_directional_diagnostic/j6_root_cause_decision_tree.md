# j6 Root Cause Decision Tree

## Starting evidence

- `requested data[5]` equals observed `/inspire/right/target.data[5]` in the thumb live test.
- `tj6` changes according to the expected mapping.
- `j6 actual` tracks one small positive direction command: `174.2 -> 176.7 deg`.
- `j6 actual` does not return for the later negative direction commands: target `174.2`, then `171.0`, while actual remains `176.7`.
- j5 tracks normally in the same session.

## Decision tree

1. Does `/inspire/right/target.data[5]` fail to match the requested value?
   - Yes: external publisher conflict or tool/topic issue.
   - No: continue. Current CSV says no.

2. Does `tj6` fail to follow the expected formula?
   - Yes: driver mapping/clamp/echo issue.
   - No: continue. Current CSV says no.

3. Does driver static audit show j6 omitted from serial `angleSet` packet?
   - Yes: software packet/index bug.
   - No: continue. Static audit says no.

4. Does `j6 actual` move in either direction?
   - No: could be disabled/error/motor/feedback issue.
   - Yes: continue. Current CSV shows one direction moved.

5. Does `j6 actual` move both directions around the same safe-start range?
   - Yes: command path and actuator can track locally; replay issue would need timing/range analysis.
   - No: current result. Directional mechanical/actuator/internal controller issue remains likely.

6. Are current/error/status diagnostics available and do they show j6 fault/limit/current saturation?
   - Yes: classify as controller error/disable/limit/current issue.
   - No: run opt-in diagnostics once to gather available driver-side status. Current driver can safely publish angleAct and target echo; current/error/status are not available without adding extra reads.

## Current branch

The present evidence lands at step 5: command generation and target echo are normal, j6 actual moves only in one direction in this small test, and j5 works normally. Software path alone does not explain the asymmetric tracking.
