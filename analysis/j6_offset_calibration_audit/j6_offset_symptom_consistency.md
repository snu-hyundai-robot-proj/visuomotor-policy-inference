# j6 Offset Symptom Consistency

## Observed symptom to explain
- Safe start: `tj6=176.6 deg`, `j6 actual=176.6 deg`.
- Command: `data[5] 0.1411 -> 0.2000`, `tj6 176.6 -> 171.0 deg`, while `j6 actual` remains `176.6..176.7 deg`.
- Earlier small upward command around `174.2 -> 176.7 deg` tracked once, but downward/return commands did not track.

## Candidate consistency
| candidate | status | reason |
|---|---|---|
| active fixed software offset on target only | does_not_explain | A fixed offset would shift both 176.6 and 171.0 commands. It would not make actual track at one point and then remain fixed while the target changes. |
| active fixed software offset on feedback only | does_not_explain | A feedback offset would bias `angleAct`, but changes in physical position should still appear as changes in `j6 actual`. It does not explain a flat actual trace during a 5.6 deg target change. |
| common target+feedback offset | does_not_explain | A common coordinate offset preserves delta. The observed problem is loss of delta tracking, not an absolute disagreement at all positions. |
| j6 target clamp to 180 deg | does_not_explain | `data[5]=0.2000` reconstructs to 171.0 deg, inside the 60..180 deg range. It is not clipped to 180 deg in the driver formula. |
| teleop percentile calibration | not_active_in_current_path | It changes Manus-to-normalized mapping only when `inspire_bridge_node` publishes. Direct test publishes normalized `data[5]` directly with bridge stopped. |
| replay `--hold-j6` | not_active_in_current_path | It can hold replay j6, but was not part of the direct staged sweep. |
| hand firmware internal zero/soft-limit/fault | partially_explains | Code can send a valid target and read static `angleAct`; firmware could internally refuse one direction due to zero/limit/fault/current protection, but current/error/status are not read in the active path. |
| actuator/mechanical endpoint/stiction/cable/gear issue | explains_symptom | Valid target echo plus unchanged `angleAct` during a 5.6 deg command, with prior one-direction movement, is consistent with one-direction mechanical/electromechanical tracking failure. |

## Logical check
A fixed software offset is an affine coordinate error. It can create a constant target/actual bias or an endpoint clipping issue, but it cannot by itself erase the commanded delta only in one direction while leaving the target echo correct. The present symptom is directional non-tracking, not merely wrong zero.
