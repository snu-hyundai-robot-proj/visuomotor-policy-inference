# j6 Next Step Recommendation

## Final classification
**3. offset으로 현재 directional non-tracking을 설명하기 어렵고, actuator/limit/mechanical 방향성 문제가 더 유력함.**

Recommended next action: **software path is sufficiently cleared; inspect hand actuator/limit/mechanical/internal controller state next**.

Reasoning:
- Direct ROS command changed `target.data[5]` and driver `tj6` exactly as expected.
- The 171.0 deg command is inside the documented driver/protocol clamp range.
- No active repository software/config offset or calibration parameter was found that can explain a flat `angleAct` response to a 5.6 deg target change.
- Current/error/status registers exist in the protocol dictionary, but the active driver path does not read them, so firmware fault/limit/current state remains outside this offline proof.

If one more software-side observation is allowed later, the single highest-value evidence would be current/error/status/limit state for j6 while issuing the same tiny normal ROS command. That requires an explicitly enabled diagnostic run or vendor tooling, not offset/mapping changes.
