# Driver j6 Static Audit

## Command path

`/inspire/right/target.data[5]` follows the same six-element path as the other fingers:

1. `InspireCommandSubscriber` subscribes to `/inspire/right/target` as `Float64MultiArray` in `inspire_driver.py:73-78`.
2. `cmd_callback()` copies the incoming list into `current_6d_vector` in `inspire_driver.py:95-99`.
3. `retarget_fingers()` maps j6 with `thumb_rot = -cur_6d_vec[5] * 950 + 1900` in `inspire_driver.py:120-132`.
4. Driver clamp applies the same `for i in range(6)` loop to j6 using `INSPIRE_FINGER_MIN_DEGREE/MAX_DEGREE`; j6 limits are `[600, 1800]` raw 0.1 degree units in `inspire_driver.py:32-33` and clamp is in `inspire_driver.py:106-113`.
5. `move_fingers(current_data)` is called in `inspire_driver.py:115`.
6. `move_fingers()` applies comm-layer limits `[900,900,900,900,1100,600]` to `[1740,1740,1740,1740,1350,1800]` in `inspire_comm.py:36-38` and `inspire_comm.py:223-227`.
7. The final serial `angleSet` payload is built by iterating all six targets in order and appending low/high bytes in `inspire_comm.py:228-233`.

For j6 specifically, a target like `1710` raw units becomes bytes:

```text
low  = 1710 & 0xFF
high = 1710 >> 8
```

Those are the sixth pair in the 12-byte `angleSet` payload because `move_fingers()` loops `i in range(6)`.

## Feedback semantics

- `j6 actual` in `/inspire/joint_states` comes from `self.ser.received_angle[5]`, published in `inspire_driver.py:139-149`.
- `received_angle` is updated only from `angleAct` reads. The register dictionary labels `angleAct` as actual finger position in `inspire_comm.py:21`; `send_get_angle()` reads that register in `inspire_comm.py:190-191`; `get_position_values()` parses six int16 values and divides by 10 in `inspire_comm.py:214-221`.
- `tj6` in `/inspire/joint_states` is not a hardware ACK. It is the driver-side command echo from `self.target_joint[5]`, assigned before the final clamp/write in `inspire_driver.py:101-113` and published in `inspire_driver.py:142-147`.

For the tested range, j6 `target_joint[5]` and final target are effectively the same because `retarget_fingers()` and clamp both keep the value in range. Still, semantically `tj6` is a driver echo, not proof that the hand accepted or executed the command.

## Required answers

1. **j6 command index bug?** No static evidence of omission, fixed value, sign reuse, or index reuse. j6 uses `cur_6d_vec[5]`, is included in six-element clamp loops, and is serialized as the sixth target pair.
2. **Final serial packet j6 source?** It comes from `/inspire/right/target.data[5] -> thumb_rot = -data[5] * 950 + 1900 -> clamp -> targets[5] -> low/high byte pair`.
3. **j6 actual source?** `angleAct` feedback parsed from hardware response, not an internal command echo.
4. **tj6 source?** Driver command echo, not hardware ACK.
5. **Different speed/force/mode/error init for j5 vs j6?** No such startup handling is present in the inspected driver. The register dictionary contains `mode`, `clearErr`, `forceClb`, `forceSet`, `speedSet`, `currAct`, `errCode`, and `statusCode`, but this driver path does not write mode/calibration/error settings and does not read current/error/status in the regular polling path.
6. **j6-only clamp/special case/write failure swallow?** There is a j6-specific upper clamp in `retarget_fingers()` for `thumb_rot > 1800`, followed by generic six-axis min/max clamp. No j6-only exception swallowing or j6-only serial write branch was found.

## Static interpretation

The code path does not explain a one-direction-only j6 physical tracking symptom. The software command and packet generation path looks structurally normal for j6, while `j6 actual` is a real `angleAct` feedback value and `tj6` is only driver command echo.
