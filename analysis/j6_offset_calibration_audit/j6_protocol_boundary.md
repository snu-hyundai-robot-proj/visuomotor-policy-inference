# j6 Protocol / Hardware Boundary

## Confirmable from code
- ROS command: `/inspire/right/target.data[5]` is accepted by `cmd_callback`.
- Driver mapping: `thumb_rot = -data[5] * 950 + 1900` raw 0.1 degree units.
- Driver clamp: j6 target is clamped to 600..1800 raw units in both `inspire_driver.py` and `inspire_comm.py`.
- Serial target payload: `angleSet` register 1040 receives six int16 values, j6 as the sixth low/high byte pair.
- Feedback: `angleAct` register 1064 is read as six int16 values and divided by 10.0; j6 is index 5.
- `tj6`: driver command echo from `target_joint[5]`; it is not an `angleSet` hardware ACK.

## Not confirmable from code alone
- Whether the RH56 firmware uses hidden zero calibration or soft limits internally.
- Whether `angleSet` and `angleAct` are guaranteed by firmware to share the same calibrated coordinate frame under all fault/calibration states.
- Whether the j6 motor is disabled, current-limited, blocked by an internal soft limit, or in a latched fault state.
- Whether there is a mechanical endpoint, gear backlash, cable slip, or internal linkage issue.

## Coordinate-system note
The code assumes `angleSet` and `angleAct` use the same 0.1 degree calibrated coordinate system because both are Inspire protocol angle registers. The repository code itself does not prove that firmware cannot apply hidden zero/limit logic between command acceptance and motor actuation.
