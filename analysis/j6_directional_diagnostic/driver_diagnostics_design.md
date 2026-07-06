# Driver Diagnostics Design

## Goal

Add an opt-in read-only ROS diagnostic topic from inside the existing `inspire_driver_node` process, without opening the serial port from another process and without adding a new register read loop.

## Implemented opt-in parameters

```text
publish_driver_diagnostics:=false
driver_diagnostics_hz:=10.0
```

Default is OFF. When OFF, no diagnostics publisher is created.

## Topic

```text
/inspire/right/driver_diagnostics
std_msgs/String
```

The string is JSON and self-describing.

## Available values

Current safe implementation publishes only values already held by the driver:

- `angle_actual_deg`: from `self.ser.received_angle`, which is populated by the existing `angleAct` polling path.
- `target_deg`: from `self.final_target_joint`, the final clamped command in raw 0.1 degree units divided by 10.
- `target_raw_0p1deg`: the same final target in raw serial units.
- `j6.angle_actual_deg`
- `j6.target_deg`

Unavailable values are explicitly `null`:

- current
- error_code
- status_code
- force_actual
- speed_actual

These are not currently read by the existing safe polling path, even though register addresses exist in `inspire_comm.py`.

## Example

```json
{
  "timestamp_monotonic": 0.0,
  "angle_actual_deg": [164.8, 164.6, 161.0, 160.6, 133.1, 176.7],
  "target_deg": [164.98, 164.88, 161.11, 160.83, 133.2, 171.0],
  "j6": {
    "angle_actual_deg": 176.7,
    "target_deg": 171.0,
    "current": null,
    "error_code": null,
    "status_code": null
  },
  "availability": {
    "angle_actual": true,
    "target_echo": true,
    "current": false,
    "error_code": false,
    "status_code": false,
    "force_actual": false,
    "speed_actual": false
  }
}
```

## Safety properties

- No mode write.
- No force calibration write.
- No clear error write.
- No calibration write.
- No separate USB serial open.
- No new arbitrary register read loop.
- Existing command timing is not changed.
- Diagnostics publish is throttled to `driver_diagnostics_hz`, default 10 Hz, and only reuses in-memory values.

## Files changed

- `system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_driver.py`

Static validation performed:

```text
PYTHONPYCACHEPREFIX=/tmp/codex_pycache_j6 python3 -m py_compile \
  system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_driver.py \
  analysis/j6_directional_diagnostic/analyze_j6_directional_behavior.py \
  tools/thumb_live_target_test.py \
  tools/j6_live_target_test.py
```

No build, launch, Docker restart, ROS publish, or hand motion was performed.
