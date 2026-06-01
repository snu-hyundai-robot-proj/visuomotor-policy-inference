# episode_manager

Episode lifecycle orchestrator for the visuomotor policy stack — the runtime of
[`../EPISODE_SYSTEM.md`](../EPISODE_SYSTEM.md).

```
BOOT --healthy--> HOMING --settled--> READY --start--> RUNNING --terminate--> STOPPING
  ^                                     ^                                          |
  |                                     +---------------- re-home -----------------+
  +-- (any) --> FAULT (safety/e-stop; latched)
```

## What it does
- **HOMING**: ramps arm+hand from the current pose to the configured **init pose**
  (`arm_home`/`hand_home`, rad), rate-limited, publishing to the same command topics the
  policy uses. Settles by tolerance + tick count, else FAULT on timeout.
- **Command arbitration**: owns the policy node **run gate** (`/vpi/set_enable`) — gate OFF
  while the manager drives (HOMING/STOPPING), ON in RUNNING (policy drives). One writer at a time.
- **Episode**: resets the policy (`/vpi_policy_control/reset`) at start; terminates on
  `timeout | success | input_lost | manual`; holds pose, then re-homes for the next episode.
- **Safety**: watchdog (input freshness, FT overload, e-stop) → STOPPING/FAULT.
- Publishes `/episode/status` (JSON) for the web console.

## Services / topics
| name | type | |
|---|---|---|
| `/episode/home` `/episode/start` `/episode/stop` `/episode/clear_fault` | `std_srvs/Trigger` | control |
| `/episode/estop` | `std_msgs/Bool` (sub) | latched soft e-stop |
| `/episode/success` | `std_msgs/Bool` (sub) | external success signal (optional) |
| `/episode/status` | `std_msgs/String` (JSON) | for the frontend |

## Build & run
```bash
ln -s /home/bi/visuomotor-policy-inference/ros2_episode_manager <ws>/src/episode_manager
cd <ws> && colcon build --packages-select episode_manager && source install/setup.bash

# set arm_home/hand_home to your real init pose first (see EPISODE_SYSTEM.md §7)
ros2 launch episode_manager episode.launch.py
```
Runs alongside the policy control node (`vpi_robot_client`), robot drivers, cameras, and
(for the console) rosbridge. With `state_source:=joint_states` it needs no custom messages.

## Key parameters
| param | default | note |
|---|---|---|
| `arm_home` / `hand_home` | zeros | **init pose (rad) — must set** (dataset episode-start) |
| `home_max_delta` | 0.02 | rad/tick homing ramp |
| `home_tol` / `home_settle_ticks` / `home_timeout_sec` | 0.02 / 15 / 15 | settle + timeout |
| `max_duration_sec` | 20 | episode timeout |
| `auto_start` / `episodes` | false / -1 | continuous vs manual; -1 = infinite |
| `ft_topic` / `ft_force_max` / `ft_torque_max` | "" / 0 / 0 | FT abort (0 = off) |
| `state_source` | frame_aligned | or `joint_states` |
