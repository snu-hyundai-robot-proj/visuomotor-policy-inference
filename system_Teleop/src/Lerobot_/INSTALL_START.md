# LeRobot Bridge Install and Start

`Lerobot_` is the middle interface between ROS2 and the LeRobot policy runner.
It does not contain the UI or robot drivers. It subscribes to robot state and camera topics, runs the policy, and publishes actions back to ROS2.

## Build

```bash
cd <workspace>/src
colcon build --packages-select lerobot_system
```

After build, source the workspace.

- Linux:

```bash
source install/setup.bash
```

- Windows PowerShell:

```powershell
.\install\setup.bat
```

## Start

Run one bridge per side.

```bash
ros2 run lerobot_system lerobot_system_left --ros-args -p side:=left -p policy_path:=<policy_path> -p enable_output:=true
ros2 run lerobot_system lerobot_system_right --ros-args -p side:=right -p policy_path:=<policy_path> -p enable_output:=true
```

Useful parameters:

- `side`: `left` or `right`
- `policy_path`: pretrained policy directory
- `enable_output`: `true` executes robot actions, `false` only prints inference results
- `state_topic`: robot state input, default is `FrameAlignedState`
- `camera_topics`: image topics such as `/system_left/d405_rgb` and `/system_left/zivid_rgb`
- `camera_keys`: observation keys that match the policy input

## Required Runtime Flow

1. Start robot state publishers for the chosen side.
2. Start Vision so it publishes `/system_<side>/d405_rgb` and `/system_<side>/zivid_rgb`.
3. Start `lerobot_system_left` or `lerobot_system_right`.
4. Use the UI to toggle between execute mode and print-only mode.

## UI Compatibility

The current `system_ui_lerobot` layout is compatible with this bridge as long as:

- the Vision nodes publish the camera topics above
- the policy path points to a valid LeRobot checkpoint
- the robot side publishes the expected frame-aligned state
- `Init Pose` is handled by the robot-side `UiCommand` path

## Quick Check

If inference should run without motion, start the bridge with `enable_output:=false`.
If actions should move the robot, switch the UI mode to execute output and restart the bridge through the UI manager.
