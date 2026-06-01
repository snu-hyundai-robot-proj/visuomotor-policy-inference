# vpi_robot_client

ROS2 node that drives **HDR35 arm + DG5F hand** from the visuomotor policy HTTP server.
It is the runtime implementation of [`../EXECUTION_PLAN.md`](../EXECUTION_PLAN.md) and
[`../INTEGRATION_HDR35.md`](../INTEGRATION_HDR35.md).

```
cameras(front/wrist) + state(26)  ──/predict──▶  action[26]=arm[0:6]+hand[6:26] (rad, absolute)
   arm  → soft-start clamp → rad→deg → JointState        → /robot/joint_target_deg  → HDR35
   hand → soft-start clamp → MultiDOFCommand(rad)         → /dg5f_left/lj_dg_pospid/reference → DG5F
```

## What it does
- Builds the 26-d state = `robot_joint(6)` + `gripper_joint(20)` (radians).
- POSTs frames + state to the server, receives the 26-d **absolute target** action.
- Splits into arm(6)/hand(20), applies **soft-start + rate limit** anchored to the measured
  current joints (kills the first-tick jump of absolute targets), clamps to joint limits,
  then publishes — only when the output gates are ON.
- Inference runs in a control thread so HTTP latency never blocks ROS callbacks.
- `~/reset` (`std_srvs/Trigger`) resets the policy queue at episode start.

## Build
This is an `ament_python` package. Copy/symlink it into your ROS2 workspace `src/` (e.g.
the `system_Teleop` workspace, which provides `system_interface` for the default
`frame_aligned` state source), then:

```bash
ln -s /home/bi/visuomotor-policy-inference/ros2_robot_client  <ws>/src/vpi_robot_client
cd <ws> && colcon build --packages-select vpi_robot_client && source install/setup.bash
pip install requests pillow            # runtime python deps (in the ROS python env)
```

## Run (staged bring-up — see EXECUTION_PLAN.md §3)
```bash
# 0) policy server on GPU
cd /home/bi/visuomotor-policy-inference && VPI_DEVICE=cuda uvicorn app.server:app --port 8000
#    (or: conda activate vpi && VPI_DEVICE=cuda uvicorn app.server:app --host 0.0.0.0 --port 8000)

# 1) INFER ONLY — never moves the robot; check it reaches the server & state/cameras flow
ros2 launch vpi_robot_client policy_control.launch.py enable_output:=false

# 3) REAL, conservative
ros2 launch vpi_robot_client policy_control.launch.py \
    enable_output:=true enable_gripper_output:=true \
    max_joint_delta:=0.02 max_gripper_delta:=0.02
```

## Key parameters
| param | default | note |
|---|---|---|
| `server_url` | `http://localhost:8000` | policy server |
| `image_format` | `JPEG` | JPEG ≈ 3× less bandwidth than PNG (BENCHMARK.md) |
| `fps` | `30.0` | run at the model's trained rate |
| `state_source` | `frame_aligned` | `frame_aligned` (needs `system_interface`) or `joint_states` |
| `enable_output` / `enable_gripper_output` | `false` | output gates — start OFF |
| `max_joint_delta` / `max_gripper_delta` | `0.02` rad/tick | soft-start + rate limit, 0=off |
| `arm_limit_min/max`, `gripper_limit_min/max` | `[]` | optional rad clamps (set from URDF) |
| `robot_topic_unit` | `deg` | HDR35 stream wants degrees (model outputs rad) |

### `joint_states` mode (no `system_interface` dependency)
```bash
ros2 launch vpi_robot_client policy_control.launch.py \
    state_source:=joint_states \
    arm_state_topic:=/system_left/joint_states arm_state_unit:=deg \
    gripper_state_topic:=/dg5f_left/joint_states gripper_state_unit:=rad
```

## ⚠️ Prerequisite
The vision node must **publish** the two camera topics (`.../camera/front/rgb`,
`.../camera/wrist/rgb`) — it currently records to disk only. See INTEGRATION_HDR35.md Step 1.

## Right arm
The default model is **left**. For the right arm, run a second server with the right model
and point a second node at it (`side:=right server_url:=http://localhost:8001`); the right
hand is Inspire (`gripper_command_type:=float64_multi_array`).
