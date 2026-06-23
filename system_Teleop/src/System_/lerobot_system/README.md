# lerobot_system

ROS2 Python bridge that runs a LeRobot policy without modifying the existing
`system_left`, `system_right`, vision, or robot stream packages.

## Data Flow

- Input state: `/system_<side>/frame_aligned_state`
- Optional input cameras: configured by `camera_topics` and `camera_keys`
- Policy input state key: `observation.state`
- Raw action debug: `/lerobot/<side>/raw_action`
- Robot joint target: `/lerobot/<side>/joint_target`
- Optional gripper output:
  - left: `/dg5f_left/lj_dg_pospid/reference`
  - right: `/inspire/right/target`

The default robot joint output is side-separated. If you want to drive the
existing `hdr_stream` node directly, either remap the topic or run that node
with its `target_joint_topic` parameter pointing to the side-specific topic.

## Examples

Dry-run with a mock policy:

```bash
ros2 run system_left system_left --ros-args \
  -p record_period:=10 \
  -p publish_state_without_record:=true

ros2 run lerobot_system lerobot_system_left --ros-args \
  -p mock_policy:=true \
  -p enable_output:=false
```

Run a real policy and publish robot actions:

```bash
ros2 run lerobot_system lerobot_system_left --ros-args \
  -p policy_path:=/path/to/pretrained_model \
  -p task:="pick up the hook" \
  -p enable_output:=true \
  -p robot_action_topic:=/lerobot/left/joint_target
```

Add camera inputs:

```bash
ros2 run lerobot_system lerobot_system_right --ros-args \
  -p policy_path:=/path/to/pretrained_model \
  -p camera_topics:="['/right/wrist/image_raw']" \
  -p camera_keys:="['observation.images.wrist']"
```

The current ACT/Diffusion configs supported by this bridge follow the LeRobot feature names from
the policy config. The two FlowMatch+DINOv2 multicam checkpoints use:

| Checkpoint | Side/hand | State | Action | Image keys |
|---|---:|---:|---:|---|
| `Ngseo/dg5f_diffusion_dinov2s_flowmatch_multicam_dr` | left / DG5F | 163 | 26 | `observation.images.d405`, `observation.images.zivid` |
| `Ngseo/rh56f1_diffusion_dinov2s_flowmatch_multicam_dr` | right / RH56F1 | 141 | 12 | `observation.images.d405`, `observation.images.zivid` |

Images are resized to the config shape, currently `240x320`, and converted to channel-first
float tensors in `[0, 1]` before the LeRobot preprocessor. The state vector is built from the
available `FrameAlignedState` fields and padded/truncated to the policy's configured state dim.

Important parameters:

- `side`: `left` or `right`
- `state_fields`: fields from `FrameAlignedState` concatenated into `observation.state`
- `action_output_unit`: unit produced by policy, default `rad`
- `robot_topic_unit`: unit expected by robot topic, default `deg`
- `enable_output`: false by default; raw action debug is still published
- `max_joint_delta`: optional per-step joint clamp in the robot topic unit
- `enable_gripper_output`: publish action slice to the configured gripper topic

## One-Side Launcher

From the workspace root:

```bash
SIDE=left POLICY_PATH=/path/to/pretrained_model bash scripts/run_lerobot_side.sh
```

The launcher starts a tmux session with the selected side's robot stream,
camera node, sensor/gripper input nodes, `system_<side>`, and
`lerobot_system_<side>`.
