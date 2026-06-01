"""Launch the visuomotor policy -> HDR35/DG5F control client.

SAFE DEFAULTS: outputs are OFF and soft-start deltas are conservative. Bring up in
stages (see EXECUTION_PLAN.md):
  1) infer only        : enable_output:=false  (watch raw actions, never moves the robot)
  2) sim               : run with HDR35 simulation + dg5f sim
  3) real, conservative: enable_output:=true enable_gripper_output:=true (small deltas)
  4) normal            : raise max_*_delta

Example (real, conservative):
  ros2 launch vpi_robot_client policy_control.launch.py \
      enable_output:=true enable_gripper_output:=true server_url:=http://localhost:8000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        "side": "left",
        "server_url": "http://localhost:8000",
        "image_format": "JPEG",
        "fps": "30.0",
        "state_source": "frame_aligned",
        "enable_output": "false",
        "enable_gripper_output": "false",
        "max_joint_delta": "0.02",
        "max_gripper_delta": "0.02",
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    node = Node(
        package="vpi_robot_client",
        executable="policy_control",
        name="vpi_policy_control",
        output="screen",
        parameters=[{
            "side": LaunchConfiguration("side"),
            "server_url": LaunchConfiguration("server_url"),
            "image_format": LaunchConfiguration("image_format"),
            "fps": LaunchConfiguration("fps"),
            "state_source": LaunchConfiguration("state_source"),

            # action split: arm[0:6] + hand[6:26]
            "arm_action_start": 0, "arm_action_size": 6,
            "gripper_action_start": 6, "gripper_action_size": 20,

            # arm output -> HDR35 (model rad -> deg for /robot/joint_target_deg)
            "robot_action_topic": "/robot/joint_target_deg",
            "robot_topic_unit": "deg",
            "enable_output": LaunchConfiguration("enable_output"),
            "max_joint_delta": LaunchConfiguration("max_joint_delta"),

            # gripper output -> DG5F pospid (rad)
            "gripper_action_topic": "/dg5f_left/lj_dg_pospid/reference",
            "gripper_command_type": "multi_dof_command",
            "enable_gripper_output": LaunchConfiguration("enable_gripper_output"),
            "max_gripper_delta": LaunchConfiguration("max_gripper_delta"),

            # cameras (publish these from the vision node — see INTEGRATION_HDR35.md Step 1)
            "front_camera_topic": "/system_left/camera/front/rgb",
            "wrist_camera_topic": "/system_left/camera/wrist/rgb",
            "camera_timeout_sec": 0.5,
        }],
    )
    return LaunchDescription(decls + [node])
