"""Launch the Episode Manager.

Brings up the orchestrator with safe defaults. Set arm_home/hand_home to your real init
pose (ideally extracted from the dataset's episode-start state). The policy control node
(vpi_robot_client) and the robot drivers must be running separately.

Example:
  ros2 launch episode_manager episode.launch.py \
      arm_home:="[0.0,-0.3,1.2,0.0,1.0,0.0]" auto_start:=false episodes:=-1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        "side": "left",
        "state_source": "frame_aligned",
        "server_url": "http://localhost:8000",
        "auto_start": "false",
        "episodes": "-1",
        "max_duration_sec": "20.0",
        "home_max_delta": "0.02",
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    node = Node(
        package="episode_manager",
        executable="episode_manager",
        name="episode_manager",
        output="screen",
        parameters=[{
            "side": LaunchConfiguration("side"),
            "state_source": LaunchConfiguration("state_source"),
            "server_url": LaunchConfiguration("server_url"),
            "auto_start": LaunchConfiguration("auto_start"),
            "episodes": LaunchConfiguration("episodes"),
            "max_duration_sec": LaunchConfiguration("max_duration_sec"),
            "home_max_delta": LaunchConfiguration("home_max_delta"),

            # init pose (rad) — REPLACE with your real home (e.g. dataset episode-start mean)
            "arm_home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "hand_home": [0.0] * 20,

            # outputs (same topics the policy uses)
            "robot_action_topic": "/robot/joint_target_deg",
            "robot_topic_unit": "deg",
            "gripper_action_topic": "/dg5f_left/lj_dg_pospid/reference",
            "gripper_command_type": "multi_dof_command",

            # cameras (for health)
            "front_camera_topic": "/system_left/camera/front/rgb",
            "wrist_camera_topic": "/system_left/camera/wrist/rgb",

            # policy node hooks
            "policy_enable_service": "/vpi/set_enable",
            "policy_reset_service": "/vpi_policy_control/reset",
        }],
    )
    return LaunchDescription(decls + [node])
