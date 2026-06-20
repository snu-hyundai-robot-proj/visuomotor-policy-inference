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

            # init pose (rad) — from the HF model repo's hand_init_pose.json (mean of
            # episode-start observation.state over all episodes). LEFT shown; for RIGHT use
            # arm [1.737826,1.765172,-0.192581,0.688188,-1.419986,0.320249] and the Inspire
            # 6-DOF hand_init (first 6 of [2.89855,2.878596,2.811947,2.807127,2.324232,2.490377]).
            "arm_home": [-1.734299, 1.679489, -0.113224, -0.657517, -1.625038, -0.202079],
            "hand_home": [-0.174693, 0.088572, -0.171882, -0.064431, -0.344696, 0.039543,
                          0.383013, 0.300303, -0.401026, 0.188575, 0.13543, 0.108224,
                          -0.372288, 0.453359, 0.029937, 0.037518, 0.000573, -0.427912,
                          0.54842, 0.033721],

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

            # replay hooks (frontend REPLAY button -> /episode/replay -> these services)
            "replay_start_service": "/episode_image_publisher/start",
            "replay_stop_service": "/episode_image_publisher/stop",
        }],
    )

    # recorded-image replay source (managed: autostart=false; Episode Manager starts/stops it)
    replay = Node(
        package="vpi_robot_client",
        executable="episode_image_publisher",
        name="episode_image_publisher",
        output="screen",
        parameters=[{
            "side": LaunchConfiguration("side"),
            "episode_dir": "/home/bi/visuomotor-policy-inference/examples/sample_episodes/left",
            "front_topic": "/system_left/camera/front/rgb",
            "wrist_topic": "/system_left/camera/wrist/rgb",
            "fps": 30.0,
            "loop": False,
            "autostart": False,     # wait for the manager (REPLAY button)
        }],
    )
    return LaunchDescription(decls + [node, replay])
