from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side = LaunchConfiguration("side")
    policy_path = LaunchConfiguration("policy_path")
    enable_output = LaunchConfiguration("enable_output")
    fps = LaunchConfiguration("fps")
    device = LaunchConfiguration("device")
    task = LaunchConfiguration("task")
    enable_ruckig = LaunchConfiguration("enable_ruckig")

    # Per-joint jerk-limited smoothing limits applied to the action vector.
    # Indices 0..5 are the HDR35_20 arm (URDF velocity + cuRobo accel/jerk); the
    # remaining hand dims reuse the last entry (the node pads to the action dim).
    # These are real numeric lists (double[]) so ROS2 types them correctly — to
    # override, edit here or pass a params YAML (a CLI string would mistype them).
    # Tighten ruckig_max_jerk for smoother (but laggier) motion.
    ruckig_max_velocity = [3.141, 3.141, 3.316, 5.410, 5.410, 7.330, 3.0]
    ruckig_max_acceleration = [12.0]
    ruckig_max_jerk = [500.0]

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("enable_output", default_value="false"),
        DeclareLaunchArgument("fps", default_value="30.0"),
        DeclareLaunchArgument("device", default_value="cuda"),
        DeclareLaunchArgument("task", default_value=""),
        DeclareLaunchArgument("enable_ruckig", default_value="true"),
        Node(
            package="lerobot_system",
            executable="lerobot_system",
            name=["lerobot_system_", side],
            output="screen",
            parameters=[{
                "side": side,
                "policy_path": policy_path,
                "enable_output": enable_output,
                "fps": fps,
                "device": device,
                "task": task,
                "enable_ruckig": enable_ruckig,
                "ruckig_max_velocity": ruckig_max_velocity,
                "ruckig_max_acceleration": ruckig_max_acceleration,
                "ruckig_max_jerk": ruckig_max_jerk,
            }],
        ),
    ])
