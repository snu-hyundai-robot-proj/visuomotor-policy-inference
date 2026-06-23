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

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("enable_output", default_value="false"),
        DeclareLaunchArgument("fps", default_value="30.0"),
        DeclareLaunchArgument("device", default_value="cuda"),
        DeclareLaunchArgument("task", default_value=""),
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
            }],
        ),
    ])
