from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declare_arguments = []
    declare_arguments.append(
    DeclareLaunchArgument(
        "simulation",
        default_value="true",
        description="simulation mode"
    )
    )

    simulation = LaunchConfiguration("simulation")

    tracker_core_node = Node(
        package="vive_tracker_core",
        executable="tracker_core",
        output="screen",
    )

    tracker_bridge_node = Node(
        package="vive_tracker_bridge",
        executable="tracker_bridge_node",
        output="screen",
        parameters=[{"simulation" : simulation}],
    )

    return LaunchDescription(declare_arguments + [tracker_core_node,tracker_bridge_node])