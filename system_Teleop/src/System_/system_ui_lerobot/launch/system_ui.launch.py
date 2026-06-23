from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package="system_ui",
			executable="system_ui_node",
			name="system_ui_node",
			output="screen",
		),
	])
