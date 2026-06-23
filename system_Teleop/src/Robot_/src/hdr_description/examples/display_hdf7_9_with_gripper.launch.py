#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


GRIPPER_CONFIG = {
    'robotiq_2f_85': {
        'xacro_file': 'hdf7_9_robotiq_2f_85.urdf.xacro',
        'robot_name': 'hdf7_9_robotiq_2f_85',
    },
    'robotiq_2f_140': {
        'xacro_file': 'hdf7_9_robotiq_2f_140.urdf.xacro',
        'robot_name': 'hdf7_9_robotiq_2f_140',
    },
    'onrobot_rg2': {
        'xacro_file': 'hdf7_9_onrobot_rg2.urdf.xacro',
        'robot_name': 'hdf7_9_onrobot_rg2',
    },
    'onrobot_rg6': {
        'xacro_file': 'hdf7_9_onrobot_rg6.urdf.xacro',
        'robot_name': 'hdf7_9_onrobot_rg6',
    },
}


def launch_setup(context, *args, **kwargs):
    """Set up and return the nodes based on gripper type selection."""
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    description_package = LaunchConfiguration('description_package')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    if gripper_type not in GRIPPER_CONFIG:
        raise ValueError(f"Unknown gripper type: {gripper_type}. "
                        f"Available types: {list(GRIPPER_CONFIG.keys())}")

    config = GRIPPER_CONFIG[gripper_type]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(description_package),
            'examples',
            config['xacro_file']
        ]),
        ' use_mock_hardware:=', use_mock_hardware,
        ' name:=', config['robot_name'],
    ])

    robot_description = {'robot_description': robot_description_content}

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(description_package),
        'rviz',
        'display_robot.rviz',
    ])

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
    )

    return [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description():
    """Generate launch description for HDF7_9 + gripper visualization."""
    declared_arguments = [
        DeclareLaunchArgument(
            'gripper_type',
            default_value='robotiq_2f_85',
            choices=['robotiq_2f_85', 'robotiq_2f_140', 'onrobot_rg2', 'onrobot_rg6'],
            description='Type of gripper to attach to HDF7_9 robot.',
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='hdr_description',
            description='Package containing robot URDF/XACRO files.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description='Use mock hardware for visualization.',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
