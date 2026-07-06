#!/usr/bin/env python3
"""Bring up the standalone planning stack for the HDR35.

Starts: robot_state_publisher (HDR35 URDF) + move_group (hdr35_20_moveit_config) +
joint_state_relay (HDR35 deg -> /joint_states rad) + a static base_link->base TF +
the plan_to_hook node.

Planning-only by default (use_mock_hardware:=true, plan_to_hook allow_execute:=false), so
nothing touches the real arm until you opt in.

    ros2 launch teleop_planning planning_bringup.launch.py side:=right
    # real-state planning needs the HDR35 stream up so /system_<side>/joint_states exists:
    #   ros2 run hdr_stream hdr_stream_node --ros-args -p robot_side:=right -p simulation:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration("side")
    robot_model = LaunchConfiguration("robot_model")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    allow_execute = LaunchConfiguration("allow_execute")
    approach_dist_mm = LaunchConfiguration("approach_dist_mm")
    approach_axis = LaunchConfiguration("approach_axis")
    hook_pose_topic = LaunchConfiguration("hook_pose_topic")
    dry_run = LaunchConfiguration("dry_run")

    moveit_pkg = get_package_share_directory("hdr35_20_moveit_config")
    initial_positions = os.path.join(moveit_pkg, "config", "initial_positions.yaml")

    # Replicate the xacro invocation that move_group.launch.py uses, so rsp's
    # robot_description matches move_group's exactly.
    xacro_path = PathJoinSubstitution([FindPackageShare("hdr_description"), "urdf", "hdr.urdf.xacro"])
    robot_description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", xacro_path,
        " use_sim:=false",
        " use_mock_hardware:=", use_mock_hardware,
        " robot_model:=", robot_model,
        " name:=hdr_robot",
        " hdr_ros2_control:=",
        " initial_positions_file:=", initial_positions,
    ])}

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[robot_description],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg, "launch", "move_group.launch.py")),
        launch_arguments={
            # move_group.launch.py references LaunchConfiguration('hdr35_20_moveit_config')
            # (the package name) without declaring it, so we must pass it through.
            "hdr35_20_moveit_config": "hdr35_20_moveit_config",
            "robot_model": robot_model,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    joint_relay = Node(
        package="teleop_planning", executable="joint_state_relay", output="screen",
        parameters=[{
            "in_topic": ParameterValue(["/system_", side, "/joint_states"], value_type=str),
            "out_topic": "/joint_states",
            "in_degrees": True,
        }],
    )

    # hook pose frame ('base') == robot base ('base_link') for this rig.
    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base"],
    )

    plan_to_hook = Node(
        package="teleop_planning", executable="plan_to_hook", output="screen",
        parameters=[{
            "side": ParameterValue(side, value_type=str),
            "hook_pose_topic": ParameterValue(hook_pose_topic, value_type=str),
            "group": "hdr_manipulator",
            "ee_link": "tool0",
            "planning_frame": "base_link",
            "approach_dist_mm": ParameterValue(approach_dist_mm, value_type=float),
            "approach_axis": ParameterValue(approach_axis, value_type=str),
            "allow_execute": ParameterValue(allow_execute, value_type=bool),
        }],
    )

    # Execution bridge: FollowJointTrajectory -> HDR35 OpenStream. dry_run defaults true,
    # so wiring is present but the arm never moves until explicitly enabled.
    followjoint_bridge = Node(
        package="teleop_planning", executable="hdr_followjoint_bridge", output="screen",
        parameters=[{
            "side": ParameterValue(side, value_type=str),
            "dry_run": ParameterValue(dry_run, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="right"),
        DeclareLaunchArgument("robot_model", default_value="hdr35_20"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("allow_execute", default_value="false"),
        DeclareLaunchArgument("approach_dist_mm", default_value="100.0"),
        DeclareLaunchArgument("approach_axis", default_value="z"),
        DeclareLaunchArgument("hook_pose_topic", default_value="/hook_pose"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        rsp, move_group, joint_relay, static_tf, plan_to_hook, followjoint_bridge,
    ])
