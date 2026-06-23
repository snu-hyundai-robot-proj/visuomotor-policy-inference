import os, yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def load_file(package_name: str, relative_path: str) -> str:

    package_path = get_package_share_directory(package_name)
    abs_path = os.path.join(package_path, relative_path)

    with open(abs_path, "r", encoding="utf-8") as f:
        return f.read()


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    abs_path = os.path.join(package_path, relative_path)

    with open(abs_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    
    declare_arguments = []
    declare_arguments.append(
    DeclareLaunchArgument(
        "robot_side",
        default_value="left",
        description="Set Robot Side"
    )
    )
    
    return LaunchDescription(declare_arguments + [OpaqueFunction(function = make_nodes)])

def make_nodes(context, *args, **kwargs):
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("hdr_description"),
            "urdf",
            "robots",
            "hdr35_20",
            "hdr35_20.urdf.xacro"
    ])

    srdf_content = load_file("hdr35_20_moveit_config", "config/hdr35_20.srdf.xacro")

    kinematics_dict = load_yaml("hdr35_20_moveit_config", "config/kinematics.yaml")

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_xacro
        ])
    }
    
    # robot_description_semantic = {
    #     "robot_description_semantic": Command([
    #         FindExecutable(name="xacro"),
    #         " ",
    #         srdf_content,
    #         "name:=hdr_robot"
    #     ])
    # }
    # # robot_description_semantic = {
    # #     "robot_description_semantic": srdf_content
    # # }
    
    # robot_description_kinematics = {
    #     "robot_description_kinematics": kinematics_dict
    # }
    
    
    # robot_description = Command([
    #     "xacro ",
    #     PathJoinSubstitution([
    #         FindPackageShare("hdr_description"),
    #         "urdf",
    #         "robots",
    #         "hdr35_20",
    #         "hdr35_20.urdf.xacro"
    #     ]),
        # " name:=hdr_robot",
    # ])

    robot_description_semantic = {
        "robot_description_semantic":Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("hdr35_20_moveit_config"),
            "config",
            "hdr35_20.srdf.xacro",
        ]),
        " name:=hdr_robot",
    ])}

    robot_kinematics_path = os.path.join(
        get_package_share_directory("hdr35_20_moveit_config"),
        "config","kinematics.yaml"
    )

    with open(robot_kinematics_path, "r") as f:
        robot_kinematics = yaml.safe_load(f)
    
    robot_side = LaunchConfiguration("robot_side")
    
    ### create Node
    robot_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description]
        )

    robot_stream_ndoe = Node(
            package="hdr_ros2_driver",
            executable="hdr_moveit_node",
            name="hdr_moveit_node",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_kinematics,
                {"robot_side": robot_side}
            ]
        )
    return [robot_publisher, robot_stream_ndoe]