# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, TimerAction

# def generate_launch_description():
#     pkg = "net_ft_driver"
#     exe = "axia_reader"
    
#     # ft sensor on the right hand
#     ft_rh = Node(
#         package=pkg,
#         executable=exe,
#         name="axia_reader_rh",
#         output="screen",
#         parameters=[{
#             "sensor_type": "ati_axia",
#             "ip": "192.168.4.21",
#             "sampling_rate": 500,
#             "internal_filter": 4,
#             "topic": "/axia_right",
#         }],
#     )

#     # ft sensor on the left hand
#     ft_lh = Node(
#         package=pkg,
#         executable=exe,
#         name="axia_reader_lh",
#         output="screen",
#         parameters=[{
#             "sensor_type": "ati_axia",
#             "ip": "192.168.4.22",
#             "sampling_rate": 500,
#             "internal_filter": 4,
#             "topic": "/axia_left",
#         }],
#     )

#     set_zero_left = ExecuteProcess(
#         cmd=[
#             'ros2', 'topic', 'pub',
#             '--once',
#             '/set_zero/axia_left',
#             'std_msgs/msg/Bool',
#             '{data: true}'
#         ],
#         output='screen'
#     )

#     set_zero_right = ExecuteProcess(
#         cmd=[
#             'ros2', 'topic', 'pub',
#             '--once',
#             '/set_zero/axia_right',
#             'std_msgs/msg/Bool',
#             '{data: true}'
#         ],
#         output='screen'
#     )

#     return LaunchDescription([ft_rh, ft_lh, TimerAction(
#             period=2.0,
#             actions=[set_zero_left, set_zero_right]
#         )
#     ])


# dual_ft_launch_with_zero.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    pkg = "net_ft_driver"
    exe = "dual_axia_reader"

    # dual FT reader node
    ft_node = Node(
        package=pkg,
        executable=exe,
        name="dual_ft_reader",
        output="screen",
        parameters=[{
            # Sensor 1
            "sensor_type1": "ati_axia",
            "ip1": "192.168.4.22",
            "sampling_rate1": 500,
            # Sensor 2
            "sensor_type2": "ati_axia",
            "ip2": "192.168.4.21",
            "sampling_rate2": 500,
            # Common
            "internal_filter": 4,
            "topic": "/ft_combined",
        }],
    )

    # set_zero topic for dual_ft_reader
    set_zero = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--once',
            '/set_zero/ft_combined',   # dual_ft_reader에서 사용하는 set_zero 토픽
            'std_msgs/msg/Bool',
            '{data: true}'
        ],
        output='screen'
    )

    # TimerAction으로 2초 뒤 set_zero 발행
    zero_timer = TimerAction(
        period=2.0,
        actions=[set_zero]
    )

    return LaunchDescription([ft_node, zero_timer])