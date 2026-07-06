#!/usr/bin/env python3
"""Relay the HDR35 joint state into the form MoveIt expects.

The hdr_stream bridge publishes ``/system_<side>/joint_states`` with positions in
**degrees** (and controller-specific names). MoveIt's planning scene monitor needs
``/joint_states`` in **radians** with the URDF joint names (j1..j6). This thin node
bridges the two so move_group always has a valid current robot state to plan from.

    ros2 run teleop_planning joint_state_relay --ros-args \
        -p in_topic:=/system_right/joint_states -p out_topic:=/joint_states
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [f"j{i}" for i in range(1, 7)]


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        self.declare_parameter("in_topic", "/system_right/joint_states")
        self.declare_parameter("out_topic", "/joint_states")
        self.declare_parameter("in_degrees", True)
        self.declare_parameter("joint_names", JOINT_NAMES)

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.in_degrees = bool(self.get_parameter("in_degrees").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.create_subscription(JointState, in_topic, self._cb, 10)
        self.get_logger().info(
            f"relaying {in_topic} ({'deg' if self.in_degrees else 'rad'}) -> "
            f"{out_topic} (rad, names={self.joint_names})")

    def _cb(self, msg: JointState):
        n = min(len(self.joint_names), len(msg.position))
        if n == 0:
            return
        scale = math.pi / 180.0 if self.in_degrees else 1.0
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names[:n]
        out.position = [float(msg.position[i]) * scale for i in range(n)]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
