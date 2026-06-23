#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import argparse
import time
import rclpy
from rclpy.node import Node

from vive_tracker_core.track import ViveTrackerModule  # 기존 모듈 사용
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # 최신 1개만 유지
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

def parse_arguments(argv):
    parser = argparse.ArgumentParser(description="Vive Tracker Core (Python) -> publish Euler raw")
    parser.add_argument("-f", "--frequency", type=float, default=200.0, help="Publish frequency [Hz]")
    args, _ = parser.parse_known_args(argv)
    return args

class ViveTrackerCoreNode(Node):
    def __init__(self, hz: float):
        super().__init__('tracker_core_node', namespace='tracker')

        self.pub_ = self.create_publisher(Float64MultiArray, 'tracker_euler', 1)

        self.v_tracker = ViveTrackerModule()
        self.v_tracker.print_discovered_objects()

        self.t1 = self.v_tracker.devices.get("tracker_1", None)
        self.t2 = self.v_tracker.devices.get("tracker_2", None)

        if self.t1 is None:
            self.get_logger().warn("tracker_1 not found")
        if self.t2 is None:
            self.get_logger().warn("tracker_2 not found")
            
        # while True:
        #     self.v_tracker = ViveTrackerModule()
        #     self.v_tracker.print_discovered_objects()

        #     self.t1 = self.v_tracker.devices.get("tracker_1", None)
        #     self.t2 = self.v_tracker.devices.get("tracker_2", None)

        #     if self.t1 is None:
        #         self.get_logger().warn("tracker_1 not found")
        #     if self.t2 is None:
        #         self.get_logger().warn("tracker_2 not found")    
            
        #     if self.t1 and self.t2:
        #         self.get_logger().warn("#######################\n" + 
        #                                "   TRACKER is Ready\n" +
        #                                "#######################\n")    
        #         break
            # time.sleep(1)
            
        if hz <= 0.0:
            hz = 30.0
        self.dt = 1.0 / hz
        self.timer_ = self.create_timer(self.dt, self.on_timer)

        # self.get_logger().info(f"Publishing /vive/tracker_euler at {hz:.2f} Hz")

    def _flatten_3x4(self, m):
        return [float(m[r][c]) for r in range(3) for c in range(4)]

    def on_timer(self):
        m1 = [[0.0]*4 for _ in range(3)]
        m2 = [[0.0]*4 for _ in range(3)]
        
        # if self.t1 is None or self.t2 is None:
        #     self.get_logger().info(f"No data")

        #     msg = Float64MultiArray()
        #     msg.data = self._flatten_3x4(m1) + self._flatten_3x4(m2)  # 총 24개
        #     self.pub_.publish(msg)
        #     return

        if self.t1 is None:
            m1 = None
        else:
            m1 = self.t1.get_pose_matrix()
        
        if self.t2 is None:
            m2 = None
        else:
            m2 = self.t2.get_pose_matrix()

        if m1 is None:
            m1 = [[0.0]*4 for _ in range(3)]
            if m2 is None:
                m2 = [[0.0]*4 for _ in range(3)]
            else:
                if len(m2) != 3 or any(len(row) != 4 for row in m2):
                    m2 = [[0.0]*4 for _ in range(3)]

        if m2 is None:
            m2 = [[0.0]*4 for _ in range(3)]
            if m1 is None:
                m1 = [[0.0]*4 for _ in range(3)]
            else:
                if len(m1) != 3 or any(len(row) != 4 for row in m1):
                    m1 = [[0.0]*4 for _ in range(3)]
        
        msg = Float64MultiArray()
        msg.data = self._flatten_3x4(m1) + self._flatten_3x4(m2)  # 총 24개

        self.pub_.publish(msg)

def main():
    args = parse_arguments(sys.argv[1:])
    rclpy.init(args=sys.argv)
    node = ViveTrackerCoreNode(hz=args.frequency)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()