#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import smach
from smach import StateMachine
from smach_ros import SimpleActionState, SmachNode
from rclpy.executors import MultiThreadedExecutor
import threading

from sensor_msgs.msg import JointState
from snu_hdr_msgs.action import JointMove
from .template.joint_move_sm import JointMoveSM

import numpy as np


def main():
    rclpy.init()
    node = Node('joint_move_sm_node')

    # 예시 target_poses: [x,y,z,qx,qy,qz,qw] 리스트들의 리스트
    # target_poses =  [0.0, 0.0, 0.0, 0.0, 0.0, -1*np.pi/180]
    
    # left_target =  [-np.pi/3, np.pi/2, 0.0, 0.0, -np.pi/2, np.pi/2]
    # right_target =  [np.pi/3, np.pi/2, 0.0, 0.0, -np.pi/2, np.pi/2]
    
    left_target =  [-np.pi/2, np.pi/2, 0.0, 0.0, -np.pi/2, np.pi]
    right_target =  [np.pi/2, np.pi/2, 0.0, 0.0, -np.pi/2, np.pi/2]


    target_poses = np.concatenate([left_target, right_target])

    sm = JointMoveSM(node, target_poses, ['left', 'right'])
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()
    
    sm.execute()    
    
    executor.shutdown()
    t.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()