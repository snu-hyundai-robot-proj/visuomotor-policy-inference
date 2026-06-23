#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smach
from smach import StateMachine
from smach_ros import SimpleActionState

from sensor_msgs.msg import JointState
from snu_hdr_msgs.action import JointMove

import numpy as np

class JointMoveSM(StateMachine):
    def __init__(self, node, target_pose, arm_name=(['left'])):
        super().__init__(outcomes=['succeeded', 'aborted', 'preempted'])
           
        target_q_list = []
        for i, _ in enumerate(arm_name):
            js = JointState()
            start = i * 6
            end = (i + 1) * 6
            js.position = target_pose[start:end]
            target_q_list.append(js)
            
    
        self.userdata.joint_move_goal = JointMove.Goal()
        self.userdata.joint_move_goal.arm_names = arm_name
        self.userdata.joint_move_goal.execution_time = 15.0
        self.userdata.joint_move_goal.target_q = target_q_list

        @smach.cb_interface(input_keys=['target_action'])
        def goal_cb_with_error_check(ud, _goal):            
            return ud.target_action
        
        with self:
            StateMachine.add('JOINT_MOVE',
                SimpleActionState(node, "/snu_hdr_controller/joint_move_control", JointMove, goal_cb=goal_cb_with_error_check),
                remapping={'target_action': 'joint_move_goal'},
                transitions={'succeeded': 'succeeded','aborted': 'aborted'}
            )
