#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smach
from smach import StateMachine
from smach_ros import SimpleActionState

from snu_hdr_msgs.action import Admittance

import numpy as np

class AdmittanceSM(StateMachine):
    def __init__(self, node, mass, stiff, adm_axis, zeta,  adm_dt, arm_name=(['left'])):
        super().__init__(outcomes=['succeeded', 'aborted', 'preempted'])

        self.userdata.admittance_goal = Admittance.Goal()
        self.userdata.admittance_goal.arm_names = arm_name
        self.userdata.admittance_goal.mass = mass
        self.userdata.admittance_goal.stiff = stiff
        self.userdata.admittance_goal.adm_axis = adm_axis
        self.userdata.admittance_goal.zeta = zeta
        self.userdata.admittance_goal.adm_dt = adm_dt

        @smach.cb_interface(input_keys=['target_action'])
        def goal_cb_with_error_check(ud, _goal):            
            return ud.target_action

        with self:
            StateMachine.add('ADMITTANCE',
                SimpleActionState(node, "/snu_hdr_controller/admittance_control", Admittance, goal_cb=goal_cb_with_error_check),
                remapping={'target_action': 'admittance_goal'},
                transitions={'succeeded': 'succeeded','aborted': 'aborted'}
            )
