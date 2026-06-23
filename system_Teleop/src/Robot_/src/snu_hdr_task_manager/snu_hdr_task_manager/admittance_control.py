#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from snu_hdr_msgs.action import Admittance
from .template.admittance_sm import AdmittanceSM

import numpy as np


def main():
    rclpy.init()
    node = Node('admittance_sm_node')

    mass = [3.0, 3.0, 3.0, 0.5, 0.5, 0.5]    
    stiff = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    adm_axis = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    zeta = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    adm_dt = 0.005


    sm = AdmittanceSM(node, mass, stiff, adm_axis, zeta, adm_dt, ['left', 'right'])

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