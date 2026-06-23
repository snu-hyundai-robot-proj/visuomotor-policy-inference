from . import snu_hdr_controller_py as hdr
import numpy as np


class DualHdrController():
    def __init__(self, urdf_path: str, node_name: str = "robot_control_node"):

        hdr.ros_init()
        self.node = hdr.Node(node_name)

        self.controller = hdr.DualHdrController(urdf_path, self.node) # rc = robot controller

        self.n_joint = 6
        self.n_robots = 2
        self.dof = self.n_joint * self.n_robots
        
        self.q = np.zeros(self.dof, dtype=np.float64)
        self.qd = np.zeros(self.dof, dtype=np.float64)        
        self.cmd = np.zeros(self.dof, dtype=np.float64)

        self.ft_node = self.controller.getFtNode()


    def run(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        self.q = q
        self.qd = qd

        self.controller.update(self.q, self.qd)
        self.controller.validate() 
        self.cmd = self.controller.write()

    def get_cmd(self):
        return self.cmd

    def is_timer_reset(self):
        return self.controller.getTimerReset()

    def ros_spin_once(self):
        hdr.ros_spin_once(self.node)
        hdr.ros_spin_once(self.ft_node)