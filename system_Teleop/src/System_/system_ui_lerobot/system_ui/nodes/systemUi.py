#!/usr/bin/env python3
import sys

from system_interface.msg import *
from system_ui.panels.ui_panels import SimpleGroupPanel
from system_ui.nodes.node_manager import NodeManager

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt, QTimer, QThread

import rclpy
import signal
import threading

from rclpy.node import Node

class SystemNode(Node):
    def __init__(self, node_name: str = "system_ui_node"):
        super().__init__(node_name)

        self.get_logger().info(f"{node_name} started")


        self.ui_left_pub = self.create_publisher(
            UiCommand,
            '/system_left/ui_command',
            10)

        self.ui_right_pub = self.create_publisher(
            UiCommand,
            '/system_right/ui_command',
            10)

        self.node_manager = NodeManager(self)

    def request_left(self, command: int, value: int) -> int:
        req = UiCommand()
        req.command = int(command)
        req.value = int(value)

        self.ui_left_pub.publish(req)

    def request_right(self, command: int, value: int) -> int:
        req = UiCommand()
        req.command = int(command)
        req.value = int(value)

        self.ui_right_pub.publish(req)
 
    def send_command(self, side = str, command = int):
        if not isinstance(side, str):
            return self.get_logger().error(f"send : 1st arg isn't string")

        if not isinstance(command, int):
            return self.get_logger().error(f"send : 2nd arg isn't int")

        if side == "left":
            return self.get_logger().info(f"Left Command : {command}")
        elif side == "right":
            return self.get_logger().info(f"Right Command : {command}")

class MainWindow(QMainWindow):
    def __init__(self, ros_node: SystemNode):
        super().__init__()
        self._node = ros_node

        self.setWindowTitle("LeRobot System Manager")

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout()
        central.setLayout(main_layout)

        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        line = QFrame()
        line.setFrameShape(QFrame.VLine)      # 세로선
        line.setFrameShadow(QFrame.Sunken)    # 입체감
        line.setLineWidth(1)
        
        main_layout.addLayout(left_layout, 1)   # left 1
        main_layout.addWidget(line)             # 중앙 구분선
        main_layout.addLayout(right_layout, 2)  # right 2 (예: 로그가 더 넓게)
        
        self.list_left_panels = self.make_left_panels()
        self.list_right_panels = self.make_right_panels()
        self.left_lerobot_panel = self.list_left_panels[-1]
        self.right_lerobot_panel = self.list_right_panels[-1]
        
        for i in self.list_left_panels:
            i.sig_clicked.connect(self.btn_clicked)
            left_layout.addWidget(i,alignment=Qt.AlignLeft)

        for i in self.list_right_panels:
            i.sig_clicked.connect(self.btn_clicked)
            right_layout.addWidget(i,alignment=Qt.AlignRight)

        self.setCentralWidget(central)
        self.start_update()

    def make_left_panels(self):
        spawn_node_panel = SimpleGroupPanel(
            title="Left Robot",
            button_names=["Connect", "Stop"],
            use_line_edit=False,
            resize=(300, 90)
        )

        connect_panel = SimpleGroupPanel(
            title="Left Vision",
            button_names=["Connect", "Stop"],
            use_line_edit=False,
            resize=(300, 90)
        )

        lerobot_panel = SimpleGroupPanel(
            title="Left LeRobot",
            button_names=["Inference", "Mode: Execute Action", "Init Pose", "Stop"],
            line_text="src/Lerobot_/lerobot/outputs/train/dg5f_diffusion/checkpoints/100000/pretrained_model",
            resize=(360, 180)
        )

        return [spawn_node_panel, connect_panel, lerobot_panel]

    def make_right_panels(self):
        spawn_node_panel = SimpleGroupPanel(
            title="Right Robot",
            button_names=["Connect", "Stop"],
            use_line_edit=False,
            resize=(300, 90)
        )

        connect_panel = SimpleGroupPanel(
            title="Right Vision",
            button_names=["Connect", "Stop"],
            use_line_edit=False,
            resize=(300, 90)
        )

        lerobot_panel = SimpleGroupPanel(
            title="Right LeRobot",
            button_names=["Inference", "Mode: Execute Action", "Init Pose", "Stop"],
            line_text="src/Lerobot_/lerobot/outputs/train/rh56f1_diffusion/checkpoints/100000/pretrained_model",
            resize=(360, 180)
        )

        return [spawn_node_panel, connect_panel, lerobot_panel]

    def btn_clicked(self, title: str, button_name: str, text: str):
        self._node.node_manager.recv_data = {"title": title, "button_name": button_name, "text": text}

        if title.startswith("Left"):
            result = self._node.node_manager.excute_command("Left")
            if button_name.startswith("Mode:") and isinstance(result, str):
                self.left_lerobot_panel.set_button_text(button_name, result)
        elif title.startswith("Right"):
            result = self._node.node_manager.excute_command("Right")
            if button_name.startswith("Mode:") and isinstance(result, str):
                self.right_lerobot_panel.set_button_text(button_name, result)
        else:
            self._node.get_logger().info("branch Error")

    def start_update(self):
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_datas)
        self.timer.start()

    def update_datas(self):
        return

    def clean_process(self):
        self._node.node_manager.stop_threading()
        self._node.node_manager.shutdown_()

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    node = SystemNode(node_name="system_ui_node")
    w = MainWindow(node)
    
    w.show()

    timer = QTimer()
    timer.setInterval(10)
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start()

    def handle_interrupt(sig, frame):
        node.get_logger().info(f"Keyborad Interrupt : Ctrl + C")

        timer.stop()
        w.clean_process()
        node.destroy_node()
        rclpy.shutdown()
        app.quit()

    signal.signal(signal.SIGINT, handle_interrupt)

    exit_code = app.exec_()

    timer.stop()
    w.clean_process()

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

    return exit_code