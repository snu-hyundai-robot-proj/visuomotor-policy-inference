#!/usr/bin/env python3
import sys

from system_interface.msg import *
from system_interface.srv import *
from system_ui.panels.ui_panels import SimpleGroupPanel
from system_ui.spawn.process_spawner import RosProcessSpawner
from system_ui.nodes.node_manager import NodeManager

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import Qt, QTimer, QThread

import rclpy
import signal
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

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
        
    def connect_server(self):
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /system_command ...")
        self.get_logger().info("Connected server")

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

        self.setWindowTitle("System Teleoperation")

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
        
        for i in self.list_left_panels:
            i.sig_clicked.connect(self.btn_clicked)
            left_layout.addWidget(i,alignment=Qt.AlignLeft)

        for i in self.list_right_panels:
            i.sig_clicked.connect(self.btn_clicked)
            right_layout.addWidget(i,alignment=Qt.AlignRight)

        # central.setLayout(layout)

        self.setCentralWidget(central)
        self.start_update()

    def make_left_panels(self):
        spawn_node_panel = SimpleGroupPanel(
            title = "Left Robot System",
            button_names = ["Connect", "Active", "Stop"],
            line_text="192.168.4.152",
            resize = (300,100)
        )

        connect_panel = SimpleGroupPanel(
            title="Left Gripper System",
            button_names=["Connect", "Teleop", "Stop"],
            line_text="192.168.4.73",
            resize = (300,100)
        )

        teleop_panel = SimpleGroupPanel(
            title="Left Teleop",
            button_names=["Calibrate","Start", "Stop"],
            use_line_edit = False,
            resize = (300,100)
        )

        record_panel = SimpleGroupPanel(
            title="Left Record",
            button_names=["Record", "Start", "Stop", "Play"],
            line_text = "Record/",
            resize = (300,100)
        )

        select_side = SimpleGroupPanel(
            title = "Left Switch",
            button_names= ["SET Left"],
            button_size= (250,150),
            use_button_size=True,
            use_line_edit = False,
            resize = (300,200)
        )
        # return [connect_panel, spawn_node_panel, teleop_panel, record_panel,select_side]
        return [connect_panel, spawn_node_panel, select_side]

    def make_right_panels(self):
        spawn_node_panel = SimpleGroupPanel(
            title = "Right Robot System",
            button_names = ["Connect", "Active", "Stop"],
            line_text="192.168.4.151",
            resize = (300,100)
        )

        connect_panel = SimpleGroupPanel(
            title="Right Gripper System",
            button_names=["Connect", "Teleop", "Stop"],
            line_text="192.168.4.151",
            use_line_edit = False,
            resize = (300,100)
        )

        teleop_panel = SimpleGroupPanel(
            title="Right Teleop",
            button_names=["Calibrate","Start", "Stop"],
            use_line_edit = False,
            resize = (300,100)
        )

        record_panel = SimpleGroupPanel(
            title="Right Record",
            button_names=["Record", "Start", "Stop"],
            line_text = "Record/",
            resize = (300,100)
        )
        select_side = SimpleGroupPanel(
            title = "Right Switch",
            button_names= ["SET Right"],
            button_size= (250,150),
            use_button_size=True,
            use_line_edit = False,
            resize = (300,200)
        )

        return [connect_panel, spawn_node_panel, select_side]
        # return [connect_panel, spawn_node_panel, teleop_panel, record_panel, select_side]

    def btn_clicked(self, title: str, button_name: str, text: str):
        # self._node.get_logger().info(f"UI title: {title}, clicked: {button_name}, text={text}")
        self._node.node_manager.recv_data = {"title": title,"button_name":button_name, "text":text}
        
        if "Left" in title:
            # self._node.get_logger().info("Left insert")
            self._node.node_manager.excute_command("Left")
        elif "Right" in title:
            # self._node.get_logger().info("Right insert")
            self._node.node_manager.excute_command("Right")
        else:
            self._node.get_logger().info("branch Error")

    def start_update(self):
        self.timer = QTimer()
        self.timer.setInterval(100)  # 20ms = 50Hz (상황에 따라 10~50ms 추천)
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