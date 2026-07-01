import sys
import rclpy
from rclpy.node import Node
from typing import Callable, Dict

from system_ui.spawn.process_manager import ProcessManager, ProcSpec

from pymodbus.client import ModbusSerialClient
from PyQt5.QtCore import QObject, pyqtSignal, QThread

import threading
import time

from pathlib import Path

def find_workspace_setup() -> str:
    current = Path(__file__).resolve()                  

    for parent in [current.parent] + list(current.parents):
        src_dir = parent / "src"
        install_setup = parent / "install" / "setup.bash"

        if src_dir.exists() and install_setup.exists():
            return str(install_setup)

    raise FileNotFoundError("workspace install/setup.bash not found")

class NodeManager(QObject):
    sig_calib = pyqtSignal()
    sig_start = pyqtSignal()
    sig_stop  = pyqtSignal()
    
    def __init__(self, ros_node : Node):
        super().__init__()

        self.node_ = ros_node
        self.recv_data: dict[str, str] = {}

        self.process_list = {
                                ## LEFT Process
                             "LeftSystemOn"         : "Left System",
                             "LeftSystemRecord"     : "Left Recorder",
                             "ConnectLeftGripper"   : "Left Delto Controller",
                             "LeftGripperControl"   : "Left Delto Retarget",
                             "LeftRobot"            : "Left Robot Controller",
                             "LeftServo"            : "Left IK Solver",

                                ## RIGHT Process
                             "RightSystemOn"        : "Right System",
                             "RightSystemRecord"    : "Right Recorder",
                             "ConnectRightGripper"  : "Right Inspire Controller",
                             "RightGripperControl"  : "Right Inspire Retarget",
                             "RightRobot"           : "Right Robot Controller",
                             "RightServo"           : "Right IK Solver",

                                ## common
                             "Manus"                : "Manus Core",
                             "Tracker"              : "Tracker Core",
                             "FTSensor"             : "FT Sensor",
                             "Left Camera"          : "Left Cam",
                             "Right Camera"         : "Right Cam",}

        self.pm = ProcessManager()

        self.pm.sig_log.connect(self.print_log)
        self.pm.sig_state.connect(self.print_state)
        self.pm.sig_exited.connect(self.print_err)
import shlex
import sys
from pathlib import Path

from PyQt5.QtCore import QObject

from rclpy.node import Node

from system_interface.msg import UiCommand
from system_ui.spawn.process_manager import ProcessManager, ProcSpec


def find_workspace_setup() -> str:
    current = Path(__file__).resolve()

    for parent in [current.parent] + list(current.parents):
        src_dir = parent / "src"
        install_setup = parent / "install" / "setup.bash"

        if src_dir.exists() and install_setup.exists():
            return str(install_setup)

    raise FileNotFoundError("workspace install/setup.bash not found")


class NodeManager(QObject):
    def __init__(self, ros_node: Node):
        super().__init__()

        self.node_ = ros_node
        self.recv_data: dict[str, str] = {}

        self.process_list = {
            "LeftRobot": "Left Robot System",
            "RightRobot": "Right Robot System",
            "LeftVision": "Left Vision System",
            "RightVision": "Right Vision System",
            "LeftLeRobot": "Left LeRobot System",
            "RightLeRobot": "Right LeRobot System",
        }

        self.pm = ProcessManager()
        self.pm.sig_log.connect(self.print_log)
        self.pm.sig_state.connect(self.print_state)
        self.pm.sig_exited.connect(self.print_err)

        self.lerobot_execute_output = {
            "left": True,
            "right": True,
        }

        self.register_executable_nodes()

    def print_log(self, name: str, txt: str):
        if txt is None:
            return

        if not txt.endswith("\n"):
            txt += "\n"

        sys.stdout.write(f"[{name}] {txt}")
        sys.stdout.flush()

    def print_state(self, name: str, running: bool):
        state = "RUNNING" if running else "STOPPED"
        self.node_.get_logger().info(f"[{name}] {state}")

    def print_err(self, name: str, exit_code: int, exit_status: int):
        self.node_.get_logger().error(f"[{name}] exited: code={exit_code}, status={exit_status}")

    def shutdown_(self):
        self.pm.shutdown_all()

    def _side_from_title(self, title: str) -> str | None:
        parts = title.split(maxsplit=1)
        if not parts:
            return None

        side = parts[0].lower()
        if side in ("left", "right"):
            return side
        return None

    def _system_from_title(self, title: str) -> str | None:
        parts = title.split(maxsplit=1)
        if len(parts) < 2:
            return None
        return parts[1]

    def _robot_command(self, side: str, command: int, value: int):
        msg = UiCommand()
        msg.command = int(command)
        msg.value = int(value)
        if side == "left":
            self.node_.ui_left_pub.publish(msg)
        elif side == "right":
            self.node_.ui_right_pub.publish(msg)

    def _register_lerobot(self, side: str, policy_path: str, enable_output: bool):
        process_name = self.process_list[f"{side.capitalize()}LeRobot"]

        if not policy_path:
            self.node_.get_logger().warning(f"{process_name}: policy path is empty.")

        # Map camera topics to the policy's expected keys (order-matched):
        #   d405_rgb  (RealSense, wrist) -> observation.images.wrist_rgb
        #   zivid_rgb (Zivid, scene/front) -> observation.images.front_rgb
        camera_topics = f"['/system_{side}/d405_rgb', '/system_{side}/zivid_rgb']"
        camera_keys = "['observation.images.wrist_rgb', 'observation.images.front_rgb']"

        if side == "left":
            gripper_args = "-p enable_gripper_output:=true -p gripper_action_start:=6 -p gripper_action_size:=20"
        else:
            gripper_args = "-p enable_gripper_output:=true -p gripper_action_start:=6 -p gripper_action_size:=6"

        cmd = (
            f"ros2 run lerobot_system lerobot_system_{side} --ros-args "
            f"-p policy_path:={shlex.quote(policy_path)} "
            f"-p enable_output:={'true' if enable_output else 'false'} "
            f"-p camera_topics:=\"{camera_topics}\" "
            f"-p camera_keys:=\"{camera_keys}\" "
            f"{gripper_args}"
        )

        self.pm.register(
            ProcSpec(
                name=process_name,
                cmd=cmd,
                ros_setup="/opt/ros/humble/setup.bash",
                ws_setup=find_workspace_setup(),
            )
        )

    def toggle_lerobot_mode(self, side: str, policy_path: str) -> str:
        current = self.lerobot_execute_output.get(side, True)
        next_state = not current
        self.lerobot_execute_output[side] = next_state

        process_name = self.process_list[f"{side.capitalize()}LeRobot"]
        self._register_lerobot(side, policy_path, next_state)

        if self.pm.is_running(process_name):
            self.pm.restart(process_name)

        return "Mode: Execute Action" if next_state else "Mode: Print Only"

    def excute_command(self, side_label: str):
        title = self.recv_data.get("title", "")
        button_name = self.recv_data.get("button_name", "")
        text = self.recv_data.get("text", "").strip()

        side = self._side_from_title(title)
        system = self._system_from_title(title)

        if side is None or system is None:
            self.node_.get_logger().error(f"Invalid UI command payload: {self.recv_data}")
            return

        if side_label.lower() != side:
            return

        if system == "Robot":
            process_name = self.process_list[f"{side.capitalize()}Robot"]
            if button_name == "Connect":
                if self.pm.start(process_name):
                    self.node_.get_logger().info(f"[SUCCESS] {process_name} started")
            elif button_name == "Stop":
                if self.pm.stop(process_name):
                    self.node_.get_logger().info(f"[SUCCESS] {process_name} stopped")

        elif system == "Vision":
            process_name = self.process_list[f"{side.capitalize()}Vision"]
            if button_name == "Connect":
                if self.pm.start(process_name):
                    self.node_.get_logger().info(f"[SUCCESS] {process_name} started")
            elif button_name == "Stop":
                if self.pm.stop(process_name):
                    self.node_.get_logger().info(f"[SUCCESS] {process_name} stopped")

        elif system == "LeRobot":
            process_name = self.process_list[f"{side.capitalize()}LeRobot"]
            if button_name == "Inference":
                if not text:
                    self.node_.get_logger().warning(f"{process_name}: policy path is empty.")
                    return

                self._register_lerobot(side, text, self.lerobot_execute_output.get(side, True))
                if self.pm.start(process_name):
                    self.node_.get_logger().info(f"[SUCCESS] {process_name} started")
            elif button_name.startswith("Mode:"):
                return self.toggle_lerobot_mode(side, text)
            elif button_name == "Init Pose":
                self._robot_command(side, 0, 1)
                self.node_.get_logger().info(f"{side.capitalize()} init pose requested")
            elif button_name == "Stop":
                if self.pm.stop(process_name):
                    self.node_.get_logger().info(f"[SUCCESS] {process_name} stopped")
            return None

    def register_executable_nodes(self):
        ws_setup = find_workspace_setup()

        self.pm.register(
            ProcSpec(
                name=self.process_list["LeftRobot"],
                cmd="ros2 run system_left system_left --ros-args -p record_period:=10",
                ros_setup="/opt/ros/humble/setup.bash",
                ws_setup=ws_setup,
            )
        )
        self.pm.register(
            ProcSpec(
                name=self.process_list["RightRobot"],
                cmd="ros2 run system_right system_right --ros-args -p record_period:=10",
                ros_setup="/opt/ros/humble/setup.bash",
                ws_setup=ws_setup,
            )
        )
        self.pm.register(
            ProcSpec(
                name=self.process_list["LeftVision"],
                cmd="ros2 run teleop_vision system_vision_left --ros-args -p hand_side:=left",
                ros_setup="/opt/ros/humble/setup.bash",
                ws_setup=ws_setup,
            )
        )
        self.pm.register(
            ProcSpec(
                name=self.process_list["RightVision"],
                cmd="ros2 run teleop_vision system_vision_right --ros-args -p hand_side:=right",
                ros_setup="/opt/ros/humble/setup.bash",
                ws_setup=ws_setup,
            )
        )

    def register_lerobot_default(self, side: str, policy_path: str):
        self._register_lerobot(side, policy_path)