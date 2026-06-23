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
                             "Right Camera"         : "Right Cam",
                             "SystemPlayer"         : "System Player",}

        self.pm = ProcessManager()

        self.pm.sig_log.connect(self.print_log)
        self.pm.sig_state.connect(self.print_state)
        self.pm.sig_exited.connect(self.print_err)

        self.sig_calib.connect(self.handle_calibration)
        self.sig_start.connect(self.handle_start)
        self.sig_stop.connect(self.handle_stop)

        self.register_excutable_node()

        self.pm.start(self.process_list["LeftSystemOn"])
        self.pm.start(self.process_list["RightSystemOn"])
        self.pm.start(self.process_list["Manus"])
        self.pm.start(self.process_list["Tracker"])
        self.pm.start(self.process_list["FTSensor"])
        self.pm.start(self.process_list["Left Camera"])
        self.pm.start(self.process_list["Right Camera"])

        self.switch_side = ''

        self.modbus_client = ModbusSerialClient(
            port='/dev/ttyUSB0',
            baudrate=115200,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=0.05
        )

        self.modbus_client.connect()

        self.prev_calib = 0
        self.prev_start = 0
        self.prev_stop = 0

        self.calibrated = False
        self.running = False

        self.modbus_running = True
        self.modbus_thread = threading.Thread(target=self.modbus_loop, daemon=True)
        self.modbus_thread.start()

    def modbus_loop(self):
        while self.modbus_running:
            try:
                res = self.modbus_client.read_input_registers(address=0, count=4, device_id=1)
                if res.isError():
                    time.sleep(0.01)
                    continue

                reg0, reg1, reg2, reg3 = [int(v) for v in res.registers]

                calib_pressed = (self.prev_calib == 0 and reg0 == 1)
                start_pressed = (self.prev_start == 0 and reg1 == 1)
                stop_pressed  = (self.prev_stop  == 0 and reg2 == 1)

                if calib_pressed:
                    self.sig_calib.emit()

                if start_pressed:
                    self.sig_start.emit()

                if stop_pressed:
                    self.sig_stop.emit()

                self.prev_calib = reg0
                self.prev_start = reg1
                self.prev_stop = reg2

                time.sleep(0.01)

            except Exception as e:
                print(f'modbus error: {e}')
                time.sleep(0.1)

    def handle_calibration(self):
        self.node_.get_logger().info("Detect and Calibrate")            ## vision 촬영까지 트리거해보자
        self.calibrated = True
        self.running = False

        if self.switch_side == 'left':
            self.node_.request_left(2,0)      # vision detect and calibration
            self.pm.start(self.process_list["LeftSystemRecord"])

        elif self.switch_side == 'right':
            self.node_.request_right(2,0)     # vision detect and calibrationa
            # self.pm.start()
            self.pm.start(self.process_list["RightSystemRecord"])
        else :
            self.node_.get_logger().warn("\n########### Switch setting is None #############\n")

    def handle_start(self):
        self.node_.get_logger().info("Teleoperation Start")

        if(self.calibrated == True and self.running == False):
            self.running = True
            if self.switch_side == 'left':
                self.node_.request_left(1,1)
                self.node_.request_left(0,2)
                self.node_.request_left(2,1)
            elif self.switch_side == 'right':
                self.node_.request_right(1,1)
                self.node_.request_right(0,2)
                self.node_.request_right(2,1)
        else :
            self.node_.get_logger().warn("\n########### Switch setting is None #############\n")

    def handle_stop(self):
        self.node_.get_logger().info("stop")
        self.calibrated = False
        self.running = False

        if self.switch_side == 'left':
            self.node_.request_left(2,2)
            self.node_.request_left(1,0)
            self.node_.request_left(0,0)

            self.pm.stop(self.process_list["LeftSystemRecord"])

            target = self.process_list["LeftGripperControl"]
            if self.pm.kill(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

            target = self.process_list["LeftRobot"]
            if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

            target = self.process_list["LeftServo"]
            if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

        elif self.switch_side == 'right':
            self.node_.request_right(2,2)
            self.node_.request_right(1,0)
            self.node_.request_right(0,0)
            self.pm.stop(self.process_list["RightSystemRecord"])

            target = self.process_list["RightGripperControl"]
            if self.pm.kill(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

            target = self.process_list["ConnectRightGripper"]
            if self.pm.kill(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

            target = self.process_list["RightRobot"]
            if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

            target = self.process_list["RightServo"]
            if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
            else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

        else :
            self.node_.get_logger().warn("\n########### Switch setting is None #############\n")

    def stop_threading(self):
        self.modbus_running = False
        # self.modbus_thread.quit()
        # self.modbus_thread.wait()

        if hasattr(self, 'modbus_thread') and self.modbus_thread.is_alive():
            self.modbus_thread.join(timeout=1.0)

        if hasattr(self, 'modbus_client') and self.modbus_client:
            self.modbus_client.close()

    def print_log(self, name:str, txt:str):
        if txt is None:
            return

        if not txt.endswith("\n"):
            txt+="\n"

        sys.stdout.write(f"[{name}] {txt}")
        sys.stdout.flush()

    def print_state(self, name:str, running: bool):
        state = "RUNNING" if running else "STOPPED"
        self.node_.get_logger().info(f"[{name}] {state}")

    def print_err(self, name:str, exit_code: int, exit_status:int):
        self.node_.get_logger().error(f"[{name}] exited: code={exit_code}, status={exit_status}")

    def excute_command(self, str:str):
        # print(self.recv_data)
        title = self.recv_data.get("title","")

        title_split = title.split()

        self.side = title_split[0]

        self.system = title_split[1]
        self.command = self.recv_data["button_name"]
        self.arg = self.recv_data["text"]

        if str == "Left":
            self.node_.get_logger().info("Left")
            match self.system:
                case "Switch":
                    self.switch_side = 'left'
                case "Gripper":
                    if self.command == "Connect":
                        target = self.process_list["ConnectLeftGripper"]
                        self.pm.register(ProcSpec(
                            name= target,
                            cmd = "ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py delto_ip:="+self.arg + " fingertip_sensor:=true",
                            # cmd = "ros2 launch dg5f_driver dg5f_left_pid_all_controller.launch.py delto_ip:="+"127.0.0.1" + " fingertip_sensor:=true delto_port:=1024",
                            ros_setup="/opt/ros/humble/setup.bash",
                            ws_setup="install/setup.bash",
                        ))
                        self.node_.request_left(0,0)
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start")

                    elif self.command == "Teleop":
                        target = self.process_list["LeftGripperControl"]
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start")
                    elif self.command == "Stop":
                        target = self.process_list["LeftGripperControl"]
                        if self.pm.kill(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

                case "Robot":
                    if self.command == "Connect":
                        target = self.process_list["LeftRobot"]
                        self.node_.request_left(0,0)
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start")
                    elif self.command == "Active":
                        target = self.process_list["LeftServo"]
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start\n %%% Must operate Calibration %%%")
                    elif self.command == "Stop":
                        # target = self.process_list["Servo"]
                        target = self.process_list["LeftRobot"]
                        if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

                        target = self.process_list["LeftServo"]
                        if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")
                        self.node_.request_left(0,0)

                case "Teleop":
                    if self.command == "Calibrate":
                        print("p")
                        # self.node_.request_left(0,1)
                        # self.node_.request_left(2,0)
                    elif self.command == "Start":
                        print("p")
                        # self.node_.request_left(0,2)
                    elif self.command == "Stop":
                        print("p")
                        # self.node_.request_left(0,0)
                    else: self.node_.get_logger().info(f"[FAILED] {target} Process stop")

                case "Record":
                    if self.command == "Record":
                        self.pm.start(self.process_list["LeftSystemRecord"])
                        # self.pm.register(ProcSpec(
                        #     name = self.process_list["LeftSystemRecord"],
                        #     cmd = "ros2 run data_exporter data_exporter --ros-args -p side:=left -p output_path:="+self.arg,
                        #     ros_setup="/opt/ros/humble/setup.bash",
                        #     ws_setup="install/setup.bash",
                        # ))
                        # self.pm.start(self.process_list["LeftSystemRecord"])

                    elif self.command == "Start":
                        # self.node_.request_left(1,1)
                        # self.node_.get_logger().info("Start Recording")
                        print("Use Switch")
                    elif self.command == "Stop":
                        # self.node_.request_left(1,0)
                        # self.node_.get_logger().info("Stop Recording")
                        # self.pm.stop(self.process_list["LeftSystemRecord"])
                        print("Use Switch")

        elif str == "Right":
            self.node_.get_logger().info("Right")
            match self.system:
                case "Switch":
                    self.switch_side = 'right'

                case "Gripper":
                    if self.command == "Connect":
                        target = self.process_list["ConnectRightGripper"]
                        self.node_.request_right(0,0)
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start")

                    elif self.command == "Teleop":
                        target = self.process_list["RightGripperControl"]
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start")
                    elif self.command == "Stop":
                        target = self.process_list["RightGripperControl"]
                        if self.pm.kill(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")
                        target = self.process_list["ConnectRightGripper"]
                        if self.pm.kill(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

                case "Robot":
                    if self.command == "Connect":
                        target = self.process_list["RightRobot"]
                        self.node_.request_right(0,0)
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start")
                    elif self.command == "Active":
                        target = self.process_list["RightServo"]
                        if self.pm.start(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process Start\n %%% Must operate Calibration %%%")
                    elif self.command == "Stop":
                        # target = self.process_list["Servo"]
                        target = self.process_list["RightRobot"]
                        if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")

                        target = self.process_list["RightServo"]
                        if self.pm.stop(target): self.node_.get_logger().info(f"[SUCCESS] {target} Process stop")
                        else : self.node_.get_logger().info(f"[FAILED] {target} Process stop")
                        self.node_.request_right(0,0)

                case "Teleop":
                    if self.command == "Calibrate":
                        self.node_.request_right(0,1)
                    elif self.command == "Start":
                        self.node_.request_right(0,2)
                    elif self.command == "Stop":
                        self.node_.request_right(0,0)
                    else: self.node_.get_logger().info(f"[FAILED] {target} Process stop")

                case "Record":
                    if self.command == "Record":
                        self.pm.start(self.process_list["RightSystemRecord"])
                        # self.pm.register(ProcSpec(
                        #     name = self.process_list["RightSystemRecord"],
                        #     cmd = "ros2 run data_exporter data_exporter --ros-args -p side:=right -p output_path:="+self.arg,
                        #     ros_setup="/opt/ros/humble/setup.bash",
                        #     ws_setup="install/setup.bash",
                        # ))
                        # self.pm.start(self.process_list["RightSystemRecord"])
                        print("Use Switch")
                    elif self.command == "Start":
                        # self.node_.request_right(1,1)
                        # self.node_.get_logger().info("Start Recording")
                        print("Use Switch")
                    elif self.command == "Stop":
                        # self.node_.request_right(1,0)
                        # self.node_.get_logger().info("Stop Recording")
                        # self.pm.kill(self.process_list["RightSystemRecord"])
                        print("Use Switch")

    def shutdown_(self):
        self.pm.shutdown_all()
        self.stop_threading()

    def register_excutable_node(self):
        ### Left Side
        self.pm.register(ProcSpec(
            name= self.process_list["LeftSystemOn"],
            cmd = "ros2 run system_left system_left --ros-args -p record_period:=10",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["LeftGripperControl"],
            cmd = "ros2 run dg5f_driver manus_retarget.py",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["LeftRobot"],
            cmd = "ros2 run hdr_stream hdr_stream_node --ros-args -p simulation:=true -p robot_side:=left",          ##  simulation Robot
            # cmd = "ros2 run hdr_stream hdr_stream_node --ros-args -p simulation:=false -p robot_side:=left",          ##  real Robot
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["LeftServo"],
            cmd = "ros2 launch hdr_ros2_driver hdr_servo_driver.launch.py robot_side:=left",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["Manus"],
            cmd = "ros2 run manus_ros2 manus_data_publisher",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))
        self.pm.register(ProcSpec(
            name= self.process_list["Tracker"],
            cmd = "ros2 launch vive_tracker_bringup vive_tracker_bringup.launch.py simulation:=false",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))
        self.pm.register(ProcSpec(
            name= self.process_list["FTSensor"],
            cmd = "ros2 launch net_ft_driver dual_axia_reader.launch.py",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["Left Camera"],
            cmd = "ros2 run teleop_vision system_vision_left --ros-args -p hand_side:=left",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["Right Camera"],
            cmd = "ros2 run teleop_vision system_vision_right --ros-args -p hand_side:=right",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name = self.process_list["LeftSystemRecord"],
            cmd = "ros2 run data_exporter data_exporter --ros-args -p side:=left -p output_path:=Record/",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name = self.process_list["RightSystemRecord"],
            cmd = "ros2 run data_exporter data_exporter --ros-args -p side:=right -p output_path:=Record/",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))
############################################################################################################################
############################################################################################################################

        # ### Right Side
        self.pm.register(ProcSpec(
            name= self.process_list["RightSystemOn"],
            cmd = "ros2 run system_right system_right --ros-args -p record_period:=10",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["ConnectRightGripper"],
            cmd = "ros2 run inspire_driver inspire_driver_node",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["RightGripperControl"],
            cmd = "ros2 run inspire_driver inspire_bridge_node",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))

        self.pm.register(ProcSpec(
            name= self.process_list["RightRobot"],
            cmd = "ros2 run hdr_stream hdr_stream_node --ros-args -p simulation:=true -p robot_side:=right",          ##  simulation Robot
            # cmd = "ros2 run hdr_stream hdr_stream_node --ros-args -p simulation:=false -p robot_side:=right",          ##  real Robot
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))
        self.pm.register(ProcSpec(
            name= self.process_list["RightServo"],
            cmd = "ros2 launch hdr_ros2_driver hdr_servo_driver.launch.py robot_side:=right",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))
        
        self.pm.register(ProcSpec(
            name= self.process_list["SystemPlayer"],
            cmd = "ros2 run system_player system_player_node --ros-args -p gripper_topic:=/dg5f_left/lj_dg_pospid/reference -p robot_topic:=/robot/joint_target_deg -p parquet_path:=/home/pin/Desktop/Tesollo/1.Project/1.hyundae/seoul_uiwang/data/converted/left/data",
            ros_setup="/opt/ros/humble/setup.bash",
            ws_setup=find_workspace_setup(),
        ))