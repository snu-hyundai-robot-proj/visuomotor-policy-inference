#!/usr/bin/env python3
import threading
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from hdr_stream.scenarios.control import http_get_joint_states

from hdr_stream.utils.net import NetClient
from hdr_stream.utils.parser import NDJSONParser
from hdr_stream.utils.dispatcher import Dispatcher
from hdr_stream.utils.api import OpenStreamAPI

GET_JOINTS = "/project/robot/joints/joint_states"
GET_POSITION = "/project/robot/po_cur"

STOP_MONITOR = "monitor"
STOP_CONTROL = "control"
STOP_ROBOT = "session"

class hdr_stream(Node):
    def __init__(self):
        super().__init__("robot_stream_node",namespace = "robot")

        self.declare_parameter("robot_ip", "192.168.4.152")
        self.declare_parameter("robot_port", 49000)
        self.declare_parameter("major", 1)
        self.declare_parameter("monitor_period_ms", 2)

        self.declare_parameter("send_dt_sec", 0.005)
        self.declare_parameter("look_ahead_time", 0.2)

        self.declare_parameter("target_joint_topic", "joint_target_deg")
        self.declare_parameter("joint_count", 6)
        self.declare_parameter("simulation", True)
        self.declare_parameter("robot_side","left")

        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.robot_port = self.get_parameter("robot_port").get_parameter_value().integer_value
        self.major = self.get_parameter("major").get_parameter_value().integer_value
        self.monitor_period_ms = self.get_parameter("monitor_period_ms").get_parameter_value().integer_value

        self.send_dt_sec = float(self.get_parameter("send_dt_sec").value)
        self.look_ahead_time = float(self.get_parameter("look_ahead_time").value)

        self.target_joint_topic = self.get_parameter("target_joint_topic").value
        self.joint_count = int(self.get_parameter("joint_count").value)

        self.joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

        # self.simulation = bool(self.get_parameter("simulation").value)
        self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
        self.robot_side = self.get_parameter("robot_side").get_parameter_value().string_value

        if self.simulation:
            self.get_logger().warn("\n##########################################################\n"
                                +    "           " + self.robot_side + " Robot Mode is SIMULAITON" 
                                + "\n##########################################################")
        else:
            self.get_logger().warn("\n##########################################################\n"
                                +    "           " + self.robot_side + " Robot Mode is REAL" 
                                + "\n##########################################################")

        ## topic
        self.sub_target = self.create_subscription(
            JointState,
            self.target_joint_topic,
            self._on_target_joint,
            1
        )

        self.pub_joint = self.create_publisher(
            JointState,
            "/system_" + self.robot_side + "/joint_states",
            1
        )

        self.pub_pose = self.create_publisher(
            Pose,
            "/system_" + self.robot_side + "/pose_states",
            1
        )

        ## Openstream API
        if self.robot_side == "left":
            self.robot_ip = "192.168.4.152"
        elif self.robot_side == "right":
            self.robot_ip = "192.168.4.151"
        else:
            self.get_logger().error("Robot IP and Side doesn't match")
            return
        self.get_logger().info("Target Robot IP : " + self.robot_ip)
        
        self.net = NetClient(self.robot_ip, int(self.robot_port))
        self.parser = NDJSONParser()
        self.dispatcher = Dispatcher()
        self.api = OpenStreamAPI(self.net)

        self.handshake_ok = threading.Event()

        self.lock = threading.Lock()
        self.current_pose: Optional[tuple] = None                 # (x,y,z,rx,ry,rz)
        self.latest_target_deg: Optional[List[float]] = None      # 최신 목표 조인트(deg)

        self._tfs = 0.0

        # recv/send period (Hz)
        self._recv_count = 0
        self._last_hz_print = time.time()

        # callback functions
        def _on_handshake_ack(m: dict) -> None:
            ok = bool(m.get("ok"))
            self.get_logger().info(f"[ack] handshake_ack ok={ok} version={m.get('version')}")
            if ok:
                self.handshake_ok.set()

        def _on_recv_data(m: dict) -> None:
            res = m.get("result")
            if not isinstance(res, dict):
                return
            
            if res.get("_type") != "Pose":
                if res.get("_type") == "JObject":
                    jointlist = res.get("position")
                    msg = JointState()
                    msg.name = list(self.joint_names)
                    msg.position = [i for i in jointlist]      ## degree
                    self.pub_joint.publish(msg)
                    # self.get_logger().info(f"RECV J : {msg.position}")

                    if self.latest_target_deg is not None and self.simulation is False:
                        joint_gap = [abs(cur - tar) for cur, tar in zip(msg.position,self.latest_target_deg)]
                        isOverflow = False
                        for i in joint_gap:
                            if i > 25.0:
                                isOverflow = True
                                self.api.stop(target=STOP_CONTROL)
                                self.overflow_workspace = True

                        if isOverflow: self.get_logger().error("overflow Joint speed")
                return

            x = res.get("x")
            y = res.get("y")
            z = res.get("z")
            rx = res.get("rx")
            ry = res.get("ry")
            rz = res.get("rz")

            # self.get_logger().info(f"RECV P : {x}, {y}, {z}, {rx}, {ry}, {rz}")
            if self.robot_side == "left":
                if x < -900.0 or x > 500.0 or y < -1200.0 or y > -900.0 or z > 1800.0 or z < 850.0:
                    self.get_logger().error("overflow robot workspace")    
                    self.api.stop(target=STOP_CONTROL)
                    self.overflow_workspace = True
            elif self.robot_side == "right":
                if x < -900.0 or x > 500.0 or y > 1200.0 or y < 900.0 or z > 1800.0 or z < 1250.0:
                    self.get_logger().error("overflow robot workspace")    
                    self.api.stop(target=STOP_CONTROL)
                    self.overflow_workspace = True
            else:
                self.get_logger().error("Can not Found Robot Side")

            msg = Pose()

            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            msg.orientation.x = rx
            msg.orientation.y = ry
            msg.orientation.z = rz

            self.pub_pose.publish(msg)

            with self.lock:
                self.current_pose = (x, y, z, rx, ry, rz)
                self._recv_count += 1

            now = time.time()
            if now - self._last_hz_print >= 1.0:
                with self.lock:
                    hz = self._recv_count
                    self._recv_count = 0
                self._last_hz_print = now
                self.get_logger().info(f"communication Hz : {hz}")

        def _on_error(e: dict) -> None:
            self.get_logger().error(
                f"[ERR] code={e.get('error')} message={e.get('message')} hint={e.get('hint')}"
            )

        self.dispatcher.on_type["handshake_ack"] = _on_handshake_ack
        self.dispatcher.on_type["monitor_ack"] = lambda m: self.get_logger().info(
            f"[ack] monitor_ack ok={m.get('ok')} url={m.get('url')} period_ms={m.get('period_ms')}"
        )
        self.dispatcher.on_type["data"] = _on_recv_data
        self.dispatcher.on_error = _on_error

        self.timer = self.create_timer(self.send_dt_sec, self.send_worker)
        self.robot_start()

    def robot_start(self) -> None:
        self.net.connect()
        self.net.start_recv_loop(lambda b: self.parser.feed(b, self.dispatcher.dispatch))

        self.api.handshake(major=int(self.major))
        self.get_logger().info("Handshake request")

    def robot_initialize(self) -> bool:
        if getattr(self, "_initialized", False):
            return True

        if not self.handshake_ok.is_set():
            return False

        self.api.monitor(url=GET_POSITION, period_ms=int(self.monitor_period_ms), args={})
        self.received_pose = True

        self.switch = self.create_timer(0.002, self.recv_worker)
        try:
            self.api.joint_traject_init()
        except Exception as e:
            self.get_logger().error(f"joint_traject_init failed: {e}")
            return False

        self._initialized = True
        self.get_logger().info("Monitor started + joint trajectory initialized.")
        return True

    def _on_target_joint(self, msg: JointState) -> None:
        if not msg.position:
            return

        if len(msg.position) < self.joint_count:
            self.get_logger().warn(
                f"target joint size too small: got={len(msg.position)} need={self.joint_count}"
            )
            return

        target = [float(msg.position[i]) for i in range(self.joint_count)]

        # self.get_logger().info(f"TARGET J : {type(target[3])}")

        with self.lock:
            self.is_recv_data = True
            self.latest_target_deg = target

    def recv_worker(self):
        if self.received_pose:
            self.received_pose = False
            self.api.monitor(url=GET_JOINTS, period_ms=int(self.monitor_period_ms), args={})
        else:
            self.api.monitor(url=GET_POSITION, period_ms=int(self.monitor_period_ms), args={})
            self.received_pose = True

    def send_worker(self) -> None:
        if getattr(self, "overflow_workspace", False):
            return

        if not self.robot_initialize():
            return

        with self.lock:
            target = None if self.latest_target_deg is None else list(self.latest_target_deg)

        if target is None:
            return

        if self.simulation:
            fmt = [f"{x:.6f}" for x in target]
            
            if self.is_recv_data:
                self.get_logger().info(f"TARGET J : {fmt}")
                
            self.is_recv_data = False
        else:
            # self.get_logger().info(f"RealMode")
            body = {
                "interval": float(self.send_dt_sec),
                "time_from_start": float(self._tfs),
                # "time_from_start": float(0),
                "look_ahead_time": float(self.look_ahead_time),
                "point": [float(x) for x in target],  # point는 deg (서버가 deg를 rad로 변환)
            }

            try:
                self.api.joint_traject_insert_point(body)
                self._tfs += self.send_dt_sec
                self.is_recv_data = False
            except Exception as e:
                self.get_logger().error(f"joint_traject_insert_point failed: {e}")

    def destroy_node(self):
        self.get_logger().info("CLOSING...")
        
        self.get_joint_thread_running = False
        
        try:
            self.api.stop(target=STOP_ROBOT)
        except Exception as e:
            self.get_logger().warn(f"api.stop failed: {e}")

        try:
            self.net.close()
        except Exception as e:
            self.get_logger().warn(f"net.close failed: {e}")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = hdr_stream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()