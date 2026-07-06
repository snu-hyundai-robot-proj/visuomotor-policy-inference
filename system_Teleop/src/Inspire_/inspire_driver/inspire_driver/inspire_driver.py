#!/usr/bin/env python3
import os
import glob
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
import threading

from inspire_driver.communicate import inspire_comm
from system_interface.msg import InspireSensor


def resolve_inspire_port():
    """Find the Inspire serial port robustly. Order: $INSPIRE_PORT (if it exists) ->
    auto-detect the Inspire's FTDI in /dev/serial/by-id (it is an FT232R / product 6001;
    the udev name may be 'FT232R_..._<serial>' OR the generic 'FTDI_6001', so match either
    and exclude the other device, an FT231X) -> /dev/ttyUSB1. This survives the by-id name
    changing across power-cycles."""
    p = os.environ.get("INSPIRE_PORT", "")
    if p and os.path.exists(p):
        return p
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        low = os.path.basename(path).lower()
        if ("ft232r" in low or "6001" in low) and "ft231x" not in low:
            return path
    return p or "/dev/ttyUSB1"

INSPIRE_FINGER_MIN_DEGREE = [880, 880, 880, 880, 1100, 600]
INSPIRE_FINGER_MAX_DEGREE = [1740, 1740, 1740, 1740, 1350, 1800]
SRBL_INSPIRE_PALM_LIST = ['palm_right', 'palm_middle', 'palm_left']

class InspireCommandSubscriber(Node):
    def __init__(self):
        super().__init__('inspire_command_subscriber')
        # Resolve the serial port robustly (env override -> auto-detect FT232R -> ttyUSB1).
        port = resolve_inspire_port()
        self.get_logger().info(f"Inspire serial port: {port}")
        self.ser = inspire_comm.SRBL_Inspire_gripper(device_name=port)
        self.current_6d_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.recv_current_joint = [0.0] * 6
        self.recv_sensor_data = [0.0] * 29
        self.target_joint = [0.0] * 6
        self.final_target_joint = [0.0] * 6
        self.publish_driver_diagnostics = self.declare_parameter(
            'publish_driver_diagnostics', False
        ).get_parameter_value().bool_value
        self.driver_diagnostics_hz = self.declare_parameter(
            'driver_diagnostics_hz', 10.0
        ).get_parameter_value().double_value
        self._last_diagnostics_publish = 0.0
        self.jointPub = self.create_publisher(
            JointState,
            '/inspire/joint_states',
            1
        )

        self.sensorPub = self.create_publisher(
            InspireSensor,
            '/inspire/tactile_sensor',
            1
        )
        self.driverDiagPub = None
        if self.publish_driver_diagnostics:
            self.driverDiagPub = self.create_publisher(
                String,
                '/inspire/right/driver_diagnostics',
                1
            )
        self.create_subscription(
            Float64MultiArray, 
            '/inspire/right/target', 
            self.cmd_callback, 
            1
        )

        self.get_logger().info("Start data Monitoring")

        # self.create_timer(0.01, self.get_current_angle)
        self.get_thread = threading.Thread(target=self.get_worker,daemon=True)
        self.get_thread.start()

        self.ser.send_get_angle()

    def get_worker(self):
        while True:
            self.get_current_angle()
            self.get_sensor_data()

            time.sleep(0.005)

    def cmd_callback(self, msg):
        self.current_6d_vector = list(msg.data)

        current_data = self.current_6d_vector
        current_data = self.retarget_fingers(current_data)

        for i in range(6):
            self.target_joint[i] = current_data[i]

        # print(current_data[0],current_data[1],current_data[2],
        #       current_data[3],current_data[4],current_data[5])
        for i in range(6):
            if current_data[i] >= INSPIRE_FINGER_MAX_DEGREE[i]:
                current_data[i] = INSPIRE_FINGER_MAX_DEGREE[i]

            if current_data[i] <= INSPIRE_FINGER_MIN_DEGREE[i]:
                current_data[i] = INSPIRE_FINGER_MIN_DEGREE[i]
        for i in range(6):
            self.final_target_joint[i] = current_data[i]
        # self.get_logger().info(f"Target Angle : {current_data}")
        self.ser.move_fingers(current_data)

    def get_latest_vector(self):
        return self.current_6d_vector

    def retarget_fingers(self, cur_6d_vec):
        # thumb(1100-1350) / thumb rotation(600-1800) / others (900-1740)
        # [pinky, ring, middle, index, thumb, thumb_rot]
        pinky = cur_6d_vec[0] * 1100 + 750 
        ring = cur_6d_vec[1] * 1100 + 750
        middle = cur_6d_vec[2] * 1100 + 750
        index = cur_6d_vec[3] * 1100 + 750
        thumb = cur_6d_vec[4] * 400 + 1100
        thumb_rot = -cur_6d_vec[5] * 950 + 1900

        if thumb_rot > 1800:
            thumb_rot = 1800
        return [float(pinky), float(ring), float(middle), float(index), float(thumb), float(thumb_rot)]

    def get_sensor_data(self):
        msg = InspireSensor()
        msg.tactile_sensor = self.ser.received_sensor
        self.sensorPub.publish(msg)

    def get_current_angle(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["j1","j2","j3","j4","j5","j6", "tj1","tj2","tj3","tj4","tj5","tj6"]
        msg.position = [self.ser.received_angle[0],self.ser.received_angle[1],self.ser.received_angle[2],
                        self.ser.received_angle[3],self.ser.received_angle[4],self.ser.received_angle[5],
                        self.target_joint[0],self.target_joint[1],self.target_joint[2],
                        self.target_joint[3],self.target_joint[4],self.target_joint[5]]

        # print(msg)
        self.jointPub.publish(msg)
        self.publish_diagnostics_if_enabled()

    def publish_diagnostics_if_enabled(self):
        if not self.publish_driver_diagnostics or self.driverDiagPub is None:
            return
        now = time.monotonic()
        hz = self.driver_diagnostics_hz if self.driver_diagnostics_hz > 0 else 10.0
        if now - self._last_diagnostics_publish < 1.0 / hz:
            return
        self._last_diagnostics_publish = now

        payload = {
            "timestamp_monotonic": now,
            "angle_actual_deg": list(self.ser.received_angle),
            "target_deg": [x / 10.0 for x in self.final_target_joint],
            "target_raw_0p1deg": list(self.final_target_joint),
            "j6": {
                "angle_actual_deg": self.ser.received_angle[5],
                "target_deg": self.final_target_joint[5] / 10.0,
                "current": None,
                "error_code": None,
                "status_code": None,
            },
            "availability": {
                "angle_actual": True,
                "target_echo": True,
                "current": False,
                "error_code": False,
                "status_code": False,
                "force_actual": False,
                "speed_actual": False,
            },
            "notes": (
                "Diagnostics use only values already held by the driver: angleAct-derived "
                "received_angle and final command target echo. No extra serial read loop is added."
            ),
        }
        self.driverDiagPub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def stop_thread(self):
        self.ser.destroy_thread()
        self.get_thread.join(timeout=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = InspireCommandSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_thread()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
