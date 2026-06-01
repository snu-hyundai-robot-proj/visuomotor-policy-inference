"""ROS2 robot-control client for the visuomotor policy server.

Pipeline (per EXECUTION_PLAN.md / INTEGRATION_HDR35.md):

    cameras (front/wrist) + robot state(26)
        -> POST /predict  (HTTP policy server)
        -> action[26] = arm[0:6] + hand[6:26]   (absolute target joints, radians)
        -> arm  : soft-start clamp -> rad->deg -> JointState  -> /robot/joint_target_deg -> HDR35
        -> hand : soft-start clamp -> MultiDOFCommand(rad)     -> /dg5f_left/lj_dg_pospid/reference -> DG5F

Safety, all parameterized:
  * enable_output / enable_gripper_output : output gates (default OFF -> infer only)
  * max_joint_delta / max_gripper_delta   : per-tick soft-start + rate limit, anchored to
                                            the MEASURED current joints (handles first-tick jump)
  * arm/gripper limit clamp               : keep commands inside joint limits
  * /<ns>/reset (std_srvs/Trigger)        : reset the policy queue at episode start

State source:
  * "frame_aligned" (default): system_interface/FrameAlignedState (robot_joint + gripper_joint, rad)
  * "joint_states"           : combine two sensor_msgs/JointState topics (units configurable)

The inference call runs in a dedicated control thread so HTTP latency never blocks
ROS callbacks. Run the policy at the model's trained rate (~30 Hz).
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from vpi_robot_client.http_client import PolicyHTTPClient

# control_msgs is only needed for the dg5f (left) gripper command type.
try:
    from control_msgs.msg import MultiDOFCommand
except Exception:  # pragma: no cover
    MultiDOFCommand = None

# system_interface is only needed for the "frame_aligned" state source.
try:
    from system_interface.msg import FrameAlignedState
except Exception:  # pragma: no cover
    FrameAlignedState = None

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None


LEFT_DELTO_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]
ARM_JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


def rad2deg(x: np.ndarray) -> np.ndarray:
    return np.asarray(x, dtype=np.float64) * 180.0 / math.pi


class PolicyControlNode(Node):
    def __init__(self):
        super().__init__("vpi_policy_control")

        # --- general ---
        self.declare_parameter("side", "left")
        self.declare_parameter("server_url", "http://localhost:8000")
        self.declare_parameter("http_timeout_sec", 5.0)
        self.declare_parameter("image_format", "JPEG")     # JPEG | PNG
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("reset_on_start", True)
        # run gate: Episode Manager toggles this at runtime (/vpi/set_enable).
        # Effective output = launch enable flag AND run_gate. Defaults True so the node
        # also works standalone (enable_output alone moves it).
        self.declare_parameter("enable_service_name", "/vpi/set_enable")

        # --- state source ---
        self.declare_parameter("state_source", "frame_aligned")  # frame_aligned | joint_states
        self.declare_parameter("state_topic", "")                # frame_aligned mode
        # joint_states mode:
        self.declare_parameter("arm_state_topic", "")
        self.declare_parameter("gripper_state_topic", "")
        self.declare_parameter("arm_state_unit", "deg")          # unit on arm_state_topic
        self.declare_parameter("gripper_state_unit", "rad")

        # --- cameras ---
        self.declare_parameter("front_camera_topic", "")
        self.declare_parameter("wrist_camera_topic", "")
        self.declare_parameter("camera_timeout_sec", 0.5)

        # --- action split ---
        self.declare_parameter("arm_action_start", 0)
        self.declare_parameter("arm_action_size", 6)
        self.declare_parameter("gripper_action_start", 6)
        self.declare_parameter("gripper_action_size", 20)

        # --- arm output ---
        self.declare_parameter("robot_action_topic", "")
        self.declare_parameter("robot_topic_unit", "deg")        # HDR35 stream wants deg
        self.declare_parameter("enable_output", False)
        self.declare_parameter("max_joint_delta", 0.02)          # rad/tick, 0=off (soft-start)
        self.declare_parameter("arm_limit_min", [])              # rad, len arm_action_size, [] = no clamp
        self.declare_parameter("arm_limit_max", [])

        # --- gripper output ---
        self.declare_parameter("gripper_action_topic", "")
        self.declare_parameter("gripper_command_type", "auto")   # auto | multi_dof_command | float64_multi_array
        self.declare_parameter("enable_gripper_output", False)
        self.declare_parameter("max_gripper_delta", 0.02)        # rad/tick, 0=off
        self.declare_parameter("gripper_limit_min", [])
        self.declare_parameter("gripper_limit_max", [])

        self.side = self.get_parameter("side").value
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        self.fps = float(self.get_parameter("fps").value)
        self.camera_timeout = float(self.get_parameter("camera_timeout_sec").value)
        self.state_source = self.get_parameter("state_source").value

        self.arm_start = int(self.get_parameter("arm_action_start").value)
        self.arm_size = int(self.get_parameter("arm_action_size").value)
        self.grip_start = int(self.get_parameter("gripper_action_start").value)
        self.grip_size = int(self.get_parameter("gripper_action_size").value)

        self.robot_topic_unit = self.get_parameter("robot_topic_unit").value
        self.enable_output = bool(self.get_parameter("enable_output").value)
        self.max_joint_delta = float(self.get_parameter("max_joint_delta").value)
        self.arm_min = self._opt_vec("arm_limit_min")
        self.arm_max = self._opt_vec("arm_limit_max")

        self.enable_gripper_output = bool(self.get_parameter("enable_gripper_output").value)
        self.max_gripper_delta = float(self.get_parameter("max_gripper_delta").value)
        self.grip_min = self._opt_vec("gripper_limit_min")
        self.grip_max = self._opt_vec("gripper_limit_max")
        self.gripper_command_type = self._resolve_gripper_cmd_type(
            self.get_parameter("gripper_command_type").value
        )

        # resolved topics
        robot_action_topic = self.get_parameter("robot_action_topic").value or "/robot/joint_target_deg"
        gripper_action_topic = self.get_parameter("gripper_action_topic").value or self._default_gripper_topic()

        # --- HTTP client ---
        self.client = PolicyHTTPClient(
            url=self.get_parameter("server_url").value,
            timeout=float(self.get_parameter("http_timeout_sec").value),
            image_format=self.get_parameter("image_format").value,
            jpeg_quality=int(self.get_parameter("jpeg_quality").value),
        )
        try:
            info = self.client.info()
            self.get_logger().info(f"policy server: {info}")
        except Exception as exc:  # don't die if server starts later
            self.get_logger().warn(f"could not reach policy server /info yet: {exc}")
        if bool(self.get_parameter("reset_on_start").value):
            self._try_reset()

        # --- shared state (guarded) ---
        self._lock = threading.Lock()
        self._cur_arm_rad: Optional[np.ndarray] = None      # measured arm joints (rad)
        self._cur_grip_rad: Optional[np.ndarray] = None     # measured gripper joints (rad)
        self._front: Optional[np.ndarray] = None
        self._wrist: Optional[np.ndarray] = None
        self._front_t = 0.0
        self._wrist_t = 0.0
        self.bridge = CvBridge() if CvBridge is not None else None

        # --- subscriptions ---
        self._setup_state_subs()
        front_topic = self.get_parameter("front_camera_topic").value or f"/system_{self.side}/camera/front/rgb"
        wrist_topic = self.get_parameter("wrist_camera_topic").value or f"/system_{self.side}/camera/wrist/rgb"
        self.create_subscription(Image, front_topic, self._on_front, 1)
        self.create_subscription(Image, wrist_topic, self._on_wrist, 1)

        # --- publishers ---
        self.arm_pub = self.create_publisher(JointState, robot_action_topic, 1)
        self.grip_pub = self._make_gripper_pub(gripper_action_topic)

        # --- reset service ---
        self.create_service(Trigger, f"~/reset", self._on_reset_srv)
        self._run_gate = True
        self.create_service(SetBool, self.get_parameter("enable_service_name").value, self._on_set_enable)

        # --- control thread ---
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"vpi_policy_control up: side={self.side}, state_source={self.state_source}, "
            f"front={front_topic}, wrist={wrist_topic}, arm->{robot_action_topic}({self.robot_topic_unit}), "
            f"grip->{gripper_action_topic}({self.gripper_command_type}), "
            f"output={'ON' if self.enable_output else 'OFF'}/grip={'ON' if self.enable_gripper_output else 'OFF'}, "
            f"max_joint_delta={self.max_joint_delta}, max_gripper_delta={self.max_gripper_delta}"
        )

    # ---------- helpers ----------
    def _opt_vec(self, name: str) -> Optional[np.ndarray]:
        vals = list(self.get_parameter(name).value or [])
        return np.asarray(vals, dtype=np.float64) if vals else None

    def _default_gripper_topic(self) -> str:
        return "/dg5f_left/lj_dg_pospid/reference" if self.side == "left" else "/inspire/right/target"

    def _resolve_gripper_cmd_type(self, t: str) -> str:
        if t != "auto":
            return t
        return "multi_dof_command" if self.side == "left" else "float64_multi_array"

    def _make_gripper_pub(self, topic: str):
        if self.gripper_command_type == "multi_dof_command":
            if MultiDOFCommand is None:
                raise RuntimeError("control_msgs not available for multi_dof_command gripper output")
            return self.create_publisher(MultiDOFCommand, topic, 1)
        if self.gripper_command_type == "float64_multi_array":
            return self.create_publisher(Float64MultiArray, topic, 1)
        if self.gripper_command_type == "none":
            return None
        raise ValueError(f"unsupported gripper_command_type: {self.gripper_command_type}")

    def _try_reset(self):
        try:
            self.client.reset()
            self.get_logger().info("policy queue reset")
        except Exception as exc:
            self.get_logger().warn(f"reset failed: {exc}")

    # ---------- state subscriptions ----------
    def _setup_state_subs(self):
        if self.state_source == "frame_aligned":
            if FrameAlignedState is None:
                raise RuntimeError(
                    "state_source=frame_aligned needs system_interface.msg.FrameAlignedState "
                    "(build the system_Teleop interfaces, or use state_source:=joint_states)."
                )
            topic = self.get_parameter("state_topic").value or f"/system_{self.side}/frame_aligned_state"
            self.create_subscription(FrameAlignedState, topic, self._on_frame_aligned, 1)
            self.get_logger().info(f"state from FrameAlignedState: {topic}")
        elif self.state_source == "joint_states":
            arm_topic = self.get_parameter("arm_state_topic").value or f"/system_{self.side}/joint_states"
            grip_topic = self.get_parameter("gripper_state_topic").value or f"/dg5f_{self.side}/joint_states"
            self._arm_unit = self.get_parameter("arm_state_unit").value
            self._grip_unit = self.get_parameter("gripper_state_unit").value
            self.create_subscription(JointState, arm_topic, self._on_arm_state, 1)
            self.create_subscription(JointState, grip_topic, self._on_grip_state, 1)
            self.get_logger().info(f"state from joint_states: arm={arm_topic}({self._arm_unit}), grip={grip_topic}({self._grip_unit})")
        else:
            raise ValueError(f"unknown state_source: {self.state_source}")

    def _on_frame_aligned(self, msg) -> None:
        if getattr(msg, "side", "") and msg.side != self.side:
            return
        with self._lock:
            self._cur_arm_rad = np.asarray(msg.robot_joint, dtype=np.float64)[: self.arm_size]
            self._cur_grip_rad = np.asarray(msg.gripper_joint, dtype=np.float64)[: self.grip_size]

    def _on_arm_state(self, msg: JointState) -> None:
        v = np.asarray(msg.position, dtype=np.float64)[: self.arm_size]
        if self._arm_unit == "deg":
            v = v * math.pi / 180.0
        with self._lock:
            self._cur_arm_rad = v

    def _on_grip_state(self, msg: JointState) -> None:
        v = np.asarray(msg.position, dtype=np.float64)[: self.grip_size]
        if self._grip_unit == "deg":
            v = v * math.pi / 180.0
        with self._lock:
            self._cur_grip_rad = v

    # ---------- cameras ----------
    def _img(self, msg: Image) -> np.ndarray:
        if self.bridge is None:
            raise RuntimeError("cv_bridge not available")
        return np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), dtype=np.uint8)

    def _on_front(self, msg: Image) -> None:
        img = self._img(msg)
        with self._lock:
            self._front, self._front_t = img, time.monotonic()

    def _on_wrist(self, msg: Image) -> None:
        img = self._img(msg)
        with self._lock:
            self._wrist, self._wrist_t = img, time.monotonic()

    def _on_reset_srv(self, request, response):
        self._try_reset()
        response.success = True
        response.message = "policy reset"
        return response

    def _on_set_enable(self, request, response):
        self._run_gate = bool(request.data)
        self.get_logger().info(f"run gate -> {'ON' if self._run_gate else 'OFF'}")
        response.success = True
        response.message = f"run_gate={self._run_gate}"
        return response

    # ---------- control loop ----------
    def _control_loop(self):
        period = 1.0 / self.fps
        next_t = time.monotonic()
        while not self._stop.is_set() and rclpy.ok():
            next_t += period
            now = time.monotonic()
            sleep = next_t - now
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # fell behind; resync

            with self._lock:
                arm_cur = None if self._cur_arm_rad is None else self._cur_arm_rad.copy()
                grip_cur = None if self._cur_grip_rad is None else self._cur_grip_rad.copy()
                front = None if self._front is None else self._front
                wrist = None if self._wrist is None else self._wrist
                ft, wt = self._front_t, self._wrist_t

            if arm_cur is None or grip_cur is None or front is None or wrist is None:
                continue
            t = time.monotonic()
            if (t - ft) > self.camera_timeout or (t - wt) > self.camera_timeout:
                continue

            state = np.concatenate([arm_cur, grip_cur]).astype(np.float32)
            try:
                action = self.client.predict(front, wrist, state)
            except Exception as exc:
                self.get_logger().warn(f"predict failed: {exc}")
                continue

            self._publish(action, arm_cur, grip_cur)

    def _slice(self, action: np.ndarray, start: int, size: int) -> Optional[np.ndarray]:
        if size <= 0 or len(action) < start + size:
            return None
        return np.asarray(action[start:start + size], dtype=np.float64)

    def _soft(self, target: np.ndarray, current: np.ndarray, max_delta: float,
              lo: Optional[np.ndarray], hi: Optional[np.ndarray]) -> np.ndarray:
        out = np.asarray(target, dtype=np.float64)
        if max_delta > 0.0 and current is not None and len(current) == len(out):
            out = current + np.clip(out - current, -max_delta, max_delta)
        if lo is not None and len(lo) == len(out):
            out = np.maximum(out, lo)
        if hi is not None and len(hi) == len(out):
            out = np.minimum(out, hi)
        return out

    def _publish(self, action: np.ndarray, arm_cur: np.ndarray, grip_cur: np.ndarray):
        # arm
        arm = self._slice(action, self.arm_start, self.arm_size)
        if arm is not None:
            arm = self._soft(arm, arm_cur, self.max_joint_delta, self.arm_min, self.arm_max)
            out = rad2deg(arm) if self.robot_topic_unit == "deg" else arm
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ARM_JOINT_NAMES[: len(out)]
            msg.position = [float(x) for x in out]
            if self.enable_output and self._run_gate:
                self.arm_pub.publish(msg)

        # gripper
        grip = self._slice(action, self.grip_start, self.grip_size)
        if grip is not None and self.grip_pub is not None:
            grip = self._soft(grip, grip_cur, self.max_gripper_delta, self.grip_min, self.grip_max)
            if self.enable_gripper_output and self._run_gate:
                if self.gripper_command_type == "multi_dof_command":
                    m = MultiDOFCommand()
                    m.dof_names = LEFT_DELTO_JOINT_NAMES[: len(grip)]
                    m.values = [float(x) for x in grip]
                    m.values_dot = [0.0] * len(grip)
                    self.grip_pub.publish(m)
                elif self.gripper_command_type == "float64_multi_array":
                    self.grip_pub.publish(Float64MultiArray(data=[float(x) for x in grip]))

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
