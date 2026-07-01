import threading
from typing import Dict, List, Optional

import numpy as np
import rclpy
from control_msgs.msg import MultiDOFCommand
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from system_interface.msg import FrameAlignedState

from lerobot_system.policy_runner import (
    LeRobotPolicyRunner,
    PolicyRunnerConfig,
    convert_units,
)


LEFT_DELTO_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]

# Must match the TRAINING observation.state layout: arm 6 (robot_joint) + hand 20
# (gripper_joint) = 26, both in rad. The convert pipeline built observation.state as
# robot_joint + gripper_joint, so the ROS node must compose it identically — otherwise
# the hand portion is silently filled with target/pose/FT and predictions are garbage.
DEFAULT_STATE_FIELDS = [
    "robot_joint",
    "gripper_joint",
]


class LeRobotSystemNode(Node):
    def __init__(self, forced_side: Optional[str] = None):
        super().__init__("lerobot_system")

        self.declare_parameter("side", forced_side or "left")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("task", "")
        self.declare_parameter("robot_type", "hyundai")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("mock_policy", False)
        self.declare_parameter("mock_action_size", 6)
        self.declare_parameter("local_files_only", False)
        self.declare_parameter("use_amp", False)
        self.declare_parameter("enable_output", False)

        self.declare_parameter("state_topic", "")
        self.declare_parameter("state_key", "observation.state")
        self.declare_parameter("state_fields", DEFAULT_STATE_FIELDS)
        self.declare_parameter("state_padding_mode", "pad_or_truncate")
        self.declare_parameter("camera_topics", [])
        self.declare_parameter("camera_keys", [])
        self.declare_parameter("camera_timeout_sec", 1.0)

        self.declare_parameter("raw_action_topic", "")
        self.declare_parameter("robot_action_topic", "")
        self.declare_parameter("robot_action_start", 0)
        self.declare_parameter("robot_action_size", 6)
        self.declare_parameter("action_output_unit", "rad")
        self.declare_parameter("robot_topic_unit", "deg")
        self.declare_parameter("max_joint_delta", 0.0)

        self.declare_parameter("enable_gripper_output", False)
        self.declare_parameter("gripper_action_topic", "")
        self.declare_parameter("gripper_action_start", 6)
        self.declare_parameter("gripper_action_size", 0)
        self.declare_parameter("gripper_command_type", "auto")

        # Ruckig jerk-limited smoothing of the policy action stream (opt-in).
        # Limits are applied to the full action vector; pass either a single value
        # (broadcast to every dim) or one value per action dim. Arm defaults below
        # match the HDR35_20 URDF/cuRobo limits; hand dims fall back to the scalar.
        self.declare_parameter("enable_ruckig", False)
        self.declare_parameter("ruckig_max_velocity", [3.141])
        self.declare_parameter("ruckig_max_acceleration", [12.0])
        self.declare_parameter("ruckig_max_jerk", [500.0])
        self.declare_parameter("ruckig_target_velocity_mode", "zero")

        self.side = self.get_parameter("side").value
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'.")

        self.enable_output = bool(self.get_parameter("enable_output").value)
        self.state_key = self.get_parameter("state_key").value
        self.state_fields = list(self.get_parameter("state_fields").value)
        self.state_padding_mode = self.get_parameter("state_padding_mode").value
        self.camera_timeout_sec = float(self.get_parameter("camera_timeout_sec").value)

        state_topic = self.get_parameter("state_topic").value or f"/system_{self.side}/frame_aligned_state"
        raw_action_topic = self.get_parameter("raw_action_topic").value or f"/lerobot/{self.side}/raw_action"
        robot_action_topic = self.get_parameter("robot_action_topic").value or f"/lerobot/{self.side}/joint_target"
        gripper_action_topic = self.get_parameter("gripper_action_topic").value or self._default_gripper_topic()

        self.robot_action_start = int(self.get_parameter("robot_action_start").value)
        self.robot_action_size = int(self.get_parameter("robot_action_size").value)
        self.action_output_unit = self.get_parameter("action_output_unit").value
        self.robot_topic_unit = self.get_parameter("robot_topic_unit").value
        self.max_joint_delta = float(self.get_parameter("max_joint_delta").value)

        self.enable_gripper_output = bool(self.get_parameter("enable_gripper_output").value)
        self.gripper_action_start = int(self.get_parameter("gripper_action_start").value)
        self.gripper_action_size = int(self.get_parameter("gripper_action_size").value)
        self.gripper_command_type = self._resolve_gripper_command_type(
            self.get_parameter("gripper_command_type").value
        )

        runner_cfg = PolicyRunnerConfig(
            policy_path=self.get_parameter("policy_path").value,
            device=self.get_parameter("device").value,
            task=self.get_parameter("task").value,
            robot_type=self.get_parameter("robot_type").value,
            local_files_only=bool(self.get_parameter("local_files_only").value),
            use_amp=bool(self.get_parameter("use_amp").value),
            mock_policy=bool(self.get_parameter("mock_policy").value),
            mock_action_size=int(self.get_parameter("mock_action_size").value),
        )
        self.runner = LeRobotPolicyRunner(runner_cfg)
        self.required_state_dim = self.runner.state_dim
        self.expected_action_dim = self.runner.action_dim
        self.image_shapes = self.runner.image_features

        if self.gripper_action_size <= 0 and self.expected_action_dim > self.robot_action_start + self.robot_action_size:
            self.gripper_action_start = self.robot_action_start + self.robot_action_size
            self.gripper_action_size = self.expected_action_dim - self.gripper_action_start

        self.smoother = self._build_ruckig_smoother()

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_state: Optional[FrameAlignedState] = None
        self.latest_images: Dict[str, np.ndarray] = {}
        self.latest_image_times: Dict[str, float] = {}
        self.warned_state_dim_mismatch = False

        self.state_sub = self.create_subscription(FrameAlignedState, state_topic, self._on_state, 1)
        self.raw_action_pub = self.create_publisher(Float64MultiArray, raw_action_topic, 1)
        self.robot_action_pub = self.create_publisher(JointState, robot_action_topic, 1)
        self.gripper_pub = self._create_gripper_publisher(gripper_action_topic)

        camera_topics = list(self.get_parameter("camera_topics").value)
        camera_keys = list(self.get_parameter("camera_keys").value)
        if not camera_keys and self.image_shapes:
            camera_keys = list(self.image_shapes.keys())
        elif camera_topics and not camera_keys:
            camera_keys = [f"observation.images.camera_{idx}" for idx in range(len(camera_topics))]
        if camera_topics and len(camera_topics) != len(camera_keys):
            raise ValueError("camera_topics and camera_keys must have the same length.")
        self.camera_keys = camera_keys

        self.image_subs = []
        for topic, key in zip(camera_topics, camera_keys):
            self.image_subs.append(
                self.create_subscription(Image, topic, self._make_image_callback(key), 1)
            )

        period = 1.0 / float(self.get_parameter("fps").value)
        self.timer = self.create_timer(period, self._run_once)

        mode = "ENABLED" if self.enable_output else "DISABLED"
        self.get_logger().info(
            f"LeRobot system ready: side={self.side}, state={state_topic}, output={mode}, "
            f"policy_type={self.runner.policy_type}, state_dim={self.required_state_dim}, "
            f"action_dim={self.expected_action_dim}, image_keys={self.camera_keys}, "
            f"robot_action_topic={robot_action_topic}"
        )
        if self.camera_keys and not camera_topics:
            self.get_logger().warn(
                "Policy expects camera observations, but camera_topics is empty. "
                "The node will wait until matching image topics are configured."
            )

    def _build_ruckig_smoother(self):
        if not bool(self.get_parameter("enable_ruckig").value):
            return None
        fps = float(self.get_parameter("fps").value)
        if fps <= 0:
            self.get_logger().error("Ruckig requested but fps<=0; smoothing disabled.")
            return None
        vmax = list(self.get_parameter("ruckig_max_velocity").value) or [3.141]
        amax = list(self.get_parameter("ruckig_max_acceleration").value) or [12.0]
        jmax = list(self.get_parameter("ruckig_max_jerk").value) or [500.0]
        mode = self.get_parameter("ruckig_target_velocity_mode").value or "zero"
        try:
            from lerobot_system.ruckig_smoother import RuckigSmoother

            smoother = RuckigSmoother(
                dof=self.expected_action_dim,
                control_dt=1.0 / fps,
                max_velocity=vmax,
                max_acceleration=amax,
                max_jerk=jmax,
                target_velocity_mode=mode,
            )
            self.get_logger().info(
                f"Ruckig smoother ENABLED: dof={self.expected_action_dim}, dt={1.0/fps:.4f}s, "
                f"vmax={vmax}, amax={amax}, jmax={jmax}, mode={mode}"
            )
            return smoother
        except Exception as exc:  # ruckig missing or bad limits -> run without smoothing
            self.get_logger().error(
                f"Failed to initialize Ruckig smoother ({exc}); continuing WITHOUT smoothing. "
                "Install it in the ROS env with `pip install ruckig`."
            )
            return None

    def _default_gripper_topic(self) -> str:
        if self.side == "left":
            return "/dg5f_left/lj_dg_pospid/reference"
        return "/inspire/right/target"

    def _resolve_gripper_command_type(self, command_type: str) -> str:
        if command_type != "auto":
            return command_type
        return "multi_dof_command" if self.side == "left" else "float64_multi_array"

    def _create_gripper_publisher(self, topic: str):
        if not self.enable_gripper_output or self.gripper_action_size <= 0:
            return None
        if self.gripper_command_type == "multi_dof_command":
            return self.create_publisher(MultiDOFCommand, topic, 1)
        if self.gripper_command_type == "float64_multi_array":
            return self.create_publisher(Float64MultiArray, topic, 1)
        if self.gripper_command_type == "none":
            return None
        raise ValueError(f"Unsupported gripper_command_type: {self.gripper_command_type}")

    def _on_state(self, msg: FrameAlignedState) -> None:
        if msg.side and msg.side != self.side:
            return
        with self.lock:
            self.latest_state = msg

    def _make_image_callback(self, key: str):
        def _callback(msg: Image) -> None:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            image = self._resize_image_for_policy(key, image)
            with self.lock:
                self.latest_images[key] = np.asarray(image, dtype=np.uint8)
                self.latest_image_times[key] = self.get_clock().now().nanoseconds / 1e9

        return _callback

    def _resize_image_for_policy(self, key: str, image: np.ndarray) -> np.ndarray:
        shape = self.image_shapes.get(key)
        if not shape or len(shape) != 3:
            return image

        channels, height, width = shape
        if channels != 3:
            self.get_logger().warn(f"Unexpected channel count for {key}: {channels}")
            return image
        if image.shape[:2] == (height, width):
            return image

        try:
            import cv2
        except Exception as exc:
            raise RuntimeError(
                f"{key} must be resized to {(height, width)}, but OpenCV is not available."
            ) from exc

        return cv2.resize(image, (int(width), int(height)), interpolation=cv2.INTER_AREA)

    def _run_once(self) -> None:
        with self.lock:
            state = self.latest_state
            images = dict(self.latest_images)
            image_times = dict(self.latest_image_times)

        if state is None:
            return
        if not self._images_ready(images, image_times):
            return

        observation = self._build_observation(state, images)
        try:
            action = self.runner.select_action(observation)
        except Exception as exc:
            self.get_logger().error(f"LeRobot inference failed: {exc}")
            return

        # publish the raw (un-smoothed) policy output for logging/debug
        self.raw_action_pub.publish(Float64MultiArray(data=[float(x) for x in action]))

        # apply Ruckig jerk-limited smoothing before sending commands to the robot
        command = action
        if self.smoother is not None:
            try:
                command = self.smoother.step(action)
            except Exception as exc:
                self.get_logger().warn(f"Ruckig smoothing failed ({exc}); sending raw action.")
                command = action

        robot_action = self._slice_action(command, self.robot_action_start, self.robot_action_size)
        if robot_action is not None:
            self._publish_robot_action(robot_action, state)

        gripper_action = self._slice_action(command, self.gripper_action_start, self.gripper_action_size)
        if gripper_action is not None:
            self._publish_gripper_action(gripper_action)

    def _images_ready(self, images: Dict[str, np.ndarray], image_times: Dict[str, float]) -> bool:
        if not self.camera_keys:
            return True

        now_sec = self.get_clock().now().nanoseconds / 1e9
        for key in self.camera_keys:
            if key not in images:
                return False
            if now_sec - image_times.get(key, 0.0) > self.camera_timeout_sec:
                return False
        return True

    def _build_observation(self, state: FrameAlignedState, images: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        pieces: List[np.ndarray] = []
        for field in self.state_fields:
            pieces.append(np.asarray(getattr(state, field), dtype=np.float32))

        state_vector = np.concatenate(pieces).astype(np.float32) if pieces else np.zeros(0, dtype=np.float32)
        state_vector = self._fit_state_dim(state_vector)
        observation = {
            self.state_key: state_vector
        }
        observation.update(images)
        return observation

    def _fit_state_dim(self, state_vector: np.ndarray) -> np.ndarray:
        if self.required_state_dim <= 0 or len(state_vector) == self.required_state_dim:
            return state_vector

        if self.state_padding_mode != "pad_or_truncate":
            raise ValueError(
                f"State dim mismatch: built={len(state_vector)} required={self.required_state_dim}. "
                "Set state_padding_mode:=pad_or_truncate to allow automatic fitting."
            )

        if not self.warned_state_dim_mismatch:
            self.get_logger().warn(
                f"State dim mismatch: built={len(state_vector)} required={self.required_state_dim}. "
                "Applying pad/truncate fallback. Check state_fields and policy config."
            )
            self.warned_state_dim_mismatch = True

        if len(state_vector) < self.required_state_dim:
            return np.pad(state_vector, (0, self.required_state_dim - len(state_vector))).astype(np.float32)
        return state_vector[: self.required_state_dim].astype(np.float32)

    def _slice_action(self, action: np.ndarray, start: int, size: int) -> Optional[np.ndarray]:
        if size <= 0:
            return None
        end = start + size
        if len(action) < end:
            self.get_logger().warn(f"Action too small: got={len(action)} need={end}")
            return None
        return np.asarray(action[start:end], dtype=np.float64)

    def _publish_robot_action(self, robot_action: np.ndarray, state: FrameAlignedState) -> None:
        target = convert_units(robot_action, self.action_output_unit, self.robot_topic_unit)

        if self.max_joint_delta > 0.0:
            current = convert_units(np.asarray(state.robot_joint, dtype=np.float64), "rad", self.robot_topic_unit)
            delta = np.clip(target - current, -self.max_joint_delta, self.max_joint_delta)
            target = current + delta

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["j1", "j2", "j3", "j4", "j5", "j6"][: len(target)]
        msg.position = [float(x) for x in target]

        if self.enable_output:
            self.robot_action_pub.publish(msg)

    def _publish_gripper_action(self, gripper_action: np.ndarray) -> None:
        if not self.enable_output or self.gripper_pub is None:
            return

        if self.gripper_command_type == "multi_dof_command":
            msg = MultiDOFCommand()
            msg.dof_names = LEFT_DELTO_JOINT_NAMES[: len(gripper_action)]
            msg.values = [float(x) for x in gripper_action]
            msg.values_dot = [0.0] * len(gripper_action)
            self.gripper_pub.publish(msg)
        elif self.gripper_command_type == "float64_multi_array":
            self.gripper_pub.publish(Float64MultiArray(data=[float(x) for x in gripper_action]))


def main(args=None, forced_side: Optional[str] = None):
    rclpy.init(args=args)
    node = LeRobotSystemNode(forced_side=forced_side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def left_main(args=None):
    main(args=args, forced_side="left")


def right_main(args=None):
    main(args=args, forced_side="right")


if __name__ == "__main__":
    main()
