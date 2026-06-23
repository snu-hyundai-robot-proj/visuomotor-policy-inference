#!/usr/bin/env python3
# -*- coding: utf-8 -*-
LEFT_DELTO_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]

import os
import math
from typing import Tuple

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState


class ParquetReplayBuffer:
    """
    parquet에서 읽은 replay 데이터를 메모리에 저장하는 클래스입니다.

    저장 데이터:
        timestamps      : shape = (N,)
        gripper_action  : shape = (N, 20)
        robot_joint     : shape = (N, 6)
    """

    def __init__(
        self,
        parquet_path: str,
        use_degree: bool = False,
        loop: bool = False,
        left_side: bool = False,
    ):
        self.parquet_path = parquet_path
        self.use_degree = use_degree

        self.loop = loop

        self.index = 0

        self.timestamps: np.ndarray | None = None
        self.gripper_action: np.ndarray | None = None
        self.robot_joint: np.ndarray | None = None
        self.left_side = left_side
        
        self._load_parquet()

    def _load_parquet(self) -> None:
        if not os.path.exists(self.parquet_path):
            raise FileNotFoundError(f"parquet file not found: {self.parquet_path}")

        df = pd.read_parquet(self.parquet_path)

        required_columns = [
            "timestamp",
            "action",
            "observation.state",
        ]

        for col in required_columns:
            if col not in df.columns:
                raise KeyError(
                    f"required column not found in parquet: {col}\n"
                    f"available columns: {list(df.columns)}"
                )

        timestamps = df["timestamp"].to_numpy(dtype=np.float64)
        action = np.array(df["action"].tolist(), dtype=np.float64)
        observ = np.array(df["observation.state"].tolist(), dtype=np.float64)

        if timestamps.shape[0] == 0:
            raise ValueError("timestamp data is empty")

        if action.ndim != 2:
            raise ValueError(f"action must be 2D array, but got shape: {action.shape}")

        if observ.ndim != 2:
            raise ValueError(
                f"observation.state must be 2D array, but got shape: {observ.shape}"
            )

        if action.shape[1] < 26:
            raise ValueError(
                f"action length must be at least 26. "
                f"Expected action[6:26] for gripper 20 joints, "
                f"but got shape: {action.shape}"
            )

        if observ.shape[1] < 6:
            raise ValueError(
                f"observation.state length must be at least 6. "
                f"Expected observation.state[0:6] for robot 6 joints, "
                f"but got shape: {observ.shape}"
            )

        # timestamp, action, observ row 개수가 모두 같은지 검사
        row_count = timestamps.shape[0]

        if action.shape[0] != row_count:
            raise ValueError(
                f"timestamp/action row count mismatch: "
                f"{row_count} vs {action.shape[0]}"
            )

        if observ.shape[0] != row_count:
            raise ValueError(
                f"timestamp/observation.state row count mismatch: "
                f"{row_count} vs {observ.shape[0]}"
            )

        # gripper action 20개를 저장
        if self.left_side:
            gripper_action = action[:, 6:26]    # Tesollo
        else:
            gripper_action = action[:, 6:12]    # Inspire

        # robot joint 6개를 저장
        robot_joint = observ[:, 0:6]

        # 기본값은 Radian
        if self.use_degree:
            gripper_action = gripper_action * (180.0 / math.pi)
        if not self.left_side:
            gripper_action = gripper_action * (1800.0 / math.pi)
            
        # 최종 배열을 멤버 변수에 저장
        self.timestamps = timestamps
        self.gripper_action = gripper_action
        self.robot_joint = robot_joint * (180.0 / math.pi)

    @property
    def size(self) -> int:
        if self.timestamps is None:
            return 0

        return int(self.timestamps.shape[0])

    def is_finished(self) -> bool:
        return (not self.loop) and (self.index >= self.size)

    def get_current_sample(self) -> Tuple[float, np.ndarray, np.ndarray]:

        if self.is_finished():
            raise IndexError("replay buffer is finished")

        current_index = self.index % self.size

        timestamp = float(self.timestamps[current_index])
        gripper_sample = self.gripper_action[current_index].copy()
        robot_sample = self.robot_joint[current_index].copy()

        return timestamp, gripper_sample, robot_sample

    def step(self) -> None:
        if self.loop:
            self.index = (self.index + 1) % self.size
        else:
            self.index += 1

    def reset(self) -> None:
        self.index = 0


class ParquetReplayNode(Node):
    def __init__(self):
        super().__init__("parquet_replay_node")
        self.declare_parameter("parquet_path", "")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("use_degree", False)
        self.declare_parameter("left_side", True)

        # gripper topic 이름 parameter
        self.declare_parameter("gripper_topic", "/gripper/command")

        # robot topic 이름 parameter
        self.declare_parameter("robot_topic", "/robot/joint_state_array")
        self.declare_parameter("gripper_dof_prefix", "gripper_joint_")

        # parquet 경로
        parquet_path = str(self.get_parameter("parquet_path").value)

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        self.loop = bool(
            self.get_parameter("loop").value
        )

        self.use_degree = bool(
            self.get_parameter("use_degree").value
        )

        self.gripper_topic = str(
            self.get_parameter("gripper_topic").value
        )

        self.robot_topic = str(
            self.get_parameter("robot_topic").value
        )

        self.gripper_dof_prefix = str(
            self.get_parameter("gripper_dof_prefix").value
        )

        self.left_side = bool(
            self.get_parameter("left_side").value
        )
        # parquet_path가 비어 있으면 실행할 수 없습니다.
        if parquet_path == "":
            raise ValueError(
                "parquet_path parameter is empty. "
                "Run with: --ros-args -p parquet_path:=/path/to/file.parquet"
            )

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than 0")

        self.timer_period_sec = 1.0 / self.publish_rate_hz

        # gripper 20개 DoF 이름입니다.
        self.gripper_dof_names = LEFT_DELTO_JOINT_NAMES

        self.buffer = ParquetReplayBuffer(
            parquet_path=parquet_path,
            use_degree=self.use_degree,
            loop=self.loop,
            left_side=self.left_side
        )

        if self.left_side:
            self.gripper_pub = self.create_publisher(
                MultiDOFCommand,
                self.gripper_topic,
                10,
            )
        else:
            self.gripper_pub = self.create_publisher(
                Float64MultiArray,
                self.gripper_topic, 
                10,
            )

        self.robot_pub = self.create_publisher(
            JointState,
            self.robot_topic,
            10,
        )

        # 30Hz timer
        self.timer = self.create_timer(
            self.timer_period_sec,
            self.on_timer,
        )

        # 시작 로그
        self.get_logger().info("========== PARQUET REPLAY NODE START ==========")
        self.get_logger().info(f"parquet_path          : {parquet_path}")
        self.get_logger().info(f"sample count          : {self.buffer.size}")
        self.get_logger().info(f"publish_rate_hz       : {self.publish_rate_hz}")
        self.get_logger().info(f"timer_period_sec      : {self.timer_period_sec:.9f}")
        self.get_logger().info(f"loop                  : {self.loop}")
        self.get_logger().info(f"use_degree            : {self.use_degree}")
        self.get_logger().info(f"gripper_topic         : {self.gripper_topic}")
        self.get_logger().info(f"robot_topic           : {self.robot_topic}")
        self.get_logger().info(f"gripper_action shape  : {self.buffer.gripper_action.shape}")
        self.get_logger().info(f"robot_joint shape     : {self.buffer.robot_joint.shape}")
        self.get_logger().info(
            f"timestamp first/last  : "
            f"{self.buffer.timestamps[0]:.6f} / {self.buffer.timestamps[-1]:.6f}"
        )
        self.get_logger().info(
            f"gripper_dof_names[0]  : {self.gripper_dof_names[0]}"
        )
        self.get_logger().info(
            f"gripper_dof_names[-1] : {self.gripper_dof_names[-1]}"
        )

    def make_gripper_command_msg(self, gripper_sample: np.ndarray) -> MultiDOFCommand:
        # gripper sample 길이 검사
        if gripper_sample.shape[0] != 20:
            raise ValueError(
                f"gripper_sample must have 20 values, "
                f"but got shape: {gripper_sample.shape}"
            )

        msg = MultiDOFCommand()
        msg.dof_names = self.gripper_dof_names
        msg.values = gripper_sample.tolist()
        msg.values_dot = [0.0] * 20

        return msg

    def make_inspire_command_msg(self, gripper_sample: np.ndarray) -> Float64MultiArray:
        # gripper sample 길이 검사
        if gripper_sample.shape[0] != 6:
            raise ValueError(
                f"gripper_sample must have 6 values, "
                f"but got shape: {gripper_sample.shape}"
            )
        msg = Float64MultiArray()
        msg.data = gripper_sample.tolist()

        return msg

    def make_robot_joint_msg(self, robot_sample: np.ndarray) -> JointState:
        if robot_sample.shape[0] != 6:
            raise ValueError(
                f"robot_sample must have 6 values, "
                f"but got shape: {robot_sample.shape}"
            )
        msg = JointState()
        msg.position = robot_sample.tolist()

        return msg

    def on_timer(self) -> None:
        if self.buffer.is_finished():
            self.get_logger().info("Replay finished. Timer stopped.")
            self.timer.cancel()
            return

        timestamp, gripper_sample, robot_sample = self.buffer.get_current_sample()

        if self.left_side:
            gripper_msg = self.make_gripper_command_msg(gripper_sample)
        else:
            gripper_msg = self.make_inspire_command_msg(gripper_sample)

        self.gripper_pub.publish(gripper_msg)

        robot_msg = self.make_robot_joint_msg(robot_sample)
        self.robot_pub.publish(robot_msg)

        if self.buffer.index % 30 == 0:
            self.get_logger().info(
                f"publish index={self.buffer.index}, "
                f"timestamp={timestamp:.6f}"
            )

        self.buffer.step()

    def stop(self) -> None:
        if hasattr(self, "timer") and self.timer is not None:
            self.timer.cancel()


def main(args=None):
    node = None

    try:
        rclpy.init(args=args)
        node = ParquetReplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            try:
                node.stop()
            except Exception:
                pass

        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
            
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()