#!/usr/bin/env python3
"""Bridge MoveIt execution to the HDR35: FollowJointTrajectory action -> OpenStream.

MoveIt's simple controller manager executes a planned trajectory by sending a
``control_msgs/action/FollowJointTrajectory`` goal to
``/joint_trajectory_controller/follow_joint_trajectory`` (per moveit_controllers.yaml).
The HDR35, however, is driven over OpenStream TCP (same mechanism as
examples/run_robot_loop.py): ``joint_traject_init`` then ``joint_traject_insert_point``
with joint positions in **degrees**.

This node accepts the FollowJointTrajectory goal and streams its points to the arm.

SAFETY: ``dry_run`` defaults to **true** — it logs the trajectory it WOULD send and does
NOT connect to or move the robot. Set ``dry_run:=false`` only with the arm powered,
workspace clear, and an e-stop in reach.

    ros2 run teleop_planning hdr_followjoint_bridge --ros-args -p side:=right -p dry_run:=true
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory

SEND_DT = 0.005
LOOK_AHEAD = 0.1
ROBOT_IP = {"right": "192.168.4.151", "left": "192.168.4.152"}
JOINT_NAMES = [f"j{i}" for i in range(1, 7)]


class HdrFollowJointBridge(Node):
    def __init__(self):
        super().__init__("hdr_followjoint_bridge")
        self.declare_parameter("side", "right")
        self.declare_parameter("robot_ip", "")
        self.declare_parameter("robot_port", 49000)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("action_name", "/joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("max_step_deg", 25.0)  # reject wild jumps between points

        self.side = str(self.get_parameter("side").value).lower()
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.robot_ip = str(self.get_parameter("robot_ip").value).strip() or ROBOT_IP.get(self.side, "")
        self.max_step_deg = float(self.get_parameter("max_step_deg").value)

        self.net = None
        self.api = None
        if not self.dry_run:
            self._connect()
        else:
            self.get_logger().warn(
                "DRY-RUN: not connecting to the robot; trajectories will be logged only. "
                "Set dry_run:=false (arm clear, e-stop ready) to actually move.")

        cb = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda c: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info(
            f"hdr_followjoint_bridge ready (side={self.side}, ip={self.robot_ip}, "
            f"dry_run={self.dry_run})")

    def _connect(self):
        from hdr_stream.utils.net import NetClient
        from hdr_stream.utils.api import OpenStreamAPI
        from hdr_stream.utils.parser import NDJSONParser
        from hdr_stream.utils.dispatcher import Dispatcher
        port = int(self.get_parameter("robot_port").value)
        self.net = NetClient(self.robot_ip, port)
        self.api = OpenStreamAPI(self.net)
        parser = NDJSONParser()
        disp = Dispatcher()
        self.net.connect()
        self.net.start_recv_loop(lambda b: parser.feed(b, disp.dispatch))
        self.api.handshake(major=1)
        time.sleep(0.5)
        self.api.joint_traject_init()
        self.get_logger().info(f"connected to HDR35 at {self.robot_ip}:{port} (OpenStream)")

    def _reorder(self, names, positions):
        """Map a trajectory point's positions onto j1..j6 order. Returns deg list or None."""
        idx = {n: i for i, n in enumerate(names)}
        if not all(j in idx for j in JOINT_NAMES):
            return None
        return [math.degrees(positions[idx[j]]) for j in JOINT_NAMES]

    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        pts = traj.points
        result = FollowJointTrajectory.Result()
        if not pts:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        deg_pts = []
        for p in pts:
            deg = self._reorder(traj.joint_names, list(p.positions))
            if deg is None:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = f"joint names {traj.joint_names} missing j1..j6"
                return result
            tfs = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            deg_pts.append((tfs, deg))

        # safety: reject inter-point jumps larger than max_step_deg
        for i in range(1, len(deg_pts)):
            step = max(abs(a - b) for a, b in zip(deg_pts[i][1], deg_pts[i - 1][1]))
            if step > self.max_step_deg:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"inter-point jump {step:.1f}deg > max_step_deg"
                self.get_logger().error(result.error_string)
                return result

        self.get_logger().info(
            f"executing trajectory: {len(deg_pts)} pts, duration "
            f"{deg_pts[-1][0]:.2f}s, dry_run={self.dry_run}")
        if self.dry_run:
            self.get_logger().info(f"  [dry-run] first={[f'{v:.1f}' for v in deg_pts[0][1]]} "
                                   f"last={[f'{v:.1f}' for v in deg_pts[-1][1]]}")

        t0 = time.time()
        fb = FollowJointTrajectory.Feedback()
        for tfs, deg in deg_pts:
            if goal_handle.is_cancel_requested:
                if not self.dry_run and self.api:
                    self.api.stop(target="control")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            now = time.time() - t0
            if tfs > now:
                time.sleep(tfs - now)
            if self.dry_run:
                pass
            else:
                self.api.joint_traject_insert_point({
                    "interval": SEND_DT,
                    "time_from_start": tfs,
                    "look_ahead_time": LOOK_AHEAD,
                    "point": [float(x) for x in deg],
                })
            fb.desired.positions = [math.radians(d) for d in deg]
            goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("trajectory execution complete")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HdrFollowJointBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.net is not None:
            try:
                node.net.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
