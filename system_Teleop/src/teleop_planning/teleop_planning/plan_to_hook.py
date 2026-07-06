#!/usr/bin/env python3
"""Plan the HDR35 end-effector to a detected hook 6D pose (standalone, no VLA).

Pipeline:  hook_pose (PoseStamped, base, **mm**)  ->  grasp/approach EE pose (m)
           ->  MoveIt MoveGroup action (plan, optionally execute).

The goal pose is derived from the hook pose by a configurable grasp offset
(``T_hook_from_grasp``) plus an approach stand-off along one grasp-frame axis. Both the
grasp pose and the stand-off ("pregrasp") are published as PoseStamped + TF so the
offset can be tuned live in RViz before anything moves.

Talks to move_group via ``moveit_msgs/action/MoveGroup`` directly — no pymoveit2/moveit_py
needed. Execution is gated: ``~/plan`` only plans (preview); ``~/execute`` plans+executes
and additionally requires the ``allow_execute`` param to be true.

    ros2 run teleop_planning plan_to_hook --ros-args -p side:=right \
        -p grasp_offset_xyz_mm:="[0.0, 0.0, 0.0]" \
        -p grasp_offset_rpy_deg:="[0.0, 0.0, 0.0]" \
        -p approach_dist_mm:=100.0 -p approach_axis:=z
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    MotionPlanRequest, PlanningOptions, WorkspaceParameters, BoundingVolume,
)


def make_T(rot, trans):
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = np.asarray(trans, float).reshape(3)
    return T


def pose_to_T(pose: Pose):
    """geometry Pose (quaternion) -> 4x4. Units pass through (mm in, mm out)."""
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot = R.from_quat(q).as_matrix() if any(q) else np.eye(3)
    return make_T(rot, [pose.position.x, pose.position.y, pose.position.z])


def T_to_pose_m(T):
    """4x4 in mm -> geometry Pose in metres."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = (T[:3, 3] / 1000.0).tolist()
    q = R.from_matrix(T[:3, :3]).as_quat()
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return p


def T_from_xyz_rpy(xyz, rpy_deg):
    rot = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    return make_T(rot, xyz)


class PlanToHook(Node):
    def __init__(self):
        super().__init__("plan_to_hook")
        self.declare_parameter("side", "right")
        self.declare_parameter("hook_pose_topic", "/hook_pose")
        self.declare_parameter("group", "hdr_manipulator")
        self.declare_parameter("ee_link", "tool0")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("pos_tol_m", 0.01)
        self.declare_parameter("ori_tol_rad", 0.05)
        self.declare_parameter("grasp_offset_xyz_mm", [0.0, 0.0, 0.0])
        self.declare_parameter("grasp_offset_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("approach_dist_mm", 100.0)
        self.declare_parameter("approach_axis", "z")
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("vel_scale", 0.1)
        self.declare_parameter("acc_scale", 0.1)
        self.declare_parameter("allow_execute", False)

        self.group = self.get_parameter("group").value
        self.ee_link = self.get_parameter("ee_link").value
        self.planning_frame = self.get_parameter("planning_frame").value

        self.latest_hook: PoseStamped | None = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_grasp = self.create_publisher(PoseStamped, "~/grasp_pose", 1)
        self.pub_pregrasp = self.create_publisher(PoseStamped, "~/pregrasp_pose", 1)

        # Reentrant group + MultiThreadedExecutor so the blocking action wait inside the
        # service callback doesn't deadlock the executor (single-threaded would).
        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseStamped, self.get_parameter("hook_pose_topic").value, self._on_hook, 10)
        self.create_service(Trigger, "~/plan", self._srv_plan, callback_group=self.cb_group)
        self.create_service(Trigger, "~/execute", self._srv_execute, callback_group=self.cb_group)

        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.get_logger().info(
            f"plan_to_hook ready: group={self.group}, ee={self.ee_link}, "
            f"frame={self.planning_frame}. Preview on ~/grasp_pose & ~/pregrasp_pose; "
            f"call ~/plan to plan, ~/execute to move (allow_execute required).")

    # ---- target geometry --------------------------------------------------
    def _compute_targets(self):
        """Return (grasp_pose_m, pregrasp_pose_m) as Pose in planning frame, or None."""
        if self.latest_hook is None:
            return None
        T_pf_hook = pose_to_T(self.latest_hook.pose)  # mm
        off_xyz = list(self.get_parameter("grasp_offset_xyz_mm").value)
        off_rpy = list(self.get_parameter("grasp_offset_rpy_deg").value)
        T_pf_grasp = T_pf_hook @ T_from_xyz_rpy(off_xyz, off_rpy)

        axis = {"x": 0, "y": 1, "z": 2}[str(self.get_parameter("approach_axis").value)]
        d = float(self.get_parameter("approach_dist_mm").value)
        standoff = np.zeros(3)
        standoff[axis] = -d  # pregrasp sits behind grasp along the approach axis
        T_pf_pregrasp = T_pf_grasp @ make_T(np.eye(3), standoff)
        return T_to_pose_m(T_pf_grasp), T_to_pose_m(T_pf_pregrasp)

    def _on_hook(self, msg: PoseStamped):
        self.latest_hook = msg
        out = self._compute_targets()
        if out is None:
            return
        grasp, pregrasp = out
        now = self.get_clock().now().to_msg()
        for pub, pose, child in ((self.pub_grasp, grasp, "grasp_target"),
                                 (self.pub_pregrasp, pregrasp, "pregrasp_target")):
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self.planning_frame
            ps.pose = pose
            pub.publish(ps)
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.planning_frame
            tf.child_frame_id = child
            tf.transform.translation = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
            tf.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    # ---- MoveGroup goal ---------------------------------------------------
    def _build_goal(self, target: Pose, plan_only: bool) -> MoveGroup.Goal:
        pos_tol = float(self.get_parameter("pos_tol_m").value)
        ori_tol = float(self.get_parameter("ori_tol_rad").value)

        pc = PositionConstraint()
        pc.header.frame_id = self.planning_frame
        pc.link_name = self.ee_link
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        region_pose = Pose()
        region_pose.position = target.position
        region_pose.orientation.w = 1.0
        bv.primitive_poses.append(region_pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.planning_frame
        oc.link_name = self.ee_link
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        req = MotionPlanRequest()
        req.group_name = self.group
        req.goal_constraints.append(constraints)
        req.num_planning_attempts = 10
        req.allowed_planning_time = float(self.get_parameter("planning_time").value)
        req.max_velocity_scaling_factor = float(self.get_parameter("vel_scale").value)
        req.max_acceleration_scaling_factor = float(self.get_parameter("acc_scale").value)
        ws = WorkspaceParameters()
        ws.header.frame_id = self.planning_frame
        ws.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
        ws.max_corner = Vector3(x=2.0, y=2.0, z=2.0)
        req.workspace_parameters = ws

        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = plan_only
        goal.planning_options = opts
        return goal

    def _wait(self, future, timeout_sec):
        """Wait for a future without nested spinning (executor runs on other threads)."""
        t0 = time.time()
        while not future.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _plan_to(self, target: Pose, plan_only: bool):
        """Send goal, wait for result. Returns (ok, message)."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return False, "move_group action server /move_action not available"
        goal = self._build_goal(target, plan_only)
        gh = self._wait(self.move_client.send_goal_async(goal), 10.0)
        if gh is None:
            return False, "no goal response from move_group (timeout)"
        if not gh.accepted:
            return False, "goal rejected by move_group"
        result = self._wait(gh.get_result_async(), 60.0)
        if result is None:
            return False, "no result from move_group (timeout)"
        code = result.result.error_code.val
        n = len(result.result.planned_trajectory.joint_trajectory.points)
        ok = (code == 1)  # SUCCESS
        return ok, f"error_code={code}, trajectory_points={n}, plan_only={plan_only}"

    # ---- services ---------------------------------------------------------
    def _srv_plan(self, request, response):
        out = self._compute_targets()
        if out is None:
            response.success = False
            response.message = "no hook pose received yet"
            return response
        _, pregrasp = out
        ok, msg = self._plan_to(pregrasp, plan_only=True)
        response.success = ok
        response.message = f"[plan-only to pregrasp] {msg}"
        self.get_logger().info(response.message)
        return response

    def _srv_execute(self, request, response):
        if not bool(self.get_parameter("allow_execute").value):
            response.success = False
            response.message = "execution blocked: set param allow_execute:=true to enable"
            return response
        out = self._compute_targets()
        if out is None:
            response.success = False
            response.message = "no hook pose received yet"
            return response
        _, pregrasp = out
        ok, msg = self._plan_to(pregrasp, plan_only=False)
        response.success = ok
        response.message = f"[execute to pregrasp] {msg}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanToHook()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
