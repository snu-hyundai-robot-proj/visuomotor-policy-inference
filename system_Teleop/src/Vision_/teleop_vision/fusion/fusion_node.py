#!/usr/bin/env python3
"""Fuse the Zivid + D405 clouds into one base-frame cloud, on demand.

This node OWNS both cameras, so the Zivid is exclusive: stop the regular vision node
(`teleop_vision system_vision_<side>`) before running this, or fold `fuse()` into that
node instead (see README). On each `~/fuse` trigger it:

  1. captures a Zivid 2D+3D frame  -> cloud in zivid frame (mm)
  2. captures a D405 depth+color   -> cloud in d405 frame (mm)
  3. reads the latest flange pose  -> T_flange2base
  4. transforms both to base, ICP-refines the wrist cloud, merges
  5. saves a coloured .ply (mm) and publishes a PointCloud2 (metres, frame 'base')

    ros2 run teleop_vision fusion_node --ros-args -p side:=right \
        -p handeye_path:=/workspace/src/Vision_/camera/T_flange_from_d405_right.npy \
        -p euler_order:=xyz
"""
from __future__ import annotations

import os
import time

import numpy as np
import open3d as o3d
import rclpy
import zivid
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from .fuse import fuse, zivid_xyz_to_cloud
from .integration import cloud_to_pointcloud2
from .rs_cloud import RealSenseD405
from .transforms import make_T, pose_msg_to_T

from geometry_msgs.msg import Pose

# Static Zivid extrinsics (T_zivid2base), mm + xyz-Euler deg — same values the vision node uses.
ZIVID_EXTRINSIC = {
    "left":  dict(t=(-1478.151, -972.564, 1603.593), rpy=(-121.411, -2.267, -88.093)),
    "right": dict(t=(-1464.766, 996.191, 1663.695),  rpy=(-119.828, 5.292, -96.986)),
}
ZIVID_SERIAL = {"left": "23352865", "right": "2051707B"}
ZIVID_YML = {"left": "camera_setting_left.yml", "right": "camera_setting_right.yml"}
RS_SERIALS = {"left": "409122273797", "right": "409122273122"}


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")
        self.declare_parameter("side", "right")
        self.declare_parameter("handeye_path", "")
        self.declare_parameter("euler_order", "xyz")
        self.declare_parameter("voxel_mm", 2.0)
        self.declare_parameter("do_icp", True)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("rs_any", os.environ.get("VISION_RS_ANY") is not None)

        self.side = str(self.get_parameter("side").value).lower()
        self.euler_order = str(self.get_parameter("euler_order").value)
        self.voxel_mm = float(self.get_parameter("voxel_mm").value)
        self.do_icp = bool(self.get_parameter("do_icp").value)

        home = os.getcwd()
        self.package_dir = os.path.join(home, "src", "Vision_")
        out = str(self.get_parameter("output_dir").value).strip()
        self.output_dir = out or os.path.join(home, "Record", self.side, "fused")
        os.makedirs(self.output_dir, exist_ok=True)

        # --- static Zivid extrinsic ---
        ext = ZIVID_EXTRINSIC[self.side]
        self.T_zivid2base = make_T(
            R.from_euler("xyz", ext["rpy"], degrees=True).as_matrix(), ext["t"])

        # --- hand-eye (T_flange_from_d405) ---
        he_path = str(self.get_parameter("handeye_path").value).strip() or \
            os.path.join(self.package_dir, "camera", f"T_flange_from_d405_{self.side}.npy")
        if not os.path.isfile(he_path):
            raise FileNotFoundError(
                f"Hand-eye file not found: {he_path}. Run `ros2 run teleop_vision "
                f"handeye_calibrate -p side:={self.side}` first.")
        self.T_flange_from_d405 = np.load(he_path)
        self.get_logger().info(f"Loaded hand-eye from {he_path}")

        self.latest_pose: Pose | None = None
        self.create_subscription(Pose, f"/system_{self.side}/pose_states", self._on_pose, 10)

        self._init_zivid()
        serial = None if bool(self.get_parameter("rs_any").value) else RS_SERIALS.get(self.side)
        self.d405 = RealSenseD405(serial=serial)

        self.cloud_pub = self.create_publisher(PointCloud2, f"/system_{self.side}/fused_cloud", 1)
        self.create_service(Trigger, "~/fuse", self._on_fuse)
        self.index = 1
        self.get_logger().info(f"FusionNode ready (side={self.side}). Call '~/fuse' to capture.")

    def _on_pose(self, msg: Pose):
        self.latest_pose = msg

    def _init_zivid(self):
        self.zivid_app = zivid.Application()
        serial = ZIVID_SERIAL[self.side]
        last = None
        for attempt in range(1, 11):
            try:
                self.camera = self.zivid_app.connect_camera(serial_number=serial)
                break
            except Exception as e:
                last = e
                self.get_logger().warning(f"Zivid connect {attempt}/10 failed: {e}")
                time.sleep(2.0)
        else:
            raise RuntimeError(f"Could not connect Zivid {serial}: {last}")
        yml = os.path.join(self.package_dir, "camera", ZIVID_YML[self.side])
        self.settings_3d = zivid.Settings.load(yml)
        self.get_logger().info(f"Zivid {serial} connected; settings {yml}")

    # ---- fusion service ---------------------------------------------------
    def _on_fuse(self, request, response):
        try:
            if self.latest_pose is None:
                response.success = False
                response.message = "No flange pose on pose_states yet."
                return response

            frame = self.camera.capture_2d_3d(self.settings_3d)
            xyz = frame.point_cloud().copy_data("xyz")
            rgba = frame.point_cloud().copy_data("rgba")
            zivid_cam = zivid_xyz_to_cloud(xyz, rgba)

            _, d405_cam = self.d405.capture()

            T_flange2base = pose_msg_to_T(self.latest_pose, order=self.euler_order)
            T_d405_2base = T_flange2base @ self.T_flange_from_d405

            fused, info = fuse(
                zivid_cam, self.T_zivid2base, d405_cam, T_d405_2base,
                voxel_mm=self.voxel_mm, do_icp=self.do_icp)

            ply = os.path.join(self.output_dir, f"{self.side}_{self.index:03d}_fused.ply")
            o3d.io.write_point_cloud(ply, fused)
            self.cloud_pub.publish(
                cloud_to_pointcloud2(fused, "base", self.get_clock().now().to_msg()))
            self.index += 1

            msg = (f"fused {info['n_fused']} pts (zivid={info['n_zivid']}, "
                   f"d405={info['n_d405']}, icp={info['icp_applied']}/"
                   f"fit={info['icp_fitness']:.2f}) -> {ply}")
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        except Exception as e:
            self.get_logger().error(f"fuse failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        try:
            self.d405.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
