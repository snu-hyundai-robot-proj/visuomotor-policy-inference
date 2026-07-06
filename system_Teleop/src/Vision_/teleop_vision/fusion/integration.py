"""Glue helpers so the existing vision node can fuse without owning a 2nd RS pipeline.

The vision node already holds one ``rs.pipeline`` (color). These helpers reuse it: we
add a depth stream + an ``rs.align`` in the node, then grab one aligned frame on demand
(under the node's RealSense lock, so the publish thread doesn't race us).
"""
from __future__ import annotations

import os

import cv2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField

from .rs_cloud import deproject_to_cloud


def load_handeye(path: str, logger=None):
    """Load T_flange_from_d405 (.npy). Returns None (fusion disabled) if absent."""
    if path and os.path.isfile(path):
        T = np.load(path)
        if logger:
            logger.info(f"[fusion] loaded hand-eye {path}")
        return T
    if logger:
        logger.warn(f"[fusion] hand-eye not found ({path}); fuse service disabled. "
                    "Run `ros2 run teleop_vision handeye_calibrate` first.")
    return None


def grab_d405_cloud(pipeline, align, fx, fy, cx, cy, depth_scale, lock,
                    depth_range_m=(0.07, 1.0), color_is_bgr=True, timeout_ms=2000):
    """One aligned depth+color frame from the node's existing pipeline -> cloud (mm)."""
    with lock:
        frames = pipeline.wait_for_frames(timeout_ms)
        frames = align.process(frames)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("D405: incomplete depth/color frame")
        depth_m = np.asanyarray(depth.get_data()).astype(np.float64) * depth_scale
        rgb = np.asanyarray(color.get_data()).copy()
    if color_is_bgr:
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    return deproject_to_cloud(depth_m, rgb, fx, fy, cx, cy, depth_range_m)


def cloud_to_pointcloud2(pcd: o3d.geometry.PointCloud, frame_id: str, stamp) -> PointCloud2:
    """Open3D cloud (mm) -> sensor_msgs/PointCloud2 (metres, xyzrgb) for RViz."""
    pts_m = (np.asarray(pcd.points) / 1000.0).astype(np.float32)
    cols = np.asarray(pcd.colors)
    if len(cols) == len(pts_m) and len(cols):
        u8 = (cols * 255).astype(np.uint32)
        packed = (u8[:, 0] << 16) | (u8[:, 1] << 8) | u8[:, 2]
    else:
        packed = np.zeros(len(pts_m), dtype=np.uint32)

    rec = np.zeros(len(pts_m), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    rec["x"], rec["y"], rec["z"] = pts_m[:, 0], pts_m[:, 1], pts_m[:, 2]
    rec["rgb"] = packed

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(pts_m)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(pts_m)
    msg.is_dense = True
    msg.data = rec.tobytes()
    return msg
