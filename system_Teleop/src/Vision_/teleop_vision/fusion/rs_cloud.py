"""Intel RealSense D405 wrist camera: depth+color capture -> metric point cloud.

The existing vision node opens the D405 in *color only*. For fusion we additionally
need depth, so this helper owns its own pipeline (depth aligned to color). Output is an
Open3D cloud expressed in the **color optical frame, in millimetres** — matching the
Zivid convention so both clouds live in the same units once transformed to base.

D405 minimum range is ~7 cm; the default depth clip reflects that.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d
import pyrealsense2 as rs


def deproject_to_cloud(depth_m, rgb, fx, fy, cx, cy, depth_range_m=(0.07, 1.0)):
    """Deproject an aligned depth image (metres) + rgb to an Open3D cloud in **mm**.

    Shared by :class:`RealSenseD405` and the embedded vision-node path so both produce
    identical clouds. ``rgb`` must already be aligned to the depth/color frame.
    """
    h, w = depth_m.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_m
    valid = (z > depth_range_m[0]) & (z < depth_range_m[1]) & np.isfinite(z)
    x = (uu - cx) / fx * z
    y = (vv - cy) / fy * z
    pts_m = np.stack((x, y, z), axis=-1)[valid]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_m * 1000.0)  # -> mm
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb[valid].astype(np.float64) / 255.0)
    return pcd


class RealSenseD405:
    def __init__(
        self,
        serial: str | None = None,
        width: int = 848,
        height: int = 480,
        fps: int = 30,
        depth_range_m: tuple[float, float] = (0.07, 1.0),
    ):
        self.depth_min_m, self.depth_max_m = depth_range_m
        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)  # depth -> color frame

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())  # raw units -> metres

        # After alignment, depth shares the color intrinsics.
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self.width, self.height = intr.width, intr.height
        self.fx, self.fy, self.cx, self.cy = intr.fx, intr.fy, intr.ppx, intr.ppy
        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1]], dtype=np.float64)
        self.dist = np.array(intr.coeffs, dtype=np.float64)  # usually ~0 (rectified)

    def _grab_aligned(self, timeout_ms: int = 2000):
        frames = self.pipeline.wait_for_frames(timeout_ms)
        frames = self.align.process(frames)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("D405: incomplete depth/color frame")
        depth_m = np.asanyarray(depth.get_data()).astype(np.float64) * self.depth_scale
        rgb = np.asanyarray(color.get_data())  # HxWx3 uint8 (rgb8)
        return depth_m, rgb

    def capture(self, timeout_ms: int = 2000):
        """Return (rgb_uint8 HxWx3, o3d.PointCloud in color frame, **mm**, coloured)."""
        depth_m, rgb = self._grab_aligned(timeout_ms)
        pcd = deproject_to_cloud(depth_m, rgb, self.fx, self.fy, self.cx, self.cy,
                                 (self.depth_min_m, self.depth_max_m))
        return rgb, pcd

    def capture_color(self, timeout_ms: int = 2000) -> np.ndarray:
        """Just the rgb image (used by hand-eye calibration)."""
        _, rgb = self._grab_aligned(timeout_ms)
        return rgb

    def close(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
