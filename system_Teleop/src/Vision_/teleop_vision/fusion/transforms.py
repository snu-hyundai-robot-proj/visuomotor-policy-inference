"""4x4 rigid-transform helpers, shared by calibration and fusion.

All translations are kept in **millimetres** to match the Zivid point cloud and the
HDR35 Cartesian pose (``/system_<side>/pose_states`` reports x/y/z in mm).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


def make_T(rot: np.ndarray, trans) -> np.ndarray:
    """Assemble a 4x4 homogeneous transform from a 3x3 rotation and a 3-vector."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rot, dtype=np.float64)
    T[:3, 3] = np.asarray(trans, dtype=np.float64).reshape(3)
    return T


def invert(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform (cheaper and more stable than np.linalg.inv)."""
    Rm = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti


def T_from_xyz_euler(xyz_mm, euler_deg, order: str = "xyz") -> np.ndarray:
    """Build T from a translation (mm) and Euler angles (degrees).

    ``order`` is any scipy convention string ('xyz', 'ZYX', ...). Intrinsic (capital)
    vs extrinsic (lowercase) matters — pick the one that matches your controller and
    verify with :mod:`handeye_calibrate`, which sweeps several orders and reports the
    residual for each.
    """
    rot = R.from_euler(order, np.asarray(euler_deg, dtype=np.float64), degrees=True).as_matrix()
    return make_T(rot, xyz_mm)


def pose_msg_to_T(pose, order: str = "xyz") -> np.ndarray:
    """geometry_msgs/Pose from HDR35 -> 4x4 (T_flange2base).

    NOTE: the HDR35 bridge (`hdr_stream`) packs Euler angles rx/ry/rz (degrees) into
    ``orientation.x/y/z`` — it is *not* a quaternion. position is in mm.
    """
    xyz = (pose.position.x, pose.position.y, pose.position.z)
    rxyz = (pose.orientation.x, pose.orientation.y, pose.orientation.z)
    return T_from_xyz_euler(xyz, rxyz, order=order)


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply T to an (N,3) array of points. Returns (N,3)."""
    pts = np.asarray(pts, dtype=np.float64)
    return pts @ T[:3, :3].T + T[:3, 3]


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    """OpenCV rvec/tvec -> 4x4. Caller is responsible for the tvec units."""
    import cv2

    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return make_T(rot, np.asarray(tvec, dtype=np.float64).reshape(3))
