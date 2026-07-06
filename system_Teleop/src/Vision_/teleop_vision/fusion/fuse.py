"""Core fusion: bring both clouds into the base frame and merge them.

Everything operates in millimetres. Inputs are clouds in their *camera* frames plus the
camera->base transforms; output is a single cloud in the base frame.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d

from .transforms import transform_points


def zivid_xyz_to_cloud(xyz: np.ndarray, rgba: np.ndarray | None = None) -> o3d.geometry.PointCloud:
    """Zivid ``copy_data('xyz')`` (HxWx3, mm, NaN where invalid) -> Open3D cloud.

    Optionally colour it from the matching ``copy_data('rgba')`` image.
    """
    flat = xyz.reshape(-1, 3)
    finite = np.isfinite(flat).all(axis=1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(flat[finite].astype(np.float64))
    if rgba is not None:
        cols = rgba.reshape(-1, 4)[:, :3].astype(np.float64) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(cols[finite])
    return pcd


def _refine_icp(source_base, target_base, max_corr_mm, voxel_mm):
    """Point-to-plane ICP refining `source` onto `target` (both already in base).

    Returns the 4x4 correction to LEFT-multiply onto the source. The hand-eye + flange
    pose already put them within a few mm, so this only mops up residual calibration
    error. Falls back to identity if the overlap is too small to trust.
    """
    src = source_base.voxel_down_sample(voxel_mm)
    tgt = target_base.voxel_down_sample(voxel_mm)
    if len(src.points) < 200 or len(tgt.points) < 200:
        return np.eye(4), 0.0
    tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3 * voxel_mm, max_nn=30))
    reg = o3d.pipelines.registration.registration_icp(
        src, tgt, max_corr_mm, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )
    return reg.transformation, reg.fitness


def fuse(
    zivid_pcd_cam: o3d.geometry.PointCloud,
    T_zivid2base: np.ndarray,
    d405_pcd_cam: o3d.geometry.PointCloud,
    T_d405_2base: np.ndarray,
    voxel_mm: float = 2.0,
    do_icp: bool = True,
    icp_max_corr_mm: float = 10.0,
    icp_min_fitness: float = 0.3,
    remove_outliers: bool = True,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Transform both clouds to base, optionally ICP-refine the wrist cloud onto the
    Zivid cloud, then merge + downsample. Returns (fused_cloud, info)."""
    zivid_base = o3d.geometry.PointCloud(zivid_pcd_cam).transform(T_zivid2base)
    d405_base = o3d.geometry.PointCloud(d405_pcd_cam).transform(T_d405_2base)

    info = {"icp_applied": False, "icp_fitness": 0.0,
            "n_zivid": len(zivid_base.points), "n_d405": len(d405_base.points)}

    if do_icp and len(zivid_base.points) and len(d405_base.points):
        correction, fitness = _refine_icp(d405_base, zivid_base, icp_max_corr_mm, voxel_mm)
        info["icp_fitness"] = float(fitness)
        if fitness >= icp_min_fitness:
            d405_base.transform(correction)
            info["icp_applied"] = True

    fused = zivid_base + d405_base
    if voxel_mm > 0:
        fused = fused.voxel_down_sample(voxel_mm)
    if remove_outliers and len(fused.points) > 50:
        fused, _ = fused.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    info["n_fused"] = len(fused.points)
    return fused, info
