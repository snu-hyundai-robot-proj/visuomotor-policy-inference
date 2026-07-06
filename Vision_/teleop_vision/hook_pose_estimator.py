import contextlib
import os
import time
from pathlib import Path

import cv2
import torch
import numpy as np
import open3d as o3d
from PIL import Image
from scipy.spatial.transform import Rotation as R

from rfdetr import RFDETRSeg2XLarge

import zivid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

try:
    from system_interface.srv import DetectObject
    DETECT_OBJECT_RESULT_FIELD = 'result'
except ImportError:
    from std_srvs.srv import Trigger as DetectObject
    DETECT_OBJECT_RESULT_FIELD = 'success'

class HookPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('hook_pose_estimator_node')

        self.declare_parameter('hand_side', 'left')
        self.declare_parameter('package_dir', '')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('service_name', 'detect_object')
        self.declare_parameter('pose_topic', 'hook_pose')
        self.declare_parameter('zivid_serial', '')
        self.declare_parameter('zivid_connect_attempts', 10)
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('save_debug_images', True)

        self.hand_side = str(self.get_parameter('hand_side').value).lower()
        if self.hand_side not in ('left', 'right'):
            raise ValueError(f"Invalid hand_side: '{self.hand_side}'. Must be 'left' or 'right'.")

        self.package_dir = self.resolve_package_dir()
        output_dir = str(self.get_parameter('output_dir').value).strip()
        self.output_dir = output_dir or str(self.package_dir / 'data')

        os.makedirs(self.output_dir, exist_ok=True)

        self.service_name = str(self.get_parameter('service_name').value).strip() or 'detect_object'
        self.pose_topic = str(self.get_parameter('pose_topic').value).strip() or 'hook_pose'

        # ICP parameters
        self.icp_iterations = 100
        self.max_correspondence = 1000.0
        self.n_random_init = 30
        self.ply_scale = 1.0
        self.rand_rot_deg = 90.0
        self.rand_trans = 5.0
        self.icp_score_mode = 'rmse'
        self.icp_seed = 0
        self.voxel_live = 1.0
        self.voxel_reg = 1.0

        # Calibration Data
        if self.hand_side.lower() == 'left':
            tx, ty, tz =-1481.770, -992.169, 1599.589
            rx_c, ry_c, rz_c = -120.782, -2.326, -87.213
        else:
            tx, ty, tz = -1468.350, 1004.437, 1665.196
            rx_c, ry_c, rz_c = -119.738, 1.822, -95.193
        rot_c = R.from_euler('xyz', [rx_c, ry_c, rz_c], degrees=True)
        self.T_cam2base = np.eye(4, dtype=np.float64)
        self.T_cam2base[:3, :3] = rot_c.as_matrix()
        self.T_cam2base[:3, 3] = [tx, ty, tz]

        existing_imgs = [
            f for f in os.listdir(self.output_dir)
            if f.startswith(f"{self.hand_side}_") and f.endswith("_segmentation_result.png")
        ]
        self.index = max([int(f.split('_')[1]) for f in existing_imgs]) + 1 if existing_imgs else 1

        self.init_seg_model()
        self.init_zivid_camera()
        self.init_icp_model()

        self.srv = self.create_service(DetectObject, self.service_name, self.detect_object_callback)
        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.get_logger().info(
            "Hook Pose Estimator Node initialized. "
            f"side={self.hand_side}, package_dir={self.package_dir}, output_dir={self.output_dir}, "
            f"service={self.service_name}, pose_topic={self.pose_topic}"
        )

    def resolve_package_dir(self):
        requested = str(self.get_parameter('package_dir').value).strip()
        env_path = os.environ.get('VISION_PACKAGE_DIR', '').strip()

        candidates = []
        if requested:
            candidates.append(Path(requested).expanduser())
        if env_path:
            candidates.append(Path(env_path).expanduser())

        here = Path(__file__).resolve()
        candidates.extend(parent for parent in here.parents if parent.name == 'Vision_')
        cwd = Path.cwd()
        candidates.extend([
            cwd / 'Vision_',
            cwd / 'src' / 'Vision_',
            Path('/workspace/Vision_'),
            Path('/workspace/src/Vision_'),
            Path.home() / 'workspace' / 'src' / 'Vision_',
        ])

        for candidate in candidates:
            if (
                (candidate / 'camera').is_dir()
                and (candidate / 'models' / 'hook_model.ply').is_file()
                and (candidate / 'segmentation' / 'weights' / 'rf_detr_best.pth').is_file()
            ):
                return candidate

        checked = ', '.join(str(path) for path in candidates)
        raise FileNotFoundError(
            "Could not locate Vision_ package assets. "
            "Set the ROS parameter package_dir or VISION_PACKAGE_DIR. "
            f"Checked: {checked}"
        )

    def detect_object_callback(self, request, response):
        try:
            self.get_logger().info("DetectObject request received.")
            success = self.detect_hook_pose()
            if success:
                setattr(response, DETECT_OBJECT_RESULT_FIELD, True)
                if hasattr(response, 'message'):
                    response.message = 'hook pose detected'
                return response
            else:
                self.get_logger().error("Failed to detect hook pose.")
        except Exception as e:
            self.get_logger().error(f"Error during hook pose estimation: {e}")
        
        setattr(response, DETECT_OBJECT_RESULT_FIELD, False)
        if hasattr(response, 'message'):
            response.message = 'failed to detect hook pose'
        return response

    def init_seg_model(self):
        try:
            self.CONF_THRESH = float(self.get_parameter('confidence_threshold').value)
            weight_path = self.package_dir / 'segmentation' / 'weights' / 'rf_detr_best.pth'

            self.model = RFDETRSeg2XLarge(pretrain_weights=str(weight_path))
            if hasattr(self.model, 'optimize_for_inference'):
                self.model.optimize_for_inference()
            device_msg = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.get_logger().info(f"Loaded RF-DETR segmentation weights from {weight_path} ({device_msg}).")
        except Exception as e:
            self.get_logger().error(f"Error initializing Segmentation model: {e}")
            raise

    def init_zivid_camera(self):
        try:
            cam_configs = {
                'left': {'file': 'camera_setting_left.yml', 'serial': '23352865'},
                'right': {'file': 'camera_setting_right.yml', 'serial': '2051707B'},
            }
            config = cam_configs[self.hand_side]
            cam_yml_path = self.package_dir / 'camera' / config['file']
            requested_serial = str(self.get_parameter('zivid_serial').value).strip()
            target_serial = requested_serial or config['serial']
            attempts = int(self.get_parameter('zivid_connect_attempts').value)

            self.zivid_app = zivid.Application()
            last_error = None
            for attempt in range(1, attempts + 1):
                try:
                    cameras = list(self.zivid_app.cameras())
                    self.get_logger().info(
                        f"Zivid discovery attempt {attempt}/{attempts}: "
                        f"{[str(camera) for camera in cameras]}"
                    )
                    if target_serial:
                        self.camera = self.zivid_app.connect_camera(serial_number=target_serial)
                    else:
                        self.camera = self.zivid_app.connect_camera()
                    self.get_logger().info(
                        f"Connected to {self.hand_side} Zivid camera"
                        f"{f' (SN: {target_serial})' if target_serial else ''}."
                    )
                    break
                except Exception as e:
                    last_error = e
                    self.get_logger().warning(
                        f"Failed to connect to Zivid camera on attempt {attempt}/{attempts}: {e}"
                    )
                    time.sleep(2.0)
            else:
                raise RuntimeError(f"Could not connect to Zivid camera. Last error: {last_error}")

            self.settings = zivid.Settings.load(str(cam_yml_path))
            self.get_logger().info(f"Loaded Zivid settings from {cam_yml_path}.")
        except Exception as e:
            self.get_logger().error(f"Error connecting Zivid camera: {e}")
            raise

    def init_icp_model(self):
        try:
            ply_path = self.package_dir / 'models' / 'hook_model.ply'
            self.gt_o3d = o3d.io.read_point_cloud(str(ply_path))
            if self.gt_o3d.is_empty():
                raise RuntimeError(f"Open3D loaded an empty point cloud from {ply_path}")

            gt_points = np.asarray(self.gt_o3d.points).copy()
            if self.ply_scale != 1.0:
                gt_points = gt_points * float(self.ply_scale)
            
            junction_point = np.array([908.921284, 3.411187, 33.298282], dtype=np.float64)
            gt_points = gt_points - junction_point
            
            self.gt_o3d.points = o3d.utility.Vector3dVector(gt_points)
            self.gt_o3d_reg = self.gt_o3d.voxel_down_sample(voxel_size=self.voxel_reg)
            self.get_logger().info(f"Loaded hook ICP model from {ply_path}.")
        except Exception as e:
            self.get_logger().error(f"Error loading ICP model: {e}")
            raise


    def detect_hook_pose(self):
        try:
            self.get_logger().info("Capturing 2D+3D frame from Zivid...")
            frame = self.camera.capture_2d_3d(self.settings)
            self.get_logger().info("Zivid capture complete.")
        except Exception as e:
            self.get_logger().error(f"Error capturing frame from Zivid camera: {e}")
            return False

        xyz = frame.point_cloud().copy_data("xyz")
        rgba = frame.point_cloud().copy_data("rgba")
        image_pil = Image.fromarray(rgba[:, :, :3]).convert("RGB")
        image_np = np.array(image_pil)
        
        self.get_logger().info(f"Running RF-DETR segmentation at threshold {self.CONF_THRESH}...")
        autocast_context = (
            torch.autocast("cuda", dtype=torch.float16)
            if torch.cuda.is_available()
            else contextlib.nullcontext()
        )
        with autocast_context, torch.no_grad():
            inference_result = self.model.predict(image_pil, threshold = self.CONF_THRESH)
        self.get_logger().info("RF-DETR segmentation complete.")
            
        if inference_result.mask is not None and len(inference_result.mask) > 0:
            mask = np.asarray(inference_result.mask[0]).astype(bool)
        else:
            self.get_logger().error("No masks detected by segmentation model.")
            self.save_failed_rgb(image_np, "no_mask")
            return False


        masked_xyz = xyz[mask].reshape(-1, 3)
        masked_xyz = masked_xyz[np.isfinite(masked_xyz).all(axis=1)]
        self.get_logger().info(f"Masked point count: {len(masked_xyz)}")

        if len(masked_xyz) < 500:
            self.get_logger().error("Not enough valid points in the masked point cloud.")
            self.save_failed_rgb(image_np, "too_few_points")
            return False

        # ICP CODE
        self.get_logger().info("Running ICP registration...")
        live_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(masked_xyz.astype(np.float64)))

        live_vis = o3d.geometry.PointCloud(live_o3d)

        live_o3d = live_o3d.voxel_down_sample(voxel_size=self.voxel_live)
        live_o3d_reg = live_o3d.voxel_down_sample(voxel_size=self.voxel_reg)

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_iterations)
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

        T_prev_best = np.eye(4)
        T_prev_best[:3, 3] = -live_o3d.get_center()

        best_reg = None
        rng = np.random.default_rng(self.icp_seed)
        init_list = [T_prev_best]

        if self.hand_side.lower() == 'left':
            for axis in ['x', 'y', 'z']:
                T_flip = np.eye(4)
                T_flip[:3, :3] = R.from_euler(axis, 180, degrees=True).as_matrix()
                init_list.append(T_flip @ T_prev_best)
        else:
            for axis in ['x', 'y', 'z']:
                T_flip = np.eye(4)
                T_flip[:3, :3] = R.from_euler(axis, 180, degrees=True).as_matrix()
                init_list.append(T_flip @ T_prev_best)

        n_rand = max(0, int(self.n_random_init) - 1)
        for _ in range(n_rand):
            T_rand = self.random_transform(self.rand_rot_deg, self.rand_trans, rng)
            init_list.append(T_rand @ T_prev_best)

        for init in init_list:
            reg = o3d.pipelines.registration.registration_icp(
                source=live_o3d_reg,
                target=self.gt_o3d_reg,
                max_correspondence_distance=self.max_correspondence,
                init=init,
                estimation_method=estimation,
                criteria=criteria,
            )
            if self.pick_better(reg, best_reg, self.icp_score_mode):
                best_reg = reg

        if best_reg is None or best_reg.fitness < 0.1:
            self.get_logger().error("ICP failed to find a good alignment.")
            return False
        self.get_logger().info(
            f"ICP complete. fitness={best_reg.fitness:.4f}, rmse={best_reg.inlier_rmse:.4f}"
        )

        T_target_from_source = best_reg.transformation
        T_source_from_target = np.linalg.inv(T_target_from_source)

        if bool(self.get_parameter('save_debug_images').value):
            live_aligned_vis = o3d.geometry.PointCloud(live_vis)
            live_aligned_vis.transform(T_target_from_source)
            icp_aligned_path = os.path.join(self.output_dir, f"{self.hand_side}_{self.index:02d}_icp_aligned.png")
            self.save_icp_alignment_screenshot(live_aligned_vis, icp_aligned_path)

        T_target2base = np.dot(self.T_cam2base, T_source_from_target)
        
        self.save_results(image_np, mask, T_target2base, xyz)
        self.publish_pose(T_target2base)
        return True

    def save_failed_rgb(self, rgb, reason):
        if not bool(self.get_parameter('save_debug_images').value):
            return
        base_name = f"{self.hand_side}_{self.index:02d}_{reason}"
        path = os.path.join(self.output_dir, f"{base_name}_rgb.png")
        cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self.get_logger().info(f"Saved failed detection RGB debug image to: {path}")

    def publish_pose(self, T_pose):
        translation = T_pose[:3, 3]
        quat = R.from_matrix(T_pose[:3, :3]).as_quat()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x = float(translation[0])
        msg.pose.position.y = float(translation[1])
        msg.pose.position.z = float(translation[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self.pose_publisher.publish(msg)

    def save_results(self, rgb, mask, T_pose, xyz=None):
        base_name = f"{self.hand_side}_{self.index:02d}"

        mask_overlay = rgb.copy()
        mask_overlay[mask] = [0, 255, 0]
        combined_seg = cv2.addWeighted(rgb, 0.7, mask_overlay, 0.3, 0)
        cv2.imwrite(os.path.join(self.output_dir, f"{base_name}_segmentation_result.png"), cv2.cvtColor(combined_seg, cv2.COLOR_RGB2BGR))
        
        H, W = mask.shape
        mask_bin_path = os.path.join(self.output_dir, f"{base_name}_mask.bin")
        with open(mask_bin_path, 'wb') as f:
            mask_bool = mask.astype(bool).reshape(H, W, 1)
            mask_bool.tofile(f)
        
        bin_path = os.path.join(self.output_dir, f"{base_name}_data.bin")
        with open(bin_path, 'wb') as f:
            if T_pose is not None:
                translation = T_pose[:3, 3]
                rotation = T_pose[:3, :3]
                r = R.from_matrix(rotation)
                quat = r.as_quat()
                pose_7d_vec = np.concatenate((translation, quat)).astype(np.float32)
                pose_7d_vec.tofile(f)
                self.save_pose_overlay(rgb, mask, xyz, T_pose, pose_7d_vec, base_name)
                
        self.get_logger().info(f"Saved RGB image, mask, and pose data for {self.hand_side} hand with index {self.index:02d}.")
        self.index += 1

    def save_pose_overlay(self, rgb, mask, xyz, T_pose, pose_7d_vec, base_name):
        if xyz is None:
            return

        overlay = rgb.copy()
        mask_layer = overlay.copy()
        mask_layer[mask] = [0, 255, 0]
        overlay = cv2.addWeighted(overlay, 0.65, mask_layer, 0.35, 0)

        T_target2cam = np.linalg.inv(self.T_cam2base) @ T_pose
        origin_cam = T_target2cam[:3, 3]
        axes_cam = T_target2cam[:3, :3]
        axis_length_mm = 120.0

        valid = np.isfinite(xyz).all(axis=2)
        if not valid.any():
            return

        flat_xyz = xyz[valid].astype(np.float64)
        valid_pixels = np.argwhere(valid)

        def nearest_pixel(point):
            dists = np.linalg.norm(flat_xyz - point.reshape(1, 3), axis=1)
            row, col = valid_pixels[int(np.argmin(dists))]
            return int(col), int(row)

        origin_px = nearest_pixel(origin_cam)
        axis_specs = [
            ("X", (255, 40, 40), origin_cam + axes_cam[:, 0] * axis_length_mm),
            ("Y", (40, 220, 40), origin_cam + axes_cam[:, 1] * axis_length_mm),
            ("Z", (40, 120, 255), origin_cam + axes_cam[:, 2] * axis_length_mm),
        ]

        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        cv2.circle(overlay_bgr, origin_px, 7, (255, 255, 255), -1)
        cv2.circle(overlay_bgr, origin_px, 9, (0, 0, 0), 2)

        for label, rgb_color, endpoint_cam in axis_specs:
            endpoint_px = nearest_pixel(endpoint_cam)
            bgr_color = (rgb_color[2], rgb_color[1], rgb_color[0])
            cv2.arrowedLine(overlay_bgr, origin_px, endpoint_px, bgr_color, 4, tipLength=0.18)
            cv2.putText(
                overlay_bgr,
                label,
                (endpoint_px[0] + 8, endpoint_px[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                bgr_color,
                2,
                cv2.LINE_AA,
            )

        pose_text = (
            f"x={pose_7d_vec[0]:.1f} y={pose_7d_vec[1]:.1f} z={pose_7d_vec[2]:.1f} "
            f"q=({pose_7d_vec[3]:.3f},{pose_7d_vec[4]:.3f},{pose_7d_vec[5]:.3f},{pose_7d_vec[6]:.3f})"
        )
        cv2.rectangle(overlay_bgr, (12, 12), (min(overlay_bgr.shape[1] - 12, 1050), 58), (0, 0, 0), -1)
        cv2.putText(
            overlay_bgr,
            pose_text,
            (24, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        overlay_path = os.path.join(self.output_dir, f"{base_name}_pose_overlay.png")
        cv2.imwrite(overlay_path, overlay_bgr)

    def save_icp_alignment_screenshot(self, live_aligned_pcd, save_path):
        try:
            render = o3d.visualization.rendering.OffscreenRenderer(640, 480)
            
            gt_colored = o3d.geometry.PointCloud(self.gt_o3d)
            gt_colored.paint_uniform_color([1.0, 0.0, 0.0]) 
            
            gt_mat = o3d.visualization.rendering.MaterialRecord()
            gt_mat.shader = "defaultUnlit" 
            gt_mat.point_size = 3.0
            
            live_colored = o3d.geometry.PointCloud(live_aligned_pcd)
            live_colored.paint_uniform_color([0.0, 1.0, 0.0]) 
            
            live_mat = o3d.visualization.rendering.MaterialRecord()
            live_mat.shader = "defaultUnlit"
            live_mat.point_size = 3.0

            render.scene.add_geometry("gt", gt_colored, gt_mat)
            render.scene.add_geometry("live", live_colored, live_mat)

            render.setup_camera(60.0, [0, 0, 0], [0, 0, 150], [0, -1, 0])
            
            img = render.render_to_image()
            o3d.io.write_image(save_path, img)
            self.get_logger().info(f"Saved ICP Red/Green alignment screenshot to: {save_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save ICP alignment screenshot: {e}")


    # Utility functions
    def align_pcd_principal_axes_to_world(self, pcd):
        pts = np.asarray(pcd.points, dtype=np.float64)
        if pts.shape[0] < 3: return
        center = pts.mean(axis=0)
        C = np.cov((pts - center).T)
        eigvals, eigvecs = np.linalg.eigh(C)
        R_obj = eigvecs[:, np.argsort(eigvals)[::-1]]
        if np.linalg.det(R_obj) < 0:
            R_obj[:, 2] *= -1
        pcd.translate(-center)
        pcd.rotate(R_obj.T, center=(0.0, 0.0, 0.0))
        pcd.translate(center)

    def pca_align_pointcloud_inplace(self, pcd, center_to_origin=True):
        pts = np.asarray(pcd.points, dtype=np.float64)
        if pts.shape[0] < 3: return
        center = pts.mean(axis=0)
        C = np.cov((pts - center).T)
        eigvals, eigvecs = np.linalg.eigh(C)
        R_obj = eigvecs[:, np.argsort(eigvals)[::-1]]
        if np.linalg.det(R_obj) < 0:
            R_obj[:, 2] *= -1
        if center_to_origin:
            pcd.translate(-center)
            pcd.rotate(R_obj.T, center=(0.0, 0.0, 0.0))
        else:
            pcd.rotate(R_obj.T, center=center)

    def random_transform(self, max_rot_deg, max_trans, rng):
        max_rad = np.deg2rad(max_rot_deg)
        angle = rng.uniform(-max_rad, max_rad)
        axis = rng.normal(size=3)
        if np.linalg.norm(axis) < 1e-9:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis /= np.linalg.norm(axis)
        x, y, z = axis
        c, s, C = np.cos(angle), np.sin(angle), 1.0 - np.cos(angle)
        R_mat = np.array([
            [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
            [z*x*C - y*s, z*y*C + x*s, z*z*C + c  ]
        ], dtype=np.float64)

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_mat
        T[:3, 3] = rng.uniform(-max_trans, max_trans, size=3)
        return T

    def pick_better(self, reg_a, reg_b, score_mode):
        if reg_b is None:
            return True
        if score_mode == 'rmse':
            if reg_a.inlier_rmse < reg_b.inlier_rmse -1e-12:
                return True
            if abs(reg_a.inlier_rmse - reg_b.inlier_rmse) < 1e-12 and reg_a.fitness > reg_b.fitness + 1e-12:
                return True
            return False
        else:
            if reg_a.fitness > reg_b.fitness + 1e-12:
                return True
            if abs(reg_a.fitness - reg_b.fitness) <= 1e-12 and reg_a.inlier_rmse < reg_b.inlier_rmse - 1e-12:
                return True
            return False



def main(args=None):
    rclpy.init(args=args)
    node = HookPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
