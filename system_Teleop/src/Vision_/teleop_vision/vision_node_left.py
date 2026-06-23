#!/usr/bin/env python3
import os, time, threading, cv2, torch, queue, collections, datetime

import numpy as np
import open3d as o3d
from PIL import Image
from scipy.spatial.transform import Rotation as R

from rfdetr import RFDETRSeg2XLarge

import zivid
import pyrealsense2 as rs
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Int32

from system_interface.srv import DetectObject
from system_interface.msg import StartRecording

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node_left')
        # self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.declare_parameter('hand_side', 'left')
        self.hand_side = self.get_parameter('hand_side').value
        self.system_side = 'system_' + self.hand_side

        home_dir = os.getcwd()
        self.package_dir = os.path.join(home_dir, 'src', 'Vision_')
        self.output_dir = os.path.join(home_dir, 'Record', self.hand_side, 'zivid_data')
        self.save_dir = os.path.join(home_dir, 'Record', self.hand_side, 'videos')
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.save_dir, exist_ok=True)

        self.camera_lock = threading.Lock()
        self.pub = self.system_side + '/frame_index'
        self.srv = self.create_service(DetectObject, self.system_side + '/detect_object', self.detect_object_callback)
        self.subcriber = self.create_subscription(StartRecording, self.system_side + '/start_recording', self.record_cmd_callback, 10)
        self.frame_index_publisher = self.create_publisher(Int32, self.system_side + '/frame_index', qos_profile = 1)
        self.bridge = CvBridge()
        self.d405_image_publisher = self.create_publisher(RosImage, self.system_side + '/d405_rgb', 10)
        self.zivid_image_publisher = self.create_publisher(RosImage, self.system_side + '/zivid_rgb', 10)

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

        if self.hand_side.lower() == 'left':
            tx, ty, tz = -1478.151, -972.564, 1603.593
            rx_c, ry_c, rz_c = -121.411, -2.267, -88.093
        else:
            tx, ty, tz = -1464.766, 996.191, 1663.695
            rx_c, ry_c, rz_c = -119.828, 5.292, -96.986
        rot_c = R.from_euler('xyz', [rx_c, ry_c, rz_c], degrees=True)
        self.T_cam2base = np.eye(4, dtype=np.float64)
        self.T_cam2base[:3, :3] = rot_c.as_matrix()
        self.T_cam2base[:3, 3] = [tx, ty, tz]

        existing_imgs = [f for f in os.listdir(self.output_dir) if f.startswith(f"{self.hand_side}_") and f.endswith("_segmentation_result.png")]
        self.index = max([int(f.split('_')[1]) for f in existing_imgs]) + 1 if existing_imgs else 1

        existing_vids = [f for f in os.listdir(self.save_dir) if f.startswith(f"{self.hand_side}_") and f.endswith("_zivid_video.mp4")]
        self.video_count = max([int(f.split('_')[1]) for f in existing_vids]) + 1 if existing_vids else 1

        # self.index = 1
        self.is_recording = False
        # self.video_count = 1
        self.write_queue = queue.Queue(maxsize=1000)
        self.rs_buffer = collections.deque(maxlen=30)

        self.init_seg_model()
        self.init_zivid_camera()
        self.init_icp_model()
        self.init_rs_camera()
        self.publish_running = True
        self.publish_thread = threading.Thread(target=self.publish_camera_stream, daemon=True)
        self.publish_thread.start()

        self.offscreen_renderer = None

        self.get_logger().info("\n##########################################################\n"+
                               "##########################################################\n"+
                              f"     {self.hand_side} Vision Node initialized and ready       \n"+
                               "##########################################################\n"+
                               "##########################################################\n")

    def publish_camera_stream(self):
        while self.publish_running:
            try:
                with self.camera_lock:
                    with self.camera.capture(self.settings_2d) as frame_2d:
                        zivid_rgba = frame_2d.image_rgba_srgb().copy_data()

                frames = self.rs_pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                wrist_cam_bgr = np.asanyarray(color_frame.get_data()).copy()
                wrist_cam_rgb = cv2.cvtColor(wrist_cam_bgr, cv2.COLOR_BGR2RGB)
                zivid_rgb = cv2.cvtColor(zivid_rgba, cv2.COLOR_RGBA2RGB)

                self.rs_buffer.append((time.time(), wrist_cam_bgr))

                zivid_msg = self.bridge.cv2_to_imgmsg(zivid_rgb, encoding='rgb8')
                zivid_msg.header.stamp = self.get_clock().now().to_msg()
                zivid_msg.header.frame_id = f'{self.hand_side}_zivid'
                self.zivid_image_publisher.publish(zivid_msg)

                wrist_msg = self.bridge.cv2_to_imgmsg(wrist_cam_rgb, encoding='rgb8')
                wrist_msg.header.stamp = self.get_clock().now().to_msg()
                wrist_msg.header.frame_id = f'{self.hand_side}_d405'
                self.d405_image_publisher.publish(wrist_msg)

            except Exception as e:
                self.get_logger().error(f"Failed to publish camera stream: {e}")
                time.sleep(0.1)

            time.sleep(0.033)

    def detect_object_callback(self, request, response):
        try:
            success = self.detect_hook_pose()
            if success:
                response.result = True
                return response
            else:
                self.get_logger().error("Failed to detect hook pose.")
        except Exception as e:
            self.get_logger().error(f"Error during hook pose estimation: {e}")

        response.result = False
        return response

    def record_cmd_callback(self, msg):
        if msg.start_record and not self.is_recording:
            self.is_recording = True
            self.rs_buffer.clear()
            self.estimated_fps = 0.0

            while not self.write_queue.empty():
                self.write_queue.get()

            self.threads = [
                threading.Thread(target=self.zivid_record),
                threading.Thread(target=self.video_writer)
            ]
            for t in self.threads:
                t.start()

        elif not msg.start_record and self.is_recording:
            self.is_recording = False
            for t in self.threads:
                t.join()
            self.get_logger().info(f"Recording stopped. Videos saved with index {self.video_count}.")
            self.video_count += 1

    def init_seg_model(self):
        self.CONF_THRESH = 0.8
        if os.environ.get("VISION_SKIP_SEG"):
            self.model = None
            self.get_logger().warn("VISION_SKIP_SEG set — skipping RF-DETR segmentation model load "
                                   "(RGB streams still publish; object-pose detection disabled)")
            return
        try:
            weight_path = os.path.join(self.package_dir, 'segmentation', 'weights', 'rf_detr_best.pth')

            self.model = RFDETRSeg2XLarge(pretrain_weights=weight_path)
            if hasattr(self.model, 'optimize_for_inference'):
                self.model.optimize_for_inference()
        except Exception as e:
            self.get_logger().error(f"Error initializing Segmentation model: {e}")
            raise

    def init_zivid_camera(self):
        self.zivid_app = zivid.Application()
        cam_configs = {
            'left': {'file': 'camera_setting_left.yml', 'serial': '23352865'},
            'right': {'file': 'camera_setting_right.yml', 'serial': '2051707B'}
        }
        side = self.hand_side.lower()
        if side not in cam_configs:
            raise ValueError(f"Invalid hand_side: '{side}'. Must be 'left' or 'right'.")

        cam_yml_path = os.path.join(self.package_dir, 'camera', cam_configs[side]['file'])
        target_serial = cam_configs[side]['serial']

        last_error = None
        for attempt in range(1, 11):
            try:
                cameras = list(self.zivid_app.cameras())
                self.get_logger().info(
                    f"Zivid discovery attempt {attempt}/10: {[str(camera) for camera in cameras]}"
                )
                self.camera = self.zivid_app.connect_camera(serial_number=target_serial)
                self.get_logger().info(f"Successfully connected to {side.capitalize()} Zivid camera.")
                break

            except Exception as e:
                last_error = e
                self.get_logger().warning(
                    f"Failed to connect to {side.capitalize()} Zivid camera "
                    f"(SN: {target_serial}) on attempt {attempt}/10. Error: {e}"
                )
                time.sleep(2.0)
        else:
            self.get_logger().error(
                f"Failed to connect to {side.capitalize()} Zivid camera "
                f"(SN: {target_serial}). Last error: {last_error}"
            )
            raise RuntimeError(f"Required {side} camera is not connected.")

        try:
            self.settings_3d = zivid.Settings.load(cam_yml_path)

            self.settings_2d = zivid.Settings2D()
            acquisition = zivid.Settings2D.Acquisition()
            acquisition.exposure_time = datetime.timedelta(microseconds=3000)
            self.settings_2d.acquisitions.append(acquisition)

        except Exception as e:
            self.get_logger().error(f"Error connecting Zivid camera: {e}")
            raise

    def init_rs_camera(self):
        try:
            self.rs_pipeline = rs.pipeline()
            self.rs_config = rs.config()
            ctx = rs.context()
            devices = ctx.query_devices()
            discovered = []
            for device in devices:
                name = device.get_info(rs.camera_info.name)
                serial = device.get_info(rs.camera_info.serial_number)
                discovered.append(f"{name} (SN: {serial})")
            self.get_logger().info(f"Discovered RealSense devices: {discovered}")
            
            rs_configs = {
                'left': '409122273797',
                'right': '409122273122'
            }
            side = self.hand_side.lower()
            if side not in rs_configs:
                raise ValueError(f"Invalid hand_side: '{side}'")
            
            target_serial = rs_configs[side]
            available = [device.get_info(rs.camera_info.serial_number) for device in devices]
            if target_serial not in available:
                if os.environ.get("VISION_RS_ANY") and available:
                    self.get_logger().warn(
                        f"Required RealSense {target_serial} not found; VISION_RS_ANY set — "
                        f"falling back to available device {available[0]}")
                    target_serial = available[0]
                else:
                    raise RuntimeError(
                        f"Required RealSense serial {target_serial} was not found. "
                        f"Discovered devices: {discovered}"
                    )
            self.rs_config.enable_device(target_serial)
            
            self.rs_config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
            self.rs_pipeline.start(self.rs_config)
            self.get_logger().info(f"Successfully connected to {side.capitalize()} RealSense camera (SN: {target_serial}).")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize RealSense camera: {e}")
            raise

    def init_icp_model(self):
        try:
            ply_path = os.path.join(self.package_dir, 'models', 'hook_model.ply')
            self.gt_o3d = o3d.io.read_point_cloud(ply_path)

            gt_points = np.asarray(self.gt_o3d.points).copy()
            if self.ply_scale != 1.0:
                gt_points = gt_points *float(self.ply_scale)
            
            junction_point = np.array([908.921284, 3.411187, 33.298282], dtype=np.float64)
            gt_points = gt_points - junction_point
            
            # if self.ply_scale != 1.0:
            #     gt_points = np.asarray(self.gt_o3d.points)
            #     self.gt_o3d.points = o3d.utility.Vector3dVector(gt_points * float(self.ply_scale))
            

            # self.gt_o3d.translate(junction_point_float) #####Error####
            # R_fix = self.gt_o3d.get_rotation_matrix_from_xyz([
            #     np.deg2rad(0),
            #     np.deg2rad(0),
            #     np.deg2rad(0)
            # ])
            
            self.gt_o3d.points = o3d.utility.Vector3dVector(gt_points)
            
            # self.gt_o3d.rotate(R_fix, center=(0, 0, 0))
            
            self.gt_o3d_reg = self.gt_o3d.voxel_down_sample(voxel_size=self.voxel_reg)
        except Exception as e:
            self.get_logger().error(f"Error loading ICP model: {e}")
            raise

    def detect_hook_pose(self):
        try:
            with self.camera_lock:
                frame = self.camera.capture_2d_3d(self.settings_3d)
        except Exception as e:
            self.get_logger().error(f"Error capturing frame from Zivid camera: {e}")
            return False
        xyz = frame.point_cloud().copy_data("xyz")
        rgba = frame.point_cloud().copy_data("rgba")
        image_pil = Image.fromarray(rgba[:, :, :3]).convert("RGB")
        image_np = np.array(image_pil)
        
        with torch.autocast("cuda", dtype=torch.float16), torch.no_grad():
            inference_result = self.model.predict(image_pil, threshold = self.CONF_THRESH)
        torch.cuda.empty_cache()
        if inference_result.mask is not None and len(inference_result.mask) > 0:
            mask = inference_result.mask[0]
        else:
            self.get_logger().error("No masks detected by segmentation model.")
            return False

        masked_xyz = xyz[mask].reshape(-1, 3)
        masked_xyz = masked_xyz[np.isfinite(masked_xyz).all(axis=1)]

        if len(masked_xyz) < 500:
            self.get_logger().error("Not enough valid points in the masked point cloud.")
            return False

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

        for axis in ['x', 'y', 'z']:
            T_flip = np.eye(4)
            T_flip[:3, :3] = R.from_euler(axis, 180, degrees=True).as_matrix()
            init_list.append(T_flip @ T_prev_best)

        n_rand = max(0, int(self.n_random_init) - 1 - 3)
        for _ in range(n_rand):
            T_rand = self.random_transform(self.rand_rot_deg, self.rand_trans, rng)
            init_list.append(T_rand @ T_prev_best)
        try:
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

        except Exception as e:
            self.get_logger().error(f"Registration Fault :  {e}")
            
        if best_reg is None or best_reg.fitness < 0.1:
            self.get_logger().error("ICP failed to find a good alignment.")
            return False

        T_target_from_source = best_reg.transformation
        T_source_from_target = np.linalg.inv(T_target_from_source)

        live_aligned_vis = o3d.geometry.PointCloud(live_vis) 
        live_aligned_vis.transform(T_target_from_source)
        icp_aligned_path = os.path.join(self.output_dir, f"{self.hand_side}_{self.index:02d}_icp_aligned.png")
        self.save_icp_alignment_screenshot(live_aligned_vis, icp_aligned_path)

        T_target2base = np.dot(self.T_cam2base, T_source_from_target)
        # self.get_logger().info(f"Estimated Pose (T_target2base):{T_target2base}\n")
        self.save_results(image_np, mask, T_target2base)
        
        try:
            del xyz, rgba, masked_xyz, live_o3d, live_vis, live_o3d_reg, live_aligned_vis, frame
        except Exception:
            pass
        
        return True

    def save_results(self, rgb, mask, T_pose):
        base_name = f"{self.hand_side}_{self.index:02d}"

        mask_overlay = rgb.copy()
        mask_overlay[mask] = [0, 255, 0]
        combined_seg = cv2.addWeighted(rgb, 0.7, mask_overlay, 0.3, 0)
        cv2.imwrite(os.path.join(self.output_dir, f"{base_name}_segmentation_result.png"), cv2.cvtColor(combined_seg, cv2.COLOR_RGB2BGR))
        
        # 1200, 1944, 1
        H,W = mask.shape
        mask_bin_path = os.path.join(self.output_dir, f"{base_name}_mask.bin")
        with open(mask_bin_path, 'wb') as f:
            mask_bool = mask.astype(bool).reshape(H,W,1)
            mask_bool.tofile(f)
        
        bin_path = os.path.join(self.output_dir, f"{base_name}_data.bin")
        with open(bin_path, 'wb') as f:
            if T_pose is not None:
                translation = T_pose[:3,3]
                rotation = T_pose[:3,:3]
                r = R.from_matrix(rotation)
                quat = r.as_quat()
                pose_7d_vec = np.concatenate((translation,quat)).astype(np.float32)
                self.get_logger().info(f"Estimated Pose (x,y,z,quaternion):{pose_7d_vec}\n")
                pose_7d_vec.tofile(f)
                # T_pose.tofile(f)
        print(bin_path)
        self.get_logger().info(f"Saved RGB image and pose data for {self.hand_side} hand with index {self.index:02d}.")
        self.index += 1

    def save_icp_alignment_screenshot(self, live_aligned_pcd, save_path):
        try:
            if self.offscreen_renderer is None:
                self.offscreen_renderer = o3d.visualization.rendering.OffscreenRenderer(640,480)

            self.offscreen_renderer.scene.clear_geometry()

            # render = o3d.visualization.rendering.OffscreenRenderer(640, 480)

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

            self.offscreen_renderer.scene.add_geometry("gt", gt_colored, gt_mat)
            self.offscreen_renderer.scene.add_geometry("live", live_colored, live_mat)

            self.offscreen_renderer.setup_camera(60.0, [0, 0, 0], [0, 150, 0], [0, -1, 0])

            img = self.offscreen_renderer.render_to_image()
            o3d.io.write_image(save_path, img)

        except Exception as e:
            self.get_logger().error(f"Failed to save ICP alignment screenshot: {e}")

    def zivid_record(self):
        frame_index = 1
        while self.is_recording:
            img = None
            zivid_image = None
            try:
                with self.camera_lock:
                    with self.camera.capture(self.settings_2d) as frame_2d:
                        zivid_capture_time = time.time()
                        img = frame_2d.image_rgba_srgb().copy_data()

                if len(self.rs_buffer) > 0:
                    zivid_image = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

                    del img

                    current_rs_buffer = list(self.rs_buffer)
                    closest_rs = min(current_rs_buffer, key=lambda x: abs(x[0] - zivid_capture_time))
                    time_diff = abs(closest_rs[0] - zivid_capture_time)
                    if time_diff > 0.1:
                        self.get_logger().warning(f"Large time difference between Zivid and RealSense frames: {time_diff:.3f} seconds")
                        # self.is_recording = False
                        # break
                        continue
                    else:
                        wrist_cam_image = closest_rs[1]

                    index_msg = Int32()
                    index_msg.data = frame_index
                    self.frame_index_publisher.publish(index_msg)

                    try:
                        self.write_queue.put((zivid_image, wrist_cam_image), block=False)
                        frame_index += 1
                    except queue.Full:
                        self.get_logger().warning("Write queue is full. Dropping frame.")
                        del zivid_image
                        del wrist_cam_image

            except Exception as e:
                self.get_logger().error(f"Zivid capture error during recording: {e}")
                time.sleep(0.1)

    def wrist_cam_record(self):
        while self.is_recording:
            try:
                frames = self.rs_pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if color_frame:
                    wrist_cam_image = np.asanyarray(color_frame.get_data()).copy()
                    wrist_cam_capture_time = time.time()
                    self.rs_buffer.append((wrist_cam_capture_time, wrist_cam_image))
            except Exception as e:
                self.get_logger().error(f"Failed to capture from RealSense camera: {e}")
                continue

    def video_writer(self):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        zivid_out = None
        wrist_cam_out = None

        try:
            while self.is_recording or not self.write_queue.empty():
                try:
                    zivid_image, wrist_cam_image = self.write_queue.get(timeout=0.5)
                    if zivid_out is None:
                        zivid_h, zivid_w = zivid_image.shape[:2]
                        wrist_cam_h, wrist_cam_w = wrist_cam_image.shape[:2]
                        # zivid_filename = os.path.join(self.save_dir,f'{self.hand_side}' ,f'{self.hand_side}_{self.video_count}_zivid_video.mp4')
                        # wrist_cam_filename = os.path.join(self.save_dir, f'{self.hand_side}' ,f'{self.hand_side}_{self.video_count}_wrist_cam_video.mp4')
                        zivid_filename = os.path.join(self.save_dir,f'{self.hand_side}_{self.video_count}_zivid_video.mp4')
                        wrist_cam_filename = os.path.join(self.save_dir, f'{self.hand_side}_{self.video_count}_wrist_cam_video.mp4')
                        zivid_out = cv2.VideoWriter(zivid_filename, fourcc, 30.0, (zivid_w, zivid_h))
                        wrist_cam_out = cv2.VideoWriter(wrist_cam_filename, fourcc, 30.0, (wrist_cam_w, wrist_cam_h))
                    zivid_out.write(zivid_image)
                    wrist_cam_out.write(wrist_cam_image)
                    del zivid_image
                    del wrist_cam_image
                except queue.Empty:
                    continue
                except Exception as e:
                    self.get_logger().error(f"Error writing frame to dist: {e}")

        finally:
            if zivid_out:
                zivid_out.release()
            if wrist_cam_out:
                wrist_cam_out.release()

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

    def destroy_node(self):
        self.publish_running = False
        self.is_recording = False
        if hasattr(self, 'offscreen_renderer') and self.offscreen_renderer is not None:
            try:
                self.offscreen_renderer.release_resources()
            except Exception:
                pass
        if hasattr(self, 'publish_thread') and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
        if hasattr(self, 'threads'):
            for t in self.threads:
                if t.is_alive():
                    t.join()
        if hasattr(self, 'rs_pipeline'):
            try:
                self.rs_pipeline.stop()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
