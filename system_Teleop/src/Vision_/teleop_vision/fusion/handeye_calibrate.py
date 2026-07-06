#!/usr/bin/env python3
"""Eye-in-hand calibration for the D405 wrist camera.

Computes ``T_flange_from_d405`` (the constant transform from the wrist camera optical
frame to the robot flange) so the live flange pose can place the wrist cloud in the
base frame:  ``T_d405_2base = T_flange2base @ T_flange_from_d405``.

Procedure
---------
1. Tape a ChArUco board down somewhere both visible to the D405 and reachable.
2. Run this node; it streams the D405 view and subscribes ``/system_<side>/pose_states``.
3. Jog the arm to a varied pose (translate AND rotate the wrist), press <Enter> to grab
   a sample. Collect >= ~12 well-spread poses.
4. Type ``solve`` (or just <Enter> once you have enough) to calibrate.

Because the HDR35 Euler convention is ambiguous, the solver sweeps several scipy Euler
orders and reports the per-sample consistency residual for each, then saves the best.

    ros2 run teleop_vision handeye_calibrate --ros-args \
        -p side:=right -p squares_x:=5 -p squares_y:=7 \
        -p square_len_mm:=30.0 -p marker_len_mm:=22.0 -p aruco_dict:=DICT_4X4_50
"""
from __future__ import annotations

import os
import sys
import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from .rs_cloud import RealSenseD405
from .transforms import make_T, pose_msg_to_T, rvec_tvec_to_T, invert

# Euler orders to try for /system_<side>/pose_states. Whichever yields the most
# self-consistent board pose across samples is almost certainly the controller's.
CANDIDATE_ORDERS = ["xyz", "ZYX", "ZYZ", "XYZ", "zyx"]

RS_SERIALS = {"left": "409122273797", "right": "409122273122"}


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__("handeye_calibrate")
        self.declare_parameter("side", "right")
        self.declare_parameter("squares_x", 5)
        self.declare_parameter("squares_y", 7)
        self.declare_parameter("square_len_mm", 30.0)
        self.declare_parameter("marker_len_mm", 22.0)
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("min_samples", 12)
        self.declare_parameter("output", "")
        self.declare_parameter("rs_any", os.environ.get("VISION_RS_ANY") is not None)

        self.side = str(self.get_parameter("side").value).lower()
        self.min_samples = int(self.get_parameter("min_samples").value)
        out = str(self.get_parameter("output").value).strip()
        self.output = out or os.path.join(
            os.getcwd(), "src", "Vision_", "camera", f"T_flange_from_d405_{self.side}.npy")

        self._build_board()

        serial = None if bool(self.get_parameter("rs_any").value) else RS_SERIALS.get(self.side)
        self.cam = RealSenseD405(serial=serial)
        self.get_logger().info(
            f"D405 intrinsics fx={self.cam.fx:.1f} fy={self.cam.fy:.1f} "
            f"cx={self.cam.cx:.1f} cy={self.cam.cy:.1f}")

        self.latest_pose: Pose | None = None
        self.create_subscription(Pose, f"/system_{self.side}/pose_states",
                                 self._on_pose, 10)

        # samples: list of (T_flange2base_raw_xyzrpy, rvec_target2cam, tvec_target2cam_mm)
        self.samples: list[tuple[tuple, np.ndarray, np.ndarray]] = []
        self._stop = False
        threading.Thread(target=self._cli_loop, daemon=True).start()
        self.get_logger().info("Ready. Jog the arm, press <Enter> to capture, 'solve' to finish.")

    # ---- board / detector -------------------------------------------------
    def _build_board(self):
        dict_name = str(self.get_parameter("aruco_dict").value)
        sx = int(self.get_parameter("squares_x").value)
        sy = int(self.get_parameter("squares_y").value)
        sl = float(self.get_parameter("square_len_mm").value)
        ml = float(self.get_parameter("marker_len_mm").value)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self._new_api = hasattr(cv2.aruco, "CharucoDetector")
        if self._new_api:
            self.board = cv2.aruco.CharucoBoard((sx, sy), sl, ml, self.aruco_dict)
            self.charuco_detector = cv2.aruco.CharucoDetector(self.board)
        else:
            self.board = cv2.aruco.CharucoBoard_create(sx, sy, sl, ml, self.aruco_dict)

    def _detect_board_pose(self, rgb):
        """Return (rvec, tvec_mm) of board->camera, or None if not seen well enough."""
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if self._new_api:
            ch_corners, ch_ids, _, _ = self.charuco_detector.detectBoard(gray)
            if ch_ids is None or len(ch_ids) < 6:
                return None
            obj_pts, img_pts = self.board.matchImagePoints(ch_corners, ch_ids)
            if obj_pts is None or len(obj_pts) < 6:
                return None
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.cam.K, self.cam.dist)
            return (rvec, tvec) if ok else None
        # legacy API
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        if ids is None or len(ids) == 0:
            return None
        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
        if ch_ids is None or len(ch_ids) < 6:
            return None
        rvec = np.zeros((3, 1)); tvec = np.zeros((3, 1))
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, self.board, self.cam.K, self.cam.dist, rvec, tvec)
        return (rvec, tvec) if ok else None

    def _on_pose(self, msg: Pose):
        self.latest_pose = msg

    # ---- interactive capture ---------------------------------------------
    def _cli_loop(self):
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd == "solve":
                self._solve_and_exit()
                return
            if cmd in ("q", "quit"):
                self._stop = True
                rclpy.shutdown(); return
            self._capture_sample()
            if len(self.samples) >= self.min_samples:
                self.get_logger().info(
                    f"Have {len(self.samples)} samples — type 'solve' when done.")

    def _capture_sample(self):
        if self.latest_pose is None:
            self.get_logger().warn("No flange pose received yet on pose_states — skipped.")
            return
        try:
            rgb = self.cam.capture_color()
        except Exception as e:
            self.get_logger().error(f"D405 capture failed: {e}")
            return
        det = self._detect_board_pose(rgb)
        if det is None:
            self.get_logger().warn("ChArUco board not detected clearly — reposition & retry.")
            return
        rvec, tvec = det
        p = self.latest_pose
        pose_raw = (p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z)
        self.samples.append((pose_raw, np.asarray(rvec).reshape(3),
                             np.asarray(tvec).reshape(3)))
        self.get_logger().info(
            f"[{len(self.samples)}] captured  flange xyz="
            f"({pose_raw[0]:.1f},{pose_raw[1]:.1f},{pose_raw[2]:.1f})  "
            f"board z={tvec.reshape(3)[2]:.1f}mm")

    # ---- solve ------------------------------------------------------------
    def _solve_for_order(self, order):
        """Run calibrateHandEye assuming `order`; return (X, residual_mm)."""
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for pose_raw, rvec, tvec in self.samples:
            xyz = pose_raw[:3]; rpy = pose_raw[3:]
            T_g2b = make_T(R.from_euler(order, rpy, degrees=True).as_matrix(), xyz)
            T_t2c = rvec_tvec_to_T(rvec, tvec)
            R_g2b.append(T_g2b[:3, :3]); t_g2b.append(T_g2b[:3, 3])
            R_t2c.append(T_t2c[:3, :3]); t_t2c.append(T_t2c[:3, 3])
        R_c2g, t_c2g = cv2.calibrateHandEye(
            R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_TSAI)
        X = make_T(R_c2g, t_c2g)  # T_flange_from_d405

        # Consistency: the board is fixed in base, so T_t2b should be constant.
        t_board = []
        for i, (pose_raw, rvec, tvec) in enumerate(self.samples):
            T_g2b = make_T(R.from_euler(order, pose_raw[3:], degrees=True).as_matrix(),
                           pose_raw[:3])
            T_t2c = rvec_tvec_to_T(rvec, tvec)
            T_t2b = T_g2b @ X @ T_t2c
            t_board.append(T_t2b[:3, 3])
        residual = float(np.linalg.norm(np.std(np.asarray(t_board), axis=0)))
        return X, residual

    def _solve_and_exit(self):
        if len(self.samples) < 6:
            self.get_logger().error(f"Need >=6 samples, have {len(self.samples)}.")
            return
        results = []
        for order in CANDIDATE_ORDERS:
            try:
                X, res = self._solve_for_order(order)
                results.append((res, order, X))
                self.get_logger().info(f"euler '{order}': board-pose residual = {res:.2f} mm")
            except Exception as e:
                self.get_logger().warn(f"euler '{order}' failed: {e}")
        if not results:
            self.get_logger().error("All Euler orders failed."); return
        results.sort(key=lambda r: r[0])
        best_res, best_order, best_X = results[0]
        np.save(self.output, best_X)
        self.get_logger().info(
            "\n========== HAND-EYE RESULT ==========\n"
            f" best euler order : {best_order}  (residual {best_res:.2f} mm)\n"
            f" T_flange_from_d405 =\n{np.array2string(best_X, precision=3, suppress_small=True)}\n"
            f" saved -> {self.output}\n"
            f"  Pass these to fusion_node:  -p euler_order:={best_order} "
            f"-p handeye_path:={self.output}\n"
            "=====================================")
        if best_res > 5.0:
            self.get_logger().warn(
                "Residual is high (>5mm): add more/varied poses, check board size params, "
                "or the wrong Euler order won.")
        self.cam.close()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cam.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
