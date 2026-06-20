"""Replay a recorded episode's frames onto the camera topics.

Purpose: run the REAL robot pipeline on recorded images, before live camera publishing
is ready. This node publishes the sample episode's front/wrist JPEGs to the same camera
topics that `vpi_robot_client` subscribes to — so:

    episode_image_publisher  ──/system_left/camera/{front,wrist}/rgb──▶  vpi_robot_client
                                                                            │ /predict
                                                                            ▼ action -> HDR35/DG5F

The robot STATE still comes from the real drivers (closed-loop on state, replayed vision).
Home the robot to the episode's start pose first (episode_manager arm_home/hand_home), then
START — the policy drives the robot using the recorded camera stream.

Run:
    ros2 run vpi_robot_client episode_image_publisher \
        --ros-args -p side:=left -p fps:=30.0 \
        -p episode_dir:=/home/bi/visuomotor-policy-inference/examples/sample_episodes/left
"""
from __future__ import annotations

import glob
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    import cv2
    from cv_bridge import CvBridge
except Exception as exc:  # pragma: no cover
    raise RuntimeError("episode_image_publisher needs cv_bridge + opencv (cv2).") from exc


class EpisodeImagePublisher(Node):
    def __init__(self):
        super().__init__("episode_image_publisher")
        self.declare_parameter("side", "left")
        self.declare_parameter("episode_dir", "")          # default: <repo>/examples/sample_episodes/<side>
        self.declare_parameter("front_topic", "")
        self.declare_parameter("wrist_topic", "")
        self.declare_parameter("front_subdir", "front_rgb")
        self.declare_parameter("wrist_subdir", "wrist_rgb")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("end_index", -1)            # -1 = to end
        self.declare_parameter("autostart", True)          # False: wait for ~/start (managed by Episode Manager)

        self.side = self.get_parameter("side").value
        ep = self.get_parameter("episode_dir").value or \
            f"/home/bi/visuomotor-policy-inference/examples/sample_episodes/{self.side}"
        front_dir = os.path.join(ep, self.get_parameter("front_subdir").value)
        wrist_dir = os.path.join(ep, self.get_parameter("wrist_subdir").value)

        self.front_files = sorted(glob.glob(os.path.join(front_dir, "*.jpg")))
        self.wrist_files = sorted(glob.glob(os.path.join(wrist_dir, "*.jpg")))
        if not self.front_files or not self.wrist_files:
            raise RuntimeError(f"no frames found under {ep} ({front_dir}, {wrist_dir})")
        n = min(len(self.front_files), len(self.wrist_files))
        s = int(self.get_parameter("start_index").value)
        e = int(self.get_parameter("end_index").value)
        e = n if e < 0 else min(e, n)
        self.front_files = self.front_files[s:e]
        self.wrist_files = self.wrist_files[s:e]
        self.loop = bool(self.get_parameter("loop").value)

        front_topic = self.get_parameter("front_topic").value or f"/system_{self.side}/camera/front/rgb"
        wrist_topic = self.get_parameter("wrist_topic").value or f"/system_{self.side}/camera/wrist/rgb"
        self.front_pub = self.create_publisher(Image, front_topic, 1)
        self.wrist_pub = self.create_publisher(Image, wrist_topic, 1)
        self.bridge = CvBridge()

        self.idx = 0
        self._active = bool(self.get_parameter("autostart").value)
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/stop", self._on_stop)
        period = 1.0 / float(self.get_parameter("fps").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"loaded {len(self.front_files)} frames @ {1.0/period:.1f}Hz -> "
            f"{front_topic} , {wrist_topic} (loop={self.loop}, active={self._active}); "
            f"control via ~/start ~/stop"
        )

    def _on_start(self, req, res):
        self.idx = 0
        self._active = True
        self.get_logger().info("replay START")
        res.success = True; res.message = "replay started"
        return res

    def _on_stop(self, req, res):
        self._active = False
        self.get_logger().info("replay STOP")
        res.success = True; res.message = "replay stopped"
        return res

    def _publish(self, pub, path):
        img = cv2.imread(path, cv2.IMREAD_COLOR)   # BGR
        if img is None:
            self.get_logger().warn(f"failed to read {path}")
            return
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

    def _tick(self):
        if not self._active:
            return
        if self.idx >= len(self.front_files):
            if self.loop:
                self.idx = 0
                self.get_logger().info("replay looped")
            else:
                self.get_logger().info("replay finished")
                self._active = False
                return
        self._publish(self.front_pub, self.front_files[self.idx])
        self._publish(self.wrist_pub, self.wrist_files[self.idx])
        if self.idx % 60 == 0:
            self.get_logger().info(f"frame {self.idx}/{len(self.front_files)}")
        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeImagePublisher()
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
