"""Episode Manager — orchestrates init-state homing + policy-driven episode execution.

State machine (see EPISODE_SYSTEM.md):

    BOOT --healthy--> HOMING --settled--> READY --start--> RUNNING --terminate--> STOPPING
      ^                                     ^                                          |
      |                                     +---------------- re-home ----------------+
      +-- (any) --> FAULT  (safety/e-stop; latched; needs /episode/clear_fault)

Responsibilities:
  * HOMING : ramp arm+hand from current pose to the configured init pose (rate-limited),
             publishing to the same command topics the policy uses. Settle detection.
  * GATE   : owns command arbitration via the policy node's run gate (/vpi/set_enable):
             gate OFF in HOMING/STOPPING/READY/FAULT (manager drives), ON in RUNNING
             (policy drives). One writer at a time.
  * RESET  : calls the policy reset service at the start of each episode.
  * SAFETY : watchdog (input freshness, FT overload, e-stop, episode timeout) -> STOP/FAULT.
  * Publishes /episode/status (std_msgs/String JSON) for the web console.

State source (current joints) mirrors vpi_robot_client: "frame_aligned" (system_interface,
default) or "joint_states" (no custom-msg dependency -> fully containerizable).
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

try:
    from control_msgs.msg import MultiDOFCommand
except Exception:  # pragma: no cover
    MultiDOFCommand = None
try:
    from geometry_msgs.msg import WrenchStamped
except Exception:  # pragma: no cover
    WrenchStamped = None
try:
    from system_interface.msg import FrameAlignedState
except Exception:  # pragma: no cover
    FrameAlignedState = None
try:
    import requests
except Exception:  # pragma: no cover
    requests = None

LEFT_DELTO_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]
ARM_JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

BOOT, HOMING, READY, RUNNING, STOPPING, FAULT = "BOOT", "HOMING", "READY", "RUNNING", "STOPPING", "FAULT"


def rad2deg(x):
    return np.asarray(x, dtype=np.float64) * 180.0 / math.pi


class EpisodeManager(Node):
    def __init__(self):
        super().__init__("episode_manager")
        gp = self.declare_parameter

        # general / rates
        gp("side", "left")
        gp("control_rate", 50.0)     # homing/hold publish + watchdog tick
        gp("status_rate", 10.0)

        # state source
        gp("state_source", "frame_aligned")          # frame_aligned | joint_states
        gp("state_topic", "")
        gp("arm_state_topic", "")
        gp("gripper_state_topic", "")
        gp("arm_state_unit", "deg")
        gp("gripper_state_unit", "rad")

        # cameras (freshness/health only — not decoded)
        gp("front_camera_topic", "")
        gp("wrist_camera_topic", "")
        gp("camera_timeout_sec", 0.5)

        # FT safety (optional)
        gp("ft_topic", "")
        gp("ft_force_max", 0.0)       # N, 0=off
        gp("ft_torque_max", 0.0)      # Nm, 0=off

        # policy server health (optional)
        gp("server_url", "http://localhost:8000")
        gp("server_check_period", 2.0)

        # init pose (radians) — set from the dataset's episode-start state ideally
        gp("arm_home", [0.0] * 6)
        gp("hand_home", [0.0] * 20)

        # command outputs (same topics the policy uses)
        gp("robot_action_topic", "/robot/joint_target_deg")
        gp("robot_topic_unit", "deg")
        gp("gripper_action_topic", "")
        gp("gripper_command_type", "auto")
        gp("arm_action_size", 6)
        gp("gripper_action_size", 20)

        # homing dynamics
        gp("home_max_delta", 0.02)    # rad/tick toward home
        gp("home_tol", 0.02)          # rad settle tolerance
        gp("home_settle_ticks", 15)
        gp("home_timeout_sec", 15.0)

        # episode
        gp("hold_sec", 0.5)           # STOPPING hold before re-home
        gp("max_duration_sec", 20.0)  # episode timeout
        gp("auto_start", False)       # auto-run after homing
        gp("episodes", -1)            # -1 = infinite (manual or auto)
        gp("use_success", True)

        # policy node hooks
        gp("policy_enable_service", "/vpi/set_enable")
        gp("policy_reset_service", "/vpi_policy_control/reset")

        # replay (recorded-image playback) hooks
        gp("use_replay", False)            # if True, every RUNNING also starts image replay
        gp("replay_start_service", "/episode_image_publisher/start")
        gp("replay_stop_service", "/episode_image_publisher/stop")

        self.side = self.get_parameter("side").value
        self.ctrl_dt = 1.0 / float(self.get_parameter("control_rate").value)
        self.arm_size = int(self.get_parameter("arm_action_size").value)
        self.grip_size = int(self.get_parameter("gripper_action_size").value)
        self.arm_home = np.asarray(self.get_parameter("arm_home").value, dtype=np.float64)[: self.arm_size]
        self.hand_home = np.asarray(self.get_parameter("hand_home").value, dtype=np.float64)[: self.grip_size]
        self.robot_topic_unit = self.get_parameter("robot_topic_unit").value
        self.home_max_delta = float(self.get_parameter("home_max_delta").value)
        self.home_tol = float(self.get_parameter("home_tol").value)
        self.home_settle_ticks = int(self.get_parameter("home_settle_ticks").value)
        self.home_timeout = float(self.get_parameter("home_timeout_sec").value)
        self.hold_sec = float(self.get_parameter("hold_sec").value)
        self.max_duration = float(self.get_parameter("max_duration_sec").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.episodes_remaining = int(self.get_parameter("episodes").value)
        self.camera_timeout = float(self.get_parameter("camera_timeout_sec").value)
        self.ft_force_max = float(self.get_parameter("ft_force_max").value)
        self.ft_torque_max = float(self.get_parameter("ft_torque_max").value)
        self.gripper_command_type = self._resolve_grip_cmd(self.get_parameter("gripper_command_type").value)

        if len(self.arm_home) != self.arm_size or float(np.abs(self.arm_home).sum()) == 0.0:
            self.get_logger().warn("arm_home is unset/zeros — set a real init pose (ideally from the dataset).")

        # shared state
        self._lock = threading.Lock()
        self._arm_rad: Optional[np.ndarray] = None
        self._grip_rad: Optional[np.ndarray] = None
        self._front_t = self._wrist_t = self._state_t = 0.0
        self._ft_force = self._ft_torque = 0.0
        self._estop = False
        self._success = False
        self._server_ok = False

        self.state = BOOT
        self.fault_reason = ""
        self.last_termination = ""
        self.policy_enabled = False
        self._ramp_arm: Optional[np.ndarray] = None
        self._ramp_grip: Optional[np.ndarray] = None
        self._settle = 0
        self._t_home0 = 0.0
        self._t_ep0 = 0.0
        self._t_stop0 = 0.0
        self._hold_arm: Optional[np.ndarray] = None
        self._hold_grip: Optional[np.ndarray] = None
        self._pending_replay = False
        self._replay_active = False

        # subs
        self._setup_state_subs()
        front = self.get_parameter("front_camera_topic").value or f"/system_{self.side}/camera/front/rgb"
        wrist = self.get_parameter("wrist_camera_topic").value or f"/system_{self.side}/camera/wrist/rgb"
        self.create_subscription(Image, front, self._on_front, 1)
        self.create_subscription(Image, wrist, self._on_wrist, 1)
        ft_topic = self.get_parameter("ft_topic").value
        if ft_topic and WrenchStamped is not None:
            self.create_subscription(WrenchStamped, ft_topic, self._on_ft, 1)
        self.create_subscription(Bool, "/episode/estop", self._on_estop, 1)
        if bool(self.get_parameter("use_success").value):
            self.create_subscription(Bool, "/episode/success", self._on_success, 1)

        # pubs
        self.arm_pub = self.create_publisher(JointState, self.get_parameter("robot_action_topic").value, 1)
        grip_topic = self.get_parameter("gripper_action_topic").value or self._default_grip_topic()
        self.grip_pub = self._make_grip_pub(grip_topic)
        self.status_pub = self.create_publisher(String, "/episode/status", 1)

        # services
        self.create_service(Trigger, "/episode/home", self._srv_home)
        self.create_service(Trigger, "/episode/start", self._srv_start)
        self.create_service(Trigger, "/episode/stop", self._srv_stop)
        self.create_service(Trigger, "/episode/clear_fault", self._srv_clear)
        self.create_service(Trigger, "/episode/replay", self._srv_replay)

        # policy node clients
        self.enable_cli = self.create_client(SetBool, self.get_parameter("policy_enable_service").value)
        self.reset_cli = self.create_client(Trigger, self.get_parameter("policy_reset_service").value)
        self.use_replay = bool(self.get_parameter("use_replay").value)
        self.replay_start_cli = self.create_client(Trigger, self.get_parameter("replay_start_service").value)
        self.replay_stop_cli = self.create_client(Trigger, self.get_parameter("replay_stop_service").value)

        # timers
        self.create_timer(self.ctrl_dt, self._tick)
        self.create_timer(1.0 / float(self.get_parameter("status_rate").value), self._publish_status)

        # server health poller
        self._server_url = self.get_parameter("server_url").value
        self._server_period = float(self.get_parameter("server_check_period").value)
        threading.Thread(target=self._server_poll, daemon=True).start()

        # take command authority immediately (gate closed)
        self._set_gate(False)
        self.get_logger().info(f"EpisodeManager up (side={self.side}, state_source={self.get_parameter('state_source').value})")

    # ---------- helpers ----------
    def _resolve_grip_cmd(self, t):
        if t != "auto":
            return t
        return "multi_dof_command" if self.side == "left" else "float64_multi_array"

    def _default_grip_topic(self):
        return "/dg5f_left/lj_dg_pospid/reference" if self.side == "left" else "/inspire/right/target"

    def _make_grip_pub(self, topic):
        if self.gripper_command_type == "multi_dof_command":
            if MultiDOFCommand is None:
                raise RuntimeError("control_msgs missing for multi_dof_command")
            return self.create_publisher(MultiDOFCommand, topic, 1)
        if self.gripper_command_type == "float64_multi_array":
            from std_msgs.msg import Float64MultiArray
            self._F64 = Float64MultiArray
            return self.create_publisher(Float64MultiArray, topic, 1)
        return None

    def _setup_state_subs(self):
        src = self.get_parameter("state_source").value
        if src == "frame_aligned":
            if FrameAlignedState is None:
                raise RuntimeError("frame_aligned needs system_interface; use state_source:=joint_states")
            topic = self.get_parameter("state_topic").value or f"/system_{self.side}/frame_aligned_state"
            self.create_subscription(FrameAlignedState, topic, self._on_fa, 1)
        elif src == "joint_states":
            at = self.get_parameter("arm_state_topic").value or f"/system_{self.side}/joint_states"
            gt = self.get_parameter("gripper_state_topic").value or f"/dg5f_{self.side}/joint_states"
            self._au = self.get_parameter("arm_state_unit").value
            self._gu = self.get_parameter("gripper_state_unit").value
            self.create_subscription(JointState, at, self._on_arm_js, 1)
            self.create_subscription(JointState, gt, self._on_grip_js, 1)
        else:
            raise ValueError(f"unknown state_source: {src}")

    # ---------- callbacks ----------
    def _on_fa(self, msg):
        if getattr(msg, "side", "") and msg.side != self.side:
            return
        with self._lock:
            self._arm_rad = np.asarray(msg.robot_joint, dtype=np.float64)[: self.arm_size]
            self._grip_rad = np.asarray(msg.gripper_joint, dtype=np.float64)[: self.grip_size]
            self._state_t = time.monotonic()

    def _on_arm_js(self, msg):
        v = np.asarray(msg.position, dtype=np.float64)[: self.arm_size]
        if self._au == "deg":
            v = v * math.pi / 180.0
        with self._lock:
            self._arm_rad = v
            self._state_t = time.monotonic()

    def _on_grip_js(self, msg):
        v = np.asarray(msg.position, dtype=np.float64)[: self.grip_size]
        if self._gu == "deg":
            v = v * math.pi / 180.0
        with self._lock:
            self._grip_rad = v

    def _on_front(self, _):
        with self._lock:
            self._front_t = time.monotonic()

    def _on_wrist(self, _):
        with self._lock:
            self._wrist_t = time.monotonic()

    def _on_ft(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        with self._lock:
            self._ft_force = math.sqrt(f.x**2 + f.y**2 + f.z**2)
            self._ft_torque = math.sqrt(t.x**2 + t.y**2 + t.z**2)

    def _on_estop(self, msg):
        self._estop = bool(msg.data)
        if self._estop and self.state != FAULT:
            self._to_fault("e-stop")

    def _on_success(self, msg):
        if bool(msg.data):
            self._success = True

    # ---------- services ----------
    def _srv_home(self, req, res):
        if self.state == FAULT:
            res.success = False; res.message = "in FAULT; clear first"; return res
        self._to_homing()
        res.success = True; res.message = "homing"; return res

    def _srv_start(self, req, res):
        if self.state != READY:
            res.success = False; res.message = f"not READY (state={self.state})"; return res
        if not self._healthy():
            res.success = False; res.message = "inputs not healthy"; return res
        self._to_running()
        res.success = True; res.message = "running"; return res

    def _srv_stop(self, req, res):
        if self.state == RUNNING:
            self._to_stopping("manual")
            res.success = True; res.message = "stopping"
        else:
            res.success = False; res.message = f"not RUNNING (state={self.state})"
        return res

    def _srv_replay(self, req, res):
        # one-button "replay sample on the robot": home if needed, then run with image replay
        if self.state == FAULT:
            res.success = False; res.message = "in FAULT; clear first"; return res
        if self.state == RUNNING:
            res.success = False; res.message = "already RUNNING"; return res
        self._pending_replay = True
        if self.state == READY:
            self._to_running()
        else:
            self._to_homing()
        res.success = True; res.message = "replay requested (home -> run -> playback)"; return res

    def _srv_clear(self, req, res):
        if self.state != FAULT:
            res.success = False; res.message = "not in FAULT"; return res
        if self._estop:
            res.success = False; res.message = "e-stop still engaged"; return res
        self.fault_reason = ""
        self._to_homing()
        res.success = True; res.message = "cleared -> homing"; return res

    # ---------- transitions ----------
    def _set_gate(self, on: bool):
        self.policy_enabled = on
        if self.enable_cli.service_is_ready():
            self.enable_cli.call_async(SetBool.Request(data=on))
        else:
            self.get_logger().warn(f"policy enable service not ready (gate {on})", throttle_duration_sec=5.0)

    def _call_reset(self):
        self._call_trigger(self.reset_cli, "policy reset")

    def _call_trigger(self, cli, label):
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())
        else:
            self.get_logger().warn(f"{label}: service not ready", throttle_duration_sec=5.0)

    def _to_homing(self):
        self._set_gate(False)
        with self._lock:
            arm = self._arm_rad.copy() if self._arm_rad is not None else self.arm_home.copy()
            grip = self._grip_rad.copy() if self._grip_rad is not None else self.hand_home.copy()
        self._ramp_arm = arm
        self._ramp_grip = grip
        self._settle = 0
        self._t_home0 = time.monotonic()
        self.state = HOMING
        self.get_logger().info("-> HOMING")

    def _to_ready(self):
        self._set_gate(False)
        self.state = READY
        self.get_logger().info("-> READY")

    def _to_running(self):
        self._call_reset()
        self._success = False
        self._replay_active = self._pending_replay or self.use_replay
        if self._replay_active:
            self._call_trigger(self.replay_start_cli, "replay start")   # feed recorded images
        self._pending_replay = False
        self._set_gate(True)          # policy now drives
        self._t_ep0 = time.monotonic()
        if self.episodes_remaining > 0:
            self.episodes_remaining -= 1
        self.state = RUNNING
        self.get_logger().info(f"-> RUNNING (replay={self._replay_active})")

    def _to_stopping(self, reason):
        self._set_gate(False)         # cut policy output first
        if self._replay_active:
            self._call_trigger(self.replay_stop_cli, "replay stop")
            self._replay_active = False
        self.last_termination = reason
        with self._lock:
            self._hold_arm = self._arm_rad.copy() if self._arm_rad is not None else self.arm_home.copy()
            self._hold_grip = self._grip_rad.copy() if self._grip_rad is not None else self.hand_home.copy()
        self._t_stop0 = time.monotonic()
        self.state = STOPPING
        self.get_logger().info(f"-> STOPPING ({reason})")

    def _to_fault(self, reason):
        self._set_gate(False)
        self.fault_reason = reason
        self.state = FAULT
        self.get_logger().error(f"-> FAULT: {reason}")

    # ---------- health / watchdog ----------
    def _healthy(self):
        t = time.monotonic()
        with self._lock:
            cams = (t - self._front_t) < self.camera_timeout and (t - self._wrist_t) < self.camera_timeout
            st = (t - self._state_t) < self.camera_timeout and self._arm_rad is not None and self._grip_rad is not None
        return cams and st

    def _safety_violation(self):
        if self._estop:
            return "e-stop"
        with self._lock:
            f, tq = self._ft_force, self._ft_torque
        if self.ft_force_max > 0 and f > self.ft_force_max:
            return f"FT force {f:.1f}>{self.ft_force_max}"
        if self.ft_torque_max > 0 and tq > self.ft_torque_max:
            return f"FT torque {tq:.1f}>{self.ft_torque_max}"
        return ""

    # ---------- main tick ----------
    def _tick(self):
        if self.state == FAULT:
            return

        v = self._safety_violation()
        if v and self.state in (HOMING, READY, RUNNING, STOPPING):
            self._to_fault(v)
            return

        if self.state == BOOT:
            if self._healthy():
                self._to_homing()
            return

        if self.state == HOMING:
            self._home_tick()
            return

        if self.state == READY:
            if self._pending_replay and self._healthy():
                self._to_running()                       # replay requested -> run now (homed)
            elif self.auto_start and self.episodes_remaining != 0 and self._healthy():
                self._to_running()
            return

        if self.state == RUNNING:
            # policy publishes; manager only watches for termination
            if (time.monotonic() - self._t_ep0) > self.max_duration:
                self._to_stopping("timeout")
            elif self._success:
                self._to_stopping("success")
            elif not self._healthy():
                self._to_stopping("input_lost")
            return

        if self.state == STOPPING:
            self._publish_cmd(self._hold_arm, self._hold_grip)   # hold pose
            if (time.monotonic() - self._t_stop0) > self.hold_sec:
                if self.episodes_remaining == 0:
                    self._to_ready()         # batch done; sit at READY (already home next cycle)
                    self.get_logger().info("episodes exhausted")
                else:
                    self._to_homing()         # return to init for next episode
            return

    def _home_tick(self):
        if (time.monotonic() - self._t_home0) > self.home_timeout:
            self._to_fault("home timeout")
            return
        # open-loop ramp toward home (works with or without live feedback)
        self._ramp_arm = self._ramp_arm + np.clip(self.arm_home - self._ramp_arm, -self.home_max_delta, self.home_max_delta)
        self._ramp_grip = self._ramp_grip + np.clip(self.hand_home - self._ramp_grip, -self.home_max_delta, self.home_max_delta)
        self._publish_cmd(self._ramp_arm, self._ramp_grip)

        cmd_reached = (np.max(np.abs(self.arm_home - self._ramp_arm)) < 1e-3 and
                       np.max(np.abs(self.hand_home - self._ramp_grip)) < 1e-3)
        with self._lock:
            arm_m, grip_m, st_t = self._arm_rad, self._grip_rad, self._state_t
        feedback_fresh = (time.monotonic() - st_t) < self.camera_timeout and arm_m is not None and grip_m is not None
        meas_ok = True
        if feedback_fresh:
            meas_ok = (np.max(np.abs(self.arm_home - arm_m)) < self.home_tol and
                       np.max(np.abs(self.hand_home - grip_m[: self.grip_size])) < self.home_tol)
        if cmd_reached and meas_ok:
            self._settle += 1
            if self._settle >= self.home_settle_ticks:
                self._call_reset()
                self._to_ready()
        else:
            self._settle = 0

    # ---------- command publish (manager-owned during HOMING/STOPPING) ----------
    def _publish_cmd(self, arm_rad, grip_rad):
        if arm_rad is not None:
            out = rad2deg(arm_rad) if self.robot_topic_unit == "deg" else np.asarray(arm_rad)
            m = JointState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.name = ARM_JOINT_NAMES[: len(out)]
            m.position = [float(x) for x in out]
            self.arm_pub.publish(m)
        if grip_rad is not None and self.grip_pub is not None:
            if self.gripper_command_type == "multi_dof_command":
                c = MultiDOFCommand()
                c.dof_names = LEFT_DELTO_JOINT_NAMES[: len(grip_rad)]
                c.values = [float(x) for x in grip_rad]
                c.values_dot = [0.0] * len(grip_rad)
                self.grip_pub.publish(c)
            elif self.gripper_command_type == "float64_multi_array":
                self.grip_pub.publish(self._F64(data=[float(x) for x in grip_rad]))

    # ---------- status ----------
    def _publish_status(self):
        t = time.monotonic()
        with self._lock:
            cam_f = (t - self._front_t) < self.camera_timeout
            cam_w = (t - self._wrist_t) < self.camera_timeout
            st = (t - self._state_t) < self.camera_timeout and self._arm_rad is not None
            f, tq = self._ft_force, self._ft_torque
        ft_ok = True
        if self.ft_force_max > 0:
            ft_ok = f <= self.ft_force_max and (self.ft_torque_max <= 0 or tq <= self.ft_torque_max)
        elapsed = (t - self._t_ep0) if self.state == RUNNING else 0.0
        msg = {
            "state": self.state,
            "episode_elapsed_s": round(elapsed, 2),
            "episodes_remaining": self.episodes_remaining,
            "policy_enabled": self.policy_enabled,
            "fault_reason": self.fault_reason,
            "last_termination": self.last_termination,
            "health": {"front_cam": cam_f, "wrist_cam": cam_w, "state": st,
                       "server": self._server_ok, "ft": ft_ok},
        }
        self.status_pub.publish(String(data=json.dumps(msg)))

    def _server_poll(self):
        while rclpy.ok():
            ok = False
            if requests is not None:
                try:
                    r = requests.get(f"{self._server_url}/health", timeout=0.5)
                    ok = (r.status_code == 200 and r.json().get("model_loaded", False))
                except Exception:
                    ok = False
            self._server_ok = ok
            time.sleep(self._server_period)


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeManager()
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
