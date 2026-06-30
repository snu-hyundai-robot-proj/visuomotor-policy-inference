"""Loads the FlowMatch Diffusion Policy from the Hugging Face Hub and runs inference.

The model uses the `FlowMatch` noise scheduler, which is a custom addition in the
snu-hyundai-robot-proj fork of LeRobot — so the `lerobot` package MUST be installed
from that fork (see requirements.txt), not from PyPI.
"""

from __future__ import annotations

import base64
import io
import os
import threading

import numpy as np
import torch
from PIL import Image

from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors

FRONT_KEY = "observation.images.front_rgb"
WRIST_KEY = "observation.images.wrist_rgb"
STATE_KEY = "observation.state"
GRIPPER_KEY = "observation.gripper_sensor"
FT_KEY = "observation.wrist_ft_sensor"
ACTION_KEY = "action"


def _resolve_device(device: str) -> str:
    if device == "cuda" and not torch.cuda.is_available():
        return "cpu"
    if device == "mps" and not torch.backends.mps.is_available():
        return "cpu"
    return device


def decode_image(b64: str) -> np.ndarray:
    """base64 PNG/JPEG -> RGB uint8 (H, W, 3)."""
    raw = base64.b64decode(b64)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    return np.asarray(img, dtype=np.uint8)


class PolicyRunner:
    """Thread-safe single-robot policy runner (one action queue)."""

    def __init__(self, model_id: str | None = None, device: str | None = None):
        self.model_id = model_id or os.environ.get("VPI_MODEL_ID", "Ngseo/hyundai-uiwang-left-flowmatch")
        self.device = _resolve_device(device or os.environ.get("VPI_DEVICE", "cpu"))
        self._lock = threading.Lock()

        # Make the deployed lerobot handle DINOv3 register tokens (no-op for resnet/dinov2).
        from app.dinov3_compat import apply as _apply_dinov3_compat
        _apply_dinov3_compat()

        policy = DiffusionPolicy.from_pretrained(self.model_id)
        policy.config.device = self.device  # saved config pins cuda; align to runtime device
        policy.to(self.device).eval()
        policy.reset()
        self.policy = policy
        self.preprocess, self.postprocess = make_pre_post_processors(
            policy.config, self.model_id, preprocessor_overrides={"device_processor": {"device": self.device}}
        )

        self.state_dim = int(policy.config.input_features[STATE_KEY].shape[0])
        self.action_dim = int(policy.config.output_features[ACTION_KEY].shape[0])
        self.cameras = [k for k in policy.config.input_features if "image" in k]
        # Expected (H, W) per camera (shape is (C, H, W)). The two camera views are
        # stacked together inside the policy, so each frame must be resized to its
        # declared size first — real cameras (Zivid vs RealSense) differ in resolution.
        self.image_hw = {
            k: (int(policy.config.input_features[k].shape[1]), int(policy.config.input_features[k].shape[2]))
            for k in self.cameras
        }

        # Extra proprio inputs some checkpoints require (e.g. the *_full / DINOv3 models).
        feats = policy.config.input_features
        self.needs_gripper = GRIPPER_KEY in feats
        self.needs_ft = FT_KEY in feats
        self.gripper_dim = int(feats[GRIPPER_KEY].shape[0]) if self.needs_gripper else 0
        self.ft_dim = int(feats[FT_KEY].shape[0]) if self.needs_ft else 0

        self.smoother = self._build_smoother()
        self.rtc = self._build_rtc()

    def _build_rtc(self):
        """Optional Real-Time Chunking: serve actions from lerobot's RTC ActionQueue while a
        background thread regenerates the next chunk (no inference on the request path).
        VPI_RTC=0 disables it (falls back to synchronous select_action). VPI_RTC_HORIZON
        overrides the execution horizon."""
        if os.environ.get("VPI_RTC", "1").lower() in ("0", "false", "no", ""):
            return None
        if str(getattr(self.policy.config, "noise_scheduler_type", "")).lower() != "flowmatch":
            import logging
            logging.getLogger("vpi").warning("VPI_RTC requested but policy is not FlowMatch; RTC disabled.")
            return None
        try:
            from app.rtc_chunker import RTCChunker

            fps = float(os.environ.get("VPI_FPS", "30"))
            horizon = os.environ.get("VPI_RTC_HORIZON")
            rtc = RTCChunker(self.policy, self.postprocess, self.device, fps=fps, use_rtc=True,
                             refill_threshold=(int(horizon) + 1) if horizon else None)
            self._rtc_cfg = {"fps": fps, "execution_horizon": rtc.exec_horizon,
                             "refill_threshold": rtc.refill_threshold}
            return rtc
        except Exception as exc:
            import logging
            logging.getLogger("vpi").warning("RTC disabled (%s); using synchronous select_action.", exc)
            self._rtc_cfg = None
            return None

    def _build_smoother(self):
        """Optional Ruckig jerk-limited smoother for the action stream (env-configured).

        VPI_RUCKIG=0 disables it. VPI_FPS sets the control rate (default 30 Hz). Limits:
        VPI_RUCKIG_MAX_{VEL,ACC,JERK} (scalars, broadcast); the default velocity is the
        per-joint HDR35_20 arm limit with hand dims padded to the action dim.
        """
        if os.environ.get("VPI_RUCKIG", "1").lower() in ("0", "false", "no", ""):
            return None
        fps = float(os.environ.get("VPI_FPS", "30"))
        # arm = HDR35_20 URDF joint velocity limits; hand dims pad to action_dim.
        vel_env = os.environ.get("VPI_RUCKIG_MAX_VEL")
        vmax = float(vel_env) if vel_env else [3.141, 3.141, 3.316, 5.410, 5.410, 7.330, 3.0]
        amax = float(os.environ.get("VPI_RUCKIG_MAX_ACC", "12.0"))
        jmax = float(os.environ.get("VPI_RUCKIG_MAX_JERK", "500.0"))
        try:
            from app.ruckig_smoother import RuckigSmoother

            sm = RuckigSmoother(self.action_dim, 1.0 / fps, vmax, amax, jmax)
            self._ruckig_cfg = {"fps": fps, "max_acceleration": amax, "max_jerk": jmax}
            return sm
        except Exception as exc:  # ruckig missing / bad limits -> serve without smoothing
            import logging

            logging.getLogger("vpi").warning(
                "Ruckig smoothing disabled (%s). `pip install ruckig` to enable.", exc
            )
            self._ruckig_cfg = None
            return None

    @staticmethod
    def _fit(img: np.ndarray, hw: tuple[int, int]) -> np.ndarray:
        """Resize RGB uint8 (H, W, 3) to (H, W) = hw if needed (bilinear)."""
        h, w = hw
        if img.shape[0] == h and img.shape[1] == w:
            return img
        resized = Image.fromarray(np.asarray(img, dtype=np.uint8)).resize((w, h), Image.BILINEAR)
        return np.asarray(resized, dtype=np.uint8)

    def reset(self) -> None:
        with self._lock:
            if self.rtc is not None:
                self.rtc.reset()       # resets the policy + clears the RTC action queue
            else:
                self.policy.reset()
            if self.smoother is not None:
                self.smoother.clear()  # next predict re-seeds the smoother at the new start

    def _vec(self, x, dim, name):
        if x is None:
            raise ValueError(f"this model requires '{name}' ({dim} dims), but none was provided")
        x = np.asarray(x, dtype=np.float32).reshape(-1)
        if x.shape[-1] != dim:
            raise ValueError(f"{name} must have {dim} dims, got {x.shape[-1]}")
        return torch.from_numpy(x).unsqueeze(0).to(self.device)

    @torch.no_grad()
    def predict(self, front_rgb: np.ndarray, wrist_rgb: np.ndarray, state: np.ndarray,
                gripper_sensor=None, wrist_ft_sensor=None) -> np.ndarray:
        if state.shape[-1] != self.state_dim:
            raise ValueError(f"state must have {self.state_dim} dims, got {state.shape[-1]}")
        front_rgb = self._fit(front_rgb, self.image_hw.get(FRONT_KEY, front_rgb.shape[:2]))
        wrist_rgb = self._fit(wrist_rgb, self.image_hw.get(WRIST_KEY, wrist_rgb.shape[:2]))
        obs = {
            FRONT_KEY: torch.from_numpy(front_rgb).float().div(255).permute(2, 0, 1).unsqueeze(0).to(self.device),
            WRIST_KEY: torch.from_numpy(wrist_rgb).float().div(255).permute(2, 0, 1).unsqueeze(0).to(self.device),
            STATE_KEY: torch.from_numpy(state.astype(np.float32)).unsqueeze(0).to(self.device),
            "task": "",
            "robot_type": "",
        }
        if self.needs_gripper:  # *_full / DINOv3 checkpoints also condition on these sensors
            obs[GRIPPER_KEY] = self._vec(gripper_sensor, self.gripper_dim, "gripper_sensor")
        if self.needs_ft:
            obs[FT_KEY] = self._vec(wrist_ft_sensor, self.ft_dim, "wrist_ft_sensor")
        if self.rtc is not None:
            # RTC: serve instantly from the action queue; inference overlaps execution in a bg thread
            act = self.rtc.step(self.preprocess(obs))
            action = act.float().cpu().numpy().reshape(-1)
            if self.smoother is not None:
                with self._lock:
                    action = self.smoother.step(action).astype(np.float32)
            return action
        with self._lock:
            action = self.postprocess(self.policy.select_action(self.preprocess(obs)))
            action = action.squeeze(0).float().cpu().numpy()
            if self.smoother is not None:
                action = self.smoother.step(action).astype(np.float32)  # jerk-limited command
        return action

    def info(self) -> dict:
        return {
            "model_id": self.model_id,
            "device": self.device,
            "cameras": self.cameras,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "scheduler": getattr(self.policy.config, "noise_scheduler_type", "unknown"),
            "num_inference_steps": getattr(self.policy.config, "num_inference_steps", None),
            "ruckig": getattr(self, "_ruckig_cfg", None) if self.smoother is not None else None,
            "rtc": getattr(self, "_rtc_cfg", None) if self.rtc is not None else None,
        }
