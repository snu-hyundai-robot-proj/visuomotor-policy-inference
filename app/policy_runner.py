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
            self.policy.reset()

    @torch.no_grad()
    def predict(self, front_rgb: np.ndarray, wrist_rgb: np.ndarray, state: np.ndarray) -> np.ndarray:
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
        with self._lock:
            action = self.postprocess(self.policy.select_action(self.preprocess(obs)))
        return action.squeeze(0).float().cpu().numpy()

    def info(self) -> dict:
        return {
            "model_id": self.model_id,
            "device": self.device,
            "cameras": self.cameras,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "scheduler": getattr(self.policy.config, "noise_scheduler_type", "unknown"),
            "num_inference_steps": getattr(self.policy.config, "num_inference_steps", None),
        }
