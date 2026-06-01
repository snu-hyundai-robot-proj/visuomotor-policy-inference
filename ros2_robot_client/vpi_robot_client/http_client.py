"""Thin HTTP client for the visuomotor-policy-inference server.

Encodes two RGB frames + a state vector, POSTs /predict, returns the action.
JPEG is the default transport (≈3x smaller than PNG — see BENCHMARK.md); the server
decodes either via PIL. Use a persistent session to avoid per-call TCP setup.
"""
from __future__ import annotations

import base64
import io

import numpy as np
import requests
from PIL import Image


class PolicyHTTPClient:
    def __init__(self, url: str, timeout: float = 5.0, image_format: str = "JPEG", jpeg_quality: int = 90):
        self.url = url.rstrip("/")
        self.timeout = timeout
        self.fmt = image_format.upper()
        self.quality = int(jpeg_quality)
        self.session = requests.Session()

    def info(self) -> dict:
        r = self.session.get(f"{self.url}/info", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def reset(self) -> None:
        r = self.session.post(f"{self.url}/reset", timeout=self.timeout)
        r.raise_for_status()

    def _encode(self, img: np.ndarray) -> str:
        buf = io.BytesIO()
        kw = {"quality": self.quality} if self.fmt in ("JPEG", "JPG") else {}
        Image.fromarray(np.asarray(img, dtype=np.uint8)).save(buf, format=self.fmt, **kw)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def predict(self, front_rgb: np.ndarray, wrist_rgb: np.ndarray, state: np.ndarray) -> np.ndarray:
        body = {
            "front_rgb": self._encode(front_rgb),
            "wrist_rgb": self._encode(wrist_rgb),
            "state": np.asarray(state, dtype=np.float32).reshape(-1).tolist(),
        }
        r = self.session.post(f"{self.url}/predict", json=body, timeout=self.timeout)
        r.raise_for_status()
        return np.asarray(r.json()["action"], dtype=np.float32).reshape(-1)
