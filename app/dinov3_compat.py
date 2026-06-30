"""Make the deployed lerobot handle DINOv3 ViT **register tokens** at serving time.

The stock `Dinov2Backbone.forward` drops only the CLS token. DINOv3 ViT prepends
`CLS + N register tokens`, so the remaining sequence isn't a square patch grid and the
stock code raises "non-square patch grid" (the model fails to load/run). This applies a
monkeypatch so forward strips CLS + all register tokens before reshaping to (B, D, H, W).
It is a no-op for plain DINOv2 (0 registers) and self-guards against double application.

Importing the deployed lerobot from the fork is enough — call `apply()` once before the
policy is constructed (app.policy_runner does this), so no change to the fork is required.
"""
from __future__ import annotations

import contextlib
import logging

import torch
import torch.nn.functional as F

logger = logging.getLogger("vpi")


def apply() -> bool:
    try:
        from lerobot.policies.diffusion.modeling_diffusion import Dinov2Backbone
    except Exception as exc:  # lerobot not importable / different layout
        logger.warning("dinov3_compat: could not import Dinov2Backbone (%s)", exc)
        return False

    if getattr(Dinov2Backbone, "_dinov3_compat", False):
        return True

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.shape[-2:] != (224, 224):
            x = F.interpolate(x, size=(224, 224), mode="bilinear", align_corners=False)
        x = (x.clamp(0.0, 1.0) - self._mean) / self._std
        ctx = torch.no_grad() if getattr(self, "_freeze", True) else contextlib.nullcontext()
        with ctx:
            out = self.model(pixel_values=x)
        n_reg = int(
            getattr(self.model.config, "num_register_tokens",
                    getattr(self.model.config, "num_registers", 0)) or 0
        )
        feat = out.last_hidden_state[:, 1 + n_reg:, :]   # drop CLS + register tokens
        b, n, d = feat.shape
        h = w = int(round(n ** 0.5))
        if h * w != n:                                   # fallback: trailing square block
            h = w = int(n ** 0.5)
            feat = feat[:, n - h * w:, :]
        return feat.transpose(1, 2).reshape(b, d, h, w).contiguous()

    Dinov2Backbone.forward = forward
    Dinov2Backbone._dinov3_compat = True
    logger.info("dinov3_compat: patched Dinov2Backbone.forward (DINOv3 register tokens)")
    return True
