"""Pydantic request/response schemas for the inference server."""

from __future__ import annotations

from pydantic import BaseModel, Field


class PredictRequest(BaseModel):
    """One control-step observation.

    Images are base64-encoded PNG/JPEG bytes (any resolution — the policy resizes
    internally). State is the raw joint vector.
    """

    front_rgb: str = Field(..., description="base64-encoded PNG/JPEG, scene/zivid camera (RGB)")
    wrist_rgb: str = Field(..., description="base64-encoded PNG/JPEG, wrist camera (RGB)")
    state: list[float] = Field(..., description="robot state vector (arm 6 + hand 20 = 26)")


class PredictResponse(BaseModel):
    action: list[float] = Field(..., description="target joints (arm 6 + hand 20 = 26)")


class InfoResponse(BaseModel):
    model_id: str
    device: str
    cameras: list[str]
    state_dim: int
    action_dim: int
    scheduler: str
    num_inference_steps: int | None
    ruckig: dict | None = Field(default=None, description="jerk-limited smoothing config, or null if disabled")
    rtc: dict | None = Field(default=None, description="real-time chunking (async RTC) config, or null if disabled")


class HealthResponse(BaseModel):
    status: str
    model_loaded: bool


class ResetResponse(BaseModel):
    status: str
