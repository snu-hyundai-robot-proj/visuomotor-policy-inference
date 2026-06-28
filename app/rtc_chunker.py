"""Real-Time Chunking (RTC) async serving wrapper for a lerobot DiffusionPolicy (FlowMatch).

RTC is already implemented in lerobot (ActionQueue / RTCProcessor / LatencyTracker +
the use_rtc guidance inside DiffusionPolicy.generate_actions). The stock serving path,
however, calls `policy.select_action()` synchronously, so every `n_action_steps` the
action queue empties and a full inference runs *on the request path* -> the periodic
"slow in the middle" stall.

This wrapper wires our /predict serving to lerobot's built-in RTC:
  - /predict -> step() ingests one observation and returns one action INSTANTLY from
    lerobot's ActionQueue;
  - a background thread regenerates the next chunk (DINOv3 encode + flow denoise) while
    the current chunk executes, so inference overlaps execution (no stall);
  - use_rtc guidance aligns each new chunk with the unexecuted tail of the previous one.

Heavy inference runs OUTSIDE the policy lock (only the cheap obs-queue snapshot is locked),
so the request path never blocks on inference.
"""
from __future__ import annotations

import math
import threading
import time

import torch

from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE


class RTCChunker:
    def __init__(self, policy, postprocess, device, *, fps: float = 30.0,
                 use_rtc: bool = True, refill_threshold: int | None = None):
        from lerobot.policies.rtc.action_queue import ActionQueue
        from lerobot.policies.rtc.configuration_rtc import RTCConfig
        from lerobot.policies.rtc.latency_tracker import LatencyTracker
        from lerobot.policies.rtc.modeling_rtc import RTCProcessor

        self.policy = policy
        self.post = postprocess
        self.device = device
        self.fps = float(fps)
        self.tpc = 1.0 / self.fps
        cfg = policy.config
        self.n_action_steps = int(cfg.n_action_steps)
        self.use_rtc = bool(use_rtc)
        self.exec_horizon = int(getattr(cfg, "rtc_execution_horizon", 0) or max(1, self.n_action_steps // 2))

        # Enable RTC on the policy at inference time (no retrain needed).
        cfg.use_rtc = self.use_rtc
        cfg.rtc_execution_horizon = self.exec_horizon
        cfg.rtc_inference_delay = int(getattr(cfg, "rtc_inference_delay", 0) or 0)
        if self.use_rtc and getattr(policy.diffusion, "rtc_processor", None) is None:
            policy.diffusion.rtc_processor = RTCProcessor(RTCConfig())

        rcfg = RTCConfig()
        rcfg.enabled = self.use_rtc
        if hasattr(rcfg, "execution_horizon"):
            rcfg.execution_horizon = self.exec_horizon
        self._rcfg = rcfg
        self.queue = ActionQueue(rcfg)
        self.latency = LatencyTracker()
        # refill once the remaining buffer drops to this many actions (must exceed
        # exec_horizon + inference_delay so the next chunk is ready before the queue drains)
        self.refill_threshold = int(refill_threshold if refill_threshold is not None else self.exec_horizon + 1)

        self._image_keys = list(cfg.image_features) if cfg.image_features else []
        self._gen_keys = None                  # obs keys actually present (set on first populate)
        self._policy_lock = threading.Lock()   # guards the (cheap) obs-queue snapshot/populate
        self._gen_flag_lock = threading.Lock()
        self._generating = False
        self._primed = False

    def reset(self):
        with self._policy_lock:
            self.policy.reset()
        self.queue = self.queue.__class__(self._rcfg)
        self._primed = False

    @torch.no_grad()
    def _populate_obs(self, batch: dict):
        from lerobot.policies.utils import populate_queues

        b = dict(batch)
        b.pop(ACTION, None)  # the preprocessor adds action=None (eval convention); never queue it
        if self._image_keys:
            b[OBS_IMAGES] = torch.stack([b[k] for k in self._image_keys], dim=-4)
        self.policy._queues = populate_queues(self.policy._queues, b)
        if self._gen_keys is None:  # the obs keys to stack for generation (mirror predict_action_chunk)
            self._gen_keys = [k for k in b if k in self.policy._queues]

    @torch.no_grad()
    def _generate(self):
        """Snapshot obs (locked) -> generate chunk (UNLOCKED, heavy) -> merge."""
        t0 = time.perf_counter()
        idx_before = self.queue.get_action_index()
        with self._policy_lock:
            q = self.policy._queues
            if not self._gen_keys or OBS_STATE not in q or len(q[OBS_STATE]) == 0:
                return
            stacked = {k: torch.stack(list(q[k]), dim=1) for k in self._gen_keys}
            self.policy._rtc_prev_left_over = self.queue.get_left_over()
            try:
                lat = float(self.latency.max())
            except Exception:
                lat = 0.0
            self.policy.config.rtc_inference_delay = math.ceil(lat / self.tpc) if lat > 0 else 0
        original = self.policy.diffusion.generate_actions(stacked)   # (1, n_action_steps, D) normalized
        processed = self.post(original)                              # unnormalized
        dt = time.perf_counter() - t0
        self.latency.add(dt)
        # clamp the delay so discarding stale actions can never fully drain the queue
        real_delay = min(max(0, math.ceil(dt / self.tpc)), self.n_action_steps - 1)
        self.queue.merge(original.squeeze(0), processed.squeeze(0), real_delay, idx_before)

    def _maybe_spawn(self):
        if self.queue.qsize() > self.refill_threshold:
            return
        with self._gen_flag_lock:
            if self._generating:
                return
            self._generating = True

        def run():
            try:
                self._generate()
            finally:
                with self._gen_flag_lock:
                    self._generating = False

        threading.Thread(target=run, daemon=True).start()

    @torch.no_grad()
    def step(self, batch: dict) -> torch.Tensor:
        """Ingest one preprocessed observation; return one postprocessed action (action_dim,)."""
        with self._policy_lock:
            self._populate_obs(batch)
        if self.queue.empty() and not self._primed:
            self._generate()           # cold start: first chunk synchronously
            self._primed = True
        self._maybe_spawn()            # background refill while executing
        act = self.queue.get()
        if act is None:                # rare: background not ready -> synchronous catch-up
            self._generate()
            act = self.queue.get()
        return act
