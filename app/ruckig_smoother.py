"""Stateful, jerk-limited smoother for the served policy action stream (Ruckig OTG).

The policy emits a new target action per `/predict` call. Sending those verbatim to a
robot produces unbounded acceleration/jerk (the output is noisy, like the teleop data it
imitates). This wraps Ruckig's online trajectory generator to turn the raw target stream
into a velocity/acceleration/jerk-limited command stream, preserving units and dimension.

Stateful: construct once, call `step()` each control tick; call `clear()` on episode
reset so the next `step()` re-seeds at the new starting action.

`ruckig` is imported lazily, so importing this module never requires it; only constructing
a smoother does.
"""
from __future__ import annotations

from typing import Optional, Sequence, Union

import numpy as np

Limit = Union[float, Sequence[float], np.ndarray]


class RuckigSmoother:
    def __init__(
        self,
        dof: int,
        control_dt: float,
        max_velocity: Limit,
        max_acceleration: Limit,
        max_jerk: Limit,
        *,
        target_velocity_mode: str = "zero",
    ) -> None:
        from ruckig import InputParameter, OutputParameter, Result, Ruckig  # lazy

        self._Result = Result
        self.dof = int(dof)
        self.dt = float(control_dt)
        if self.dof <= 0:
            raise ValueError("dof must be positive")
        if self.dt <= 0:
            raise ValueError("control_dt must be positive")
        if target_velocity_mode not in ("zero", "fd"):
            raise ValueError("target_velocity_mode must be 'zero' or 'fd'")

        self.vmax = self._as_vec(max_velocity)
        self.amax = self._as_vec(max_acceleration)
        self.jmax = self._as_vec(max_jerk)
        self.target_velocity_mode = target_velocity_mode

        self._otg = Ruckig(self.dof, self.dt)
        self._inp = InputParameter(self.dof)
        self._out = OutputParameter(self.dof)
        self._inp.max_velocity = self.vmax.tolist()
        self._inp.max_acceleration = self.amax.tolist()
        self._inp.max_jerk = self.jmax.tolist()

        self._prev_target: Optional[np.ndarray] = None
        self.initialized = False

    def _as_vec(self, x: Limit) -> np.ndarray:
        a = np.asarray(x, dtype=float).reshape(-1)
        if a.size == 1:
            a = np.full(self.dof, float(a[0]))            # scalar -> broadcast to every dim
        elif a.size < self.dof:
            a = np.concatenate([a, np.full(self.dof - a.size, a[-1])])  # pad remaining dims with last entry
        elif a.size > self.dof:
            raise ValueError(f"limit length {a.size} exceeds dof {self.dof}")
        if not np.all(a > 0):
            raise ValueError("all limits must be > 0")
        return a

    def clear(self) -> None:
        """Forget state so the next step() re-seeds at the new starting action."""
        self.initialized = False
        self._prev_target = None

    def reset(self, position: Sequence[float], velocity: Optional[Sequence[float]] = None) -> None:
        p = np.asarray(position, dtype=float).reshape(-1)
        if p.size != self.dof:
            raise ValueError(f"position length {p.size} does not match dof {self.dof}")
        v = np.zeros(self.dof) if velocity is None else np.asarray(velocity, dtype=float).reshape(-1)
        self._inp.current_position = p.tolist()
        self._inp.current_velocity = v.tolist()
        self._inp.current_acceleration = [0.0] * self.dof
        self._prev_target = p.copy()
        self.initialized = True

    def step(self, target: Sequence[float]) -> np.ndarray:
        """Advance one control cycle toward `target`; return the jerk-limited command."""
        t = np.asarray(target, dtype=float).reshape(-1)
        if t.size != self.dof:
            raise ValueError(f"target length {t.size} does not match dof {self.dof}")
        if not self.initialized:
            self.reset(t)
            return t.copy()

        self._inp.target_position = t.tolist()
        if self.target_velocity_mode == "fd" and self._prev_target is not None:
            tv = np.clip((t - self._prev_target) / self.dt, -self.vmax, self.vmax)
        else:
            tv = np.zeros(self.dof)
        self._inp.target_velocity = tv.tolist()
        self._inp.target_acceleration = [0.0] * self.dof

        result = self._otg.update(self._inp, self._out)
        if int(result) < 0:  # Ruckig error -> degrade gracefully to the raw target
            self._prev_target = t.copy()
            return t.copy()
        self._out.pass_to_input(self._inp)
        self._prev_target = t.copy()
        return np.asarray(self._out.new_position, dtype=float)
