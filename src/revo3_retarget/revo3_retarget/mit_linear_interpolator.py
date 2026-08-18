from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


_MIN_DURATION_S = 1e-6


@dataclass(frozen=True)
class MitCommandSample:
    joint_names: list[str]
    position: list[float]
    velocity: list[float]
    kp: list[float]
    kd: list[float]


def duration_from_hz(hz: float, name: str = "hz") -> float:
    value = float(hz)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a finite positive value.")
    return 1.0 / value


def _as_vector(name: str, values, expected_size: int | None = None) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.ndim != 1:
        raise ValueError(f"{name} must be a 1-D sequence.")
    if expected_size is not None and arr.size != expected_size:
        raise ValueError(f"{name} length must be {expected_size}, got {arr.size}.")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain only finite values.")
    return arr


class LinearMitCommandInterpolator:
    """Linearly interpolates between retarget targets for high-rate MIT commands."""

    def __init__(self, default_duration_s: float):
        self.default_duration_s = self._valid_duration(default_duration_s)
        self._joint_names: list[str] = []
        self._start_position: np.ndarray | None = None
        self._target_position: np.ndarray | None = None
        self._target_velocity: np.ndarray | None = None
        self._kp: list[float] = []
        self._kd: list[float] = []
        self._start_time_s = 0.0
        self._end_time_s = 0.0
        self._last_target_time_s: float | None = None

    @staticmethod
    def _valid_duration(duration_s: float) -> float:
        value = float(duration_s)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("default_duration_s must be a finite positive value.")
        return value

    def update_target(
        self,
        joint_names: list[str],
        position,
        kp,
        kd,
        now_s: float,
    ) -> None:
        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite.")

        joint_names = list(joint_names)
        position_arr = _as_vector("position", position)
        if len(joint_names) != position_arr.size:
            raise ValueError(f"joint_names length must be {position_arr.size}, got {len(joint_names)}.")
        kp_arr = _as_vector("kp", kp, position_arr.size)
        kd_arr = _as_vector("kd", kd, position_arr.size)

        previous_joint_names = list(self._joint_names)
        previous_target_position = None if self._target_position is None else self._target_position.copy()
        previous_target_time_s = self._last_target_time_s

        current = self.sample(now_s)
        compatible = (
            current is not None
            and current.joint_names == joint_names
            and len(current.position) == position_arr.size
        )
        if compatible:
            start_position = np.asarray(current.position, dtype=float)
        else:
            start_position = position_arr.copy()

        duration_s = self.default_duration_s
        target_velocity = np.zeros_like(position_arr)
        target_velocity_compatible = (
            previous_target_time_s is not None
            and previous_target_position is not None
            and previous_joint_names == joint_names
            and previous_target_position.size == position_arr.size
        )
        if previous_target_time_s is not None:
            interval_s = now_s - previous_target_time_s
            if math.isfinite(interval_s) and interval_s > _MIN_DURATION_S:
                duration_s = interval_s
                if target_velocity_compatible:
                    target_velocity = (position_arr - previous_target_position) / interval_s

        self._joint_names = joint_names
        self._start_position = start_position
        self._target_position = position_arr.copy()
        self._target_velocity = target_velocity.copy()
        self._kp = kp_arr.tolist()
        self._kd = kd_arr.tolist()
        self._start_time_s = now_s
        self._end_time_s = now_s + max(duration_s, _MIN_DURATION_S)
        self._last_target_time_s = now_s

    def sample(self, now_s: float) -> MitCommandSample | None:
        if self._start_position is None or self._target_position is None:
            return None

        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite.")

        duration_s = self._end_time_s - self._start_time_s
        delta = self._target_position - self._start_position
        if duration_s <= _MIN_DURATION_S or now_s >= self._end_time_s:
            position = self._target_position.copy()
            velocity = np.zeros_like(position)
        else:
            alpha = float(np.clip((now_s - self._start_time_s) / duration_s, 0.0, 1.0))
            position = self._start_position + alpha * delta
            velocity = (
                self._target_velocity.copy()
                if self._target_velocity is not None
                else np.zeros_like(position)
            )

        return MitCommandSample(
            joint_names=list(self._joint_names),
            position=position.tolist(),
            velocity=velocity.tolist(),
            kp=list(self._kp),
            kd=list(self._kd),
        )
