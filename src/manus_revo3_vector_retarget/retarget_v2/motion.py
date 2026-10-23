"""Position, velocity and acceleration bounded reference trajectories.

This module generates *references*. It does not communicate with hardware and
does not claim that an actuator follows the reference or that collisions are
safe. A hardware adapter must supply measured-state checks and its own stop.

Each joint follows an analytic acceleration/cruise/braking trajectory ending
at the latest bounded target with zero velocity. Replanning starts at the
current reference position AND velocity. In particular, position limits are
not enforced by clipping the moving output, which would hide an impossible
instantaneous stop. Reversal first respects the existing stopping distance.

A stale sample latches the guard into braking, then hold. Fresh samples alone
do not re-enable movement: call ``rearm()`` after it has stopped. ``reset(q)``
is an explicit declaration of a stationary starting state and also re-arms.
Use it only with a known stationary initial state, never to conceal tracking
error. These semantics avoid unexpectedly restarting when a stream returns.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MotionState:
    """A detached snapshot; editing its arrays cannot edit the guard."""

    q: np.ndarray
    dq: np.ndarray
    mode: str


class MotionGuard:
    """Generate joint references using radians and seconds.

    ``lower`` and ``upper`` are nonempty one-dimensional vectors.
    ``velocity_limit`` and ``acceleration_limit`` may be scalars or vectors
    of the same size and must be strictly positive. All values must be finite.
    The default initial position is the midpoint of each joint range; real
    hardware should always provide its known stationary initial position.

    Acceleration is piecewise constant and bounded, but jerk is not bounded.
    The elapsed ``dt`` must be positive and finite. Timing freshness is the
    caller's responsibility: set ``fresh=False`` for stale input, invalid
    tracking, a missed deadline, or an intentionally requested smooth stop.
    Independent joint profiles respect all per-joint bounds but do not impose
    a synchronized arrival time or collision constraints.
    """

    def __init__(
        self,
        lower,
        upper,
        velocity_limit,
        acceleration_limit,
        initial_q=None,
    ):
        self._lower = self._vector(lower, "lower")
        self._upper = self._vector(upper, "upper", self._lower.shape)
        if np.any(self._lower >= self._upper):
            raise ValueError("Every lower position limit must be below upper")
        self._velocity_limit = self._limit(velocity_limit, "velocity_limit")
        self._acceleration_limit = self._limit(
            acceleration_limit, "acceleration_limit"
        )
        if initial_q is None:
            initial_q = 0.5 * self._lower + 0.5 * self._upper
        self.reset(initial_q)

    @staticmethod
    def _vector(value, name, shape=None):
        try:
            result = np.array(value, dtype=np.float64, copy=True)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite numeric vector") from exc
        if result.ndim != 1 or result.size == 0:
            raise ValueError(f"{name} must be a nonempty one-dimensional vector")
        if shape is not None and result.shape != shape:
            raise ValueError(f"{name} must have shape {shape}, got {result.shape}")
        if not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain only finite values")
        return result

    def _limit(self, value, name):
        array = np.asarray(value, dtype=np.float64)
        if array.ndim == 0:
            array = np.full(self._lower.shape, float(array))
        array = self._vector(array, name, self._lower.shape)
        if np.any(array <= 0):
            raise ValueError(f"{name} must be strictly positive")
        return array

    @property
    def state(self):
        return MotionState(self._q.copy(), self._dq.copy(), self._mode)

    def reset(self, q):
        """Start armed at a declared stationary position inside the limits."""
        q = self._vector(q, "q", self._lower.shape)
        if np.any(q < self._lower) or np.any(q > self._upper):
            raise ValueError("Reset position must be inside the position limits")
        self._q = q
        self._dq = np.zeros_like(q)
        self._mode = "armed"

    def rearm(self):
        """Enable following fresh targets after braking has fully stopped.

        Re-arming an already armed guard is a no-op. A moving, stale-latched
        guard raises instead of cancelling its stop midway through braking.
        """
        if self._mode == "braking" or (
            self._mode != "armed" and np.any(self._dq != 0)
        ):
            raise RuntimeError("Cannot rearm until the guard is stopped in hold")
        self._mode = "armed"

    def step(self, target, dt, fresh=True):
        """Return detached ``(q, dq)`` references after ``dt`` seconds.

        Fresh targets must be finite vectors and are saturated to position
        limits *before* trajectory planning. During stale braking/hold the
        target is ignored and may be ``None``. Invalid active input raises
        without altering state; the caller should then request a stale stop.
        Returned velocity is the instantaneous end-of-step velocity; because
        phases may switch within a step, it is not the average displacement
        divided by ``dt``.
        """
        if isinstance(dt, (bool, np.bool_)) or np.ndim(dt) != 0:
            raise ValueError("dt must be a finite positive scalar in seconds")
        try:
            dt = float(dt)
        except (TypeError, ValueError) as exc:
            raise ValueError("dt must be a finite positive scalar in seconds") from exc
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be a finite positive scalar in seconds")
        if not isinstance(fresh, (bool, np.bool_)):
            raise ValueError("fresh must be a boolean")

        if not fresh or self._mode != "armed":
            self._brake(dt)
            return self._q.copy(), self._dq.copy()

        target = self._vector(target, "target", self._lower.shape)
        target = np.clip(target, self._lower, self._upper)
        for joint in range(self._q.size):
            self._q[joint], self._dq[joint] = self._advance_joint(
                self._q[joint], self._dq[joint], target[joint],
                self._velocity_limit[joint], self._acceleration_limit[joint], dt,
            )
        return self._q.copy(), self._dq.copy()

    def _brake(self, dt):
        # Integrate only until velocity reaches zero, even when dt is longer
        # than the complete stopping time. No position clipping is used.
        magnitude = np.abs(self._dq)
        duration = np.minimum(dt, magnitude / self._acceleration_limit)
        self._q += np.sign(self._dq) * duration * (
            magnitude - 0.5 * self._acceleration_limit * duration
        )
        self._dq = np.sign(self._dq) * np.maximum(
            magnitude - self._acceleration_limit * dt, 0.0
        )
        self._mode = "braking" if np.any(self._dq != 0) else "hold"

    @staticmethod
    def _advance_joint(q, velocity, target, vmax, amax, dt):
        # Choosing the direction relative to the stopping point handles an
        # abrupt target reversal: the unavoidable old-direction excursion is
        # exactly the feasible stopping distance, then motion reverses.
        stopping_point = q + velocity * abs(velocity) / (2.0 * amax)
        direction = np.sign(target - stopping_point)
        if direction == 0:
            direction = np.sign(velocity) if velocity != 0 else 1.0
        distance = direction * (target - q)
        initial_speed = direction * velocity
        peak = np.sqrt(max(amax * distance + 0.5 * initial_speed**2, 0.0))
        peak = min(vmax, max(peak, initial_speed, 0.0))

        acceleration_time = max((peak - initial_speed) / amax, 0.0)
        cruise_distance = max(
            distance - (peak**2 - 0.5 * initial_speed**2) / amax, 0.0
        )
        cruise_time = cruise_distance / peak if peak > 0 else 0.0
        braking_time = peak / amax

        if dt >= acceleration_time + cruise_time + braking_time:
            # This is an analytically reached endpoint, not position clipping.
            return target, 0.0

        used = min(dt, acceleration_time)
        q += velocity * used + 0.5 * direction * amax * used**2
        velocity += direction * amax * used
        dt -= used
        if dt <= 0:
            return q, velocity

        velocity = direction * peak
        used = min(dt, cruise_time)
        q += velocity * used
        dt -= used
        if dt <= 0:
            return q, velocity

        q += velocity * dt - 0.5 * direction * amax * dt**2
        velocity -= direction * amax * dt
        return q, velocity
