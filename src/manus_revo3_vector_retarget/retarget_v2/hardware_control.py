"""Direct or interpolated MIT references with operator start and pause.

The selected hand's last valid JointState supplies interpolated startup position.
Motor faults, current, bus counters, and feedback age are not control gates.
Glove target expiry stops publication and requires an explicit new arm request.
Interpolated output also checks command timer gaps. The downstream MIT timeout
holds position when publication stops.
"""
from __future__ import annotations

import math

import numpy as np

from .hardware_interpolation import CommandInterpolator


class HardwareController:
    """ROS-independent controller; serialize calls under one outer lock.

    A first valid JointState and a fresh target are sufficient to arm. Direct
    output publishes each subsequent target once, without a startup ramp or
    position smoothing. Optional interpolation starts from the last measured
    pose and retains the configured startup ramp and motion limits.
    """

    def __init__(self, profile, interpolate=False):
        if type(interpolate) is not bool:
            raise ValueError("interpolate must be a boolean")
        self.profile = profile
        self.interpolate = interpolate
        self.guard = self._new_guard()
        self.enabled = False
        self.reason = "startup"
        self.last_error = None
        self._closed = False
        self._joint_at = None
        self._q_joint = self._dq_joint = None
        self._feedback_delivery_count = 0
        self._target = self._target_at = None
        self._target_valid = False
        self._applied_target_at = None
        self._last_sample = None
        self._last_command = self._last_velocity = None
        self._direct_command = self._direct_target_at = None
        self.commands_generated = 0

    def _new_guard(self):
        return CommandInterpolator(
            self.profile.names, self.profile.lower, self.profile.upper, self.profile.command)

    @staticmethod
    def _number(value, name):
        if isinstance(value, (bool, np.bool_)) or np.ndim(value) != 0:
            raise ValueError(f"{name} must be a finite scalar")
        try:
            value = float(value)
        except (ValueError, TypeError) as exc:
            raise ValueError(f"{name} must be a finite scalar") from exc
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite scalar")
        return value

    def _latch(self, reason, error=None):
        self.enabled = False
        self.reason = str(reason)
        self.guard.set_enabled(False)
        self._direct_command = self._direct_target_at = None
        if error is not None:
            self.last_error = str(error)

    def pause(self, reason="paused"):
        """Stop publishing while retaining the target for an explicit resume."""
        if not self._closed:
            self._latch(reason)

    def clear_target(self, reason="recalibration"):
        """Pause and discard the old glove target before recalibration."""
        self.pause(reason)
        self._target = self._target_at = None
        self._target_valid = False
        self._applied_target_at = None

    def shutdown(self):
        self._latch("shutdown")
        self._closed = True

    def update_feedback(self, names, position, velocity, stamp, received_at,
                        now_ros, now_monotonic):
        """Keep the latest valid named JointState, without sampling watchdogs.

        ROS stamp/clock arguments remain accepted for transport compatibility;
        they do not gate feedback, including simulation clock resets. Malformed
        packets are rejected without replacing the last valid measured state.
        """
        if self._closed:
            return False
        try:
            received = self._number(received_at, "received_at")
            if received < 0:
                raise ValueError("Feedback receipt time must be nonnegative")
            q = self.profile.ordered(names, position)
            dq = self.profile.ordered(names, velocity)
            self.guard.update_feedback(self.profile.names, q, received)
            self._q_joint, self._dq_joint = q.copy(), dq.copy()
            self._joint_at = received
            self._feedback_delivery_count += 1
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self.last_error = str(exc)
            return False

    def update_target(self, model_q, received_at):
        """Cache a model-order target with its original host receipt timestamp."""
        if self._closed:
            return False
        try:
            received_at = self._number(received_at, "target receipt")
            if received_at < 0:
                raise ValueError("Target receipt must be nonnegative")
            if self._target_at is not None and received_at <= self._target_at:
                return False
            target = self.profile.model_to_hardware(model_q)
            if self.enabled and self._target_at is not None and received_at - self._target_at > self.profile.command.target_timeout_s:
                self._latch("target_gap")
            self._target, self._target_at = target.copy(), received_at
            self._target_valid = True
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._target_valid = False
            self._latch("invalid_target", exc)
            return False

    @staticmethod
    def _age(now, timestamp):
        return None if timestamp is None else now - timestamp

    def _requirements(self, now):
        if self._closed:
            return "shutdown"
        if self._q_joint is None:
            return "waiting_feedback"
        if not self._target_valid or self._target_at is None:
            return "waiting_target"
        if not 0 <= now - self._target_at <= self.profile.command.target_timeout_s:
            return "stale_target"
        return None

    def arm(self, now):
        """Arm output; direct mode waits for the next accepted target."""
        try:
            now = self._number(now, "now")
            blocked = self._requirements(now)
            if blocked:
                self._latch(blocked)
                return False
            if self.enabled:
                return True
            if self.interpolate:
                guard = self._new_guard()
                guard.update_feedback(self.profile.names, self._q_joint, self._joint_at)
                guard.set_enabled(True)
                guard.update_target(self.profile.names, self._target, now, self._target_at,
                                    source_age=now - self._target_at)
                self.guard = guard
            self._direct_command = self._direct_target_at = None
            self._applied_target_at = self._target_at
            self._last_sample = now
            self.enabled, self.reason, self.last_error = True, None, None
            return True
        except (ValueError, TypeError) as exc:
            self._latch("arming_error", exc)
            return False

    def check_watchdog(self, now):
        """Check target expiry without consuming a target or generating output."""
        if not self.enabled or self._closed:
            return False
        try:
            now = self._number(now, "now")
            blocked = self._requirements(now)
            if blocked:
                self._latch(blocked)
                return False
            return True
        except (ValueError, TypeError) as exc:
            self._latch("command_error", exc)
            return False

    def sample(self, now):
        """Return a complete hardware-order reference, or None to publish nothing."""
        if not self.enabled or self._closed:
            return None
        try:
            now = self._number(now, "now")
            blocked = self._requirements(now)
            if blocked:
                self._latch(blocked)
                return None
            if self.interpolate:
                interval = now - self._last_sample
                if interval < 0 or interval > self.profile.command.max_timer_gap_s:
                    self._latch("timer_gap")
                    return None
                if self._target_at != self._applied_target_at:
                    self.guard.update_target(self.profile.names, self._target, now, self._target_at,
                                             source_age=now - self._target_at)
                    self._applied_target_at = self._target_at
                result = self.guard.sample(now)
                if result is None:
                    self._latch("interpolator_stopped", self.guard.state)
                    return None
                q, dq = result
            else:
                if self._target_at == self._applied_target_at:
                    return None
                q = self._target.copy()
                dq = np.zeros_like(q)
                if self._direct_command is not None and self.profile.command.velocity_feedforward:
                    dt = self._target_at - self._direct_target_at
                    dq = np.clip((q - self._direct_command) / dt,
                                 -self.profile.command.max_velocity_rad_s,
                                 self.profile.command.max_velocity_rad_s)
                self._direct_command, self._direct_target_at = q.copy(), self._target_at
                self._applied_target_at = self._target_at
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
                raise ValueError("Non-finite command output")
            self._last_sample = now
            self._last_command, self._last_velocity = q.copy(), dq.copy()
            self.commands_generated += 1
            return q, dq
        except (ValueError, TypeError) as exc:
            self._latch("command_error", exc)
            return None

    def status(self, now):
        now = self._number(now, "now")
        blocked = self._requirements(now)
        if self._closed:
            state = "shutdown"
        elif self.enabled:
            state = "ramping" if self.interpolate and self.guard.state == "RAMPING" else "active"
        elif self.reason == "startup":
            state = "ready" if blocked is None else "waiting"
        elif self.reason in ("paused", "paused by user", "operator_pause", "recalibration",
                             "operator_exit", "exit"):
            state = "paused"
        else:
            state = "fault"
        projection = None if self._q_joint is None else np.clip(
            self._q_joint, self.profile.lower, self.profile.upper
        ) - self._q_joint
        return dict(
            state=state, enabled=self.enabled, reason=self.reason, last_error=self.last_error,
            interpolate=self.interpolate,
            output_mode="interpolated" if self.interpolate else "direct",
            ready_to_arm=blocked is None, arming_blocked_by=blocked,
            feedback_model_q=None if self._q_joint is None else self.profile.feedback_to_model(self._q_joint).tolist(),
            feedback_velocity_hardware=None if self._dq_joint is None else self._dq_joint.tolist(),
            feedback_age_s=self._age(now, self._joint_at),
            feedback_source="joint_state",
            feedback_delivery_count=self._feedback_delivery_count,
            target_age_s=self._age(now, self._target_at),
            startup_projection_rad=None if projection is None else projection.tolist(),
            command_q=None if self._last_command is None else self._last_command.tolist(),
            command_dq=None if self._last_velocity is None else self._last_velocity.tolist(),
            commands_generated=self.commands_generated,
            stop_behavior="Publication stops; downstream controller timeout holds position.",
        )
