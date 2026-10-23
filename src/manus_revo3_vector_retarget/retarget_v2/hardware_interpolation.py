# Hardware command interpolation copied with user authorization from the existing
# manus-revo3-retargeting repository, 2026-10-08:
# src/capabilities/manus_revo3_retarget/manus_revo3_vector_retarget/teleoperation/interpolation.py
# Feedback supplies a startup position; hardware sampling watchdogs are omitted.
# This is the known hardware output
# interpolation, not the independent solver or the MuJoCo MotionGuard.
# Its hard-bound fallback can override acceleration bounds; do not claim the
# analytic MotionGuard invariant for this compatibility layer.
"""High-rate command generation; all times are monotonic seconds.

Continuous target interpolation with measured-state startup, target-distance
braking, bounded command changes, and a glove-target watchdog.
"""
import numpy as np

from .hardware_profile import CommandConfig


class CommandInterpolator:
    def __init__(self, names, lower, upper, config: CommandConfig):
        self.names = tuple(names)
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.config = config
        self.feedback = self.feedback_at = self.feedback_raw = None
        self.enabled = False
        self.last_source_stamp = -np.inf
        self.state = 'DISABLED'
        self.reset_motion()

    def reset_motion(self):
        self.position = self.velocity = self.target = None
        self.target_at = self.target_received_at = self.sample_at = self.startup_until = None
        self.segment_start = self.segment_at = self.segment_duration = None

    def set_enabled(self, value):
        self.enabled = bool(value)
        self.reset_motion()
        self.state = 'WAITING TARGET/FEEDBACK' if value else 'DISABLED'

    def ordered(self, names, values):
        names = tuple(names)
        a = np.asarray(values, dtype=float)
        if len(names) != len(self.names) or len(set(names)) != len(names) or set(names) != set(self.names):
            raise ValueError('Require exactly the configured 21 unique joint names')
        if a.shape != (len(names),) or not np.isfinite(a).all():
            raise ValueError('Joint values must be finite and match joint names')
        indices = {n: i for i,n in enumerate(names)}
        return a[[indices[n] for n in self.names]]

    def update_feedback(self, names, positions, now):
        q = self.ordered(names, positions)
        if not np.isfinite(now):
            raise ValueError('Feedback time must be finite')
        # Only the startup/recovery seed is projected into command bounds;
        # retain the original measurement for the caller's display.
        self.feedback_raw = q.copy()
        self.feedback, self.feedback_at = np.clip(q, self.lower, self.upper), float(now)

    def update_target(self, names, positions, now, source_stamp, source_age=0.):
        q = self.ordered(names, positions)
        if np.any(q < self.lower-1e-8) or np.any(q > self.upper+1e-8):
            raise ValueError('Target exceeds modeled joint limits')
        if not np.isfinite([now, source_stamp]).all() or source_stamp <= self.last_source_stamp:
            raise ValueError('Target source stamp must be finite and strictly increasing')
        self.last_source_stamp = source_stamp
        if not self.enabled: return False
        if self.feedback is None:
            self.reset_motion()
            self.state = 'WAITING FEEDBACK'
            return False
        stale = self.target_at is None or now-self.target_at > self.config.target_timeout_s
        if self.position is None or stale:
            self.position = self.feedback.copy()
            self.velocity = np.zeros_like(q)
            self.sample_at = now
            self.startup_until = now+self.config.startup_duration_s
            reference = self.position.copy()
        else:
            # Keep interpolation continuous independently of the limited command.
            # Starting every segment at self.position repeatedly drags a lagging
            # command back into the reference and makes latency depend on rate.
            reference = self.reference(now)
        interval = 1/30 if stale else now-self.target_received_at
        remaining_startup = self.startup_until-now
        # Repeated targets must preserve the original startup deadline.
        self.segment_duration = (remaining_startup if remaining_startup > 0
                                 else max(interval, 1/self.config.publish_hz))
        self.segment_start, self.segment_at = reference, now
        self.target = np.clip(q, self.lower, self.upper)
        self.target_at, self.target_received_at = now-source_age, now
        self.state = 'RAMPING' if now < self.startup_until else 'LIVE'
        return True

    def reference(self, now):
        """Unconstrained interpolation point, separate from commanded position."""
        alpha = np.clip((now-self.segment_at)/self.segment_duration, 0., 1.)
        return self.segment_start+alpha*(self.target-self.segment_start)

    @staticmethod
    def braking_speed(distance, acceleration, dt):
        """Limit next-step speed so remaining distance covers stopping distance.

        distance >= v*dt + v**2/(2*a). The first term reserves this command
        sample's travel; using sqrt(2*a*distance) alone misses that travel.
        Rationalized form avoids cancellation close to a stationary target.
        """
        twice_ad = 2*acceleration*np.maximum(distance, 0.)
        adt = acceleration*dt
        return twice_ad/(np.sqrt(adt*adt+twice_ad)+adt)

    def sample(self, now):
        if not self.enabled: return None
        if self.feedback is None:
            self.reset_motion()
            self.state = 'WAITING FEEDBACK'
            return None
        if self.target_at is None: return None
        if now-self.target_at > self.config.target_timeout_s:
            self.reset_motion(); self.state = 'STALE TARGET'
            return None
        dt = now-self.sample_at
        if dt < 0 or dt > self.config.max_timer_gap_s:
            self.reset_motion(); self.state = 'TIMER GAP - WAITING TARGET'
            return None
        if dt > 0:
            desired = self.reference(now)
            v = np.clip((desired-self.position)/dt, -self.config.max_velocity_rad_s, self.config.max_velocity_rad_s)
            a = self.config.max_acceleration_rad_s2
            # Brake for the actual target as well as joint bounds. A suddenly
            # reversing target may be inside the current stopping distance;
            # preserve the acceleration limit while stopping and reversing.
            forward = np.minimum(np.maximum(self.target-self.position, 0.), self.upper-self.position)
            backward = np.minimum(np.maximum(self.position-self.target, 0.), self.position-self.lower)
            v = np.clip(v, -self.braking_speed(backward, a, dt), self.braking_speed(forward, a, dt))
            v = np.clip(v, self.velocity-a*dt, self.velocity+a*dt)
            # Hard bounds remain a last resort if an infeasible state reaches
            # this layer; clipping there can override the acceleration bound.
            q = np.clip(self.position+v*dt, self.lower, self.upper)
            self.velocity = (q-self.position)/dt
            self.position = q
            self.sample_at = now
        self.state = 'RAMPING' if now < self.startup_until else 'LIVE'
        velocity = self.velocity.copy() if self.config.velocity_feedforward else np.zeros_like(self.velocity)
        return self.position.copy(), velocity
