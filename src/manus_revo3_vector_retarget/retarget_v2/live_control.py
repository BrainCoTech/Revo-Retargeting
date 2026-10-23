"""ROS-independent live session state and causal simulated servo control."""
from __future__ import annotations

import math

import numpy as np

from .input import Calibration, chains, decode, normalize_hand_side, palm_coordinates
from .motion import MotionGuard
from .simulation import ServoSimulation
from .solver import Retargeter, SolverConfig


class LiveController:
    """One owner thread calls ingest/tick/controls; no hardware transport.

    Receipt time is a monotonic host timestamp, not a MANUS sensor timestamp.
    Calibration estimates bone lengths, not an artificial zero finger pose.
    A calibrated session starts in READY and needs resume(). All stops latch.
    """

    def __init__(self, robot, calibration_frames=120, timeout=.25, normal_sign=-1., config=None):
        if not isinstance(calibration_frames, int) or calibration_frames < 30:
            raise ValueError("calibration_frames must be an integer >=30")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        if normal_sign not in (-1., 1.):
            raise ValueError("normal_sign must be -1 or +1")
        self.robot, self.required_frames = robot, calibration_frames
        self.hand_side = normalize_hand_side(getattr(robot, "hand_side", "right"))
        self.timeout, self.normal_sign = timeout, normal_sign
        self.config = config or SolverConfig()
        initial = np.clip(np.zeros(len(robot.joint_names)), robot.lower, robot.upper)
        self.guard = MotionGuard(robot.lower, robot.upper, self.config.velocity_limit,
                                 self.config.acceleration_limit, initial_q=initial)
        self.simulation = ServoSimulation(robot)
        self.simulation.reset(initial)
        self.solver = None
        self._calibration = []
        self._target = initial.copy()
        self._reference = initial.copy()
        self._enabled = False
        self.stop_reason = "startup"
        self.last_error = None
        self.last_received = None
        self.last_sequence = -1
        self.glove_id = None
        self.valid_frames = self.solved_frames = self.rejected_frames = 0
        self.solve_status = None
        self.target_chain = None
        self.dropped_wall_s = 0.
        self.deadline_misses = 0
        self.max_command_velocity = self.max_command_acceleration = 0.
        self.last_tick = None

    def _fresh(self, now):
        return (self.last_received is not None
                and 0 <= now-self.last_received <= self.timeout)

    def stop(self, reason):
        self._enabled = False
        self.stop_reason = reason

    def pause(self):
        self.stop("paused")

    def request_calibration(self):
        """Brake without resetting simulated/measured or reference positions."""
        self.stop("recalibration")
        self._calibration.clear()
        self.solver = None
        self.target_chain = None
        self.last_received = None

    def resume(self, now):
        if (not math.isfinite(now) or self.solver is None or not self._fresh(now)
                or self.last_error is not None
                or self.guard.state.mode == "braking"
                or (not self._enabled and np.any(self.guard.state.dq != 0))):
            return False
        self.guard.rearm()
        self._enabled = True
        self.stop_reason = None
        return True

    def ingest(self, message, received_at, sequence, now):
        """Validate and solve a new frame; rejected frames never refresh age."""
        if sequence <= self.last_sequence:
            return False
        self.last_sequence = sequence
        try:
            if not all(math.isfinite(t) for t in (received_at, now)):
                raise ValueError("Non-finite receipt time")
            if received_at > now or now-received_at > self.timeout:
                raise ValueError("Received sample is future-dated or already stale")
            if self.last_received is not None and received_at <= self.last_received:
                raise ValueError("Receipt time did not increase")
            # Check a gap even if a new packet arrived before the next tick.
            if self.last_received is not None and received_at-self.last_received > self.timeout:
                self.stop("stale_input")
                if self.solver is None:
                    self._calibration.clear()
            points = decode(message, self.hand_side)
            local = chains(palm_coordinates(points, self.normal_sign))
            lengths = np.linalg.norm(np.diff(local, axis=1), axis=2)
            if np.any(lengths < .008) or np.any(lengths > .1):
                raise ValueError("Unexpected bone lengths; expected SDK common coordinates in metres")
            identity = message["glove_id"]
            if isinstance(identity, bool) or not isinstance(identity, int):
                raise ValueError("glove_id must be an integer")
            if self.glove_id is not None and identity != self.glove_id:
                raise ValueError("Glove identity changed")
            if self.solver is None:
                # A requested recalibration cannot cancel a moving stop.
                if np.any(self.guard.state.dq != 0):
                    return False
                self._calibration.append(message)
                if len(self._calibration) >= self.required_frames:
                    calibration = Calibration.from_frames(self._calibration, self.normal_sign,
                                                          self.hand_side)
                    self.solver = Retargeter(self.robot, calibration, self.config)
                    # Keep the optimizer's interior numerical seed. Seeding TRF
                    # from the zero paused reference can make its initial trust
                    # region tiny and trigger ftol before it moves meaningfully.
                    # Command continuity belongs to the unchanged MotionGuard;
                    # a fresh solve never resets its reference or enables motion.
                    self.stop("ready")
            if self.solver is not None:
                target, result = self.solver.solve(message)
                self.solve_status = result
                if not result["accepted"]:
                    raise ValueError("Optimizer rejected this frame")
                self._target = target
                root = self.robot.points(self.simulation.measured_q)[self.solver.chain_ids][:, :1]
                self.target_chain = np.concatenate((root, root+np.cumsum(
                    self.solver.last["directions_world"]*self.robot.finger_lengths[:, :, None], axis=1)), axis=1)
                self.solved_frames += 1
            self.glove_id = identity
            self.last_received = received_at
            self.valid_frames += 1
            self.last_error = None
            return True
        except (KeyError, TypeError, ValueError, FloatingPointError) as exc:
            self.rejected_frames += 1
            self.last_error = str(exc)
            self.stop("invalid_input")
            if self.solver is None:
                self._calibration.clear()
            return False

    def tick(self, dt, now):
        """Advance the previous command, then compute the next reference.

        No newly received reference is applied retroactively. If wall time jumps
        >100 ms, latch a stop and advance only 20 ms: report discarded wall time
        instead of fast-forwarding a supposedly live simulation.
        """
        if (isinstance(dt, (bool, np.bool_)) or np.ndim(dt) != 0
                or isinstance(now, (bool, np.bool_)) or np.ndim(now) != 0
                or not math.isfinite(dt) or dt <= 0 or not math.isfinite(now)):
            raise ValueError("tick requires finite dt>0 and monotonic now")
        if self.last_tick is not None and now <= self.last_tick:
            raise ValueError("tick time must increase")
        self.last_tick = now
        if dt > .1:
            self.deadline_misses += 1
            self.dropped_wall_s += dt-.02
            dt = .02
            self.stop("loop_stall")
        if not self._fresh(now):
            if self._enabled:
                self.stop("stale_input")
            if self.solver is None:
                self._calibration.clear()
        # Gravity and damping continue during pause and stream interruption.
        self.simulation.step(self._reference, dt)
        before = self.guard.state.dq
        self._reference, velocity = self.guard.step(
            self._target, dt, fresh=bool(self._enabled and self._fresh(now)))
        self.max_command_velocity = max(self.max_command_velocity, float(np.max(np.abs(velocity))))
        self.max_command_acceleration = max(self.max_command_acceleration,
                                            float(np.max(np.abs(velocity-before)))/dt)
        return self._reference.copy(), velocity

    def status(self, now):
        motion = self.guard.state
        if motion.mode == "braking":
            state = "braking"
        elif self.solver is None:
            state = "calibrating" if self._calibration else "waiting"
        elif self._enabled:
            state = "tracking"
        else:
            state = "ready" if self.stop_reason == "ready" else "hold"
        return dict(state=state, stop_reason=self.stop_reason, guard_mode=motion.mode,
                    glove_id=self.glove_id, hand_side=self.hand_side, last_sequence=self.last_sequence,
                    input_age_s=None if self.last_received is None else max(0., now-self.last_received),
                    fresh=self._fresh(now), calibration_count=len(self._calibration),
                    calibration_frames=self.required_frames, valid_frames=self.valid_frames,
                    solved_frames=self.solved_frames, rejected_frames=self.rejected_frames,
                    last_error=self.last_error, solve=self.solve_status,
                    command_q=motion.q.tolist(), command_dq=motion.dq.tolist(),
                    measured_q=self.simulation.measured_q.tolist(),
                    max_command_velocity_rad_s=self.max_command_velocity,
                    max_command_acceleration_rad_s2=self.max_command_acceleration,
                    deadline_misses=self.deadline_misses, dropped_wall_s=self.dropped_wall_s,
                    simulation_time_s=float(self.simulation.data.time))

    def close(self):
        self.simulation.close()
