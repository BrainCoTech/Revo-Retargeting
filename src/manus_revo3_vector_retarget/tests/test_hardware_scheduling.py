"""Exercise the real hardware loop with deterministic arrivals, no ROS or devices."""
from contextlib import nullcontext
from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from retarget_v2 import hardware as hardware_module
from retarget_v2.ros_input import LatestSample


@dataclass
class Arrival:
    at: float
    sequence: int | None = None
    received_at: float | None = None
    recalibrate: bool = False
    thread_error: str | None = None


class FakeSource:
    """Advance virtual time to arrivals; retain only the newest delivered frame."""

    def __init__(self, runtime):
        self.runtime = runtime
        self.arrivals = []
        self.sample = None
        self.thread_error = None
        self.waits = []

    def _deliver(self):
        while self.arrivals and self.arrivals[0].at <= self.runtime.now:
            arrival = self.arrivals.pop(0)
            if arrival.sequence is not None:
                self.sample = LatestSample(arrival.sequence,
                    arrival.at if arrival.received_at is None else arrival.received_at,
                    {"sequence": arrival.sequence})
            if arrival.recalibrate:
                self.runtime.transport.request_recalibration()
            if arrival.thread_error is not None:
                self.thread_error = arrival.thread_error

    def latest(self, after_sequence=None):
        self._deliver()
        if self.sample is None or (after_sequence is not None and self.sample.sequence <= after_sequence):
            return None
        return deepcopy(self.sample)

    def wait_for_sample(self, after_sequence=None, timeout=None):
        assert timeout is not None and 0 < timeout <= .05
        self.waits.append((self.runtime.now, after_sequence, timeout))
        deadline = self.runtime.now + timeout
        while True:
            sample = self.latest(after_sequence)
            if sample is not None:
                return sample
            if self.runtime.now >= deadline:
                return None
            next_event = self.arrivals[0].at if self.arrivals else float("inf")
            wake_at = min(deadline, next_event)
            if wake_at >= self.runtime.stop_at:
                self.runtime.now = self.runtime.stop_at
                raise KeyboardInterrupt
            self.runtime.now = wake_at

    def status(self):
        return dict(thread_error=self.thread_error, last_error=None)


class FakeTransport:
    def __init__(self, runtime):
        self.runtime = runtime
        self.generation = 0
        self.pending = None
        # This double starts enabled so watchdog tests can observe a latch.
        # Actual startup/enable semantics are covered in test_ros_hardware.py.
        self.enabled = True
        self.targets = []
        self.attempts = []
        self.pauses = []
        self.statuses = []
        self.front = dict(phase="WAITING_INPUT", reason="Waiting for glove input")

    def request_recalibration(self):
        self.generation += 1
        self.pending = self.generation
        self.pause("recalibration")

    def take_recalibration_request(self):
        request, self.pending = self.pending, None
        return request

    def set_target(self, q, received_at, generation):
        row = dict(at=self.runtime.now, sequence=int(q[0]), received_at=received_at,
                   generation=generation, enabled=self.enabled)
        self.attempts.append(row)
        if generation != self.generation:
            return False
        self.targets.append(row)
        return True

    def pause(self, reason):
        self.enabled = False
        self.pauses.append((self.runtime.now, reason))

    def update_retarget_status(self, front, generation):
        if generation != self.generation:
            return False
        self.front = deepcopy(front)
        self.statuses.append((self.runtime.now, generation, deepcopy(front)))
        return True

    def snapshot(self):
        return dict(self.front, last_error=None, thread_error=None)


class FakeCalibration:
    def __init__(self, runtime):
        self.runtime = runtime
        self.count = 0
        self.ready = False
        self.resets = []
        self.clears = []

    def advance(self, message):
        self.count += 1
        self.ready = self.count >= self.runtime.calibration_frames
        return object() if self.ready else None

    def reset(self, force=True):
        assert force
        self.count, self.ready = 0, False
        self.resets.append(self.runtime.now)

    def clear_pending(self):
        self.clears.append(self.runtime.now)
        if not self.ready:
            self.count = 0

    def status(self):
        return dict(state="ready" if self.ready else "calibrating", count=self.count,
                    required_frames=self.runtime.calibration_frames, glove_id=123,
                    cache_used=False, cache_path=None, cache_error=None)


class Runtime:
    def __init__(self, monkeypatch):
        self.now = 1.
        self.stop_at = 1.1
        self.calibration_frames = 1
        self.solve_durations = {}
        self.during_solve = None
        self.solves = []
        self.solvers = []
        self.source = FakeSource(self)
        self.transport = FakeTransport(self)
        self.calibration = FakeCalibration(self)
        profile = SimpleNamespace(model_lower=np.zeros(21), model_upper=np.ones(21),
            model_to_hardware=lambda q: q.copy(), command=SimpleNamespace(
                max_velocity_rad_s=12., max_acceleration_rad_s2=24.,
                max_timer_gap_s=.1, target_timeout_s=.25))

        def sleep(_):
            pytest.fail("The hardware loop must wait for new input, not poll with sleep")

        def solver_factory(*args, **kwargs):
            instance = len(self.solvers)
            self.solvers.append(instance)

            def solve(message):
                sequence = message["sequence"]
                self.solves.append(dict(at=self.now, sequence=sequence, solver=instance))
                duration = self.solve_durations.get(sequence, .002)
                self.now += duration
                if self.during_solve is not None:
                    self.during_solve(sequence)
                return np.full(21, sequence, dtype=float), dict(accepted=True, solve_ms=duration * 1000)

            return SimpleNamespace(solve=solve)

        monkeypatch.setattr(hardware_module, "time", SimpleNamespace(
            monotonic=lambda: self.now, perf_counter=lambda: self.now, sleep=sleep))
        monkeypatch.setattr(hardware_module, "RobotModel", lambda **kwargs: nullcontext(object()))
        monkeypatch.setattr(hardware_module, "HardwareProfile", lambda *args, **kwargs: profile)
        monkeypatch.setattr(hardware_module, "HardwareTransport", lambda *args, **kwargs: nullcontext(self.transport))
        monkeypatch.setattr(hardware_module, "ManusSubscriber", lambda **kwargs: nullcontext(self.source))
        monkeypatch.setattr(hardware_module, "CalibrationSession", lambda **kwargs: self.calibration)
        monkeypatch.setattr(hardware_module, "Retargeter", solver_factory)

    def run(self, arrivals, stop_at, hand_side="right"):
        self.source.arrivals = list(arrivals)
        self.stop_at = stop_at
        args = SimpleNamespace(model="may3", hand_side=hand_side, namespace=None,
            domain=197, profile=None, motion_scale=None, glove_id=None,
            calibration_frames=120, normal_sign=-1., cache_dir=None)
        hardware_module.hardware(args)


@pytest.fixture
def runtime(monkeypatch):
    return Runtime(monkeypatch)


@pytest.mark.parametrize("hand_side", ["left", "right"])
def test_each_fresh_frame_starts_without_a_second_rate_limit(runtime, hand_side):
    arrivals = [Arrival(1. + index * .01, index + 1) for index in range(5)]
    runtime.run(arrivals, stop_at=1.09, hand_side=hand_side)
    assert [row["sequence"] for row in runtime.solves] == [1, 2, 3, 4, 5]
    assert [row["at"] for row in runtime.solves] == pytest.approx([row.at for row in arrivals])
    assert len(runtime.transport.targets) == 5


def test_idle_timeout_never_resolves_a_cached_frame_and_preserves_receipt_time(runtime):
    runtime.run([Arrival(1., 1)], stop_at=1.23)
    assert len(runtime.source.waits) >= 5
    assert len(runtime.solves) == len(runtime.transport.targets) == 1
    assert runtime.transport.targets[0]["received_at"] == 1.
    final = runtime.transport.statuses[-1][2]
    assert final["valid_frames"] == 1
    assert final["solve"]["input_to_solve_ms"] == pytest.approx(0.)
    assert final["solve"]["total_solve_ms"] == pytest.approx(2.)
    assert final["solve"]["input_to_target_ms"] == pytest.approx(2.)
    assert [reason for _, reason in runtime.transport.pauses] == ["exit"]


def test_idle_wait_still_detects_stale_input_without_reporting_solver_stall(runtime):
    runtime.run([Arrival(1., 1)], stop_at=1.38)
    stale = [at for at, reason in runtime.transport.pauses if reason == "stale_manus"]
    assert stale and 1.25 < stale[0] <= 1.302
    assert "solver_loop_stall" not in [reason for _, reason in runtime.transport.pauses]
    assert runtime.transport.statuses[-1][2]["phase"] == "WAITING_INPUT"
    assert len(runtime.solves) == 1


def test_already_stale_arrival_is_not_sent_to_solver(runtime):
    runtime.run([Arrival(1., 1, received_at=.5)], stop_at=1.03)
    assert not runtime.solves and not runtime.transport.targets
    assert "invalid_manus_or_target" in [reason for _, reason in runtime.transport.pauses]


def test_frames_arriving_during_a_solve_are_coalesced_to_the_newest(runtime):
    runtime.solve_durations[1] = .035
    runtime.run([Arrival(1., 1), Arrival(1.01, 2), Arrival(1.02, 3),
                 Arrival(1.03, 4), Arrival(1.04, 5)], stop_at=1.09)
    assert [row["sequence"] for row in runtime.solves] == [1, 4, 5]
    assert [row["at"] for row in runtime.solves] == pytest.approx([1., 1.035, 1.04])
    assert [row["received_at"] for row in runtime.transport.targets] == [1., 1.03, 1.04]


def test_recalibration_during_wait_discards_returned_frame_before_old_solver_runs(runtime):
    runtime.run([Arrival(1., 1), Arrival(1.01, 2, recalibrate=True),
                 Arrival(1.02, 3)], stop_at=1.08)
    assert [row["sequence"] for row in runtime.solves] == [1, 3]
    assert [row["solver"] for row in runtime.solves] == [0, 1]
    assert [row["generation"] for row in runtime.transport.targets] == [0, 1]
    assert runtime.calibration.resets == [1.01]
    assert not runtime.transport.targets[-1]["enabled"]


def test_recalibration_during_solve_cannot_apply_the_obsolete_result(runtime):
    runtime.during_solve = lambda sequence: (
        runtime.transport.request_recalibration() if sequence == 1 else None)
    runtime.run([Arrival(1., 1), Arrival(1.02, 2)], stop_at=1.08)
    assert [row["sequence"] for row in runtime.transport.attempts] == [1, 2]
    assert [row["sequence"] for row in runtime.transport.targets] == [2]
    assert runtime.transport.targets[0]["generation"] == 1
    assert not runtime.transport.targets[0]["enabled"]
    assert [row["solver"] for row in runtime.solves] == [0, 1]


def test_slow_solve_latches_pause_before_the_result_reaches_command_transport(runtime):
    runtime.solve_durations[1] = .12
    runtime.run([Arrival(1., 1), Arrival(1.14, 2)], stop_at=1.20)
    stalled = [at for at, reason in runtime.transport.pauses if reason == "solver_loop_stall"]
    assert stalled and stalled[0] == pytest.approx(1.12)
    assert len(runtime.transport.targets) == 2
    assert all(not row["enabled"] for row in runtime.transport.targets)


def test_input_gap_clears_partial_calibration_while_idle(runtime):
    runtime.calibration_frames = 3
    runtime.run([Arrival(1., 1), Arrival(1.01, 2), Arrival(1.4, 3),
                 Arrival(1.41, 4), Arrival(1.42, 5)], stop_at=1.48)
    assert runtime.calibration.clears
    assert [row["sequence"] for row in runtime.solves] == [5]
    assert runtime.transport.statuses[-1][2]["calibration_count"] == 3
    assert "solver_loop_stall" not in [reason for _, reason in runtime.transport.pauses]


def test_receiver_failure_is_reported_even_when_no_frame_arrives(runtime):
    with pytest.raises(RuntimeError, match="executor failed"):
        runtime.run([Arrival(1.08, thread_error="executor failed")], stop_at=1.3)
    assert runtime.now <= 1.13
    assert [reason for _, reason in runtime.transport.pauses] == ["thread_error", "exit"]
    assert not runtime.solves
