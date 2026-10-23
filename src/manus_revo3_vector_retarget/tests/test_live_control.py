"""Live controller state/timing contracts using synthetic SDK input.

These tests exercise the real new motion guard and MuJoCo servo. They do not
connect ROS, read user recordings, or send any hardware commands.
"""

from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.live_control import LiveController
from retarget_v2.model import RobotModel
from test_input import sdk_hand, transformed


@pytest.fixture(scope="module")
def robot():
    with RobotModel() as model:
        yield model


def ready_controller(robot):
    live = LiveController(robot, calibration_frames=30, timeout=.25, normal_sign=-1)
    frame = sdk_hand(pip_bend_degrees=30.)
    for sequence in range(1, 32):
        now = sequence * .01
        live.ingest(deepcopy(frame), received_at=now, sequence=sequence, now=now)
    assert live.solver is not None
    assert live.status(now)["state"] == "ready"
    return live, frame, now, sequence


def moving_controller(robot):
    live, frame, now, sequence = ready_controller(robot)
    assert live.resume(now)
    for _ in range(12):
        now += .01
        sequence += 1
        live.ingest(deepcopy(frame), received_at=now, sequence=sequence, now=now)
        live.tick(.01, now)
    assert np.max(np.abs(live.guard.state.dq)) > .05
    return live, frame, now, sequence


def settle_stopped(live, now, maximum_steps=200):
    previous = live.guard.state
    for _ in range(maximum_steps):
        now += .01
        q, dq = live.tick(.01, now)
        # Stopping must preserve continuity and not teleport qpos or command.
        assert np.max(np.abs(dq - previous.dq)) <= 8 * .01 + 1e-9
        assert np.max(np.abs(q - previous.q)) <= 2 * .01 + 1e-9
        assert np.all(np.abs(dq) <= np.abs(previous.dq) + 1e-9)
        previous = live.guard.state
        if np.max(np.abs(dq)) == 0:
            return now
    raise AssertionError("Controller did not finish its bounded braking trajectory")


def test_initial_calibration_is_paused_until_explicit_resume(robot):
    live, _, now, _ = ready_controller(robot)
    stationary = live.guard.state.q
    for _ in range(5):
        now += .01
        q, dq = live.tick(.01, now)
        np.testing.assert_array_equal(q, stationary)
        np.testing.assert_array_equal(dq, np.zeros(21))
    assert live.status(now)["state"] == "ready"
    assert live.resume(now) is True
    now += .01
    q, dq = live.tick(.01, now)
    assert np.max(np.abs(dq)) > 0
    assert np.max(np.abs(q - stationary)) > 0


@pytest.mark.parametrize("model_version", ["may3"])
def test_zero_reference_does_not_prevent_initial_or_recalibrated_ik_convergence(model_version):
    # The official model permits negative angles at most joints. Overwriting
    # the optimizer seed with the held zero command used to shrink TRF's first
    # trust region enough to falsely report convergence at essentially zero.
    with RobotModel(model_version=model_version) as model:
        live = LiveController(model, calibration_frames=30)
        try:
            frame = sdk_hand(pip_bend_degrees=30.)
            stationary = live.guard.state.q.copy()
            measured = live.simulation.measured_q.copy()
            sequence = 0
            for recalibrate in (False, True):
                if recalibrate:
                    live.request_calibration()
                for _ in range(30):
                    sequence += 1
                    now = sequence * .01
                    assert live.ingest(deepcopy(frame), now, sequence, now)
                assert live.status(now)["state"] == "ready"
                assert live.solve_status["accepted"]
                target = live.solver.last
                before, _ = live.solver.evaluate(stationary, target, stationary)
                after, _ = live.solver.evaluate(live._target, target, stationary)
                assert np.isfinite(live._target).all()
                assert np.linalg.norm(live._target - stationary) > .25
                assert np.dot(after, after) < .2 * np.dot(before, before)
                # Solving is not commanding: calibration changes neither the
                # stopped trajectory nor measured simulation state.
                np.testing.assert_array_equal(live.guard.state.q, stationary)
                np.testing.assert_array_equal(live.guard.state.dq, np.zeros(21))
                np.testing.assert_array_equal(live._reference, stationary)
                np.testing.assert_array_equal(live.simulation.measured_q, measured)
        finally:
            live.close()


def test_tick_applies_previous_reference_before_generating_next(robot, monkeypatch):
    live, _, now, _ = ready_controller(robot)
    assert live.resume(now)
    applied = []
    original = live.simulation.step

    def record_and_integrate(reference, dt):
        applied.append((np.asarray(reference).copy(), dt))
        return original(reference, dt)

    monkeypatch.setattr(live.simulation, "step", record_and_integrate)
    previous_reference = live.guard.state.q
    for _ in range(3):
        now += .01
        current_reference, _ = live.tick(.01, now)
        assert len(applied) > 0
        np.testing.assert_array_equal(applied[-1][0], previous_reference)
        assert applied[-1][1] == pytest.approx(.01)
        previous_reference = current_reference.copy()
    assert np.max(np.abs(applied[0][0] - current_reference)) > 0


@pytest.mark.parametrize("sequence_delta", [0, -1])
def test_duplicate_or_out_of_order_sequence_does_not_refresh_input(robot, sequence_delta):
    live, frame, now, sequence = ready_controller(robot)
    original = live.status(now)
    live.ingest(deepcopy(frame), received_at=now + .20,
                sequence=sequence + sequence_delta, now=now + .20)
    status = live.status(now + .20)
    assert status["last_sequence"] == original["last_sequence"]
    assert status["valid_frames"] == original["valid_frames"]
    assert status["input_age_s"] == pytest.approx(.20)
    live.tick(.01, now + .30)
    assert live.resume(now + .30) is False


def test_delayed_delivery_uses_receipt_time_instead_of_ingest_time(robot):
    live, frame, now, sequence = ready_controller(robot)
    live.ingest(deepcopy(frame), received_at=now + .01,
                sequence=sequence + 1, now=now + .40)
    assert live.resume(now + .40) is False
    q_before = live.guard.state.q
    q, dq = live.tick(.01, now + .41)
    np.testing.assert_array_equal(q, q_before)
    np.testing.assert_array_equal(dq, np.zeros(21))


def test_timeout_brakes_and_fresh_recovery_does_not_automatically_restart(robot):
    live, frame, now, sequence = moving_controller(robot)
    last_receipt = now
    # Wall time has advanced past freshness, but the control integration step
    # remains short; this distinguishes source loss from loop-stall handling.
    now = last_receipt + .26
    previous = live.guard.state.dq
    _, dq = live.tick(.01, now)
    assert np.all(np.abs(dq) <= np.abs(previous) + 1e-12)
    assert live.status(now)["state"] in ("braking", "hold")
    assert live.resume(now) is False
    now = settle_stopped(live, now)
    stopped = live.guard.state.q
    now += .01
    sequence += 1
    live.ingest(deepcopy(frame), received_at=now, sequence=sequence, now=now)
    q, dq = live.tick(.01, now)
    np.testing.assert_array_equal(q, stopped)
    np.testing.assert_array_equal(dq, np.zeros(21))
    assert live.resume(now) is True
    q, dq = live.tick(.01, now + .01)
    assert np.max(np.abs(dq)) <= 8 * .01 + 1e-9
    assert np.max(np.abs(q - stopped)) <= .5 * 8 * .01**2 + 1e-9


def test_packet_gap_latches_stop_even_if_a_packet_arrives_before_tick(robot):
    live, frame, now, sequence = moving_controller(robot)
    previous = live.guard.state.dq
    now += .26
    live.ingest(deepcopy(frame), received_at=now, sequence=sequence + 1, now=now)
    assert live.status(now)["input_age_s"] == 0
    _, dq = live.tick(.01, now)
    assert np.all(np.abs(dq) <= np.abs(previous) + 1e-12)
    assert live.status(now)["state"] in ("braking", "hold")
    assert live.resume(now) is False


def test_pause_cannot_be_cancelled_while_still_moving(robot):
    live, frame, now, sequence = moving_controller(robot)
    live.pause()
    # pause() has not yet had a tick to change MotionGuard.mode, so checking
    # only the mode instead of actual reference velocity would be incorrect.
    assert live.resume(now) is False
    previous_q = live.guard.state.q
    now = settle_stopped(live, now)
    assert np.max(np.abs(live.guard.state.q - previous_q)) > 0
    now += .01
    live.ingest(deepcopy(frame), received_at=now, sequence=sequence + 1, now=now)
    assert live.resume(now) is True


@pytest.mark.parametrize("mutation", [
    lambda m: m.update(side="Left"),
    lambda m: m.update(glove_id=m["glove_id"] + 1),
    lambda m: m["raw_nodes"][4]["pose"]["position"].update(x=np.nan),
])
def test_bad_frame_or_identity_change_stops_without_refreshing_good_input(robot, mutation):
    live, frame, now, sequence = moving_controller(robot)
    original = live.status(now)
    bad = deepcopy(frame)
    mutation(bad)
    live.ingest(bad, received_at=now + .01, sequence=sequence + 1, now=now + .01)
    status = live.status(now + .01)
    assert status["valid_frames"] == original["valid_frames"]
    assert status["input_age_s"] == pytest.approx(.01)
    assert status["stop_reason"] == "invalid_input"
    previous = live.guard.state.dq
    _, dq = live.tick(.01, now + .01)
    assert np.all(np.abs(dq) <= np.abs(previous) + 1e-12)
    assert live.resume(now + .01) is False


def test_bone_length_jump_and_future_timestamp_cannot_refresh_freshness(robot):
    live, frame, now, sequence = ready_controller(robot)
    valid_count = live.status(now)["valid_frames"]
    bad = transformed(frame, np.eye(3) * 1.3, np.zeros(3))
    live.ingest(bad, received_at=now + .01, sequence=sequence + 1, now=now + .01)
    assert live.status(now + .01)["valid_frames"] == valid_count
    assert live.status(now + .01)["stop_reason"] == "invalid_input"
    live.ingest(deepcopy(frame), received_at=now + 1,
                sequence=sequence + 2, now=now + .02)
    assert live.status(now + .02)["input_age_s"] == pytest.approx(.02)
    assert live.status(now + .02)["valid_frames"] == valid_count


def test_invalid_frame_at_rest_needs_a_good_frame_before_resume(robot):
    live, frame, now, sequence = ready_controller(robot)
    bad = deepcopy(frame)
    bad["side"] = "Left"
    now += .01
    live.ingest(bad, received_at=now, sequence=sequence + 1, now=now)
    assert np.max(np.abs(live.guard.state.dq)) == 0
    assert live.resume(now) is False
    now += .01
    live.ingest(deepcopy(frame), received_at=now, sequence=sequence + 2, now=now)
    assert live.resume(now) is True


def test_loop_stall_drops_wall_time_and_latches_stop(robot):
    live, frame, now, sequence = moving_controller(robot)
    before = live.guard.state
    simulated_before = live.simulation.data.time
    now += .5
    # Even a freshly received frame cannot conceal a missed control deadline.
    live.ingest(deepcopy(frame), received_at=now, sequence=sequence + 1, now=now)
    q, dq = live.tick(.5, now)
    assert live.simulation.data.time - simulated_before <= .020000001
    assert np.max(np.abs(q - before.q)) <= 2 * .02 + 1e-9
    assert np.max(np.abs(dq - before.dq)) <= 8 * .02 + 1e-9
    assert np.all(np.abs(dq) <= np.abs(before.dq) + 1e-12)
    assert live.resume(now) is False
    assert live.status(now)["state"] in ("braking", "hold")


def test_recalibration_does_not_reset_a_moving_reference_or_auto_resume(robot):
    live, frame, now, sequence = moving_controller(robot)
    previous = live.guard.state
    live.request_calibration()
    np.testing.assert_array_equal(live.guard.state.q, previous.q)
    np.testing.assert_array_equal(live.guard.state.dq, previous.dq)
    assert live.resume(now) is False
    # Frames seen during braking must not populate a new calibration window.
    now += .001
    sequence += 1
    live.ingest(deepcopy(frame), received_at=now, sequence=sequence, now=now)
    assert live.status(now)["calibration_count"] == 0
    now = settle_stopped(live, now)
    stopped = live.guard.state.q
    for _ in range(31):
        now += .01
        sequence += 1
        live.ingest(deepcopy(frame), received_at=now, sequence=sequence, now=now)
        q, dq = live.tick(.01, now)
        np.testing.assert_array_equal(q, stopped)
        np.testing.assert_array_equal(dq, np.zeros(21))
    assert live.status(now)["state"] == "ready"
    assert live.resume(now) is True


def test_tick_rejects_repeated_and_backwards_monotonic_time(robot):
    live, _, now, _ = ready_controller(robot)
    live.tick(.01, now + .01)
    before = live.guard.state
    simulation_time = live.simulation.data.time
    for bad_now in (now + .01, now, np.nan, np.inf):
        with pytest.raises(ValueError):
            live.tick(.01, bad_now)
        np.testing.assert_array_equal(live.guard.state.q, before.q)
        assert live.simulation.data.time == simulation_time


@pytest.mark.parametrize("dt", [0, -1, np.nan, np.inf, True])
def test_invalid_tick_duration_cannot_advance_simulation(robot, dt):
    live, _, now, _ = ready_controller(robot)
    before = live.guard.state
    time_before = live.simulation.data.time
    with pytest.raises(ValueError):
        live.tick(dt, now + .01)
    np.testing.assert_array_equal(live.guard.state.q, before.q)
    np.testing.assert_array_equal(live.guard.state.dq, before.dq)
    assert live.simulation.data.time == time_before
