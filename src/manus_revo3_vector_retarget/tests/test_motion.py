"""Checks on observable trajectory bounds, reversals and stale-stream stops."""

import numpy as np
import pytest

from retarget_v2.motion import MotionGuard


def assert_constraints(q, dq, previous_q, previous_dq, dt, lo, hi, vel, acc):
    eps = 2e-10
    assert np.all(q >= lo - eps)
    assert np.all(q <= hi + eps)
    assert np.all(np.abs(dq) <= vel + eps)
    assert np.all(np.abs(dq - previous_dq) <= acc * dt + eps)
    assert np.all(np.abs(q - previous_q) <= vel * dt + eps)
    stopping_point = q + dq * np.abs(dq) / (2 * acc)
    assert np.all(stopping_point >= lo - eps)
    assert np.all(stopping_point <= hi + eps)


def test_random_reversal_variable_steps_and_limits():
    rng = np.random.default_rng(721)
    lo = rng.uniform(-1.8, -0.2, 21)
    hi = rng.uniform(0.2, 1.8, 21)
    vel = rng.uniform(0.15, 3.0, 21)
    acc = rng.uniform(0.3, 12.0, 21)
    guard = MotionGuard(lo, hi, vel, acc)
    q, dq = guard.state.q, guard.state.dq
    for index in range(5000):
        # Includes requests far beyond a joint boundary, abrupt reversals,
        # tiny steps and steps longer than a full acceleration phase.
        target = rng.uniform(lo - 2, hi + 2)
        if index % 11 == 0:
            target = np.where(rng.random(21) > 0.5, lo, hi)
        dt = float(10 ** rng.uniform(-4, -0.25))
        previous_q, previous_dq = q, dq
        q, dq = guard.step(target, dt)
        assert_constraints(q, dq, previous_q, previous_dq, dt, lo, hi, vel, acc)


@pytest.mark.parametrize("target", [-1.0, 1.0])
def test_reaches_boundary_at_rest_without_hard_stop(target):
    guard = MotionGuard([-1], [1], 0.6, 0.8, initial_q=[0])
    q, dq = guard.state.q, guard.state.dq
    for _ in range(400):
        previous_q, previous_dq = q, dq
        q, dq = guard.step([target], 0.01)
        assert_constraints(q, dq, previous_q, previous_dq, 0.01, -1, 1, 0.6, 0.8)
    np.testing.assert_allclose(q, [target], atol=1e-12)
    np.testing.assert_array_equal(dq, [0])


def test_target_reversal_cannot_instantly_reverse_motion():
    guard = MotionGuard([-2], [2], 1, 2, initial_q=[0])
    q, dq = guard.step([2], 0.4)
    np.testing.assert_allclose(q, [0.16])
    np.testing.assert_allclose(dq, [0.8])
    reversed_q, reversed_dq = guard.step([-2], 0.1)
    assert reversed_q[0] > q[0]  # Old direction until it can physically stop.
    np.testing.assert_allclose(reversed_q, [0.23])
    np.testing.assert_allclose(reversed_dq, [0.6])


def test_stale_brakes_then_latches_hold_and_requires_rearm():
    guard = MotionGuard([-2], [2], 1, 2, initial_q=[0])
    q, dq = guard.step([2], 0.4)
    stopping_point = q + dq * np.abs(dq) / 4
    q, dq = guard.step(None, 0.05, fresh=False)
    assert guard.state.mode == "braking"
    with pytest.raises(RuntimeError, match="stopped"):
        guard.rearm()
    for _ in range(10):
        previous_q, previous_dq = q, dq
        # Fresh samples do not cancel a latched stop.
        q, dq = guard.step([-2], 0.05, fresh=True)
        assert_constraints(q, dq, previous_q, previous_dq, 0.05, -2, 2, 1, 2)
    np.testing.assert_allclose(q, stopping_point, atol=1e-12)
    np.testing.assert_array_equal(dq, [0])
    assert guard.state.mode == "hold"
    np.testing.assert_array_equal(guard.step([-2], 5)[0], q)
    guard.rearm()
    restarted_q, restarted_dq = guard.step([-2], 0.01)
    np.testing.assert_allclose(restarted_q, q - 0.0001)
    np.testing.assert_allclose(restarted_dq, [-0.02])
    assert guard.state.mode == "armed"


def test_stale_large_step_integrates_only_until_stop():
    guard = MotionGuard([-1], [1], 1, 2, initial_q=[0])
    q, dq = guard.step([1], 0.4)
    expected_stop = q + dq**2 / 4
    q, dq = guard.step(None, 20, fresh=False)
    np.testing.assert_allclose(q, expected_stop)
    np.testing.assert_array_equal(dq, [0])
    assert guard.state.mode == "hold"


def test_twenty_one_joint_stops_remain_feasible_near_boundaries():
    rng = np.random.default_rng(54)
    lo = np.full(21, -0.4)
    hi = np.full(21, 0.7)
    vel = rng.uniform(0.3, 2.0, 21)
    acc = rng.uniform(0.1, 5.0, 21)
    for _ in range(80):
        initial = np.where(rng.random(21) > 0.5, lo, hi)
        guard = MotionGuard(lo, hi, vel, acc, initial_q=initial)
        target = np.where(initial == lo, hi, lo)
        q, dq = guard.step(target, rng.uniform(0.01, 2.0))
        stopping_point = q + dq * np.abs(dq) / (2 * acc)
        while guard.state.mode != "hold":
            dt = float(rng.uniform(0.001, 0.1))
            previous_q, previous_dq = q, dq
            q, dq = guard.step(None, dt, fresh=False)
            assert_constraints(q, dq, previous_q, previous_dq, dt, lo, hi, vel, acc)
            assert np.all(np.abs(dq) <= np.abs(previous_dq) + 1e-12)
        np.testing.assert_allclose(q, stopping_point, atol=1e-12)


@pytest.mark.parametrize("dt", [0, -1, np.nan, np.inf, True, [0.01]])
def test_invalid_time_does_not_change_state(dt):
    guard = MotionGuard([-1], [1], 1, 1, initial_q=[0])
    with pytest.raises(ValueError, match="dt"):
        guard.step([1], dt)
    np.testing.assert_array_equal(guard.state.q, [0])
    np.testing.assert_array_equal(guard.state.dq, [0])


@pytest.mark.parametrize("target", [[np.nan], [np.inf], [0, 1], None, 0.5])
def test_invalid_target_does_not_change_state(target):
    guard = MotionGuard([-1], [1], 1, 1, initial_q=[0])
    with pytest.raises(ValueError, match="target"):
        guard.step(target, 0.01)
    np.testing.assert_array_equal(guard.state.q, [0])


@pytest.mark.parametrize("kwargs", [
    {"lower": [1]}, {"upper": [-1]}, {"upper": [np.inf]},
    {"velocity_limit": 0}, {"acceleration_limit": -1},
    {"velocity_limit": [1, 2]}, {"initial_q": [2]},
])
def test_invalid_configuration(kwargs):
    config = dict(lower=[-1], upper=[1], velocity_limit=1, acceleration_limit=2)
    config.update(kwargs)
    with pytest.raises(ValueError):
        MotionGuard(**config)


def test_reset_and_external_arrays_do_not_mutate_internal_state():
    initial = np.array([0.2, -0.3])
    guard = MotionGuard([-1, -1], [1, 1], 1, 2, initial_q=initial)
    initial[:] = 99
    snapshot = guard.state
    snapshot.q[:] = 88
    np.testing.assert_array_equal(guard.state.q, [0.2, -0.3])
    q, dq = guard.step([1, -1], 0.1)
    q[:] = 77
    dq[:] = 66
    assert np.all(np.abs(guard.state.q) < 1)
    guard.step(None, 5, fresh=False)
    guard.reset([0, 0])
    assert guard.state.mode == "armed"
    np.testing.assert_array_equal(guard.state.dq, [0, 0])


def test_long_step_matches_substeps_for_a_constant_target():
    # Checks the phase-integrated position, not just bounded output values.
    one = MotionGuard([-3], [3], 0.8, 1.7, initial_q=[0])
    many = MotionGuard([-3], [3], 0.8, 1.7, initial_q=[0])
    one.step([3], 0.4)
    many.step([3], 0.4)
    q_one, dq_one = one.step([-2.5], 2.173)
    for _ in range(2173):
        q_many, dq_many = many.step([-2.5], 0.001)
    np.testing.assert_allclose(q_one, q_many, atol=1e-11)
    np.testing.assert_allclose(dq_one, dq_many, atol=1e-11)
