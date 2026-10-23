"""JointState-seeded control tests without ROS or a hardware SDK."""
from pathlib import Path
from copy import copy
from dataclasses import replace
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.hardware_control import HardwareController
from retarget_v2.hardware_profile import HardwareProfile
from retarget_v2.model import RobotModel


@pytest.fixture(scope="module")
def profile():
    with RobotModel() as robot:
        yield HardwareProfile(robot)


def feed(controller, now, q=None, dq=None):
    p = controller.profile
    q = (p.lower + p.upper) / 2 if q is None else q
    dq = np.zeros(21) if dq is None else dq
    return controller.update_feedback(p.names, q, dq, 1000 + now, now, 1000 + now, now)


def ready(profile, interpolate=True):
    controller = HardwareController(profile, interpolate=interpolate)
    measured = profile.model_to_hardware(profile.model_lower + .2 * (profile.model_upper - profile.model_lower))
    assert feed(controller, 10., measured)
    target = profile.model_lower + .6 * (profile.model_upper - profile.model_lower)
    assert controller.update_target(target, 10.)
    return controller, measured, target, 10.


def test_initially_silent_and_first_armed_command_uses_one_measured_pose(profile):
    c, measured, _, now = ready(profile)
    assert c.status(now)["ready_to_arm"]
    assert c.sample(now) is None
    assert c.arm(now)
    q, dq = c.sample(now)
    np.testing.assert_allclose(q, measured, atol=1e-12)
    np.testing.assert_array_equal(dq, np.zeros(21))
    assert c.status(now)["state"] == "ramping"
    assert c.status(now)["output_mode"] == "interpolated"
    assert c.status(now)["interpolate"] is True
    assert np.max(np.abs(q)) > .1


def test_arm_requires_initial_joint_state_and_fresh_target(profile):
    c = HardwareController(profile)
    target = (profile.model_lower + profile.model_upper) / 2
    assert not c.arm(10.)
    assert c.status(10.)["arming_blocked_by"] == "waiting_feedback"
    assert feed(c, 10.)
    assert not c.arm(10.)
    assert c.status(10.)["arming_blocked_by"] == "waiting_target"
    assert c.update_target(target, 9.)
    assert not c.arm(10.)
    assert c.update_target(target, 10.)
    assert c.arm(10.)


def test_feedback_age_and_velocity_do_not_gate_output(profile):
    c, measured, target, start = ready(profile)
    assert feed(c, start, measured, np.full(21, 2.))
    now = start + 20.
    assert c.update_target(target, now)
    assert c.arm(now)
    for i in range(1, 121):
        timestamp = now + i * .005
        if i % 4 == 0:
            assert c.update_target(target, timestamp)
        assert c.sample(timestamp) is not None
    status = c.status(timestamp)
    assert status["enabled"]
    assert status["feedback_age_s"] == pytest.approx(20.6)
    np.testing.assert_array_equal(status["feedback_velocity_hardware"], np.full(21, 2.))


def test_pause_latches_and_rearm_seeds_latest_measurement(profile):
    c, measured, target, now = ready(profile)
    assert c.arm(now)
    assert c.sample(now + .005) is not None
    c.pause()
    measured += .001
    now += .02
    assert feed(c, now, measured)
    assert c.update_target(target, now)
    assert c.sample(now) is None
    assert c.arm(now)
    q, dq = c.sample(now)
    np.testing.assert_allclose(q, measured)
    np.testing.assert_array_equal(dq, np.zeros(21))


@pytest.mark.parametrize("interpolate", [False, True])
def test_recalibration_discards_old_target_until_new_solve(profile, interpolate):
    c, measured, target, now = ready(profile, interpolate)
    assert c.arm(now)
    c.clear_target()
    assert c.sample(now) is None
    assert c.status(now)["arming_blocked_by"] == "waiting_target"
    assert not c.arm(now)
    assert feed(c, now + .02, measured)
    assert not c.arm(now + .02)
    assert c.update_target(target, now + .02)
    assert c.sample(now + .02) is None
    assert c.arm(now + .02)


def test_target_expiry_latches_until_explicit_restart(profile):
    c, _, target, start = ready(profile)
    assert c.arm(start)
    for i in range(1, 61):
        now = start + i * .005
        c.sample(now)
    assert c.status(now)["reason"] == "stale_target"
    assert c.update_target(target, now + .02)
    assert c.sample(now + .02) is None
    assert c.arm(now + .02)


@pytest.mark.parametrize("gap,reason", [(.15, "timer_gap"), (.3, "target_gap")])
def test_target_and_timer_gaps_latch(profile, gap, reason):
    c, _, target, start = ready(profile)
    assert c.arm(start)
    now = start + gap
    assert c.update_target(target, now)
    assert c.sample(now) is None
    assert c.status(now)["reason"] == reason
    assert c.arm(now)


def test_out_of_range_measurement_only_projects_seed_into_command_bounds(profile):
    c, measured, target, now = ready(profile)
    measured[0] = profile.lower[0] - .5
    measured[1] = profile.upper[1] + .5
    assert feed(c, now, measured)
    assert c.arm(now)
    q, dq = c.sample(now)
    np.testing.assert_allclose(q, np.clip(measured, profile.lower, profile.upper))
    np.testing.assert_array_equal(dq, np.zeros(21))
    status = c.status(now)
    np.testing.assert_allclose(status["feedback_model_q"], profile.feedback_to_model(measured))
    np.testing.assert_allclose(status["startup_projection_rad"], q - measured)
    previous = q
    for i in range(1, 61):
        timestamp = now + i * .005
        if i % 4 == 0:
            assert c.update_target(target, timestamp)
        q, dq = c.sample(timestamp)
        assert np.all(q >= profile.lower) and np.all(q <= profile.upper)
        np.testing.assert_allclose(dq, (q - previous) / .005, atol=1e-10)
        previous = q


@pytest.mark.parametrize("damage", ["side", "duplicate", "missing", "nan_position", "nan_velocity"])
def test_malformed_feedback_preserves_last_valid_state(profile, damage):
    c, measured, _, now = ready(profile)
    assert c.arm(now)
    names, q, dq = list(profile.names), measured.copy(), np.zeros(21)
    if damage == "side":
        names = [name.replace("right_", "left_") for name in names]
    elif damage == "duplicate":
        names[-1] = names[0]
    elif damage == "missing":
        names = names[:-1]
    elif damage == "nan_position":
        q[0] = np.nan
    else:
        dq[0] = np.nan
    assert not c.update_feedback(names, q, dq, 0., now, 0., now)
    np.testing.assert_allclose(c.status(now)["feedback_model_q"], profile.feedback_to_model(measured))
    assert c.sample(now + .005) is not None


def test_joint_state_name_order_is_explicit(profile):
    c = HardwareController(profile)
    q = profile.lower + np.linspace(.2, .8, 21) * (profile.upper - profile.lower)
    dq = np.linspace(-.2, .2, 21)
    order = np.random.default_rng(18).permutation(21)
    assert c.update_feedback(np.asarray(profile.names)[order].tolist(), q[order], dq[order], 0, 10., 0, 10.)
    status = c.status(10.)
    np.testing.assert_allclose(status["feedback_model_q"], profile.feedback_to_model(q))
    np.testing.assert_array_equal(status["feedback_velocity_hardware"], dq)


@pytest.mark.parametrize("interpolate", [False, True])
@pytest.mark.parametrize("bad_target", ["outside", "nan", "shape"])
def test_invalid_target_cannot_arm_old_cached_command(profile, bad_target, interpolate):
    c, _, target, now = ready(profile, interpolate)
    assert c.arm(now)
    if bad_target == "outside":
        target[0] = profile.model_upper[0] + .1
    elif bad_target == "nan":
        target[0] = np.nan
    else:
        target = target[:-1]
    assert not c.update_target(target, now + .01)
    assert c.sample(now + .01) is None
    assert not c.arm(now + .01)


def test_continuous_targets_keep_startup_deadline_and_command_derivative(profile):
    c, measured, target, start = ready(profile)
    assert c.arm(start)
    previous = measured
    for i in range(1, 431):
        now = start + i * .005
        if i % 3 == 0:
            assert c.update_target(target, now)
        q, dq = c.sample(now)
        assert np.all(q >= profile.lower - 1e-12) and np.all(q <= profile.upper + 1e-12)
        assert np.max(np.abs(dq)) <= profile.command.max_velocity_rad_s + 1e-10
        np.testing.assert_allclose(dq, (q - previous) / .005, atol=1e-10)
        previous = q
        if i == 350:
            assert c.status(now)["state"] == "ramping"
    assert c.status(now)["state"] == "active"


@pytest.mark.parametrize("interpolate", [False, True])
def test_shutdown_never_rearms_or_returns_command(profile, interpolate):
    c, measured, target, now = ready(profile, interpolate)
    assert c.arm(now)
    c.shutdown()
    assert not feed(c, now + .02, measured)
    assert not c.update_target(target, now + .02)
    assert c.sample(now + .02) is None
    assert not c.arm(now + .02)
    assert c.status(now + .02)["state"] == "shutdown"


@pytest.mark.parametrize("value", [None, 0, 1, "false", "true", np.bool_(False)])
def test_interpolate_requires_an_explicit_boolean(profile, value):
    with pytest.raises(ValueError, match="interpolate must be a boolean"):
        HardwareController(profile, interpolate=value)


def test_direct_is_default_and_waits_for_each_new_target_without_smoothing(profile):
    c = HardwareController(profile)
    measured = profile.lower + .1 * (profile.upper - profile.lower)
    assert feed(c, 10., measured)
    target = profile.model_lower + .8 * (profile.model_upper - profile.model_lower)
    assert c.update_target(target, 10.)
    assert c.arm(10.)
    assert c.sample(10.) is None
    assert c.sample(10.005) is None
    assert c.update_target(target, 10.02)
    q, dq = c.sample(10.021)
    np.testing.assert_array_equal(q, profile.model_to_hardware(target))
    np.testing.assert_array_equal(dq, np.zeros(21))
    assert np.max(np.abs(q - measured)) > .5
    assert c.sample(10.025) is None
    # A new stationary glove frame is still a new command.
    assert c.update_target(target, 10.04)
    q2, dq2 = c.sample(10.042)
    np.testing.assert_array_equal(q2, q)
    np.testing.assert_array_equal(dq2, np.zeros(21))
    status = c.status(10.042)
    assert status["state"] == "active"
    assert status["output_mode"] == "direct" and status["interpolate"] is False
    assert status["commands_generated"] == 2


def test_direct_velocity_uses_source_receipt_intervals_and_is_capped(profile):
    c, _, target, start = ready(profile, interpolate=False)
    assert c.arm(start)
    assert c.update_target(target, start + .02)
    q1, dq1 = c.sample(start + .10)
    np.testing.assert_array_equal(dq1, np.zeros(21))
    target2 = target + .01 * (profile.model_upper - profile.model_lower)
    assert c.update_target(target2, start + .04)
    q2, dq2 = c.sample(start + .11)
    np.testing.assert_allclose(dq2, (q2 - q1) / .02, atol=1e-10)
    target3 = profile.model_lower + .2 * (profile.model_upper - profile.model_lower)
    assert c.update_target(target3, start + .041)
    q3, dq3 = c.sample(start + .12)
    np.testing.assert_array_equal(q3, profile.model_to_hardware(target3))
    np.testing.assert_allclose(dq3, np.clip((q3 - q2) / .001,
                                         -profile.command.max_velocity_rad_s,
                                         profile.command.max_velocity_rad_s))
    assert np.max(np.abs(dq3)) == profile.command.max_velocity_rad_s


def test_direct_velocity_feedforward_can_be_disabled(profile):
    disabled_profile = copy(profile)
    disabled_profile.command = replace(profile.command, velocity_feedforward=False)
    c, _, target, start = ready(disabled_profile, interpolate=False)
    assert c.arm(start)
    assert c.update_target(target, start + .02)
    c.sample(start + .02)
    assert c.update_target(profile.model_lower, start + .04)
    q, dq = c.sample(start + .04)
    np.testing.assert_array_equal(q, profile.model_to_hardware(profile.model_lower))
    np.testing.assert_array_equal(dq, np.zeros(21))


def test_direct_reads_latest_valid_feedback_without_age_or_velocity_gates(profile):
    c, measured, target, start = ready(profile, interpolate=False)
    assert c.arm(start)
    measured += .001
    assert feed(c, start + .01, measured, np.full(21, 2.))
    assert not feed(c, start + .02, np.full(21, np.nan))
    assert c.update_target(target, start + .15)
    q, _ = c.sample(start + .16)
    np.testing.assert_array_equal(q, profile.model_to_hardware(target))
    status = c.status(start + .16)
    np.testing.assert_allclose(status["feedback_model_q"], profile.feedback_to_model(measured))
    np.testing.assert_array_equal(status["feedback_velocity_hardware"], np.full(21, 2.))
    assert status["feedback_delivery_count"] == 2
    assert status["feedback_age_s"] == pytest.approx(.15)
    assert status["enabled"]


def test_direct_watchdog_never_consumes_a_target_and_stale_input_requires_rearm(profile):
    c, _, target, start = ready(profile, interpolate=False)
    assert c.arm(start)
    assert c.update_target(target, start + .02)
    assert c.check_watchdog(start + .03)
    assert c.check_watchdog(start + .04)
    assert c.status(start + .04)["commands_generated"] == 0
    assert c.sample(start + .04) is not None
    assert c.sample(start + .05) is None
    assert not c.update_target(target, start + .02)
    assert not c.update_target(target, start + .01)
    assert not c.check_watchdog(start + .28)
    assert c.status(start + .28)["reason"] == "stale_target"
    assert c.update_target(target, start + .30)
    assert c.sample(start + .30) is None
    assert c.arm(start + .30)
    assert c.sample(start + .30) is None
    assert c.update_target(target, start + .32)
    _, dq = c.sample(start + .32)
    np.testing.assert_array_equal(dq, np.zeros(21))


def test_direct_source_gap_latches_even_when_new_target_is_fresh(profile):
    c, _, target, start = ready(profile, interpolate=False)
    assert c.arm(start)
    assert c.update_target(target, start + .3)
    assert c.sample(start + .3) is None
    assert c.status(start + .3)["reason"] == "target_gap"


def test_direct_pause_and_recalibration_clear_velocity_history(profile):
    c, _, target, start = ready(profile, interpolate=False)
    assert c.arm(start)
    assert c.update_target(target, start + .02)
    assert c.sample(start + .02) is not None
    for offset, stop in ((.04, c.pause), (.10, c.clear_target)):
        stop()
        assert c.sample(start + offset) is None
        assert c.update_target(profile.model_lower, start + offset)
        assert c.arm(start + offset)
        assert c.sample(start + offset) is None
        assert c.update_target(profile.model_upper, start + offset + .02)
        q, dq = c.sample(start + offset + .02)
        np.testing.assert_array_equal(q, profile.model_to_hardware(profile.model_upper))
        np.testing.assert_array_equal(dq, np.zeros(21))


def test_direct_repeated_arm_does_not_drop_pending_target_or_velocity_history(profile):
    c, _, target, start = ready(profile, interpolate=False)
    assert c.arm(start)
    assert c.update_target(target, start + .02)
    q1, _ = c.sample(start + .02)
    target2 = target + .01 * (profile.model_upper - profile.model_lower)
    assert c.update_target(target2, start + .04)
    assert c.arm(start + .04)
    q2, dq2 = c.sample(start + .04)
    np.testing.assert_allclose(dq2, (q2 - q1) / .02, atol=1e-10)


@pytest.mark.parametrize("check", ["sample", "check_watchdog"])
def test_direct_invalid_clock_latches_until_explicit_restart(profile, check):
    c, _, _, start = ready(profile, interpolate=False)
    assert c.arm(start)
    assert not getattr(c, check)(float("nan"))
    assert c.status(start)["reason"] == "command_error"
    assert not c.status(start)["enabled"]
