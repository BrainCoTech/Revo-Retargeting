import mujoco
import numpy as np
import pytest

from retarget_v2.model import RobotModel
from retarget_v2.motion import MotionGuard
from retarget_v2.simulation import ServoSimulation


@pytest.fixture(scope="module")
def robot():
    with RobotModel() as model:
        yield model


def test_named_transmissions_have_positive_unit_joint_gear(robot):
    sim = ServoSimulation(robot)
    joint_ids = [mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                 for n in robot.joint_names]
    np.testing.assert_array_equal(sim.model.actuator_trnid[:, 0], joint_ids)
    np.testing.assert_allclose(sim.model.actuator_gear[:, 0], 1)
    np.testing.assert_allclose(sim.model.actuator_ctrlrange[:, 0], robot.lower)
    np.testing.assert_allclose(sim.model.actuator_ctrlrange[:, 1], robot.upper)
    np.testing.assert_allclose(sim.model.actuator_forcerange, np.tile([-.35, .35], (21, 1)))
    midpoint = (robot.lower + robot.upper) / 2
    for index in range(21):
        sim.reset(midpoint)
        sim.data.ctrl[sim.actuator_indices[index]] += 0.05
        mujoco.mj_forward(sim.model, sim.data)
        expected = np.zeros(21)
        expected[index] = 0.05
        np.testing.assert_allclose(sim.data.actuator_force[sim.actuator_indices], expected, atol=1e-12)
    assert sim.report["contacts_enabled"] is False
    np.testing.assert_array_equal(sim.model.opt.gravity, [0, 0, -9.81])


def test_step_integrates_dynamics_instead_of_copying_reference(robot):
    sim = ServoSimulation(robot)
    initial = (robot.lower + robot.upper) / 2
    sim.reset(initial)
    target = initial + 0.1
    measured = sim.step(target, 0.002)
    assert np.max(np.abs(measured - target)) > 0.05
    assert np.max(np.abs(measured - initial)) > 1e-6
    for _ in range(400):
        measured = sim.step(target, 0.002)
    assert np.max(np.abs(measured - target)) < 0.08
    assert sim.report["finite"]
    assert sim.report["mujoco_warning_count"] == 0
    assert sim.report["max_actuator_force_nm"] <= .35 + 1e-12


def test_explicit_simulation_effort_remains_independent_of_urdf(robot):
    assert robot.model_version == "may3"
    sim = ServoSimulation(robot, effort_limit_nm=.12)
    np.testing.assert_array_equal(sim.urdf_effort_limit, .35)
    np.testing.assert_allclose(sim.effort_limit, .12)
    np.testing.assert_allclose(sim.model.actuator_forcerange, np.tile([-.12, .12], (21, 1)))
    assert "Explicit simulation setting" in sim.report["effort_limit_source"]
    for invalid in (0, -1, float("nan"), float("inf"), True):
        with pytest.raises(ValueError):
            ServoSimulation(robot, effort_limit_nm=invalid)


@pytest.mark.parametrize("physics_dt", [.001, .002])
def test_motion_guard_abrupt_target_changes_stay_stable(robot, physics_dt):
    sim = ServoSimulation(robot, timestep=physics_dt)
    guard = MotionGuard(robot.lower, robot.upper, 0.6, 2.0, initial_q=np.zeros(21))
    for index in range(900):
        # Instant goal changes are smoothed by the independent motion layer.
        goal = robot.upper * .8 if 100 <= index < 500 else np.zeros(21)
        reference, _ = guard.step(goal, .01)
        measured = sim.step(reference, .01)
        assert np.all(np.isfinite(measured))
    report = sim.report
    assert report["max_tracking_error_rad"] < .065
    assert report["tracking_rmse_rad"] < .025
    # A held, discretely updated position setpoint can produce faster local
    # measured motion than the reference velocity; the servo is not itself a
    # hard physical velocity limiter. Check the observed stable response.
    assert report["max_measured_velocity_rad_s"] < 1.1
    assert report["max_actuator_force_nm"] <= .35 + 1e-12
    assert report["mujoco_warning_count"] == 0
    assert not report["measured_limit_violation"]
    assert report["max_measured_limit_violation_rad"] < .001


def test_slow_sinusoidal_reference_and_fractional_integration(robot):
    sim = ServoSimulation(robot)
    guard = MotionGuard(robot.lower, robot.upper, .3, 1, initial_q=np.zeros(21))
    for index in range(600):
        goal = robot.upper * (.2 + .15 * np.sin(index * .013 * 1.1))
        reference, _ = guard.step(goal, .013)
        sim.step(reference, .013)
    report = sim.report
    assert report["simulated_time_s"] == pytest.approx(7.8, abs=1e-10)
    assert sim.data.time == pytest.approx(7.8, abs=1e-10)
    assert sim.model.opt.timestep == .002
    assert report["finite"]
    assert report["max_tracking_error_rad"] < .07
    assert report["mujoco_warning_count"] == 0
    assert not report["measured_limit_violation"]


def test_sites_match_measured_pose_and_kinematic_model_is_untouched(robot):
    robot.points(np.zeros(21))
    saved_q = robot.data.qpos.copy()
    sim = ServoSimulation(robot)
    measured = sim.step(robot.upper * .3, .05)
    np.testing.assert_array_equal(robot.data.qpos, saved_q)
    expected = robot.points(measured)
    site_ids = [mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, n)
                for n in robot.point_names]
    np.testing.assert_allclose(sim.data.site_xpos[site_ids], expected, atol=1e-12)


def test_invalid_input_leaves_simulation_unadvanced(robot):
    sim = ServoSimulation(robot)
    for invalid in [0, -1, np.inf, np.nan, True, [0.01]]:
        with pytest.raises(ValueError):
            sim.step(np.zeros(21), invalid)
    for invalid in [np.full(21, np.nan), np.zeros(20), robot.upper + 1]:
        with pytest.raises(ValueError):
            sim.step(invalid, .01)
    assert sim.data.time == 0
    assert sim.report["physics_steps"] == 0
    with pytest.raises(ValueError, match="timestep"):
        ServoSimulation(robot, timestep=.01)


def test_soft_limits_report_true_excursions_without_clipping(robot):
    sim = ServoSimulation(robot)
    # Only initialize via reset; physical gravity at a limit is then resolved
    # by the simulated actuator/soft stop, not by qpos assignment or clipping.
    sim.reset(np.zeros(21))
    for _ in range(100):
        sim.step(np.zeros(21), .01)
    assert sim.report["finite"]
    assert not sim.report["measured_limit_violation"]
    assert 0 <= sim.report["max_measured_limit_violation_rad"] < .001
    sim.reset(robot.upper * .5)
    assert sim.report["physics_steps"] == 0
    assert sim.report["max_tracking_error_rad"] == 0
    np.testing.assert_array_equal(sim.data.qvel, 0)
