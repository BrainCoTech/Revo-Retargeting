"""Independent numerical solver checks; real capture is an optional local fixture.

Set REVO3_TEST_CAPTURE to a recording directory (or its raw.jsonl). Otherwise
the latest recording beside the worktree is used when present. User frames are
read at test time and are never embedded in these tests.
"""

from pathlib import Path
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.input import Calibration, read_capture
from retarget_v2.model import FINGERS, RobotModel
from retarget_v2.solver import Retargeter, SolverConfig


class FeatureCalibration:
    """Supply controlled anatomical features without depending on SDK decoding."""

    def __init__(self, lengths):
        self.lengths = np.asarray(lengths).copy()

    def features(self, message):
        return {name: value.copy() for name, value in message.items()}


@pytest.fixture(scope="module")
def robot():
    with RobotModel() as model:
        yield model


def synthetic_features(robot, q):
    points = robot.points(q)
    names = {name: index for index, name in enumerate(robot.point_names)}
    ids = [[names[f"{finger}_{part}"] for part in ("root", "pip", "dip", "tip")]
           for finger in FINGERS]
    chain = (points[np.asarray(ids)] - robot.palm_origin) @ robot.palm_basis
    segments = np.diff(chain, axis=1)
    pair = chain[0, -1] - chain[1:, -1]
    return dict(chain=chain,
                directions=segments / np.linalg.norm(segments, axis=2, keepdims=True),
                tip_vectors=pair, tip_distances=np.linalg.norm(pair, axis=1))


def away_from_clearance_kinks(retargeter, seed):
    """Choose a deterministic interior pose where a central derivative exists."""
    rng = np.random.default_rng(seed)
    robot = retargeter.robot
    a, b = retargeter.clearance_pairs.T
    threshold = np.where((a % 3 == 2) & (b % 3 == 2), .006, .012)
    for _ in range(100):
        q = robot.lower + (.15 + .7 * rng.random(21)) * (robot.upper - robot.lower)
        chain = robot.points(q)[retargeter.chain_ids]
        samples = np.stack(((chain[:, 1] + chain[:, 2]) / 2,
                            (chain[:, 2] + chain[:, 3]) / 2,
                            chain[:, 3]), axis=1).reshape(-1, 3)
        distances = np.linalg.norm(samples[a] - samples[b], axis=1)
        if np.min(abs(distances - threshold)) > 2e-4 and distances.min() > 1e-4:
            return q
    raise AssertionError("Could not find a pose away from clearance discontinuities")


def assert_residual_jacobian(retargeter, message, seed):
    target = retargeter.targets(message)
    q = away_from_clearance_kinks(retargeter, seed)
    previous = q * .7
    residual, analytic = retargeter.evaluate(q, target, previous)
    assert analytic.shape == (len(residual), 21)
    assert np.isfinite(residual).all() and np.isfinite(analytic).all()
    numeric = np.empty_like(analytic)
    epsilon = 1e-6
    for column in range(21):
        step = np.zeros(21)
        step[column] = epsilon
        positive = retargeter.evaluate(q + step, target, previous)[0]
        negative = retargeter.evaluate(q - step, target, previous)[0]
        numeric[:, column] = (positive - negative) / (2 * epsilon)
    np.testing.assert_allclose(analytic, numeric, atol=2e-8, rtol=3e-6)


def test_full_residual_jacobian_with_synthetic_features(robot):
    calibration = FeatureCalibration(robot.finger_lengths)
    retargeter = Retargeter(robot, calibration)
    q = robot.lower + .35 * (robot.upper - robot.lower)
    assert_residual_jacobian(retargeter, synthetic_features(robot, q), seed=2368)


@pytest.fixture(scope="module")
def actual_frames(robot):
    configured = os.environ.get("REVO3_TEST_CAPTURE")
    if configured:
        path = Path(configured).expanduser()
        if path.is_file():
            path = path.parent
    else:
        candidates = sorted((robot.repo_root.parent / "recordings").glob("*/raw.jsonl"))
        if not candidates:
            pytest.skip("No local real recording; set REVO3_TEST_CAPTURE to enable this integration check")
        path = candidates[-1].parent
    _, rows = read_capture(path)
    initial = [row["msg"] for row in rows if row.get("phase") == "calibration"]
    calibration = Calibration.from_frames(initial[:300])
    pinch_rows = [row for row in rows if row.get("phase") == "pinch"]
    assert pinch_rows, "Real fixture must include a pinch phase"
    pinch_row = min(pinch_rows,
                    key=lambda row: float(calibration.features(row["msg"])["tip_distances"].min()))
    assert calibration.features(pinch_row["msg"])["tip_distances"].min() < SolverConfig().pinch_on_m
    return calibration, {"first": rows[0]["msg"], "pinch": pinch_row["msg"]}


@pytest.mark.parametrize("frame_kind, seed", [("first", 1981), ("pinch", 9093)])
def test_full_residual_jacobian_on_actual_first_and_pinch_frames(robot, actual_frames, frame_kind, seed):
    calibration, frames = actual_frames
    assert_residual_jacobian(Retargeter(robot, calibration), frames[frame_kind], seed=seed)


def test_solve_is_finite_bounded_and_responds_to_a_changed_hand_shape(robot):
    calibration = FeatureCalibration(robot.finger_lengths)
    retargeter = Retargeter(robot, calibration, SolverConfig(max_evaluations=60))
    open_q = robot.lower + .10 * (robot.upper - robot.lower)
    curled_q = robot.lower + .68 * (robot.upper - robot.lower)
    open_features = synthetic_features(robot, open_q)
    curled_features = synthetic_features(robot, curled_q)
    q_open, open_info = retargeter.solve(open_features)
    target = retargeter.targets(curled_features)
    before = retargeter.evaluate(q_open, target, q_open)[0]
    q_curled, curled_info = retargeter.solve(curled_features)
    after = retargeter.evaluate(q_curled, target, q_open)[0]
    for q, info in ((q_open, open_info), (q_curled, curled_info)):
        assert np.isfinite(q).all()
        assert np.all(q >= robot.lower) and np.all(q <= robot.upper)
        assert info["accepted"]
        assert np.isfinite(info["objective"])
    assert np.linalg.norm(q_curled - q_open) > .25
    assert np.dot(after, after) < .6 * np.dot(before, before)
    four_finger_flexion = [index for index, name in enumerate(robot.joint_names)
                          if "thumb" not in name and "MPR" not in name]
    assert q_curled[four_finger_flexion].mean() > q_open[four_finger_flexion].mean() + .2


def test_identical_initial_state_and_input_give_identical_solution(robot):
    calibration = FeatureCalibration(robot.finger_lengths)
    message = synthetic_features(robot, robot.lower + .4 * (robot.upper - robot.lower))
    first, second = Retargeter(robot, calibration), Retargeter(robot, calibration)
    q1, info1 = first.solve(message)
    q2, info2 = second.solve(message)
    np.testing.assert_allclose(q1, q2, atol=1e-12, rtol=0.)
    assert info1["accepted"] == info2["accepted"]
    assert info1["nfev"] == info2["nfev"]
    assert info1["objective"] == pytest.approx(info2["objective"], abs=1e-12)


def test_hardware_limits_constrain_optimization_without_changing_urdf(robot):
    from retarget_v2.hardware_profile import HardwareProfile
    profile = HardwareProfile(robot)
    original_lower, original_upper = robot.lower.copy(), robot.upper.copy()
    calibration = FeatureCalibration(robot.finger_lengths)
    limited = Retargeter(robot, calibration, joint_limits=(profile.model_lower, profile.model_lower + .5 * (profile.model_upper - profile.model_lower)))
    # The goal intentionally uses URDF angles beyond the command envelope.
    target = synthetic_features(robot, robot.lower + .95 * (robot.upper - robot.lower))
    for _ in range(3):
        q, status = limited.solve(target)
        assert status["accepted"]
        assert np.all(q >= profile.model_lower) and np.all(q <= profile.model_upper)
        command = profile.model_to_hardware(q)
        assert np.all(command >= profile.lower) and np.all(command <= profile.upper)
    np.testing.assert_array_equal(robot.lower, original_lower)
    np.testing.assert_array_equal(robot.upper, original_upper)
    assert np.all(q <= limited.upper)
    assert np.any(limited.upper < robot.upper)


def test_solver_rejects_invalid_or_widened_hardware_limits(robot):
    calibration = FeatureCalibration(robot.finger_lengths)
    for lower, upper in [(robot.lower[:-1], robot.upper),
                         (np.full(21, np.nan), robot.upper),
                         (robot.lower-.1, robot.upper),
                         (robot.lower, robot.upper+.1),
                         (robot.upper, robot.lower),
                         (robot.lower, robot.lower+1e-7)]:
        with pytest.raises(ValueError, match="joint_limits"):
            Retargeter(robot, calibration, joint_limits=(lower, upper))


@pytest.mark.parametrize("distance", [0., 1e-12])
def test_coincident_human_tips_keep_a_finite_nonzero_robot_pinch_gap(robot, distance):
    calibration = FeatureCalibration(robot.finger_lengths)
    retargeter = Retargeter(robot, calibration)
    message = synthetic_features(robot, robot.lower + .3 * (robot.upper - robot.lower))
    message["tip_vectors"][0] = [distance, 0., 0.]
    message["tip_distances"][0] = distance
    target = retargeter.targets(message)
    for value in target.values():
        assert np.isfinite(value).all()
    assert target["pinch_blend"][0] == pytest.approx(1.)
    assert np.linalg.norm(target["pair_target"][0]) == pytest.approx(retargeter.config.pinch_gap_m, abs=1e-12)
    residual, jacobian = retargeter.evaluate(retargeter.q, target, retargeter.q)
    assert np.isfinite(residual).all() and np.isfinite(jacobian).all()


def test_thumb_offset_matrix_is_a_rotation_and_zero_is_identity():
    from retarget_v2.solver import thumb_offset_matrix
    rotation = thumb_offset_matrix(25., (.6508, .1828, -.7369))
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.)
    axis = np.array([.6508, .1828, -.7369]) / np.linalg.norm([.6508, .1828, -.7369])
    np.testing.assert_allclose(rotation @ axis, axis, atol=1e-12)
    np.testing.assert_allclose(thumb_offset_matrix(0., (0., 0., 1.)), np.eye(3), atol=1e-15)
    with pytest.raises(ValueError):
        thumb_offset_matrix(10., (0., 0., 0.))


def test_thumb_offset_rotates_only_thumb_direction_targets(robot):
    calibration = FeatureCalibration(robot.finger_lengths)
    message = synthetic_features(robot, robot.lower + .3 * (robot.upper - robot.lower))
    plain = Retargeter(robot, calibration).targets(message)
    zero = Retargeter(robot, calibration, thumb_offset=(0., (0., 0., 1.))).targets(message)
    shifted = Retargeter(robot, calibration, thumb_offset=(30., (0., 0., 1.))).targets(message)
    np.testing.assert_allclose(zero["directions_world"], plain["directions_world"], atol=1e-15)
    np.testing.assert_allclose(shifted["directions_world"][1:], plain["directions_world"][1:], atol=1e-15)
    np.testing.assert_allclose(shifted["pair_target"], plain["pair_target"], atol=1e-15)
    angle = np.degrees(np.arccos(np.clip(np.sum(shifted["directions"][0] * plain["directions"][0], axis=1), -1, 1)))
    assert np.all(angle <= 30. + 1e-9) and angle.max() > 1.
    # The caller's features must not be mutated by the offset.
    np.testing.assert_allclose(message["directions"][0], synthetic_features(
        robot, robot.lower + .3 * (robot.upper - robot.lower))["directions"][0], atol=1e-15)
