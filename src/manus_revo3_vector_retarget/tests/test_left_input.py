"""Selected-hand input contracts and a complete left-hand simulated session."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from retarget_v2.input import Calibration, chains, decode, palm_coordinates
from retarget_v2.live_control import LiveController
from retarget_v2.model import RobotModel
from retarget_v2.ros_input import LatestMessageBuffer, ManusSubscriber
from retarget_v2.solver import Retargeter
from test_input import sdk_hand, transformed
from test_ros_input import fake_ros, object_message, sdk_message


def left_hand(pip_bend_degrees=30.):
    frame = transformed(sdk_hand(pip_bend_degrees), np.diag([-1., 1., 1.]), np.zeros(3))
    frame["side"] = "Left"
    return frame


def test_left_decode_requires_explicit_selection_and_preserves_geometry():
    frame = left_hand()
    with pytest.raises(ValueError, match="right hand"):
        decode(frame)
    left = decode(frame, hand_side="left")
    right = decode(sdk_hand())
    for name in left:
        np.testing.assert_array_equal(left[name], right[name] * [-1., 1., 1.])
    with pytest.raises(ValueError, match="left hand"):
        decode(sdk_hand(), hand_side="left")
    bad = deepcopy(frame)
    bad["raw_nodes"][7]["parent_node_id"] = 0
    with pytest.raises(ValueError, match="Wrong parent"):
        decode(bad, hand_side="left")


@pytest.mark.parametrize("side", [None, "", "both", 1, True])
def test_invalid_hand_selection_fails_before_input(side):
    with pytest.raises(ValueError, match="hand_side"):
        decode(left_hand(), hand_side=side)
    with pytest.raises(ValueError, match="hand_side"):
        Calibration(np.full((5, 3), .03), hand_side=side)
    with pytest.raises(ValueError, match="hand_side"):
        LatestMessageBuffer(hand_side=side)
    with pytest.raises(ValueError, match="hand_side"):
        ManusSubscriber(hand_side=side)


def test_left_calibration_preserves_mirrored_anatomy_and_rigid_motion_invariance():
    right = sdk_hand()
    left = left_hand()
    right_calibration = Calibration.from_frames([right] * 30)
    left_calibration = Calibration.from_frames([left] * 30, hand_side="left")
    assert left_calibration.hand_side == "left"
    assert left_calibration.normal_sign == right_calibration.normal_sign == -1.
    np.testing.assert_allclose(left_calibration.lengths, right_calibration.lengths, atol=1e-14)
    right_features = right_calibration.features(right)
    left_features = left_calibration.features(left)
    # Both anatomical frames use a cross-product normal. Reflection therefore
    # reverses local z only; the selected robot's own palm frame does the same.
    # Adding another left-only normal-sign change would invert finger flexion.
    for feature in ("chain", "directions", "tip_vectors"):
        np.testing.assert_allclose(left_features[feature], right_features[feature] * [1., 1., -1.],
                                   atol=1e-14)
    np.testing.assert_allclose(left_features["tip_distances"], right_features["tip_distances"])
    rotation = Rotation.from_rotvec([.35, -.8, 1.2]).as_matrix()
    moved = transformed(left, rotation, np.array([1., -2., .4]))
    for feature, values in left_calibration.features(moved).items():
        np.testing.assert_allclose(values, left_features[feature], atol=2e-13)
    np.testing.assert_allclose(chains(palm_coordinates(decode(left, "left"))), left_features["chain"])


def test_calibration_cannot_mix_sides_or_accept_opposite_side_afterward():
    left = left_hand()
    calibration = Calibration.from_frames([left] * 30, hand_side="left")
    with pytest.raises(ValueError, match="left hand"):
        Calibration.from_frames([left] * 29 + [sdk_hand()], hand_side="left")
    with pytest.raises(ValueError, match="left hand"):
        calibration.features(sdk_hand())
    with pytest.raises(ValueError, match="Bone length changed"):
        calibration.features(transformed(left, np.eye(3) * 1.3, np.zeros(3)))


def test_left_buffer_rejects_other_side_without_locking_or_refreshing_identity():
    buffer = LatestMessageBuffer(hand_side="left")
    assert not buffer.put(sdk_message(glove_id=10, side="Right"), 1.)
    assert buffer.status()["selected_glove_id"] is None
    assert buffer.latest() is None
    assert buffer.put(sdk_message(glove_id=-20, side="Left"), 2.)
    previous = buffer.latest()
    assert not buffer.put(sdk_message(glove_id=-20, side="Right"), 3.)
    assert not buffer.put(sdk_message(glove_id=-21, side="Left"), 4.)
    assert buffer.latest() == previous
    assert buffer.status()["hand_side"] == "left"
    assert buffer.status()["selected_glove_id"] == -20
    assert buffer.put(sdk_message(glove_id=-20, side="Left"), 5.)
    assert buffer.latest().sequence == 2


def test_subscriber_passes_selected_side_to_callback_filter(fake_ros):
    with ManusSubscriber(topic="/manus_left", hand_side="left") as subscriber:
        callback = fake_ros["nodes"][0].callback
        callback(object_message(sdk_message(side="Right")))
        assert subscriber.latest() is None
        callback(object_message(sdk_message(side="Left")))
        previous = subscriber.latest()
        assert previous.message["side"] == "Left"
        callback(object_message(sdk_message(side="Right")))
        assert subscriber.latest() == previous
        assert subscriber.status()["hand_side"] == "left"


def test_left_live_session_calibrates_solves_moves_and_rejects_right_input():
    with RobotModel(hand_side="left") as robot:
        right_calibration = Calibration.from_frames([sdk_hand()] * 30)
        with pytest.raises(ValueError, match="hand side does not match"):
            Retargeter(robot, right_calibration)
        controller = LiveController(robot, calibration_frames=30)
        try:
            frame = left_hand()
            for sequence in range(1, 31):
                now = sequence * .01
                assert controller.ingest(deepcopy(frame), now, sequence, now)
            assert controller.status(now)["state"] == "ready"
            assert controller.status(now)["hand_side"] == "left"
            assert controller.solver.calibration.hand_side == "left"
            assert controller.solve_status["accepted"]
            assert np.isfinite(controller._target).all()
            assert np.all(controller._target >= robot.lower)
            assert np.all(controller._target <= robot.upper)
            assert controller.resume(now)
            q, dq = controller.tick(.01, now + .01)
            assert np.any(np.abs(dq) > 0)
            assert np.all(q >= robot.lower)
            assert np.all(q <= robot.upper)
            assert not controller.ingest(sdk_hand(), now + .02, sequence + 1, now + .02)
            status = controller.status(now + .02)
            assert status["valid_frames"] == 30
            assert status["input_age_s"] == pytest.approx(.02)
            assert status["stop_reason"] == "invalid_input"
            assert controller.resume(now + .02) is False
        finally:
            controller.close()
