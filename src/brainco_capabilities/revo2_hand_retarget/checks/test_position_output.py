"""Position output regressions without a ROS graph or robot connection."""
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import JointState

from revo2_hand_retarget.revo2_joints import joint_names_for_side
from revo2_hand_retarget.teleop_controller_node import Revo2TeleopController


def controller(mode="position"):
    node = object.__new__(Revo2TeleopController)
    node.output_mode = mode
    node.control_hz = 100.0
    node.target_timeout = node.feedback_timeout = 0.3
    node.position_rate_limit = 0.5
    node.position_max_lead = 0.10
    node.target_filter_alpha = 0.45
    node.target_filter_fast_alpha = 0.9
    node.target_filter_fast_threshold = 0.09
    node.feedback_position_scales = np.ones(6)
    node.feedback_position_offsets = np.zeros(6)
    node._warn_throttled = lambda message: None
    return node


def ready_state(now=10.0):
    state = Revo2TeleopController._make_side_state()
    state.feedback_ready = state.target_ready = True
    state.feedback_time = state.target_time = now
    state.actual_position[:] = 0.4
    state.target_position[:] = 1.0
    return state


def publisher():
    messages = []
    return SimpleNamespace(publish=lambda msg: messages.append(np.array(msg.data))), messages


def test_reorders_left_and_right_named_targets():
    node = controller()
    for side in ("left", "right"):
        names = joint_names_for_side(side)
        order = [4, 1, 5, 2, 0, 3]
        positions = np.arange(6) * 0.1
        msg = JointState(name=[names[i] for i in order], position=positions[order].tolist())
        np.testing.assert_allclose(node._joint_state_positions(msg, side, feedback=False), positions)


@pytest.mark.parametrize("fault", ["missing", "duplicate", "nan", "inf", "no_names", "wrong_side", "short"])
def test_rejects_malformed_targets(fault):
    node = controller()
    names = list(joint_names_for_side("left"))
    positions = [0.2] * 6
    if fault == "missing":
        names[-1] = "unknown_joint"
    elif fault == "duplicate":
        names[-1] = names[0]
    elif fault in ("nan", "inf"):
        positions[2] = float(fault)
    elif fault == "no_names":
        names = []
    elif fault == "wrong_side":
        names = list(joint_names_for_side("right"))
    elif fault == "short":
        positions.pop()
    with pytest.raises(ValueError):
        node._joint_state_positions(JointState(name=names, position=positions), "left", feedback=False)


def test_initial_position_comes_from_feedback_then_rate_and_lead_are_bounded():
    node, state = controller(), ready_state()
    pub, messages = publisher()
    node._publish_position("left", state, 10.0, pub)
    np.testing.assert_allclose(messages[-1], 0.4)
    # Feedback stays still: position output must not accumulate a far-away goal.
    for i in range(1, 101):
        now = 10.0 + i * 0.01
        state.feedback_time = state.target_time = now
        node._publish_position("left", state, now, pub)
        assert np.max(np.abs(messages[-1] - messages[-2])) <= 0.005 + 1e-12
        assert np.max(np.abs(messages[-1] - state.actual_position)) <= 0.10 + 1e-12
    np.testing.assert_allclose(messages[-1], 0.5)


def test_target_timeout_latches_hold_and_resume_starts_from_feedback():
    node, state = controller(), ready_state()
    pub, messages = publisher()
    node._publish_position("left", state, 10.0, pub)
    state.feedback_time = 10.4
    state.actual_position[:] = 0.42
    node._publish_position("left", state, 10.4, pub)
    np.testing.assert_allclose(messages[-1], 0.42)
    state.feedback_time = 10.5
    state.actual_position[:] = 0.43
    node._publish_position("left", state, 10.5, pub)
    np.testing.assert_allclose(messages[-1], 0.42)  # Hold once, do not chase drift.
    state.target_time = state.feedback_time = 10.6
    node._publish_position("left", state, 10.6, pub)
    np.testing.assert_allclose(messages[-1], 0.43)


def test_feedback_timeout_publishes_nothing_then_resynchronizes():
    node, state = controller(), ready_state()
    pub, messages = publisher()
    node._publish_position("left", state, 10.0, pub)
    state.target_time = 10.4
    node._publish_position("left", state, 10.4, pub)
    assert len(messages) == 1
    state.target_time = state.feedback_time = 10.5
    state.actual_position[:] = 0.7
    node._publish_position("left", state, 10.5, pub)
    np.testing.assert_allclose(messages[-1], 0.7)


def test_no_initial_feedback_never_sends_zero():
    node = controller()
    state = node._make_side_state()
    pub, messages = publisher()
    node._publish_position("left", state, 10.0, pub)
    assert not messages


def test_hold_preserves_real_feedback_outside_urdf_limit():
    node = controller()
    msg = JointState(name=list(joint_names_for_side("left")), position=[1.2] * 6)
    actual = node._joint_state_positions(msg, "left", feedback=True)
    assert actual[0] == 1.2
    target = node._joint_state_positions(msg, "left", feedback=False)
    assert target[0] < 1.2


def test_velocity_mode_keeps_legacy_unnamed_message_support():
    node = controller("velocity")
    actual = node._joint_state_positions(JointState(position=[0.2] * 6), "left", feedback=True)
    np.testing.assert_allclose(actual, 0.2)
