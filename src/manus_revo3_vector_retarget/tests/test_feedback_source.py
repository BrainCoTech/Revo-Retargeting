"""The shared MIT backend requires only named JointState feedback."""
import numpy as np
import pytest

from retarget_v2.hardware_control import HardwareController
from retarget_v2.hardware_profile import HardwareProfile
from retarget_v2.model import RobotModel


@pytest.fixture(scope="module")
def profile():
    with RobotModel() as robot:
        yield HardwareProfile(robot)


@pytest.mark.parametrize("stamp, now_ros", [(0., 0.), (100., 500.), (500., 100.)])
@pytest.mark.parametrize("interpolate", [False, True])
def test_ros_clock_and_header_age_do_not_gate_sampling(profile, stamp, now_ros, interpolate):
    c = HardwareController(profile, interpolate=interpolate)
    q = (profile.lower + profile.upper) / 2
    assert c.update_feedback(profile.names, q, np.full(21, .2), stamp, 10., now_ros, 10.)
    assert c.update_target(profile.feedback_to_model(q), 10.)
    assert c.arm(10.)
    assert c.update_target(profile.feedback_to_model(q), 10.01)
    np.testing.assert_allclose(c.sample(10.01)[0], q)
    status = c.status(10.)
    assert status["feedback_source"] == "joint_state"
    assert not hasattr(c, "update_dynamic")
    assert not {"bus_age_s", "motor_state", "current_a", "driver_read_successes_total"} & status.keys()


def test_repeated_ros_header_accepts_latest_received_values(profile):
    c = HardwareController(profile)
    q = (profile.lower + profile.upper) / 2
    for timestamp, position in ((10., q), (10.02, q + .01)):
        assert c.update_feedback(profile.names, position, np.zeros(21), 0., timestamp, 0., timestamp)
    np.testing.assert_allclose(c.status(10.02)["feedback_model_q"], profile.feedback_to_model(q + .01))
