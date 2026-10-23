"""Left hardware mapping and emitted MIT references, without ROS or a device."""
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from retarget_v2 import ros_hardware
from retarget_v2.hardware_profile import HardwareProfile
from retarget_v2.paths import description_root
from retarget_v2.ros_hardware import HardwareTransport


CONFIG = Path(__file__).resolve().parents[1] / "config"


@pytest.fixture
def left_robot():
    # Read the actual description; hardware mapping does not require MuJoCo.
    urdf = description_root() / "urdf/revo3_left_may3.urdf"
    joints = [j for j in ET.parse(urdf).getroot().findall("joint") if j.get("type") == "revolute"]
    return SimpleNamespace(
        hand_side="left", model_version="may3", urdf_path=urdf,
        joint_names=tuple(j.get("name") for j in joints),
        lower=np.array([float(j.find("limit").get("lower")) for j in joints]),
        upper=np.array([float(j.find("limit").get("upper")) for j in joints]),
    )


def test_left_profile_maps_named_joints_and_preserves_feedback(left_robot):
    profile = HardwareProfile(left_robot)
    assert profile.hand_side == "left"
    assert profile.path.name == "left_hardware_may3.yaml"
    assert profile.names[:4] == tuple(f"left_little_{j}_joint" for j in ("MPR", "MCP", "PIP", "DIP"))
    assert profile.names[-5:] == tuple(f"left_thumb_{j}_joint" for j in ("MCP", "PIP", "DIP", "CMP", "CMR"))
    assert profile.mapping_metadata["status"] == "provisional_not_physically_calibrated"
    np.testing.assert_array_equal(profile.scale, np.ones(21))
    np.testing.assert_array_equal(profile.offset, np.zeros(21))
    q = profile.model_lower + np.linspace(.2, .8, 21) * (profile.model_upper - profile.model_lower)
    indices = [left_robot.joint_names.index(name) for name in profile.names]
    np.testing.assert_array_equal(profile.model_to_hardware(q), q[indices])
    np.testing.assert_array_equal(profile.feedback_to_model(q[indices]), q)
    feedback = profile.lower - .01
    np.testing.assert_allclose(profile.feedback_to_model(feedback)[indices], feedback)
    with pytest.raises(ValueError, match="outside"):
        profile.model_to_hardware(profile.feedback_to_model(feedback))


def test_left_limits_intersect_description_without_expanding_hardware(left_robot):
    profile = HardwareProfile(left_robot)
    data = yaml.safe_load(profile.path.read_text())
    for i, name in enumerate(profile.names):
        j = left_robot.joint_names.index(name)
        limits = data["hardware_channel_bounds"][name]
        assert profile.lower[i] == max(left_robot.lower[j], limits["lower_rad"])
        assert profile.upper[i] == min(left_robot.upper[j], limits["upper_rad"])
    for q in (profile.model_lower, profile.model_upper):
        np.testing.assert_allclose(profile.feedback_to_model(profile.model_to_hardware(q)), q)
    order = np.random.default_rng(37).permutation(21)
    shuffled = SimpleNamespace(**vars(left_robot))
    shuffled.joint_names = tuple(left_robot.joint_names[i] for i in order)
    shuffled.lower, shuffled.upper = left_robot.lower[order], left_robot.upper[order]
    reordered = HardwareProfile(shuffled)
    q = (profile.model_lower + profile.model_upper) / 2
    np.testing.assert_array_equal(reordered.model_to_hardware(q[order]), profile.model_to_hardware(q))


def test_right_profile_cannot_be_used_for_left_model(left_robot):
    with pytest.raises(ValueError, match="hand_side"):
        HardwareProfile(left_robot, CONFIG / "right_hardware_may3.yaml")
    mislabeled_robot = SimpleNamespace(**{**vars(left_robot), "hand_side": "right"})
    with pytest.raises(ValueError, match="hand_side"):
        HardwareProfile(mislabeled_robot, CONFIG / "left_hardware_may3.yaml")


@pytest.mark.parametrize("mutation", [
    lambda p: p["output_calibration"].update(right={"index_MPR_joint": {"scale": 1., "offset_deg": 0.}}),
    lambda p: p["hardware_channel_bounds"].update(right_index_MPR_joint=p["hardware_channel_bounds"].pop("left_index_MPR_joint")),
    lambda p: p.update(feedback_lower_stop_overrides_deg={"right_middle_MCP_joint": 5.}),
    lambda p: p.update(feedback_lower_stop_overrides_deg={"left_middle_MCP_joint": 5.}),
])
def test_left_profile_rejects_mixed_side_data_and_right_feedback_exceptions(left_robot, tmp_path, mutation):
    data = yaml.safe_load((CONFIG / "left_hardware_may3.yaml").read_text())
    mutation(data)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        HardwareProfile(left_robot, path)


def test_left_transport_publishes_interpolated_velocity_with_left_names(left_robot, monkeypatch):
    profile = HardwareProfile(left_robot)
    assert profile.command.velocity_feedforward is True
    transport = HardwareTransport(profile, interpolate=True)
    assert transport.namespace == "/revo3_left"
    assert transport.command_topic == "/revo3_left/joint_forward_mit_controller/commands"
    assert transport.feedback_topic == "/revo3_left/revo3_joint_state/joint_states"
    control = transport.control
    measured = profile.lower + .2 * (profile.upper - profile.lower)
    for now in (10., 10.02):
        ros = 1000. + now
        assert control.update_feedback(profile.names, measured, np.zeros(21), ros, now, ros, now)
    target = profile.model_lower + .6 * (profile.model_upper - profile.model_lower)
    assert transport.set_target(target, 10.02)
    assert control.arm(10.02)
    first_q, first_dq = control.sample(10.02)
    np.testing.assert_allclose(first_q, measured)
    np.testing.assert_array_equal(first_dq, np.zeros(21))

    # Exercise the actual command controller/interpolator and transport timer;
    # only the ROS node, clock, and publisher are replaced with local doubles.
    messages = []
    transport._publisher = SimpleNamespace(publish=messages.append)
    transport._message_type = lambda: SimpleNamespace(header=SimpleNamespace())
    transport._node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=1010, nanosec=25_000_000))))
    transport._last_graph_check = 10.02
    monkeypatch.setattr(ros_hardware.time, "monotonic", lambda: 10.025)
    transport._tick()
    assert len(messages) == 1
    command = messages[0]
    assert command.header.frame_id == "left_hand_base_link"
    assert command.joint_names == list(profile.names)
    assert np.any(np.abs(command.velocity) > 0)
    np.testing.assert_allclose(command.velocity, (np.array(command.position) - first_q) / .005)
    np.testing.assert_array_equal(command.kp, profile.kp)
    np.testing.assert_array_equal(command.kd, profile.kd)
    assert command.effort == [0.] * 21
    assert transport.snapshot()["hand_side"] == "left"
    transport.pause()
    transport._tick()
    assert len(messages) == 1


def test_custom_left_namespace_and_legacy_right_default(left_robot, monkeypatch):
    profile = HardwareProfile(left_robot)
    custom = HardwareTransport(profile, namespace="/lab/revo_left")
    assert custom.namespace == "/lab/revo_left" and custom.hand_side == "left"
    monkeypatch.setattr(ros_hardware, "_new_controller", lambda *_a, **_k: object())
    legacy = HardwareTransport(SimpleNamespace())
    assert legacy.namespace == "/revo3_right" and legacy.hand_side == "right"
