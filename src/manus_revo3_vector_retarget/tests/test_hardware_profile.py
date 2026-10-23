"""Current-description mapping, command envelopes and model/profile consistency."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from retarget_v2.hardware_profile import CommandConfig, HardwareProfile
from retarget_v2.model import RobotModel


@pytest.fixture(scope="module")
def robot():
    with RobotModel() as model:
        yield model


def document():
    return yaml.safe_load((Path(__file__).resolve().parents[1] /
                           "config/right_hardware_may3.yaml").read_text())


def save(tmp_path, data):
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_named_mapping_and_feedback_excursions(robot):
    profile = HardwareProfile(robot)
    q = profile.model_lower + .35 * (profile.model_upper - profile.model_lower)
    hardware = profile.model_to_hardware(q)
    expected = np.array([q[robot.joint_names.index(name)] for name in profile.names])
    np.testing.assert_array_equal(hardware, expected)
    np.testing.assert_array_equal(profile.feedback_to_model(hardware), q)
    feedback = profile.upper + .1
    raw = profile.feedback_to_model(feedback)
    assert np.any(raw > profile.model_upper)
    with pytest.raises(ValueError, match="outside"):
        profile.model_to_hardware(raw)


def test_model_order_is_not_assumed(robot):
    profile = HardwareProfile(robot)
    order = np.random.default_rng(93).permutation(21)
    shuffled = SimpleNamespace(joint_names=tuple(robot.joint_names[i] for i in order),
                               lower=robot.lower[order], upper=robot.upper[order],
                               urdf_path=robot.urdf_path, model_version="may3")
    actual = HardwareProfile(shuffled)
    q = profile.model_lower + .4 * (profile.model_upper - profile.model_lower)
    np.testing.assert_array_equal(actual.model_to_hardware(q[order]), profile.model_to_hardware(q))
    np.testing.assert_array_equal(actual.model_lower, profile.model_lower[order])
    values = np.arange(21, dtype=float)
    np.testing.assert_array_equal(profile.ordered([profile.names[i] for i in order], values[order]), values)
    for names in ([], list(profile.names[:-1]), [profile.names[0]] * 21):
        with pytest.raises(ValueError):
            profile.ordered(names, values)


def test_hardware_envelope_intersects_current_urdf_without_mutating_it(robot):
    profile = HardwareProfile(robot)
    before_lower, before_upper = robot.lower.copy(), robot.upper.copy()
    bounds = document()["hardware_channel_bounds"]
    for i, name in enumerate(profile.names):
        mi = robot.joint_names.index(name)
        assert profile.lower[i] == max(robot.lower[mi], bounds[name]["lower_rad"])
        assert profile.upper[i] == min(robot.upper[mi], bounds[name]["upper_rad"])
    np.testing.assert_array_equal(robot.lower, before_lower)
    np.testing.assert_array_equal(robot.upper, before_upper)
    for q in (profile.model_lower, profile.model_upper):
        np.testing.assert_allclose(profile.feedback_to_model(profile.model_to_hardware(q)), q, atol=1e-15)
    with pytest.raises(ValueError, match="outside"):
        profile.model_to_hardware(robot.upper + .01)


def test_hash_binds_profile_to_current_description(robot, tmp_path):
    altered = SimpleNamespace(joint_names=robot.joint_names, lower=robot.lower,
                              upper=robot.upper, model_version="may3",
                              urdf_path=tmp_path / "changed.urdf")
    altered.urdf_path.write_bytes(robot.urdf_path.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="SHA256"):
        HardwareProfile(altered)
    changed = document()
    changed["urdf_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA256"):
        HardwareProfile(robot, save(tmp_path, changed))


def test_current_profile_defaults_and_independent_motion_limits(robot):
    profile = HardwareProfile(robot)
    configured = document()["command"]
    assert profile.command.publish_hz == 200
    assert profile.command.startup_duration_s == 2
    assert profile.command.max_velocity_rad_s == configured["max_velocity_rad_s"]
    assert profile.command.max_acceleration_rad_s2 == configured["max_acceleration_rad_s2"]
    np.testing.assert_array_equal(profile.kp, np.array([.7 if "MPR" in name else .2 for name in profile.names]))
    np.testing.assert_array_equal(profile.kd, np.array([.08 if "MPR" in name else .02 for name in profile.names]))
    tuned = HardwareProfile(robot, max_velocity_rad_s=2, max_acceleration_rad_s2=3)
    assert tuned.command.max_velocity_rad_s == 2 and tuned.command.max_acceleration_rad_s2 == 3
    scaled = HardwareProfile(robot, motion_scale=.5)
    assert scaled.command.max_velocity_rad_s == configured["max_velocity_rad_s"] * .5
    assert scaled.command.max_acceleration_rad_s2 == configured["max_acceleration_rad_s2"] * .25
    with pytest.raises(ValueError, match="cannot be combined"):
        HardwareProfile(robot, motion_scale=1, max_velocity_rad_s=1)


@pytest.mark.parametrize("hand_side", ["left", "right"])
@pytest.mark.parametrize("velocity, acceleration", [(100., 100.), (1000., 2000.)])
def test_motion_limits_above_old_ceilings_are_preserved(tmp_path, hand_side, velocity, acceleration):
    with RobotModel(hand_side=hand_side) as robot:
        data = yaml.safe_load(HardwareProfile(robot).path.read_text())
        data["command"].update(max_velocity_rad_s=velocity, max_acceleration_rad_s2=acceleration)
        path = save(tmp_path, data)
        configured = HardwareProfile(robot, path)
        assert configured.command.max_velocity_rad_s == velocity
        assert configured.command.max_acceleration_rad_s2 == acceleration
        # Independent overrides can raise or lower an already high YAML limit.
        overridden = HardwareProfile(robot, path, max_velocity_rad_s=velocity * 2,
                                     max_acceleration_rad_s2=acceleration / 2)
        assert overridden.command.max_velocity_rad_s == velocity * 2
        assert overridden.command.max_acceleration_rad_s2 == acceleration / 2


@pytest.mark.parametrize("source", ["yaml", "override"])
@pytest.mark.parametrize("option", ["max_velocity_rad_s", "max_acceleration_rad_s2"])
@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), True, "1"])
def test_invalid_motion_limits_are_rejected(robot, tmp_path, source, option, value):
    if source == "yaml":
        data = document()
        data["command"][option] = value
        path = save(tmp_path, data)
        with pytest.raises(ValueError):
            HardwareProfile(robot, path)
        return
    with pytest.raises(ValueError):
        HardwareProfile(robot, **{option: value})


@pytest.mark.parametrize("mutation", [
    lambda p: p.update(model_version="legacy"),
    lambda p: p.update(unknown=True),
    lambda p: p["command"].update(publish_hz=True),
    lambda p: p["gains"].update(default_kp=-1),
    lambda p: p["output_calibration"]["right"].pop("index_MPR_joint"),
    lambda p: p["output_calibration"]["right"]["index_MPR_joint"].update(scale=.5),
    lambda p: p["hardware_channel_bounds"]["right_index_MPR_joint"].update(lower_rad=-.27),
    lambda p: p.update(feedback_lower_stop_overrides_deg={"right_index_MCP_joint": 5}),
])
def test_invalid_mapping_or_expanded_envelope_is_rejected(robot, tmp_path, mutation):
    data = document()
    mutation(data)
    with pytest.raises(ValueError):
        HardwareProfile(robot, save(tmp_path, data))


def test_legacy_model_is_not_shipped_and_command_config_validates():
    with pytest.raises(ValueError, match="workspace revo3_description"):
        RobotModel(model_version="legacy")
    with pytest.raises(ValueError):
        CommandConfig(max_timer_gap_s=0)
