"""SDK geometry contracts using synthetic, independently specified hand poses.

No user capture is embedded and no legacy retargeting implementation is used.
"""

from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from retarget_v2.input import Calibration, FINGERS, chains, decode, palm_coordinates
from retarget_v2.solver import Retargeter, SolverConfig


def sdk_hand(pip_bend_degrees=30.0):
    """An SDK-shaped right hand with a known natural PIP bend.

    X runs toward pinky, Z toward fingertips, and flexion runs toward -Y.
    Positions and lengths are constructed here, not copied from real data.
    """
    nodes = []

    def add(finger, part, xyz, parent):
        node_id = len(nodes)
        nodes.append(dict(node_id=node_id, parent_node_id=parent,
                          chain_type=finger, joint_type=part,
                          pose=dict(position=dict(zip("xyz", map(float, xyz))),
                                    orientation=dict(x=0., y=0., z=0., w=1.))))
        return node_id

    add("Hand", "Invalid", [0., 0., 0.], 0)
    thumb_points = np.array([[-.030, -.010, .025], [-.060, -.015, .059],
                             [-.080, -.030, .084], [-.094, -.049, .099]])
    parent = 0
    for part, point in zip(("Metacarpal", "Proximal", "Distal", "Tip"), thumb_points):
        parent = add("Thumb", part, point, parent)
    for finger, x in zip(("Index", "Middle", "Ring", "Pinky"), (-.030, -.010, .010, .030)):
        parent = add(finger, "Metacarpal", [x, 0., .020], 0)
        point = np.array([x, 0., .080])
        parent = add(finger, "Proximal", point, parent)
        for part, length, flex in zip(("Intermediate", "Distal", "Tip"),
                                      (.045, .027, .022),
                                      (0., pip_bend_degrees, pip_bend_degrees + 15.)):
            radians = np.deg2rad(flex)
            point = point + length * np.array([0., -np.sin(radians), np.cos(radians)])
            parent = add(finger, part, point, parent)
    return dict(side="Right", glove_id=12345, raw_node_count=25, raw_nodes=nodes,
                raw_sensor_orientation=dict(x=0., y=0., z=0., w=1.))


def transformed(message, rotation, translation):
    result = deepcopy(message)
    for node in result["raw_nodes"]:
        xyz = np.array([node["pose"]["position"][axis] for axis in "xyz"])
        point = rotation @ xyz + translation
        node["pose"]["position"] = dict(zip("xyz", point))
    return result


def calibration(message):
    return Calibration.from_frames([deepcopy(message) for _ in range(30)])


def target_builder(cal):
    """A kinematics-free robot descriptor suffices to exercise target generation."""
    robot = SimpleNamespace(
        joint_names=tuple(f"joint_{i}" for i in range(21)),
        lower=np.full(21, -1.), upper=np.full(21, 2.),
        point_names=tuple(f"{f}_{part}" for f in FINGERS for part in ("root", "pip", "dip", "tip")),
        finger_lengths=np.tile([.041, .026, .023], (5, 1)),
        palm_basis=Rotation.from_rotvec([.2, -.1, .3]).as_matrix(),
    )
    rest = chains(palm_coordinates(decode(sdk_hand(pip_bend_degrees=0.))))
    rest = (rest @ robot.palm_basis.T).reshape(-1, 3)
    robot.points = lambda q: rest.copy()
    return Retargeter(robot, cal, SolverConfig())


def assert_feature_equal(left, right):
    assert left.keys() == right.keys()
    for name in left:
        np.testing.assert_allclose(left[name], right[name], atol=2e-13, rtol=2e-12,
                                   err_msg=f"feature {name} changed")


def test_sdk_semantics_survive_arbitrary_ids_and_wire_order():
    source = sdk_hand()
    changed = deepcopy(source)
    remap = {n["node_id"]: 100 + 7 * n["node_id"] for n in changed["raw_nodes"]}
    for node in changed["raw_nodes"]:
        node["parent_node_id"] = remap[node["parent_node_id"]]
        node["node_id"] = remap[node["node_id"]]
        if node["chain_type"] == "Pinky":
            node["chain_type"] = "FingerLittle"
    np.random.default_rng(8105).shuffle(changed["raw_nodes"])
    decoded = decode(changed)
    assert len(decoded) == 25
    assert "thumb_Intermediate" not in decoded
    np.testing.assert_array_equal(chains(decoded), chains(decode(source)))


def publisher_hand():
    frame = sdk_hand()
    labels = {"Metacarpal": "MCP", "Proximal": "PIP", "Intermediate": "IP",
              "Distal": "DIP", "Tip": "TIP"}
    for node in frame["raw_nodes"]:
        node["joint_type"] = labels.get(node["joint_type"], node["joint_type"])
    return frame


def test_current_publisher_labels_preserve_geometry_calibration_and_targets():
    source = sdk_hand()
    frame = publisher_hand()
    original = deepcopy(frame)
    np.random.default_rng(12).shuffle(frame["raw_nodes"])
    np.testing.assert_array_equal(chains(decode(frame)), chains(decode(source)))
    cal = calibration(frame)
    np.testing.assert_array_equal(cal.lengths, calibration(source).lengths)
    assert_feature_equal(cal.features(frame), calibration(source).features(source))
    assert_feature_equal(target_builder(cal).targets(frame),
                         target_builder(calibration(source)).targets(source))
    assert sorted(frame["raw_nodes"], key=lambda n: n["node_id"]) == original["raw_nodes"]
    from retarget_v2.ros_input import LatestMessageBuffer
    buffer = LatestMessageBuffer()
    assert buffer.put(frame, 1.)
    assert buffer.status()["accepted"] == 1


@pytest.mark.parametrize("mutation, error", [
    (lambda m: m["raw_nodes"][7].update(parent_node_id=0), "Wrong parent"),
    (lambda m: m["raw_nodes"][7].update(joint_type="UNKNOWN"), "labels do not match"),
    (lambda m: m["raw_nodes"][7].update(joint_type="PIP"), "Duplicate semantic"),
])
def test_current_publisher_labels_still_validate_topology(mutation, error):
    frame = publisher_hand()
    mutation(frame)
    with pytest.raises(ValueError, match=error):
        decode(frame)


@pytest.mark.parametrize("mutation, message", [
    (lambda m: m.update(side="Left"), "right hand"),
    (lambda m: m.update(raw_node_count=24), "25-node"),
    (lambda m: m["raw_nodes"][7].update(parent_node_id=11), "Wrong parent"),
    (lambda m: m["raw_nodes"][7].update(node_id=6), "Duplicate node IDs"),
    (lambda m: m["raw_nodes"][7].update(joint_type="Distal"), "Duplicate semantic"),
    (lambda m: m["raw_nodes"][4]["pose"]["position"].update(x=np.nan), "Non-finite"),
])
def test_invalid_sdk_topology_or_values_are_rejected(mutation, message):
    frame = sdk_hand()
    mutation(frame)
    with pytest.raises(ValueError, match=message):
        decode(frame)


def test_palm_coordinates_features_and_targets_are_rigid_motion_invariant():
    source = sdk_hand()
    cal = calibration(source)
    retargeter = target_builder(cal)
    reference_palm = palm_coordinates(decode(source))
    reference_features = cal.features(source)
    reference_targets = retargeter.targets(source)
    rng = np.random.default_rng(9183)
    for _ in range(12):
        rotation = Rotation.random(random_state=rng).as_matrix()
        assert np.linalg.det(rotation) == pytest.approx(1.)
        moved = transformed(source, rotation, rng.uniform(-2., 2., 3))
        for key, point in palm_coordinates(decode(moved)).items():
            np.testing.assert_allclose(point, reference_palm[key], atol=2e-14)
        assert_feature_equal(cal.features(moved), reference_features)
        assert_feature_equal(retargeter.targets(moved), reference_targets)
        np.testing.assert_allclose(calibration(moved).lengths, cal.lengths, atol=1e-14)


@pytest.mark.parametrize("scale", [.001, 1000.])
def test_millimetre_metre_unit_mistakes_are_rejected(scale):
    source = sdk_hand()
    wrong_units = transformed(source, np.eye(3) * scale, np.zeros(3))
    with pytest.raises(ValueError, match="metre units"):
        calibration(wrong_units)
    with pytest.raises(ValueError, match="Bone length changed"):
        calibration(source).features(wrong_units)


def test_calibration_rejects_bone_length_jump():
    source = sdk_hand()
    frames = [deepcopy(source) for _ in range(30)]
    frames[-1] = transformed(source, np.eye(3) * 1.2, np.zeros(3))
    with pytest.raises(ValueError, match="inconsistency"):
        Calibration.from_frames(frames)


def test_wrist_imu_is_not_part_of_local_shape_or_robot_targets():
    source = sdk_hand()
    cal = calibration(source)
    retargeter = target_builder(cal)
    reference = retargeter.targets(source)
    changed = deepcopy(source)
    changed["raw_sensor_orientation"] = dict(zip("xyzw", Rotation.from_euler("xyz", [90, -42, 177], degrees=True).as_quat()))
    assert_feature_equal(cal.features(changed), cal.features(source))
    assert_feature_equal(retargeter.targets(changed), reference)
    del changed["raw_sensor_orientation"]
    assert_feature_equal(retargeter.targets(changed), reference)


def test_calibration_preserves_natural_bend_instead_of_subtracting_it():
    source = sdk_hand(pip_bend_degrees=30.)
    cal = calibration(source)
    retargeter = target_builder(cal)
    directions = cal.features(source)["directions"][1:]
    bends = np.rad2deg(np.arccos(np.sum(directions[:, 0] * directions[:, 1], axis=1)))
    np.testing.assert_allclose(bends, 30., atol=1e-12)
    # A different reference pose with the same physical lengths must not change
    # the current gesture's target. Only morphology is calibrated.
    straight_reference = calibration(sdk_hand(pip_bend_degrees=0.))
    assert_feature_equal(retargeter.targets(source), target_builder(straight_reference).targets(source))


def test_normal_sign_is_explicit_reflection_only():
    points = decode(sdk_hand())
    source = palm_coordinates(points, normal_sign=1.)
    reflected = palm_coordinates(points, normal_sign=-1.)
    for name in source:
        np.testing.assert_allclose(reflected[name], source[name] * [1., 1., -1.], atol=1e-15)
    # SDK flexion toward -Y becomes positive target palm-normal movement.
    assert reflected["middle_Distal"][2] > reflected["middle_Intermediate"][2]


def test_degenerate_palm_frame_is_rejected():
    points = decode(sdk_hand())
    points["index_Proximal"] = points["pinky_Proximal"].copy()
    with pytest.raises(ValueError, match="Degenerate"):
        palm_coordinates(points)


@pytest.mark.parametrize("sign", [0., .5, 2., np.nan, np.inf])
def test_normal_sign_cannot_scale_or_flatten_the_palm(sign):
    with pytest.raises(ValueError, match="normal_sign"):
        palm_coordinates(decode(sdk_hand()), normal_sign=sign)
    with pytest.raises(ValueError, match="normal_sign"):
        Calibration(np.full((5, 3), .03), normal_sign=sign)


def test_coincident_tips_keep_nonzero_gap_and_last_valid_direction(monkeypatch):
    source = sdk_hand()
    cal = calibration(source)
    retargeter = target_builder(cal)
    first = retargeter.targets(source)
    original = cal.features(source)
    original["tip_vectors"][0] = 0.
    original["tip_distances"][0] = 0.
    monkeypatch.setattr(cal, "features", lambda message: deepcopy(original))
    at_contact = retargeter.targets(source)
    previous_direction = first["pair_target"][0] / np.linalg.norm(first["pair_target"][0])
    np.testing.assert_allclose(at_contact["pair_target"][0], previous_direction * retargeter.config.pinch_gap_m,
                               atol=1e-15)
    assert at_contact["pinch_blend"][0] == 1.
