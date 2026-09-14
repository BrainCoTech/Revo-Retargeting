"""Geometry records and optional sibling HumanDex FK integration, without hardware."""
import importlib.util
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
FINGERS = ('index', 'middle', 'ring', 'little', 'thumb')


def record(name):
    return yaml.safe_load((PACKAGE / 'config' / name).read_text())


def test_glove_offsets_are_mirrored_and_pad_points_are_separate():
    glove = record('humandex_fingertips.yaml')
    pad = record('revo3_pad_contacts.yaml')
    assert glove['finger_order'] == list(FINGERS)
    assert pad['enabled_in_ik'] is False
    for finger in FINGERS:
        right = glove['hands']['right'][finger]
        left = glove['hands']['left'][finger]
        np.testing.assert_array_equal(left['offset_m'], np.array(right['offset_m']) * [1, -1, 1])
        for side in ('left', 'right'):
            assert glove['hands'][side][finger]['parent_link'] == f'{side}_{finger}_DIP_Link'
            assert pad['hands'][side][finger]['parent_link'] == f'{side}_{finger}_tip_Link'
            assert np.isfinite(glove['hands'][side][finger]['offset_m']).all()


@pytest.fixture(scope='module')
def upstream():
    root = Path(os.environ.get('HUMANDEX_ROOT', WORKSPACE.parent / 'BrainCo-HumanDex')) / 'src/humandex_urdf'
    if not root.is_dir():
        pytest.skip('Set HUMANDEX_ROOT to test the sibling HumanDex publisher')
    pytest.importorskip('rclpy')
    spec = importlib.util.spec_from_file_location('geometry_test_mux', root / 'brainco_mcu/joint_state_mux.py')
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    data = yaml.safe_load((root / 'config/config.yaml').read_text())
    params = data['joint_state_mux']['ros__parameters'].copy()
    params['urdf_path'] = str(root / 'urdf/HumanDex_bimanual.urdf')
    return mod, params


def test_upstream_config_matches_geometry_record(upstream):
    _, params = upstream
    glove = record('humandex_fingertips.yaml')
    expected_names = []
    expected_offsets = []
    for side in ('left', 'right'):
        for finger in FINGERS:
            entry = glove['hands'][side][finger]
            expected_names.append(entry['parent_link'])
            expected_offsets.extend(entry['offset_m'])
    assert params['eef_tip_links'] == expected_names
    np.testing.assert_array_equal(params['eef_tip_offsets_m'], expected_offsets)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('finger', FINGERS)
def test_actual_publisher_applies_offset_once_and_dip_rotation_moves_tip(upstream, side, finger):
    mod, params = upstream
    cls = mod.JointStateMuxNode
    sent = []
    harness = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=lambda: SimpleNamespace(warn=lambda msg: pytest.fail(msg)),
        _tip_links=params['eef_tip_links'],
        _base_links={s: params[f'{s}_base_link'] for s in ('left', 'right')},
        _joint_by_child={}, _fk_chains={},
        _hand_from_link=cls._hand_from_link,
        _eef_pose_pub=SimpleNamespace(publish=sent.append),
    )
    harness._build_chain = lambda base, tip: cls._build_chain(harness, base, tip)
    cls._load_fk_model(harness)
    offsets = cls._load_tip_offsets(harness)
    poses = []
    for angle in (0., .6):
        msg = mod.JointState()
        msg.header.stamp.sec = 123
        msg.name = [f'{side}_{finger}_DIP_joint']
        msg.position = [angle]
        harness._tip_offsets = offsets
        cls._publish_eef_pose(harness, msg, {side})
        actual = sent[-1].poses[FINGERS.index(finger)]
        harness._tip_offsets = {name: np.zeros(3) for name in offsets}
        cls._publish_eef_pose(harness, msg, {side})
        origin_pose = sent[-1].poses[FINGERS.index(finger)]
        q = actual.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        xyz = lambda p: np.array([p.position.x, p.position.y, p.position.z])
        np.testing.assert_allclose(xyz(actual), xyz(origin_pose) + rotation @ offsets[f'{side}_{finger}_DIP_Link'], atol=1e-10)
        assert sent[-1].header.stamp.sec == 123
        poses.append((xyz(actual), xyz(origin_pose)))
    np.testing.assert_allclose(poses[0][1], poses[1][1], atol=1e-10)
    assert np.linalg.norm(poses[1][0] - poses[0][0]) > .001
