"""Direct JointState FK contract and independent SDK/MuJoCo comparison."""

import importlib.util
import os
from pathlib import Path

import numpy as np
import pytest

from revohuman_kinematics.core import rotation
from revohuman_kinematics.joint_fk import JointStateFK, quaternion_xyzw


LOCAL_URDF = Path(__file__).resolve().parents[1] / 'urdf/revohuman_dv1_kinematics.urdf'


def test_names_filter_wrap_and_state_validation():
    fk = JointStateFK(LOCAL_URDF, 'left', ema_alpha=0.2)
    names = list(reversed(fk.joint_names))
    q, _ = fk.compute(names, [0.] * 21, 1_000_000_000, 1_000_000_001, 'left_palm_link')
    assert np.allclose(q, 0)
    q, _ = fk.compute(names, [1.] * 21, 1_005_000_000, 1_005_000_001, 'left_palm_link')
    assert np.allclose(q, .2)
    for bad_names, positions, stamp, now, frame in [
        (names[:-1], [1.] * 20, 1_010_000_000, 1_010_000_001, 'left_palm_link'),
        ([names[0]] * 21, [1.] * 21, 1_010_000_000, 1_010_000_001, 'left_palm_link'),
        (names, [np.nan] * 21, 1_010_000_000, 1_010_000_001, 'left_palm_link'),
        (names, [1.] * 21, 1_005_000_000, 1_010_000_001, 'left_palm_link'),
        (names, [1.] * 21, 1_010_000_000, 2_000_000_000, 'left_palm_link'),
        (names, [1.] * 21, 1_010_000_000, 1_009_000_000, 'left_palm_link'),
        (names, [1.] * 21, 1_010_000_000, 1_010_000_001, 'right_palm_link'),
    ]:
        with pytest.raises(ValueError):
            fk.compute(bad_names, positions, stamp, now, frame)
        assert fk.last_stamp == 1_005_000_000
        assert np.allclose(fk.filtered, .2)
    # Reconnection gap resets the filter; wrap crossing follows the short arc.
    q, _ = fk.compute(names, [np.deg2rad(179)] * 21, 2_000_000_000, 2_000_000_001, 'left_palm_link')
    q, _ = fk.compute(names, [np.deg2rad(-179)] * 21, 2_005_000_000, 2_005_000_001, 'left_palm_link')
    assert np.allclose(q, np.deg2rad(179.4))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_dip_origin_and_local_tip_offset(side):
    raw = JointStateFK(LOCAL_URDF, side)
    offsets = [0.] * 12 + [0.01, 0.02, 0.03]
    shifted = JointStateFK(LOCAL_URDF, side, offsets)
    for step, angle in enumerate([0., .8]):
        q = np.zeros(21)
        q[16] = angle
        stamp = 1_000_000_000 + step * 10_000_000
        _, a = raw.compute(raw.joint_names, q, stamp, stamp, raw.base_link)
        _, b = shifted.compute(shifted.joint_names, q, stamp, stamp, shifted.base_link)
        assert np.allclose(b[-1][:3, 3] - a[-1][:3, 3], a[-1][:3, :3] @ offsets[-3:])
        if step == 0:
            origin = a[-1][:3, 3].copy()
        else:
            assert np.allclose(origin, a[-1][:3, 3])


@pytest.mark.parametrize('axis', [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
def test_quaternion_half_turns(axis):
    q = quaternion_xyzw(rotation(axis, np.pi))
    assert np.allclose(np.abs(q), [*axis, 0], atol=1e-9)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_sdk_mujoco_matches_fk(side, tmp_path):
    sdk_path = os.environ.get('DV1_SDK_PATH')
    if not sdk_path:
        pytest.skip('Set DV1_SDK_PATH to run the SDK MuJoCo comparison')
    import mujoco
    sdk = Path(sdk_path)
    spec = importlib.util.spec_from_file_location('sdk_fk_viewer', sdk / 'tools/fk_viewer.py')
    viewer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(viewer)
    urdf = viewer.build_single_hand_urdf(sdk / 'description', side, tmp_path / f'{side}.urdf')
    model = mujoco.MjModel.from_xml_path(str(urdf))
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    data = mujoco.MjData(model)
    fk = JointStateFK(sdk / 'description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf', side)
    rng = np.random.default_rng(42)
    for step, joints in enumerate([np.zeros(21), *rng.uniform(-1., 1.5, (20, 21))]):
        stamp = 1_000_000_000 + step * 10_000_000
        q, poses = fk.compute(fk.joint_names, joints, stamp, stamp, fk.base_link)
        for name, value in zip(fk.joint_names, q):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint_id]] = value
        mujoco.mj_forward(model, data)
        for finger, pose in zip(('index', 'middle', 'ring', 'little', 'thumb'), poses):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{side}_{finger}_DIP_Link')
            assert np.allclose(pose[:3, 3], data.xpos[body_id], atol=1e-9, rtol=0)
            assert np.allclose(pose[:3, :3], data.xmat[body_id].reshape(3, 3), atol=1e-9, rtol=0)
            expected = data.xquat[body_id][[1, 2, 3, 0]]
            assert abs(np.dot(expected, quaternion_xyzw(pose[:3, :3]))) == pytest.approx(1., abs=1e-9)
