from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from revohuman_kinematics.core import HandModel, URDFKinematics, load_config

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'urdf/revohuman_dv1_kinematics.urdf'


def config():
    return deepcopy(load_config(ROOT / 'config/kinematics.yaml'))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_three_encoders_and_no_spread_contribution(side):
    model = HandModel(URDF, config(), side)
    for finger, start in zip(('index', 'middle', 'ring', 'little'), (0, 4, 8, 12)):
        for index in range(start, start + 3):
            raw = np.zeros(21)
            raw[index] = 90
            _, joints, _ = model.compute(raw)
            assert joints[finger + '_flexion'] == pytest.approx(1.4661 / 3)
        raw = np.zeros(21)
        raw[start + 3] = 60
        assert model.compute(raw)[1][finger + '_flexion'] == 0


@pytest.mark.parametrize('side', ['left', 'right'])
def test_tip_offset_rotates_with_dip(side):
    cfg = config()
    zero = HandModel(URDF, cfg, side)
    raw = np.zeros(21)
    a = zero.compute(raw)[2]['thumb_tip']
    raw[16] = 90
    assert np.allclose(a, zero.compute(raw)[2]['thumb_tip'])
    cfg['sides'][side]['tip_offsets_m']['thumb'] = [0, 0, 0.025]
    offset = HandModel(URDF, cfg, side)
    p0 = offset.compute(np.zeros(21))[2]['thumb_tip']
    p1 = offset.compute(raw)[2]['thumb_tip']
    assert np.linalg.norm(p0-a) == pytest.approx(0.025)
    assert np.linalg.norm(p1-a) == pytest.approx(0.025)
    assert not np.allclose(p0, p1)


def test_palm_local_mirror_and_zero_thumb_position():
    left = HandModel(URDF, config(), 'left').compute(np.zeros(21))[2]['thumb_tip']
    right = HandModel(URDF, config(), 'right').compute(np.zeros(21))[2]['thumb_tip']
    assert np.allclose(left, right * [1, -1, 1])
    # Independent sum of the 5 thumb origins at q=0 from the supplied DV1 file.
    assert np.allclose(left, [0.032795701026953, -0.08602696609497, 0.12645453339389])


def test_known_chain_nonzero_origin_rpy(tmp_path):
    path = tmp_path / 'test.urdf'
    path.write_text('''<robot name="test"><link name="base"/><link name="tip"/>
    <joint name="j" type="continuous"><parent link="base"/><child link="tip"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 1"/>
    </joint></robot>''')
    fk = URDFKinematics(path)
    pose = fk.evaluate(fk.chain('base', 'tip'), {'j': np.pi / 2})
    assert np.allclose(pose @ [1, 0, 0, 1], [0, 2, 3, 1])
    with pytest.raises(KeyError):
        fk.evaluate(fk.chain('base', 'tip'), {})


def test_calibration_wrap_and_bad_config():
    cfg = config()
    cfg['sides']['left']['zero_deg'][0] = 350
    cfg['sides']['left']['sign'][0] = -1
    cfg['sides']['left']['offset_deg'][0] = 5
    raw = np.zeros(21)
    raw[0] = 10
    assert HandModel(URDF, cfg, 'left').compute(raw)[0][0] == pytest.approx(np.deg2rad(-15))
    cfg['four_finger']['closed_deg'] = [0, 0, 0]
    with pytest.raises(ValueError):
        HandModel(URDF, cfg, 'left')


def test_gate_rejects_stale_duplicate_wrong_side_and_invalid():
    from revohuman_kinematics.node import FrameGate
    from revohuman_msgs.msg import RawFrame
    frame = RawFrame()
    frame.side = 'left'
    frame.header.stamp.sec = 10
    frame.encoder_valid_mask = 0x1fffff
    gate = FrameGate('left', 0.5)
    stamp, key = gate.check(frame, 10_100_000_000)
    gate.accept(stamp, key)
    frame.header.stamp.nanosec = 1
    with pytest.raises(ValueError, match='updated'):
        gate.check(frame, 10_100_000_000)
    frame.encoder_sequence = 1
    for attr, value in [('encoder_valid_mask', 0), ('encoder_offline_mask', 1),
                        ('encoder_reconnecting_mask', 1), ('side', 'right')]:
        previous = getattr(frame, attr)
        setattr(frame, attr, value)
        with pytest.raises(ValueError):
            gate.check(frame, 10_100_000_000)
        setattr(frame, attr, previous)
    with pytest.raises(ValueError):
        gate.check(frame, 11_000_000_000)
