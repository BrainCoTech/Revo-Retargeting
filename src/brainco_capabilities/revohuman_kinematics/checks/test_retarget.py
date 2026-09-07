"""Optional downstream integration; run in the retarget_revo2 environment."""
from pathlib import Path
import numpy as np
import pytest

from revohuman_kinematics.core import HandModel, load_config


@pytest.mark.parametrize('side', ['left', 'right'])
def test_fk_to_six_revo2_targets(side):
    pytest.importorskip('mujoco')
    pytest.importorskip('revo2_hand_retarget')
    from revo2_hand_retarget.retargeters.pose_thumb_retargeter import PoseThumbRetargeter
    root = Path(__file__).resolve().parents[1]
    model = HandModel(root / 'urdf/revohuman_dv1_kinematics.urdf',
                      load_config(root / 'config/kinematics.yaml'), side)
    import revo2_hand_retarget
    target_root = Path(revo2_hand_retarget.__file__).parent
    solver = PoseThumbRetargeter(target_root / 'brainco_hand/brainco.yml', enabled_sides=(side,))
    for angle in (0, 30, 60):
        raw = np.zeros(21)
        for start in (0, 4, 8, 12):
            raw[start:start + 3] = angle
        raw[17:21] = angle / 3
        _, joints, points = model.compute(raw)
        tips = [points[f + '_tip'].tolist() for f in ('thumb', 'index', 'middle', 'ring', 'little')]
        target = solver.retarget_side(side, tips, joint_positions=joints)
        assert target.shape == (6,)
        assert np.isfinite(target).all()
        assert np.allclose(target[2:], angle / 90 * 1.4661)
