import importlib.util
import math
from pathlib import Path
import numpy as np
import pytest
from manus_revo3_retarget.migrate_calibration import migrate


def test_combines_offsets_and_preserves_unsaturated_range():
    old = dict(legacy_right_physical_thumb_joint_offset_deg=2.,
               legacy_right_physical_thumb_mcp_scale=1.3,
               legacy_right_physical_thumb_mcp_offset_deg=-3.,
               physical_right_thumb_MCP_joint_scale=.8,
               physical_right_thumb_MCP_joint_offset_deg=5.)
    new = migrate(old)
    for q in np.linspace(-1., 1., 101):
        expected = .8 * (1.3*q + math.radians(2.-3.)) + math.radians(5.)
        actual = new['physical_right_thumb_MCP_joint_scale']*q + math.radians(new['physical_right_thumb_MCP_joint_offset_deg'])
        assert actual == pytest.approx(expected)
    assert not any('legacy_right_physical_thumb' in k for k in new)


def test_normalization_migration_preserves_old_residual_coefficients():
    new = migrate(dict(thumb_ik_posture_weight=.1, thumb_ik_smooth_weight=.2,
                       legacy_left_physical_thumb_pip_ik_scale=1.4))
    assert math.sqrt(new['thumb_ik_posture_weight'])/math.radians(10) == pytest.approx(.1)
    assert math.sqrt(new['thumb_ik_smooth_weight'])/math.radians(10) == pytest.approx(.2)
    assert math.sqrt(new['left_thumb_ik_pip_weight'])/.01 == pytest.approx(.14)


def test_contact_fit_recovers_offset_and_rejects_degenerate_motion():
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('fit',root/'scripts/fit_fingertip_contact.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    offset=np.array([.018,-.007,.003]); center=np.array([.1,.2,.3]); poses=[]
    for i in range(12):
        a=i*.2;b=i*.13
        rx=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
        ry=np.array([[math.cos(b),0,math.sin(b)],[0,1,0],[-math.sin(b),0,math.cos(b)]])
        r=rx@ry;poses.append(dict(position_m=(center-r@offset).tolist(),rotation=r.tolist()))
    fit=module.fit_contact(poses)
    assert fit['offset_m'] == pytest.approx(offset,abs=1e-10)
    assert fit['rms_error_m'] < 1e-10
    with pytest.raises(ValueError,match='rotations'):
        module.fit_contact([poses[0]]*12)
