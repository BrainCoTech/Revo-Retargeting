"""DV1 left-hand conversion, coordinate transform and calibration checks."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import JointState
import yaml

from revo2_hand_retarget.calibrate_dv1_fingers import endpoints
from revo2_hand_retarget.finger_flexion import CalibratedFingerFlexion, JOINTS
from hand_input_adapters.humandex_adapter_node import HumanDexHandAdapter
from hand_input_adapters.palm_transform import PalmTransform
from revohuman_kinematics.joint_fk import JointStateFK


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT.parent / 'revohuman_kinematics/urdf/revohuman_dv1_kinematics.urdf'


def test_direct_left_message_and_transform():
    canonical, joints, poses, warnings = [], [], [], []
    processor = JointStateFK(URDF, 'left', ema_alpha=.2)
    p = yaml.safe_load((ROOT / 'config/dv1_left.yaml').read_text())['/humandex_hand_adapter']['ros__parameters']
    alignment = PalmTransform([.01, -.02, .03], [0., 0., math.pi / 2])
    harness = SimpleNamespace(
        joint_fk=processor, sides=('left',), frames={'left': 'hand_retarget_left'},
        input_frames={'left': 'left_palm_link'}, palm_transforms={'left': alignment},
        fk_joint_pub=SimpleNamespace(publish=joints.append), fk_pose_pub=SimpleNamespace(publish=poses.append),
        output_publishers={'left': SimpleNamespace(publish=canonical.append)}, published={'left': 0},
        _present_sides=HumanDexHandAdapter._present_sides,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warning=lambda text, **kw: warnings.append(text)))
    harness._publish_pair = lambda j, p: HumanDexHandAdapter._publish_pair(harness, j, p)
    msg = JointState()
    msg.header.stamp.sec = 1
    msg.header.frame_id = 'left_palm_link'
    msg.name = list(reversed(processor.joint_names))
    msg.position = [math.pi / 4] * 21
    HumanDexHandAdapter._publish_from_joint_state(harness, msg)
    assert len(canonical) == len(joints) == len(poses) == 1
    out = canonical[0]
    assert out.side == out.LEFT and len(out.joint_names) == 21 and len(out.landmarks_m) == 5
    assert out.header.stamp == joints[0].header.stamp == poses[0].header.stamp == msg.header.stamp
    values = dict(zip(out.joint_names, out.joint_positions_rad))
    assert 'index_flexion' not in values
    assert values['thumb_dip'] == pytest.approx(math.pi / 4)
    native = poses[0].poses[4].position
    result = out.landmarks_m[4]
    assert (result.x, result.y, result.z) == pytest.approx(alignment.apply((native.x, native.y, native.z)))
    # Repeated frames must not keep refreshing canonical output/robot targets.
    HumanDexHandAdapter._publish_from_joint_state(harness, msg)
    assert len(canonical) == 1 and warnings


def test_endpoint_capture_and_wrapped_ranges():
    rows = [[math.radians(179 if i % 2 else -179)] * 12 for i in range(40)]
    assert np.allclose(np.abs(endpoints(rows)), math.pi)
    with pytest.raises(ValueError, match='at least 30'):
        endpoints(rows[:2])
    with pytest.raises(ValueError, match='moved'):
        endpoints([[float(i % 2)] * 12 for i in range(40)])
    opened, closed = [math.radians(170)] * 12, [math.radians(-170)] * 12
    mapping = CalibratedFingerFlexion(opened, closed, [1.] * 3, 1.4661, True)
    assert mapping.compute(dict.fromkeys(JOINTS, -math.pi))['index_flexion'] == pytest.approx(1.4661 / 2)


def test_config_requires_separate_left_endpoints():
    p = yaml.safe_load((ROOT.parent / 'revo2_hand_retarget/config/flexion_dv1_left.yaml').read_text())
    assert p['hand_mode'] == 'left' and p['four_finger_calibration_label'].startswith('PROVISIONAL')
    assert not any(k.startswith('right_') for k in p)


@pytest.mark.parametrize("context_active", [False, True])
def test_shutdown_wait_set_error_only_ignored_after_shutdown(monkeypatch, context_active):
    from hand_input_adapters import humandex_adapter_node as module
    destroyed = []
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: context_active)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(module, 'HumanDexHandAdapter', lambda: SimpleNamespace(
        destroy_node=lambda: destroyed.append(True)))
    def stopped_spin(node):
        raise module.rclpy_implementation.RCLError('context is not valid')
    monkeypatch.setattr(module.rclpy, 'spin', stopped_spin)
    if context_active:
        with pytest.raises(module.rclpy_implementation.RCLError):
            module.main()
    else:
        assert module.main() == 0
    assert destroyed == [True]
