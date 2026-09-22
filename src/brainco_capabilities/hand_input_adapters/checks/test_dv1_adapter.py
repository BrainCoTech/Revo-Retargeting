"""DV1 message conversion, coordinate transform and calibration checks."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import JointState
import yaml

from revo2_hand_retarget.calibrate_dv1_fingers import endpoints
from revo2_hand_retarget.finger_flexion import CalibratedFingerFlexion, JOINTS
from hand_input_adapters.dv1_joint_state import DV1JointStateInput
from hand_input_adapters.humandex_adapter_node import HumanDexHandAdapter
from hand_input_adapters.palm_transform import PalmTransform
from revohuman_kinematics.joint_fk import JointStateFK


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT.parent / 'revohuman_kinematics/urdf/revohuman_dv1_kinematics.urdf'
SDK_NAMES = (
    'index_DIP_joint', 'index_PIP_joint', 'index_MCP_joint', 'index_MPR_joint',
    'middle_DIP_joint', 'middle_PIP_joint', 'middle_MCP_joint', 'middle_MPR_joint',
    'ring_DIP_joint', 'ring_PIP_joint', 'ring_MCP_joint', 'ring_MPR_joint',
    'little_DIP_joint', 'little_PIP_joint', 'little_MCP_joint', 'little_MPR_joint',
    'thumb_DIP_joint', 'thumb_PIP_joint', 'thumb_MCP_joint', 'thumb_CMR_joint', 'thumb_CMP_joint',
)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', ['sdk_single', 'sdk_pair', 'legacy'])
def test_direct_message_and_transform(side, layout):
    canonical, joints, poses, warnings = [], [], [], []
    processor = JointStateFK(URDF, side, ema_alpha=.2)
    alignment = PalmTransform([.01, -.02, .03], [0., 0., math.pi / 2])
    harness = SimpleNamespace(
        joint_fk=processor, joint_input=DV1JointStateInput(side, layout), sides=(side,),
        frames={side: f'hand_retarget_{side}'}, input_frames={side: f'{side}_palm_link'},
        palm_transforms={side: alignment},
        fk_joint_pub=SimpleNamespace(publish=joints.append), fk_pose_pub=SimpleNamespace(publish=poses.append),
        output_publishers={side: SimpleNamespace(publish=canonical.append)}, published={side: 0},
        _present_sides=HumanDexHandAdapter._present_sides,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None, warning=lambda text, **kw: warnings.append(text)))
    harness._publish_pair = lambda j, p: HumanDexHandAdapter._publish_pair(harness, j, p)
    msg = JointState()
    msg.header.stamp.sec = 1
    selected_values = [.1 + index / 100 for index in range(21)]
    if layout == 'sdk_single':
        msg.header.frame_id = f'revohuman_{side}'
        names, positions = [f'{side}_{name}' for name in SDK_NAMES], selected_values
    elif layout == 'sdk_pair':
        msg.header.frame_id = 'revohuman_pair'
        names = [f'{hand}_{name}' for hand in ('left', 'right') for name in SDK_NAMES]
        other_values = [-.7] * 21
        positions = selected_values + other_values if side == 'left' else other_values + selected_values
    else:
        msg.header.frame_id = f'{side}_palm_link'
        names, positions = [f'{side}_{name}' for name in SDK_NAMES], selected_values
    msg.name, msg.position = list(reversed(names)), list(reversed(positions))
    HumanDexHandAdapter._publish_from_joint_state(harness, msg)
    assert len(canonical) == len(joints) == len(poses) == 1
    out = canonical[0]
    assert out.side == (out.LEFT if side == 'left' else out.RIGHT)
    assert len(out.joint_names) == 21 and len(out.landmarks_m) == 5
    assert out.header.stamp == joints[0].header.stamp == poses[0].header.stamp == msg.header.stamp
    assert out.header.frame_id == f'hand_retarget_{side}'
    assert joints[0].header.frame_id == poses[0].header.frame_id == f'{side}_palm_link'
    assert list(joints[0].name) == [f'{side}_{name}' for name in SDK_NAMES]
    assert list(joints[0].position) == pytest.approx(selected_values)
    values = dict(zip(out.joint_names, out.joint_positions_rad))
    assert 'index_flexion' not in values
    assert values['thumb_dip'] == pytest.approx(selected_values[16])
    native = poses[0].poses[4].position
    result = out.landmarks_m[4]
    assert (result.x, result.y, result.z) == pytest.approx(alignment.apply((native.x, native.y, native.z)))
    # Repeated frames must not keep refreshing canonical output/robot targets.
    HumanDexHandAdapter._publish_from_joint_state(harness, msg)
    assert len(canonical) == 1 and warnings
    assert processor.last_stamp == 1_000_000_000

    # Source-frame validation happens before FK; a rejected wire frame changes no filter state.
    filtered = processor.filtered.copy()
    msg.header.frame_id = 'unexpected_sample_frame'
    HumanDexHandAdapter._publish_from_joint_state(harness, msg)
    assert len(canonical) == len(joints) == len(poses) == 1
    assert 'expected SDK frame' in warnings[-1]
    assert np.array_equal(processor.filtered, filtered)


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
