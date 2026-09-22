"""SDK 2683152 wire fixtures, independent of the FK joint-name constants."""

import pytest

from hand_input_adapters.dv1_joint_state import DV1JointStateInput


# joint_map 0.2 base names; SDK glove_node supplies handedness to joint_names().
SDK_NAMES = (
    'index_DIP_joint', 'index_PIP_joint', 'index_MCP_joint', 'index_MPR_joint',
    'middle_DIP_joint', 'middle_PIP_joint', 'middle_MCP_joint', 'middle_MPR_joint',
    'ring_DIP_joint', 'ring_PIP_joint', 'ring_MCP_joint', 'ring_MPR_joint',
    'little_DIP_joint', 'little_PIP_joint', 'little_MCP_joint', 'little_MPR_joint',
    'thumb_DIP_joint', 'thumb_PIP_joint', 'thumb_MCP_joint', 'thumb_CMR_joint', 'thumb_CMP_joint',
)
LAYOUTS = ('sdk_single', 'sdk_pair', 'legacy')
LEFT_VALUES = [index / 8 - 1.25 for index in range(21)]
RIGHT_VALUES = [value + 4 for value in LEFT_VALUES]


def wire_sample(side, layout):
    values = LEFT_VALUES if side == 'left' else RIGHT_VALUES
    if layout == 'sdk_single':
        return [f'{side}_{name}' for name in SDK_NAMES], values.copy(), f'revohuman_{side}'
    if layout == 'sdk_pair':
        return ([f'{hand}_{name}' for hand in ('left', 'right') for name in SDK_NAMES],
                LEFT_VALUES + RIGHT_VALUES, 'revohuman_pair')
    return [f'{side}_{name}' for name in SDK_NAMES], values.copy(), f'{side}_palm_link'


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', LAYOUTS)
@pytest.mark.parametrize('shuffled', [False, True])
def test_normalization_preserves_named_radians_and_selects_the_requested_hand(side, layout, shuffled):
    names, positions, frame = wire_sample(side, layout)
    if shuffled:
        order = list(range(1, len(names), 2)) + list(range(0, len(names), 2))
        names, positions = [names[i] for i in order], [positions[i] for i in order]
    original = names.copy(), positions.copy()
    actual_names, actual_positions = DV1JointStateInput(side, layout).normalize(names, positions, frame)
    assert actual_names == tuple(f'{side}_{name}' for name in SDK_NAMES)
    # Include negative angles and values above pi: no sign, offset, degree or wrap conversion.
    assert actual_positions == (LEFT_VALUES if side == 'left' else RIGHT_VALUES)
    assert (names, positions) == original


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', LAYOUTS)
def test_source_frame_override_is_checked_before_internal_names_are_produced(side, layout):
    names, positions, frame = wire_sample(side, layout)
    normalizer = DV1JointStateInput(side, layout, source_frame_id='custom_sample_frame')
    with pytest.raises(ValueError, match='expected SDK frame'):
        normalizer.normalize(names, positions, frame)
    actual_names, actual_positions = normalizer.normalize(names, positions, 'custom_sample_frame')
    assert actual_names == tuple(f'{side}_{name}' for name in SDK_NAMES)
    assert actual_positions == (LEFT_VALUES if side == 'left' else RIGHT_VALUES)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', LAYOUTS)
def test_wrong_or_empty_source_frame_is_rejected(side, layout):
    names, positions, _ = wire_sample(side, layout)
    normalizer = DV1JointStateInput(side, layout)
    for frame in ('', 'world', f'revohuman_{"right" if side == "left" else "left"}'):
        with pytest.raises(ValueError, match='expected SDK frame'):
            normalizer.normalize(names, positions, frame)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', LAYOUTS)
@pytest.mark.parametrize('defect', ['missing', 'duplicate', 'extra', 'unknown', 'short_positions'])
def test_incomplete_ambiguous_or_unexpected_joint_names_are_rejected(side, layout, defect):
    names, positions, frame = wire_sample(side, layout)
    if defect == 'missing':
        names.pop()
        positions.pop()
    elif defect == 'duplicate':
        names[-1] = names[0]
    elif defect == 'extra':
        names.append('wrist_joint')
        positions.append(0.)
    elif defect == 'unknown':
        names[-1] = 'unknown_joint'
    else:
        positions.pop()
    with pytest.raises(ValueError, match='unique joints'):
        DV1JointStateInput(side, layout).normalize(names, positions, frame)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout', LAYOUTS)
@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), float('-inf')])
def test_nonfinite_joint_values_are_rejected(side, layout, invalid):
    names, positions, frame = wire_sample(side, layout)
    # Check both halves of a pair, including the hand this adapter does not select.
    for index in ([0, 21] if layout == 'sdk_pair' else [0]):
        bad = positions.copy()
        bad[index] = invalid
        with pytest.raises(ValueError, match='finite'):
            DV1JointStateInput(side, layout).normalize(names, bad, frame)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout,wire_layout', [
    (layout, other) for layout in LAYOUTS for other in LAYOUTS
    if (layout == 'sdk_pair') != (other == 'sdk_pair')
])
def test_schema_mismatch_is_not_inferred_from_incoming_names(side, layout, wire_layout):
    names, positions, _ = wire_sample(side, wire_layout)
    _, _, frame = wire_sample(side, layout)
    with pytest.raises(ValueError, match='unique joints'):
        DV1JointStateInput(side, layout).normalize(names, positions, frame)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('prefix', ['', 'opposite'])
def test_sdk_single_rejects_unprefixed_or_opposite_hand_names(side, prefix):
    other = 'right' if side == 'left' else 'left'
    names = [f'{other}_{name}' if prefix else name for name in SDK_NAMES]
    with pytest.raises(ValueError, match='unique joints'):
        DV1JointStateInput(side).normalize(names, LEFT_VALUES, f'revohuman_{side}')


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('layout,wire_layout', [('sdk_single', 'legacy'), ('legacy', 'sdk_single')])
def test_single_and_legacy_share_joint_names_but_require_distinct_source_frames(side, layout, wire_layout):
    names, positions, frame = wire_sample(side, wire_layout)
    with pytest.raises(ValueError, match='expected SDK frame'):
        DV1JointStateInput(side, layout).normalize(names, positions, frame)


@pytest.mark.parametrize('side,layout', [('both', 'sdk_pair'), ('left', 'auto')])
def test_invalid_configuration_is_rejected(side, layout):
    with pytest.raises(ValueError):
        DV1JointStateInput(side, layout)
