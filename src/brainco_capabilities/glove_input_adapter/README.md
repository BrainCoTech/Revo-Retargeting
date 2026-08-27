# Glove Input Adapter

This capability package adapts HumanDex inputs to the existing Revo2 retargeter. HumanDex
acquisition remains in `humandex_urdf`.

```text
HumanDex MCU
  |-> /humandex_<side>/joint_states -- PIP-only four-finger flexion --|
  `-> /humandex_eef_pose ----------- thumb fingertip pose -----------|
                                                                  v
                                                     humandex_pose_adapter
                                                                  |
                                                                  v
                                                     manus_revo2_retarget
                                                     four fingers: PIP
                                                     thumb: pose IK
```

The default adapter reads each non-thumb finger's `PIP` joint and transports its normalized flexion
through MANUS-compatible ergonomics fields. The thumb is not mapped from `CMR/CMP`; its palm-local
fingertip pose continues into the retargeter's `revo3_thumb` IK.

The default `humandex_urdf/config/config.yaml` defines the per-hand fingertip order as:

```text
index, middle, ring, little, thumb
```

Left poses are relative to `left_palm_Link`; right poses are relative to `right_palm_Link`. The
adapter therefore does not perform world-to-palm conversion. It inserts an identity palm node and
reorders the five local fingertip poses for compatibility with the existing retarget node. A fixed
inverse axis map compensates for the legacy MANUS callback, so the retargeter ultimately receives
the original HumanDex palm-local XYZ coordinates unchanged.

The adapter also subscribes to `/humandex_<side>/joint_states` by default. The four HumanDex
`PIP` positions are mapped through a fixed open/closed calibration interval. The current temporary
profile reuses `+12 deg` (open) and `-12 deg` (closed); it must be recalibrated from independent PIP
recordings. Values outside the interval are clamped. No startup frame is captured as a zero.

The resulting absolute Revo2 angles are transported as MANUS-compatible
`Index/Middle/Ring/Pinky + MCP/PIP/DIP Stretch` fields because the existing thumb retarget node
already consumes that contract. All three fields carry the same absolute target angle, so the
weighted four-finger conversion reconstructs that target exactly.

For one right glove:

```bash
ros2 launch glove_input_adapter humandex_pose_adapter.launch.py \
  side:=right \
  input_hand_mode:=right
```

For bimanual `/humandex_eef_pose`, start one adapter per side with `input_hand_mode:=both`. The
combined HumanDex array contains five left poses followed by five right poses.

`ManusGlove` is a compatibility boundary only. The standalone `humandex_absolute_revo2_node` remains
available for offline regression tests, but it is no longer used by the standard HumanDex launch.
