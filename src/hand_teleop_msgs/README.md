# Hand Teleoperation Interface

`HandKinematics` is the only device-neutral input contract used by hand
retargeting.  Each message represents one side and one measurement time.

Canonical joint names use lowercase anatomical names such as `index_pip`,
`middle_mcp`, and `thumb_cmr`.  Canonical landmarks use names such as
`thumb_tip`, `thumb_pip`, and `index_tip`.  Angles are radians, positions are
metres, and every landmark is expressed in `header.frame_id`.

The standard landmark frames are `hand_retarget_left` and
`hand_retarget_right`. They name the normalized Revo retarget basis, not a
device TF frame. In that basis, native MANUS points are mapped as
`(-native_y, -native_x, native_z)`; the current HumanDex palm-local FK and Hex
position stream already use the equivalent basis. A device adapter must perform
this conversion exactly once.

Adapters may also publish the derived aggregate DOFs `index_flexion`,
`middle_flexion`, `ring_flexion`, and `little_flexion`. The current HumanDex
implementation still scales these compatibility fields to the Revo2 range;
they must not be interpreted as measured anatomical angles. Revo3 consumes the
independent joint fields instead.

`source` is diagnostic metadata.  Retargeting behavior must be selected by an
explicit profile, never by branching on `source`.
