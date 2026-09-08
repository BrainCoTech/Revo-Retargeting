# Hand Teleoperation Interface

`HandKinematics` is the only device-neutral input contract used by hand
retargeting.  Each message represents one side and one measurement time.

Canonical joint names use lowercase anatomical names such as `index_pip`,
`middle_mcp`, and `thumb_cmr`.  Canonical landmarks use names such as
`thumb_tip`, `thumb_pip`, and `index_tip`.  Angles are radians, positions are
metres, and every landmark is expressed in `header.frame_id`.

The standard landmark frames are `hand_retarget_left` and
`hand_retarget_right`. Adapters must convert source landmarks into the chosen
retarget frame before assigning these names. The MANUS adapter retains the
existing `(-native_y, -native_x, native_z)` position mapping. HumanDex supports
per-side palm rotation and translation; identity defaults are uncalibrated
assumptions, not evidence that its origin or axes match MANUS or the robot.
Coordinate calibration and the anatomical meaning of each landmark must be
verified for each device/model pair. Apply each coordinate conversion once.

Adapters may also publish the derived aggregate DOFs `index_flexion`,
`middle_flexion`, `ring_flexion`, and `little_flexion`. The current HumanDex
implementation still scales these compatibility fields to the Revo2 range;
they must not be interpreted as measured anatomical angles. Revo3 consumes the
independent joint fields instead.

`source` is diagnostic metadata.  Retargeting behavior must be selected by an
explicit profile, never by branching on `source`.
