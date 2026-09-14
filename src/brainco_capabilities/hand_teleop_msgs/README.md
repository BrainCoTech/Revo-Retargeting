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

MANUS/HumanDex adapters publish observed independent angles, not Revo2 aggregate
flexion. Robot-specific aggregate mapping is owned by `revo2_hand_retarget`.
Other legacy adapters may still publish aggregate fields during migration.
A tip means the glove/model fingertip. The upstream FK applies the model-specific
DIP-local offset once; the adapter preserves this point. Finger-pad contact
references are separate and must not replace tips. HumanDex_bimanual geometric
offsets and their model scope are documented in
[the fingertip record](../../manus_revo3_retarget/config/humandex_fingertips.yaml).

`source` is diagnostic metadata.  Retargeting behavior must be selected by an
explicit profile, never by branching on `source`.
