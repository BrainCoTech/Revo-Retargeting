# Hand Input Adapters

This package is the only place where device-specific schemas are translated to
`hand_teleop_msgs/HandKinematics`.

- `humandex_hand_adapter` pairs `/joint_states` and `/humandex_eef_pose` by the
  exact source timestamp and emits one message per side.
- `manus_hand_adapter` converts native MANUS nodes and ergonomics without
  leaking MANUS identifiers into the retargeter.
- `hex_hand_adapter` pairs raw Hex angle/position JSON using a bounded receive
  time skew.

Adapters normalize units, names, timestamps, and palm-local coordinates.  They
do not contain Revo2 joint limits or target mappings.

All adapters publish landmarks in `hand_retarget_left` or
`hand_retarget_right`. These normalized frame names prevent downstream code
from applying a second device-specific axis transform.

## HumanDex four-finger calibration

HumanDex receives all 21 joints and five end-effector positions. The legacy
`four_finger_mapping:=pip` mode only uses PIP for aggregate flexion. To use all
three bending encoders on the tested right glove:

```bash
ros2 run hand_input_adapters humandex_hand_adapter --ros-args \
  --params-file "$(ros2 pkg prefix hand_input_adapters)/share/hand_input_adapters/config/humandex_right_calibrated.yaml"
```

The `calibrated` mode normalizes each DIP/PIP/MCP angle between its measured
open and closed endpoints, clips each to [0, 1], then takes a weighted average
and multiplies by `four_finger_flexion_range_rad`. The supplied equal weights
reuse the previously tested RevoHuman mapping; they are not the historical
MANUS MCP/PIP/DIP weights of 0.50/0.35/0.15. MPR is excluded from flexion.
All original joints and tip coordinates are still forwarded.

Endpoint arrays contain 12 **upstream JointState radians**, ordered index,
middle, ring, little, with DIP/PIP/MCP within each finger. The right-hand
profile converts the recorded raw encoder endpoints through the upstream
180-degree offset and joint signs (including negative ring/little DIP).
Use it with the bringup `humandex_upstream_right.yaml` configuration. If the
upstream offset/signs or glove fit change, remeasure the endpoints. Each
enabled side requires its own endpoint arrays; right-hand calibration is
never implicitly applied to the left hand. Missing bending joints reject
the frame instead of falling back to another mapping.
