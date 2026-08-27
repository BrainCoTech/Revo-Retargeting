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
