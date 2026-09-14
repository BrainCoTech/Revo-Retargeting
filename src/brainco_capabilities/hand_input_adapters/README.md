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

## Revo2 four-finger calibration

MANUS/HumanDex adapters publish independent observed joints only. Aggregate
flexion now belongs to `revo2_hand_retarget`, selected explicitly with
`finger_flexion_config`. Calibration files and `calibrate_dv1_fingers` have moved
there. See [migration](../../../docs/revo3_architecture_migration.md).

## DV1 SDK direct input

`input_mode:=dv1_joint_states` subscribes only to the SDK JointState topic.
The adapter evaluates five DIP-link poses internally using the shared
`revohuman_kinematics.joint_fk` library. No separate FK node or external pose
publisher is needed. `urdf_path` must point to the SDK DV1 URDF; input radians
already include firmware zero/direction handling. One side per process is
required, and input `header.frame_id` must match `<side>_palm_link`.

The debug `fk_joint_topic` and `fk_pose_topic` publish the same filtered joint
sample and native palm-local poses, with the original timestamp. The adapter
then applies `<side>_palm_rpy_rad` and `<side>_palm_translation_m` to canonical
landmarks. `tip_offsets_m` is five DIP-local XYZ vectors in finger order;
`fk_ema_alpha` defaults to 0.2, with a reset after a gap longer than `max_age_sec`.

Use `ros2 launch hand_input_adapters dv1_input.launch.py hand_mode:=right urdf_path:=/absolute/path/to/Revo_Human_DV1_URDF_Bimanual.urdf`.
It selects `dv1_left.yaml` or `dv1_right.yaml`; `adapter_config` and `joint_topic`
can override the defaults. Acquisition runs independently in the upstream SDK.
This launch only starts the adapter and FK, with no robot or serial driver.
Revo2 left ranges are explicitly provisional; `ros2 run revo2_hand_retarget calibrate_dv1_fingers` records actual
open/fist ranges to a local YAML without changing firmware or FK zero.
The Revo2 DV1 mapping uses `four_finger_wrap_angles: true` for circular endpoint differences.
See the bringup [left-hand instructions](../../brainco_bringup/revo2_teleop_bringup/README_DV1.md).


### HumanDex fingertip geometry

For paired `/humandex_eef_pose` input, fingertip offsets are applied in
BrainCo-HumanDex, not in this adapter. Its `HumanDex_bimanual.urdf` default config
now uses the mirrored glove-tip offsets recorded in
[humandex_fingertips.yaml](../../manus_revo3_retarget/config/humandex_fingertips.yaml).
The direct `dv1_joint_states` profiles use a different SDK URDF (including different
DIP axes); their existing zero/trial offsets are intentionally not overwritten.
Tip means glove fingertip; a finger-pad contact reference is a separate point.

## 右手 DV1 → Revo3

三个终端使用相同的 ROS_DOMAIN_ID。以下路径适用于当前本机目录，串口按实际设备修改。

终端 1：独立 SDK 采集。

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  "$HOME/code/tele-retarget/brainco_revohuman_sdk/tools/ros2_joint_state_pub.py" \
  --hand right --port /dev/ttyACM0
```

终端 2：监听 `/humandex_right/joint_states`，计算 FK，发布 `/hand_kinematics/right`。

```bash
cd ~/code/tele-retarget/Revo-Retargeting
source ~/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo3
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
ros2 launch hand_input_adapters dv1_input.launch.py \
  hand_mode:=right \
  urdf_path:=$HOME/code/tele-retarget/brainco_revohuman_sdk/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf
```

终端 3：启动 Revo3 右手驱动和重定向，读取已有 HandKinematics。

```bash
cd ~/code/tele-retarget/Revo-Retargeting
source ~/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo3
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
bash scripts/teleop.sh right input_source:=external \
  input_config:="$PWD/src/manus_revo3_retarget/config/input_humandex.yaml"
```

首次使用新入口前，在已加载上述构建环境的终端执行：

```bash
python -m colcon build --base-paths src --symlink-install \
  --packages-select hand_input_adapters revo2_teleop_bringup
source install/setup.bash
```
