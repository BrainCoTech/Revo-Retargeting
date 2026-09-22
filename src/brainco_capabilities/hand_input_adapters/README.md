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
already include the SDK joint-map sign and offset handling. The adapter maps
names without converting the angles again. Each adapter process handles one
side, including when its input contains both hands.

The current SDK contract is documented in [Issue #13](https://github.com/HAOTianGa03/brainco_revohuman_sdk/issues/13)
for branch `feat/tracker-world-frame-and-bringup`, commit `2683152`.
Choose `joint_state_layout` explicitly when using paired or older input:

| Layout | Default source topic | Input names | Default source `frame_id` |
| --- | --- | --- | --- |
| `sdk_single` (single-hand default) | `/revohuman/{side}/joint_states` | 21 unprefixed names, e.g. `index_DIP_joint` | `revohuman_left` / `revohuman_right` |
| `sdk_pair` | `/revohuman/pair/joint_states` | 42 names with `left_` / `right_` prefixes | `revohuman_pair` |
| `legacy` | `/humandex_{side}/joint_states` | 21 names prefixed with the selected side | `left_palm_link` / `right_palm_link` |

Input uses BEST_EFFORT / VOLATILE QoS, matching the new SDK. Joint selection is
by name, not array position. The SDK's sample timestamp (`sample_wall_ns`,
CLOCK_REALTIME) passes through unchanged. Invalid NaN angles, missing or
duplicate joints, stale samples, and timestamps that move backward are rejected.
`source_frame_id` overrides the expected incoming frame. It is separate from
the internal FK frame `<side>_palm_link` and does not change palm coordinates.

The debug `fk_joint_topic` and `fk_pose_topic` publish the same filtered joint
sample and native palm-local poses, with the original timestamp. Their defaults
remain `/humandex_{side}/fk_joint_states` and `/humandex_{side}/eef_pose`. The adapter
then applies `<side>_palm_rpy_rad` and `<side>_palm_translation_m` to canonical
landmarks. `tip_offsets_m` is five DIP-local XYZ vectors in finger order;
`fk_ema_alpha` defaults to 0.2, with a reset after a gap longer than `max_age_sec`.

```bash
ros2 launch hand_input_adapters dv1_input.launch.py \
  hand_mode:=right \
  urdf_path:=/absolute/path/to/Revo_Human_DV1_URDF_Bimanual.urdf
```

It selects `dv1_left.yaml` or `dv1_right.yaml`; `adapter_config`, `joint_topic`,
`joint_state_layout`, and `source_frame_id` override the corresponding defaults.
For SDK `mode:=pair`, add `joint_state_layout:=sdk_pair` and run one adapter per
selected side. For an older publisher, use
`joint_state_layout:=legacy joint_topic:=/humandex_right/joint_states`.

Acquisition runs independently in the upstream SDK. Install its native Python
package, register hand identity by USB topology, and build its three ROS packages
in a separate workspace as described in the [SDK setup guide](../../brainco_bringup/revo2_teleop_bringup/README_DV1.md#sdk-准备独立工作区).
Source only the SDK workspace in its terminal and only this workspace in the
adapter terminal; the two workspaces contain different `revohuman_msgs` packages.
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

三个终端使用相同的 ROS_DOMAIN_ID。先完成上面的独立 SDK 工作区准备，路径与 registry 组名按本机配置修改。

终端 1：独立 SDK 采集。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_revohuman_ws/install/setup.bash
ros2 launch revohuman_bringup revohuman.launch.py mode:=right set_name:=bench \
  tracker_mode:=off publish_tactile:=off camera_mode:=off
```

终端 2：监听 `/revohuman/right/joint_states`，计算 FK，发布 `/hand_kinematics/right`。

```bash
cd ~/code/tele-retarget/Revo-Retargeting
source ~/miniforge3/etc/profile.d/conda.sh
conda activate revo_teleop
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
conda activate revo_teleop
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
bash scripts/teleop.sh right input_source:=external \
  input_config:="$PWD/src/manus_revo3_retarget/config/input_humandex.yaml"
```

也可用 `bash scripts/teleop.sh right input_source:=dv1` 合并终端 2、3 的启动。
双手默认使用 SDK `mode:=pair` 和 `bash scripts/teleop.sh both input_source:=dv1`。
上游若分别运行两个单手 SDK 进程，下游显式使用
`bash scripts/teleop.sh both input_source:=dv1 joint_state_layout:=sdk_single`。
不要同时运行 pair 与单手采集。teleop 的源帧覆盖参数为
`left_source_frame_id` / `right_source_frame_id`，源话题覆盖参数为
`left_joint_topic` / `right_joint_topic`。

首次使用或更新入口后，在本仓库构建环境中执行：

```bash
python -m colcon build --base-paths src --symlink-install \
  --packages-select hand_input_adapters revo2_teleop_bringup
source install/setup.bash
```
