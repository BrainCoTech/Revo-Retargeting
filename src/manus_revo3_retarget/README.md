# Revo3 HandKinematics Retarget

本包保留 `manus_revo3_retarget` 包名以兼容现有脚本。C++ retarget 节点现在只订阅
`hand_teleop_msgs/HandKinematics`；MANUS 和 HumanDex 由独立 adapter 接入。
Pinocchio 求解器、21 个输出关节、MIT 插值和控制器话题保持原有结构。

```text
MANUS → manus_hand_adapter ───────┐
                                 ├→ HandKinematics → Revo3 retarget → MIT controller
HumanDex mux → humandex_hand_adapter ┘
```

HumanDex 的采集和 FK 由 BrainCo-HumanDex 启动；此处只监听 `/joint_states` 和
`/humandex_eef_pose`。

## Runtime Assumptions

- Start the target repository Revo3 hardware launch first, for example
  `revo3_driver/launch/revo3_system.launch.py` or
  `revo3_driver/launch/dual_revo3_system.launch.py`.
- This package publishes `revo3_mit_controller_msgs/msg/Revo3MITCommand`.
- Retargeting runs directly from the HandKinematics subscription callback. MIT commands
  are published by a separate timer at `mit_command_publish_hz` (default 200 Hz)
  using the latest retarget target.
- Default command topics:
  - `/revo3_left/joint_forward_mit_controller/commands`
  - `/revo3_right/joint_forward_mit_controller/commands`
- The thumb IK backend is Pinocchio and uses the shared `revo3_description` URDF.

## Build

```bash
cd revoarm_hardware/Revoarm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select manus_ros2_msgs manus_ros2 hand_teleop_msgs hand_input_adapters manus_revo3_retarget
source install/setup.bash
```

## Launch

```bash
source install/setup.bash
ros2 launch manus_revo3_retarget pipeline_launch.py hand_mode:=both
```

默认启动 MANUS publisher 和 adapter。选择 HumanDex 时不会启动 MANUS 或 HumanDex 驱动：

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py input_source:=humandex hand_mode:=right
```

已有 HandKinematics 发布者时，可用 `input_source:=external` 跳过 adapter，并通过
`input_config:=/path/to/input.yaml` 指定字段映射。`left_input_topic` 和
`right_input_topic` 同时传给 adapter 和 retargeter，默认是 `/hand_kinematics/left` 和
`/hand_kinematics/right`。

输入必须包含正的源时间戳、正确的 side 和 `hand_retarget_<side>` frame；数组长度、
名称唯一性和有限数值均会检查。四指的 MCP/PIP/DIP、所选侧摆字段和五个 tip 必须完整；
缺失帧不更新目标。`thumb_pip`、`thumb_dip` 位置及拇指角度参考可省略。
`source` 仅作消息元数据，算法不根据设备名选择行为。

| 配置 | MANUS | HumanDex |
|---|---|---|
| 四指屈曲 | `index_mcp/pip/dip` 等，rad | 同名字段，rad |
| `spread_joint_suffix` | `spread` | `mpr` |
| `spread_relative_to_middle` | `true` | `false` |
| `thumb_cmr_joint_name` | `thumb_mcp_spread` | `thumb_cmr` |
| 坐标转换 | adapter 执行 `(-y, -x, z)` | adapter 透传掌心局部坐标 |

输入配置位于 `config/input_manus.yaml` 和 `config/input_humandex.yaml`，在原有
retarget 配置之后、用户 calibration override 之前加载。字段选择和输入正负号可通过
这些配置修改；retargeter 不再转换 MANUS 角度单位或使用 MANUS node ID。

HumanDex 配置只完成接口接入，尚未验证实机零位、方向和行程。当前 HumanDex adapter
仍把上游 DIP link 原点称为 tip，并保留原有四指 aggregate flexion；Revo3 不使用
这些 aggregate 值，独立读取 MCP/PIP/DIP。准确 tip/PIP/DIP 点位和手型标定仍需后续完成。
本次保留既有 MIT 定时发布行为：输入停止时继续发布最后目标，不能将输入校验当作失联停机策略。

`hand_mode:=both` starts two independent retarget processes:
`manus_revo3_retarget_left` and `manus_revo3_retarget_right`. Each process only
initializes and computes retargeting for its own side.

Useful overrides:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  launch_manus_publisher:=true \
  mit_command_publish_hz:=200
```

Default parameters are split by function. These four YAML files are loaded
before the selected `input_<source>.yaml` field mapping:

- `config/control.yaml`: topics, MIT publish rate, global MIT kp/kd.
- `config/thumb_retarget.yaml`: thumb IK and thumb calibration.
- `config/four_finger_retarget.yaml`: index/middle/ring/little flexion mapping.
- `config/spread_retarget.yaml`: spread/MPR mapping.

Use `calibration_config`, `left_calibration_config`, or `right_calibration_config`
only for a final one-off override loaded after those split configs:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  calibration_config:=/path/to/physical_joint_calibration.yaml
```

If the Revo3 system is not using `/revo3_<side>` namespaces, override the command
topics directly or use `control_config`:

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right \
  use_revo3_namespace:=false
```

## Launch And Record MCAP

To start the pipeline and record the default Manus/Revo3 topics into
`manus_revo3_retarget/log`:

```bash
cd src/brainco_capabilities/manus_revo3_retarget
./scripts/run_pipeline_record_mcap.sh
```

The script records MCAP bags with `ros2 bag record -s mcap`. If the MCAP storage
plugin is missing, install it first:

```bash
sudo apt install ros-humble-rosbag2-storage-mcap
```

By default the script activates the `manusglove` conda environment before
sourcing ROS and the workspace. Override it with `CONDA_ENV_NAME=<env>` or set
`CONDA_ENV_NAME=none` to use the current shell environment.

Default recorded topics:

- `/manus_glove_0`
- `/manus_glove_1`
- `/revo3_left/joint_forward_mit_controller/commands`
- `/revo3_right/joint_forward_mit_controller/commands`
- `/revo3_left/joint_forward_mit_controller/retarget_targets`
- `/revo3_right/joint_forward_mit_controller/retarget_targets`
- `/revo3_left/revo3_joint_state/joint_states_aligned`
- `/revo3_right/revo3_joint_state/joint_states_aligned`

The `retarget_targets` topics contain the post-retarget, pre-linear-interpolation
MIT target for each side. The high-rate `commands` topics are published by the
timer.

The script also starts `joint_state_aligner`. It republishes each Revo3
`sensor_msgs/msg/JointState` with `name`, `position`, `velocity`, and `effort`
ordered by the latest command `joint_names`, so recorded state arrays line up
with the MIT command arrays. The raw joint state topics are left untouched.

Useful overrides:

```bash
# Pass launch arguments through to pipeline_launch.py.
./scripts/run_pipeline_record_mcap.sh hand_mode:=right

# Put logs somewhere else or use a custom bag folder name.
LOG_ROOT=/tmp/revo3_logs BAG_NAME=test_right \
  ./scripts/run_pipeline_record_mcap.sh hand_mode:=right

# Record all topics instead of the default list.
RECORD_ALL=1 ./scripts/run_pipeline_record_mcap.sh

# Also record the raw, hardware-published joint state topics.
RECORD_RAW_JOINT_STATES=1 ./scripts/run_pipeline_record_mcap.sh

# Replace the default topic list.
BAG_TOPICS="/manus_glove_0 /revo3_right/joint_forward_mit_controller/commands" \
  ./scripts/run_pipeline_record_mcap.sh hand_mode:=right

# Disable joint-state alignment if you only want raw topics.
ENABLE_JOINT_STATE_ALIGNER=0 ./scripts/run_pipeline_record_mcap.sh
```

## Quintic Joint Test

To run a direct MIT command test without Manus input, use:

```bash
cd src/brainco_capabilities/manus_revo3_retarget
./scripts/run_quintic_test_record_mcap.sh
```

This publishes a quintic trajectory for every Revo3 hand joint:
`0 deg -> 40 deg -> 0 deg -> 40 deg -> 0 deg`. It starts
`joint_state_aligner` and records MCAP with:

- `/revo3_left/joint_forward_mit_controller/commands`
- `/revo3_right/joint_forward_mit_controller/commands`
- `/revo3_left/revo3_joint_state/joint_states_aligned`
- `/revo3_right/revo3_joint_state/joint_states_aligned`

Useful overrides:

```bash
./scripts/run_quintic_test_record_mcap.sh --hand-mode right
./scripts/run_quintic_test_record_mcap.sh --target-deg 40 --move-duration-s 2.0 --hold-s 1.0 --rate-hz 100
```

## Command/State Time-Series Viewer

To inspect all Revo3 MIT command values next to live joint feedback as rolling
time-series curves:

```bash
ros2 launch manus_revo3_retarget command_state_viewer.launch.py hand_mode:=both
```

For a single side:

```bash
ros2 launch manus_revo3_retarget command_state_viewer.launch.py hand_mode:=right
```

The viewer subscribes to:

- `/revo3_left/joint_forward_mit_controller/commands`
- `/revo3_left/revo3_joint_state/joint_states`
- `/revo3_right/joint_forward_mit_controller/commands`
- `/revo3_right/revo3_joint_state/joint_states`

Each side gets a tab, and each joint gets a rolling plot. Blue is command
position, green is state position, and a red plot background means
`abs(state_position - command_position)` exceeds `warn_error_rad`.

Useful overrides:

```bash
ros2 launch manus_revo3_retarget command_state_viewer.launch.py \
  hand_mode:=right \
  history_sec:=20.0 \
  update_ms:=50 \
  warn_error_rad:=0.05
```
