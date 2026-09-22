# Revo3 HandKinematics 重定向

[English](README.md) · [工作区配置](../../README_CN.md)

## 使用 HumanDex / DV1 启动 Revo3

构建命令在仓库根目录运行。

### 步骤 1：构建

已有构建时，跳过步骤 1。新电脑先按下方说明准备系统依赖并初始化子模块。

```bash
bash scripts/setup_revo_conda.sh
conda activate revo_teleop
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 python -m colcon build --base-paths src --symlink-install \
  --packages-up-to manus_revo3_retarget revo3_driver revo2_teleop_bringup \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
               -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
```

### 步骤 2：启动

**终端 1：启动 SDK 采集并保持运行。** 根据本机情况修改 SDK 路径和串口。

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
/usr/bin/python3 \
  "$HOME/code/tele-retarget/brainco_revohuman_sdk/tools/ros2_joint_state_pub.py" \
  --hand right \
  --port /dev/ttyACM0
```

**终端 2：启动 DV1 适配与 FK、Revo3 重定向和真机 driver。**

```bash
cd ~/code/tele-retarget/Revo-Retargeting
source /opt/ros/humble/setup.bash
conda activate revo_teleop
source install/setup.bash
bash scripts/teleop.sh right input_source:=dv1
```

左手将 `right` 改为 `left`；双手先分别运行两个 SDK 采集进程，各自指定手侧和串口，再运行 `bash scripts/teleop.sh both input_source:=dv1`。
适配器直接读取 SDK 关节话题并计算 FK，无需另开 FK 进程。
SDK 默认目录为 `$HOME/code/tele-retarget/brainco_revohuman_sdk`，可通过 `sdk_path:=/实际路径` 或 `urdf_path:=/实际路径/model.urdf` 覆盖模型位置。

不传 `input_source:=dv1` 时，`teleop.sh` 保留 MANUS 默认输入并启动 MANUS 采集。
`humandex` 对应旧的关节和位姿双话题输入；`external` 接收已有的 `HandKinematics` 发布者。

## Revo3 真机连接与设备命名

给手上电，通过 USB 串口连接电脑。默认配置使用 Modbus、5 Mbps，
`auto_detect: true` 会扫描串口并匹配从站 ID：左手 126、右手 127。
这种模式会忽略配置中的 `port`，无需先建立设备别名。

以下命令均在仓库根目录执行。只查看本机串口及 USB 拓扑，不打开串口通信：

```bash
bash src/brainco_revo3_ros2/revo3_driver/setup/discover_revo3_serial.sh
```

列表包含其他串口设备；可在停止 driver 后逐个插拔手的 USB 连接，确认对应端口。
当前用户需有串口读写权限；权限不足时执行以下命令，然后重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

### 可选：固定设备别名

`/dev/ttyUSB*`、`/dev/ttyACM*` 等编号可能随插拔变化。若需要固定端口，
确认右手实际端口后执行下例（将 `/dev/ttyUSB0` 换成确认的端口）：

```bash
sudo bash src/brainco_revo3_ros2/revo3_driver/setup/setup_revo3_udev_rules.sh \
  /dev/ttyUSB0 right
ls -l /dev/revo3_hand_right
```

脚本建立 `/dev/revo3_hand_right` 符号链接，保留原设备名；左手用 `left`，
对应 `/dev/revo3_hand_left`。它会覆盖 `/etc/udev/rules.d/99-revo3-hands.rules`，
移除未指定一侧的旧别名，并将匹配串口的权限设为 `0666`（所有本地用户可读写）。
双手需在同一次调用中传入两侧，例如 `/dev/ttyUSB0 right /dev/ttyUSB1 left`，
不要分两次执行单手命令。别名未出现时重新插拔 USB。

规则优先按 USB 拓扑匹配。默认 `hub-relative` 仅在路径足够深时匹配末两级端口，
较浅路径使用完整路径；缺少 `ID_PATH` 时才用 USB 序列号和接口编号。
更换手所插的 Hub 端口或 USB 拓扑后需重新检查绑定，别名不保证跟随同一只手。

### 使用固定端口启动

建立别名不会自动修改 driver 配置。先复制完整右手协议配置：

```bash
cp src/brainco_revo3_ros2/revo3_driver/config/protocol_modbus_right.yaml \
  /tmp/revo3_protocol_right.yaml
```

在副本的 `hardware` 下将 `auto_detect` 改为 `false`、`port` 改为
`/dev/revo3_hand_right`，其余字段保留。加载步骤 2 的 ROS 和工作区环境后启动：

```bash
REVO3_RIGHT_PROTOCOL_CONFIG=/tmp/revo3_protocol_right.yaml \
  bash scripts/teleop.sh right input_source:=dv1
```

左手对应 `protocol_modbus_left.yaml` 和 `REVO3_LEFT_PROTOCOL_CONFIG`。
长期使用时将副本保存在自己的配置目录，并给 launch 传入其绝对路径。


## 输入与运行说明

本包保留 `manus_revo3_retarget` 包名。C++ 节点只订阅 `hand_teleop_msgs/HandKinematics`。
DV1 适配器内置 FK，MANUS 和旧 HumanDex 双话题输入各有适配器；输出仍为 21 个关节的 MIT 命令。
`scripts/teleop.sh` 包含 driver；单独调用 `pipeline_launch.py` 时需要另外启动 driver。

默认启动 MANUS publisher 和 adapter。选择 HumanDex 时不会启动 MANUS 或 HumanDex 驱动：

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py input_source:=humandex hand_mode:=right
```

已有 HandKinematics 发布者时，可用 `input_source:=external` 跳过 adapter，并通过
`input_config:=/path/to/input.yaml` 指定字段映射。`left_input_topic` 和
`right_input_topic` 同时传给 adapter 和 retargeter，默认是 `/hand_kinematics/left` 和
`/hand_kinematics/right`。

输入必须包含正的源时间戳、正确的 side 和 `hand_retarget_<side>` frame；数组长度、
名称唯一性和有限数值均会检查。四指的 MCP/PIP/DIP、所选侧摆字段和 thumb_tip 必须完整；
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

tip 定义为手套／模型指尖；HumanDex_bimanual 的 DIP 到指尖偏移已写入 BrainCo-HumanDex 默认 FK 配置。
机器人 IK 仍使用现有 `thumb_tip_Link`。`config/revo3_pad_contacts.yaml` 单独保存指腹参考点，当前不加载到 IK。
`config/humandex_fingertips.yaml` 记录模型、偏移和使用范围；不同 DV1 SDK 模型不能直接套用。
MANUS/HumanDex adapter 不再生成 aggregate flexion，Revo2 独立执行此映射。
补偿合并、归一化 IK 和配置迁移见 [架构迁移说明](../../docs/revo3_architecture_migration.md)。
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


## 录制与调试工具

MCAP 录制、五次轨迹测试和命令/状态曲线查看器见 [英文工具说明](README.md#launch-and-record-mcap)。
