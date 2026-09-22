# Revo Retargeting

支持 Revo3 和 Revo2、共用手部输入适配器的 ROS 2 Humble 工作区。

[English](README.md) · [Revo3 使用说明](src/manus_revo3_retarget/README_CN.md)

## 使用 HumanDex / DV1 启动 Revo3

构建命令在仓库根目录运行。

### 步骤 1：构建

首次构建或更新适配器后执行步骤 1。新电脑先按下方说明准备系统依赖并初始化子模块。

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

SDK 使用 `feat/tracker-world-frame-and-bringup` 分支，接口依据 [Issue #13](https://github.com/HAOTianGa03/brainco_revohuman_sdk/issues/13) 的 `2683152`。首次使用先按 [SDK 准备步骤](src/brainco_bringup/revo2_teleop_bringup/README_DV1.md#sdk-准备独立工作区) 安装原生 Python SDK、构建其三个 ROS 包，并在 registry 中按 USB 拓扑声明手别。SDK 与本仓库继续使用独立工作区，各自终端只 source 各自工作区。

**终端 1：启动官方 SDK 采集并保持运行。** 下例启动右手；registry 组名按实际配置填写。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_revohuman_ws/install/setup.bash
ros2 launch revohuman_bringup revohuman.launch.py mode:=right set_name:=bench \
  tracker_mode:=off publish_tactile:=off camera_mode:=off
```

**终端 2：启动 DV1 适配与 FK、Revo3 重定向和真机 driver。**

```bash
cd ~/code/tele-retarget/Revo-Retargeting
source /opt/ros/humble/setup.bash
conda activate revo_teleop
source install/setup.bash
bash scripts/teleop.sh right input_source:=dv1
```

左手将两处 `right` 改为 `left`。双手将 SDK 的 `mode:=right` 改为 `mode:=pair`，下游运行 `bash scripts/teleop.sh both input_source:=dv1`。单手默认 `joint_state_layout:=sdk_single`，读取 `/revohuman/{side}/joint_states`；双手默认 `sdk_pair`，从 `/revohuman/pair/joint_states` 的 42 个关节中按侧取出 21 个。

若双手采集使用两个独立的 SDK 单手进程（`mode:=left` 和 `mode:=right`），下游显式运行 `bash scripts/teleop.sh both input_source:=dv1 joint_state_layout:=sdk_single`。不要同时启动 `pair` 和单手采集；一只手套只能由一个进程打开。

适配器以 BEST_EFFORT 订阅，保留 SDK 采样时间戳，直接使用已转换的弧度关节角计算 FK。调试话题 `/humandex_{side}/fk_joint_states`、`/humandex_{side}/eef_pose` 保留。源消息的 `frame_id` 与 FK 的掌心坐标独立；SDK 自定义 `frame_id` 时用 `left_source_frame_id:=...` / `right_source_frame_id:=...` 配套设置。
SDK 默认目录为 `$HOME/code/tele-retarget/brainco_revohuman_sdk`，可通过 `sdk_path:=/实际路径` 或 `urdf_path:=/实际路径/model.urdf` 覆盖模型位置。

旧 `/humandex_{side}/joint_states` 发布者可显式使用 `joint_state_layout:=legacy`，并通过 `left_joint_topic:=...` / `right_joint_topic:=...` 指定旧话题；关节名与帧标识须满足旧契约。三种布局见 [适配器说明](src/brainco_capabilities/hand_input_adapters/README.md#dv1-sdk-direct-input)。

不传 `input_source:=dv1` 时，`teleop.sh` 保留 MANUS 默认输入并启动 MANUS 采集。
`humandex` 对应旧的关节和位姿双话题输入；`external` 接收已有的 `HandKinematics` 发布者。

## 首次配置和硬件连接

目标环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。克隆或切换分支后先拉取 Revo3 子模块：

```bash
git submodule update --init --recursive
```

系统和 ROS 依赖安装见 [英文主说明](README.md#fresh-computer-setup)。Revo2 driver 所需 Stark SDK：

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

MANUS 用户另需安装官方 SDK，参见 [SDK 安装说明](README.md#fresh-computer-setup)。
Revo3 串口权限、自动识别和可选固定设备名见 [连接说明](src/manus_revo3_retarget/README_CN.md#revo3-真机连接与设备命名)。

## Revo2 说明

## 架构

三类输入都在同一个设备中立边界结束：

```text
设备驱动 / 上游采集
  -> hand_input_adapters
  -> hand_teleop_msgs/HandKinematics
  -> revo2_hand_retarget
  -> revo2_pid_controller target JointState
  -> revo2_driver
```

`HandKinematics` 每条消息只描述一侧手和一个源时间戳。关节角统一为弧度，关键点
统一为米；缺失数据直接省略，不再伪造 `ManusGlove` 节点或人体工学字段。坐标系、
单位和设备命名转换只存在于适配器，重定向核心不依赖 MANUS 消息，也不感知
HumanDex/Hex 的传输方式。

主要包：

```text
src/brainco_capabilities/hand_teleop_msgs       中立输入协议
src/brainco_capabilities/hand_input_adapters    HumanDex/MANUS/Hex 适配器
src/brainco_capabilities/revo2_hand_retarget    设备中立重定向核心
src/brainco_bringup/revo2_teleop_bringup        profile 驱动的统一启动层
src/brainco_drivers/hex_glove_driver             只负责原始 UDP 传输
src/brainco_drivers/manus_ros2                   MANUS SDK 原生驱动
src/brainco_drivers/revo2_driver                 Revo2 ros2_control 驱动
```

## 启动

HumanDex / RevoHuman 的设备识别、话题检查和无真机分步骤启动，见
[HumanDex 手套启动指南](docs/humandex_startup_CN.md)。该指南也说明外部采集驱动与本仓库适配器的边界。

所有手套共用一个入口，只切换 profile：

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 hand_mode:=right

ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=manus_revo2 hand_mode:=right

ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=hex_revo2 hand_mode:=right
```

HumanDex 采集进程默认由其上游工程启动；MANUS 和 Hex profile 默认启动各自驱动。
若驱动已经在运行，传 `launch_input_driver:=false`。

接真机前先跑只读离线链路：

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 hand_mode:=right \
  launch_revo2_driver:=false switch_controllers:=false
```

先确认 `/hand_kinematics/right` 和 retarget target topic 的数值、方向、频率合理，
再启用 Revo2 真机。`revo2_driver` 仍保留较安全的 position controller 默认状态；
完整遥操作启动层只在明确启用时切到 `revo2_pid_controller`。

字段定义、适配器参数和 profile 说明见各包 README。

### RevoHuman SDK + DV1 FK

新版 SDK 的 JointState 接入、Revo2 启动与四指标定见 [DV1 接入说明](src/brainco_bringup/revo2_teleop_bringup/README_DV1.md)。适配器内部完成 FK，支持 YAML tip 偏移微调。

旧 `revohuman_revo2` profile 与 [原始 RawFrame 链路](src/brainco_capabilities/revohuman_kinematics/README.md) 仍保留，使用本仓库旧串口 driver；不要与新版 SDK 采集同时启动。新 SDK 接入使用上面的 `input_source:=dv1` 或 Revo2 的 `dv1_sdk.launch.py`。
