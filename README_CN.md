# Revo3 遥操作

[English](README.md) · [Revo3 重定向与校准详解](src/manus_revo3_retarget/README_CN.md)

本工作区使用 MANUS 手套控制 BrainCo Revo3 灵巧手，目标环境为 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10。
`revo3_retargeting` 分支通过 Git 子模块管理 [BrainCoTech/brainco_revo3_ros2](https://github.com/BrainCoTech/brainco_revo3_ros2)，路径为 `src/brainco_revo3_ros2`。主仓库固定经过验证的驱动提交，其中包含在线校准所需的零增益释放和串口发现修复；安装依赖和构建前先初始化子模块。

## 首次安装

```bash
git clone --branch revo3_retargeting https://github.com/BrainCoTech/Revo-Retargeting.git
cd Revo-Retargeting
git submodule update --init --recursive
conda create -n revo_retargeting python=3.10 -y
conda activate revo_retargeting
```

MANUS SDK 的共享库需要从官方获取。驱动子模块中的 Revo3 SDK 用于机器人驱动，MANUS SDK 用于手套采集。准备 MANUS 官方 SDK 后，选择一种方式安装依赖：

```bash
MANUS_SDK_ARCHIVE=/实际路径/MANUS_SDK.zip ./scripts/install_revo3_deps.sh
# 或提供已解压的 SDK 目录：
MANUS_SDK_DIR=/实际路径/ManusSDK ./scripts/install_revo3_deps.sh
```

安装脚本准备 ROS 控制组件、Pinocchio、RViz、MCAP、Git LFS 和 Python 依赖，并将 MANUS 共享库安装到 `src/manus_ros2/ManusSDK/lib/`。
系统依赖已经齐全时，可以只安装 SDK，再检查依赖：

```bash
MANUS_SDK_ARCHIVE=/实际路径/MANUS_SDK.zip ./scripts/install_manus_sdk.sh
./scripts/check_system_deps.sh
```

## 驱动子模块版本

主仓库通过 gitlink 记录经过验证的驱动 SHA。日常更新主仓库后，按记录的固定版本同步驱动：

```bash
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

在线校准所需的修复见[驱动 PR #1](https://github.com/BrainCoTech/brainco_revo3_ros2/pull/1)（`woltium:codex/revo3-online-driver-fixes` → `develop`）。在这些修复合入 `develop` 前，在线校准按主仓库固定的提交运行。

维护者需要跟进 `.gitmodules` 中配置的 `develop` 时，先确认远端 `develop` 已包含所需修复，再执行：

```bash
git submodule update --remote src/brainco_revo3_ros2
```

这会切换到主仓库记录版本之外的驱动提交。验证构建、遥操和校准行为后，再把新的 `src/brainco_revo3_ros2` gitlink 提交到主仓库。日常同步使用前面的固定版本命令。

## 构建

以下命令在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
python -m colcon build --symlink-install --packages-up-to \
  manus_ros2 revo3_driver manus_revo3_retarget
source install/setup.bash
```

首次构建需要同时构建驱动及其依赖。更新在线校准后，也要重新构建 `revo3_driver` 和 `manus_revo3_retarget`，并重启驱动；旧驱动会把零增益替换为默认增益，无法执行校准面板请求的释放。

## 硬件与手套准备

首次连接 Revo3 时，按驱动说明配置串口权限和可选设备别名：

```bash
cd src/brainco_revo3_ros2/revo3_driver/setup
bash bootstrap_revo3.sh
bash check_revo3_setup.sh
cd -
```

首次使用 MANUS 或更换操作者后，运行手套校准：

```bash
./scripts/calibrate_manus.sh right
./scripts/calibrate_manus.sh left
```

这两个命令会覆盖对应手套的 `.mcal` 校准文件。手套校准后，再做下文的 Revo3 关节和对指校准。

## 启动遥操作

先验证单手，再使用双手：

```bash
./scripts/teleop.sh right
# 左手：
./scripts/teleop.sh left
# 双手：
./scripts/teleop.sh both
```

一键脚本启动驱动、MANUS 采集和重定向节点，并显式开启输出；`Ctrl+C` 停止脚本管理的进程。
如果 driver 或 MANUS 已在运行，分别使用 `START_REVO3_DRIVER=0` 或 `START_MANUS_PUBLISHER=0` 复用已有进程，避免同一设备被重复打开：

```bash
START_REVO3_DRIVER=0 ./scripts/teleop.sh right
START_MANUS_PUBLISHER=0 ./scripts/teleop.sh right
```

## 在线校准

完成构建并在两个终端分别加载 ROS 和 `install/setup.bash`，使用相同的 `ROS_DOMAIN_ID`。面板需要桌面图形环境。

```bash
# 终端 1：只启动右手驱动。
./scripts/start_driver.sh right

# 终端 2：启动 MANUS、右手重定向和校准面板。
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right launch_tuning_panel:=true
```

双手时，驱动使用 `./scripts/start_driver.sh both`，pipeline 使用 `hand_mode:=both`。面板一次选择一侧进行校准。
MANUS 已在运行时，在 pipeline 命令中添加 `launch_manus_publisher:=false`；重定向节点也已运行时，只打开面板：

```bash
ros2 run manus_revo3_retarget retarget_tuning_panel --side right --operator alice
```

直接启动 pipeline 默认关闭输出；在面板启用所选手后，依次进入侧摆、四指屈曲、拇指／对指三个阶段。前两个阶段会把非活动关节的目标角设为零，第三阶段保留全部调整并启用所有关节目标。

| 看到的问题 | 调整参数 | 基础配置 |
|---|---|---|
| 某根手指侧摆方向相反 | 对该指当前 `physical_<side>_<finger>_MPR_joint_scale` 取反 | `spread_retarget.yaml` |
| 方向正确，但并拢位置偏移 | 对应 `MPR_joint_offset_deg` | 同上 |
| 侧摆幅度太大／太小 | 保留符号，调整 MPR scale 的绝对值 | 同上 |
| 拇指掌根对指姿态偏移 | 先调拇指 CMP/CMR 的输出 offset，必要时调 scale | `thumb_retarget.yaml` |
| 拇指弯曲不足／过度 | 拇指 MCP/PIP/DIP 的输出 offset／scale | 同上 |
| 只有一根四指接触位置不合适 | 对应手指 MCP/PIP/DIP 的输出 offset／scale | `four_finger_retarget.yaml` |

基础配置位于 `src/manus_revo3_retarget/config/`。保留原有 legacy 映射，日常校准从最终 `physical_*` 输出参数入手。
先检查四指伸直、并拢和张开，再检查四指弯曲，最后依次做拇食、拇中、拇无名、拇小指接触。一次只改一个参数，每次修改后复查张手与四组对指。
面板 offset 步长为 0.5°、scale 步长为 0.05，可从一个步长开始调整。全量阶段沿用现有重定向算法，实际接触位置由操作者检查。

详细键名、右手 YAML 示例、节点名、参数读取及验收步骤见[侧摆方向与对指校准](src/manus_revo3_retarget/README_CN.md#侧摆方向与对指校准)。

### 保存与重新加载

在面板填写操作者和可选校准名称，选择 **Export Override YAML**。历史快照保存在 `~/.brainco/retarget_calibration/library/<side>/`，同时生成 `<operator>_revo3_<side>.yaml` 兼容副本。
历史中的 **Set as latest** 只更新保存的选择，不会应用到运行节点；pipeline 不会自动读取 `latest.json`，重启时显式传入配置：

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right launch_tuning_panel:=true \
  right_calibration_config:="$HOME/.brainco/retarget_calibration/alice_revo3_right.yaml"
```

双手可同时传入 `left_calibration_config` 和 `right_calibration_config`。如果提供共同的 `calibration_config`，分侧配置会被忽略。

### 启动与退出行为

| 入口 | 默认手侧 | 重定向输出初始状态 |
|---|---|---|
| `./scripts/start_driver.sh` | 双手 | 仅启动驱动 |
| 直接启动 `pipeline_launch.py` | 双手 | 关闭 |
| `./scripts/teleop.sh` | 右手 | 显式开启 |

直接启动 pipeline 且不使用面板时，可传 `start_enabled:=true`；它与 `launch_tuning_panel:=true` 不能同时启用。
面板关闭、`Ctrl+C` 或 SIGTERM 会请求关闭输出，并发送全部 21 个关节的零 kp/kd、零速度、零 effort 命令。释放不会要求手指回零，机械摩擦仍存在。
通过 launch 启动的面板关闭后，pipeline 及它启动的 MANUS 进程也会退出，独立启动的驱动继续运行。
如果已连接节点未确认释放请求，面板会保留窗口并显示错误。DDS 确认只说明消息送达；强制终止、通信丢失或同时停止驱动时，无法保证实际释放。

## 验证与调试

构建并加载新工作区后执行离线检查：

```bash
python3 -m pytest src/manus_revo3_retarget/test
colcon test --packages-select revo3_driver
colcon test-result --verbose
```

ROS 集成测试使用合成手套消息和独立话题，不启动硬件。显式运行：

```bash
REVO3_CALIBRATION_INTEGRATION=1 python3 -m pytest \
  src/manus_revo3_retarget/test/test_staged_calibration_ros.py
```

面板交互测试还需要可用的 `DISPLAY`。实机验收需另外检查命令与反馈方向、全部关节释放，以及四组对指的实际接触情况。
录制、命令／反馈曲线和轨迹工具见[英文包说明](src/manus_revo3_retarget/README.md#launch-and-record-mcap)。

依赖检查提示驱动文件缺失时，在仓库根目录执行 `git submodule sync --recursive` 和 `git submodule update --init --recursive`，恢复主仓库记录的驱动版本，再运行依赖检查。主动升级驱动版本时，按[驱动子模块版本](#驱动子模块版本)中的维护步骤执行。
