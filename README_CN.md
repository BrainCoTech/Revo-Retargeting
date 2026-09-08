# Revo2 Retargeting

这是一个支持 HumanDex、MANUS、Hex 手套遥操作 BrainCo Revo2 灵巧手的 ROS 2
Humble workspace。

English: [README.md](README.md)

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

## 构建

目标环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。

```bash
source /opt/ros/humble/setup.bash
python -m colcon build
source install/setup.bash
```

构建 `revo2_driver` 前按需安装 BrainCo Stark SDK：

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

使用 MANUS profile 时，还需要把官方 SDK 放在
`src/brainco_drivers/manus_ros2/ManusSDK/`。

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

新增原始 driver 与 DV1 运动学适配层，直接输出 `HandKinematics`。
四指使用三个弯曲编码器，拇指默认以 DIP_Link 原点作为末端，支持 YAML tip 偏移微调。
构建、启动与标定见 [RevoHuman 接入说明](src/brainco_capabilities/revohuman_kinematics/README.md)。
统一启动 profile 为 `revohuman_revo2`；零位、方向与拇指参数仍需实测标定。
