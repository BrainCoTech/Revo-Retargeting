# BrainCo Revo3 ROS 2

[English](README.md) | [简体中文](README_CN.md)

面向 BrainCo Revo3 21 自由度（DoF）灵巧手的 ROS 2 软件包。本仓库包含基于 Modbus 控制所需的硬件驱动、URDF/xacro 描述，以及 MIT 命令控制器。

## 软件包

```text
brainco_revo3_ros2/
├── revo3_driver/                 # ros2_control 硬件接口与 launch 文件
├── revo3_description/            # Revo3 URDF/xacro、网格模型与 RViz 配置
├── revo3_mit_controller/         # MIT 命令控制器插件
└── revo3_mit_controller_msgs/    # MIT 命令消息定义
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- `ros2_control`、`ros2_controllers`、`xacro`、`robot_state_publisher`
- 硬件模式下需通过 Modbus 串口连接 Revo3 灵巧手

## SDK

已内置经测试的 BC Revo3 SDK `v1.0.4`（位于 `revo3_driver/vendor/dist`），克隆仓库后可直接编译，无需额外下载 SDK。

如需手动更新 SDK：

```bash
cd <workspace>/src/brainco_revo3_ros2/revo3_driver
bash scripts/download_sdk.sh
```

## 编译

```bash
mkdir -p ~/revo3_ws/src
cd ~/revo3_ws/src
git clone https://github.com/BrainCoTech/brainco_revo3_ros2.git

cd ~/revo3_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to revo3_driver --symlink-install
source install/setup.bash
```

## 启动

仿真模式（无需硬件）：

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true
```

硬件模式：

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right
```

双手模式：

```bash
ros2 launch revo3_driver dual_revo3_system.launch.py
```

启动硬件前，请先配置串口别名：

```bash
cd ~/revo3_ws/src/brainco_revo3_ros2/revo3_driver/setup
bash bootstrap_revo3.sh
bash check_revo3_setup.sh
```

## 接口

硬件为每个关节暴露以下命令接口：

- `position`：弧度（rad）
- `velocity`：弧度/秒（rad/s）
- `effort`：力矩前馈，单位毫安（mA）
- `kp`
- `kd`

状态接口：

- `position`：弧度（rad）
- `velocity`：弧度/秒（rad/s）
- `current`：安培（A）
- `motor_state`：SDK 原始状态位掩码

更多细节请参阅 `revo3_driver/README.md` 与 `revo3_driver/README_INTERFACES_CN.md`。

## 许可证

Apache License 2.0。详见 [LICENSE](LICENSE)。
