# revo3_driver

[English](README.md) | [简体中文](README_CN.md)

## 概述

`revo3_driver` 是面向 BrainCo Revo3 21-DoF 灵巧手的 ROS 2 `ros2_control` 硬件接口包。

- 仅支持 Modbus 串口通信（5 Mbps）
- MIT 模态控制：每关节同时暴露 `position / velocity / effort / kp / kd` 命令接口
- 关节状态反馈：`position / velocity / current / motor_state`
- 支持单手与双手启动
- 支持 `mock_components/GenericSystem` 仿真模式

### 21 个受控关节（以右手为例，左手替换前缀 `right` → `left`）

| SDK 索引 | 关节名 | 说明 |
|--------|--------|------|
| 0 | `right_little_MPR_joint` | 小指外展（abduction） |
| 1 | `right_little_MCP_joint` | 小指掌指关节 |
| 2 | `right_little_PIP_joint` | 小指近端指间关节 |
| 3 | `right_little_DIP_joint` | 小指远端指间关节 |
| 4 | `right_ring_MPR_joint` | 无名指外展 |
| 5 | `right_ring_MCP_joint` | 无名指掌指关节 |
| 6 | `right_ring_PIP_joint` | 无名指近端指间关节 |
| 7 | `right_ring_DIP_joint` | 无名指远端指间关节 |
| 8 | `right_middle_MPR_joint` | 中指外展 |
| 9 | `right_middle_MCP_joint` | 中指掌指关节 |
| 10 | `right_middle_PIP_joint` | 中指近端指间关节 |
| 11 | `right_middle_DIP_joint` | 中指远端指间关节 |
| 12 | `right_index_MPR_joint` | 食指外展 |
| 13 | `right_index_MCP_joint` | 食指掌指关节 |
| 14 | `right_index_PIP_joint` | 食指近端指间关节 |
| 15 | `right_index_DIP_joint` | 食指远端指间关节 |
| 16 | `right_thumb_MCP_joint` | 拇指掌指关节 |
| 17 | `right_thumb_PIP_joint` | 拇指近端指间关节 |
| 18 | `right_thumb_DIP_joint` | 拇指远端指间关节 |
| 19 | `right_thumb_CMP_joint` | 拇指 CMC 外展 |
| 20 | `right_thumb_CMR_joint` | 拇指腕掌旋转 |

---

## 环境依赖

- Ubuntu 22.04 + ROS 2 Humble
- `ros2_control`, `controller_manager`, `hardware_interface`
- `joint_state_broadcaster`, `forward_command_controller`, `joint_trajectory_controller`
- `robot_state_publisher`, `launch`, `launch_ros`, `xacro`, `rviz2`
- `revo3_description`（URDF / mesh 资源包）
- `revo3_mit_controller`, `revo3_mit_controller_msgs`

---

## 构建

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-up-to revo3_driver --symlink-install
source install/setup.bash
```

### BC Revo3 SDK

公开仓随包包含已验证的 BC Revo3 SDK `v1.0.4`：

```text
revo3_driver/vendor/dist/include/stark-sdk.h
revo3_driver/vendor/dist/shared/linux/libbc_revo3_sdk.so
```

如需刷新 SDK，可运行：

```bash
cd <workspace>/src/brainco_revo3_ros2/revo3_driver
bash scripts/download_sdk.sh
```

脚本会下载固定版本的 SDK 二进制包，并解压到 `revo3_driver/vendor/dist`。构建时 driver 会直接链接该目录下的 `libbc_revo3_sdk.so`。

也可以通过环境变量覆盖 SDK 版本：

```bash
export BRAINCO_REVO3_SDK_VERSION=v1.0.4
```

---

## 配置文件

| 文件 | 说明 |
|------|------|
| `config/protocol_modbus_right.yaml` | 右手 Modbus 参数 |
| `config/protocol_modbus_left.yaml` | 左手 Modbus 参数 |
| `config/revo3_controllers.yaml` | 控制器模板（含 `HAND_PREFIX` / `UPDATE_RATE` 占位符） |
| `config/revo3.ros2_control.xacro` | ros2_control 系统 xacro（由 `revo3_description` 调用） |

### Modbus 关键参数

```yaml
hardware:
  slave_id: 127         # 右手=127，左手=126
  baudrate: 5000000     # 5 Mbps，Revo3 固定波特率
  auto_detect: true     # true 时自动扫描串口，忽略 port 字段
  auto_detect_quick: true
  auto_detect_port: ""  # 留空扫描全部串口；填 "/dev/ttyUSB" 可限制范围
  port: /dev/ttyUSB0    # auto_detect=true 时忽略；false 时手动指定
  log_level: info       # error | warn | info | debug | trace
  read_decimation: 4    # 读反馈降频；200 Hz 控制时约 50 Hz 有效反馈
```

---

## 启动方式

### 单手

```bash
# 右手（实体）
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right

# 左手（实体）
ros2 launch revo3_driver revo3_system.launch.py hand_side:=left

# 仿真模式（不连接硬件）
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true

# 开启 RViz 可视化
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true launch_rviz:=true

# 指定自定义协议配置（绝对路径或相对于 revo3_driver/config 的文件名）
ros2 launch revo3_driver revo3_system.launch.py \
  hand_side:=right \
  protocol_config_file:=/path/to/my_modbus.yaml
```

### 双手

```bash
# 默认配置同时启动左右手
ros2 launch revo3_driver dual_revo3_system.launch.py

# 仿真模式 + RViz
ros2 launch revo3_driver dual_revo3_system.launch.py if_sim:=true launch_rviz:=true

# 指定左右手各自的协议配置
ros2 launch revo3_driver dual_revo3_system.launch.py \
  left_protocol_config_file:=protocol_modbus_left.yaml \
  right_protocol_config_file:=protocol_modbus_right.yaml
```

### 完整 Launch 参数参考

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hand_side` | `right` | 手型：`left` \| `right` |
| `if_sim` | `false` | 使用 mock 硬件 |
| `use_namespace` | `true` | 节点命名空间（`revo3_left` / `revo3_right`） |
| `launch_rsp` | `true` | 启动 robot_state_publisher |
| `launch_rviz` | `false` | 启动 RViz2 可视化 |
| `rviz_config_file` | `""` | RViz 配置文件，留空使用默认 `revo3_hand.rviz` |
| `protocol_config_file` | `""` | 协议配置覆盖，留空自动按 `hand_side` 选择 |
| `controllers_file` | `""` | 控制器模板覆盖，留空使用 `revo3_controllers.yaml` |
| `description_package` | `revo3_description` | 提供 `revo3.single.system.xacro` 的包 |

---

## 控制器切换

启动后默认激活 `joint_forward_mit_controller`，`joint_forward_pos_controller` 和 `joint_traj_pos_controller` 已加载但未激活。
MIT 与位置/轨迹控制器共享 `position` command interface，**不能同时激活**。

> **命名空间与 `hand_side` 对应**：`hand_side:=right` → `/revo3_right/...`；`hand_side:=left` → `/revo3_left/...`。
> 以下示例以右手为例；左手请把路径中的 `revo3_right` 替换为 `revo3_left`。

### 切换到 JointTrajectoryController

```bash
ros2 control switch_controllers \
  --controller-manager /revo3_right/controller_manager \
  --deactivate joint_forward_mit_controller \
  --activate joint_traj_pos_controller
```

### 切换到位置直通控制器

```bash
ros2 control switch_controllers \
  --controller-manager /revo3_right/controller_manager \
  --deactivate joint_forward_mit_controller \
  --activate joint_forward_pos_controller
```

### 切换回 MIT Controller

```bash
ros2 control switch_controllers \
  --controller-manager /revo3_right/controller_manager \
  --deactivate joint_traj_pos_controller \
  --activate joint_forward_mit_controller
```

（若当前激活的是 `joint_forward_pos_controller`，把 `--deactivate` 中的控制器名换成它即可。）

---

## 测试

### 前置步骤

启动后先确认控制器已就绪：

```bash
# 查看所有控制器状态（应看到 revo3_joint_state 和 joint_forward_mit_controller 均为 active）
ros2 control list_controllers --controller-manager /revo3_right/controller_manager

# 查看硬件接口
ros2 control list_hardware_interfaces --controller-manager /revo3_right/controller_manager
```

若 `list_controllers` 长时间等待 service，说明 launch 未启动或 `hand_side` 与命令中的命名空间不一致。

### 查看关节状态

```bash
# 实时打印关节位置/速度
# 注意：当前不提供状态 effort；若 echo 中 effort 为 NaN，表示不可用。
ros2 topic echo /revo3_right/revo3_joint_state/joint_states

# 查看 current(A)/motor_state 诊断接口
ros2 topic echo /revo3_right/revo3_joint_state/dynamic_joint_states
```

### 测试 MIT Controller（实体手推荐，默认已激活）

硬件底层始终走 **MIT 模态**（`position + velocity + effort + kp + kd`）。默认激活的 `joint_forward_mit_controller` 是实体手最直接的测试方式，**无需切换控制器**。

**发布话题**：

```
/revo3_right/joint_forward_mit_controller/commands
```

消息类型：`revo3_mit_controller_msgs/msg/Revo3MITCommand`

- `position`：弧度（rad），21 个值，顺序见下文关节表
- `kp` / `kd`：刚度/阻尼，建议显式填写（默认配置 `kp=1.0`, `kd=0.1`）
- `velocity` / `effort`：可留空，使用控制器默认值
- `joint_names`：可留空（按 SDK 顺序）；也可填关节名做部分关节控制

> `command_timeout_sec` 默认为 `0.25` s：只发一次 `--once` 后控制器会超时并保持当前位置。
> 实体手测试请用 `--rate 20` 持续发布，或增大 yaml 中的 `command_timeout_sec`。

**全手握拳（持续 20 Hz 发布）：**

```bash
ros2 topic pub --rate 20 \
  /revo3_right/joint_forward_mit_controller/commands \
  revo3_mit_controller_msgs/msg/Revo3MITCommand \
  "{position: [0.0, 1.2, 1.2, 1.2,
              0.0, 1.2, 1.2, 1.2,
              0.0, 1.2, 1.2, 1.2,
              0.0, 1.2, 1.2, 1.2,
              0.8, 0.8, 0.5, 0.3,
              0.0],
    kp: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    kd: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"
```

**全手展开（归零）：**

```bash
ros2 topic pub --rate 20 \
  /revo3_right/joint_forward_mit_controller/commands \
  revo3_mit_controller_msgs/msg/Revo3MITCommand \
  "{position: [0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0],
    kp: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    kd: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"
```

**验证反馈（另开终端）：**

```bash
ros2 topic echo /revo3_right/revo3_joint_state/joint_states --once
```

### 测试 forward_command_controller（位置直通）

`joint_forward_pos_controller` 订阅 `Float64MultiArray`，顺序严格对应 21 个关节（SDK 索引 0→20）。
它只写 `position` 接口；硬件在 MIT 模式下需要非零 `kp/kd` 才能产生力矩，驱动会在 `kp/kd` 未声明时自动回退到默认值（`kp=1.0`, `kd=0.1`）。

测试前先切换：

```bash
ros2 control switch_controllers \
  --controller-manager /revo3_right/controller_manager \
  --deactivate joint_forward_mit_controller \
  --activate joint_forward_pos_controller
```

**发布话题**（以右手命名空间为例）：

```
/revo3_right/joint_forward_pos_controller/commands
```

**全手展开（所有关节归零）：**

```bash
ros2 topic pub --once \
  /revo3_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0]}"
```

**全手握拳（MCP/PIP/DIP 约 1.2 rad，外展归零）：**

```bash
ros2 topic pub --once \
  /revo3_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.2, 1.2, 1.2,
           0.0, 1.2, 1.2, 1.2,
           0.0, 1.2, 1.2, 1.2,
           0.0, 1.2, 1.2, 1.2,
           0.8, 0.8, 0.5, 0.3,
           0.0]}"
```

> 关节顺序：`little_MPR, little_MCP, little_PIP, little_DIP,`
> `ring_MPR, ring_MCP, ring_PIP, ring_DIP,`
> `middle_MPR, middle_MCP, middle_PIP, middle_DIP,`
> `index_MPR, index_MCP, index_PIP, index_DIP,`
> `thumb_MCP, thumb_PIP, thumb_DIP, thumb_CMP, thumb_CMR`

**只弯曲食指（其余归零）：**

```bash
ros2 topic pub --once \
  /revo3_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 1.2, 1.2, 1.2,
           0.0, 0.0, 0.0, 0.0,
           0.0]}"
```

**只弯曲拇指：**

```bash
ros2 topic pub --once \
  /revo3_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0,
           0.8, 0.8, 0.5, 0.3,
           0.0]}"
```

**持续周期性发布（验证实时响应，10 Hz）：**

```bash
ros2 topic pub --rate 10 \
  /revo3_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.5, 0.5, 0.5,
           0.0, 0.5, 0.5, 0.5,
           0.0, 0.5, 0.5, 0.5,
           0.0, 0.5, 0.5, 0.5,
           0.3, 0.3, 0.2, 0.1,
           0.0]}"
```

> `joint_forward_pos_controller` 会把最后一次命令保持在硬件缓冲中，`--once` 通常足够。
> 若仍无动作，先用 `ros2 topic echo .../joint_states` 确认反馈是否变化，再检查控制器是否为 `active`。

### 运行仿真模式完整测试

```bash
# 1. 启动（仿真 + RViz）
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true launch_rviz:=true

# 2. 确认控制器就绪
ros2 control list_controllers --controller-manager /revo3_right/controller_manager

# 3. 发指令
ros2 topic pub --once \
  /revo3_right/joint_forward_pos_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.2, 1.2, 1.2,
           0.0, 1.2, 1.2, 1.2,
           0.0, 1.2, 1.2, 1.2,
           0.0, 1.2, 1.2, 1.2,
           0.8, 0.8, 0.5, 0.3, 0.0]}"

# 4. 验证状态反馈
ros2 topic echo /revo3_right/revo3_joint_state/joint_states --once
ros2 topic echo /revo3_right/revo3_joint_state/dynamic_joint_states --once
```

---

### 测试 joint_traj_pos_controller（轨迹 Action 控制）

`joint_traj_pos_controller` 使用 `joint_trajectory_controller/JointTrajectoryController` 插件，并暴露一个 Action Server：

```
/revo3_right/joint_traj_pos_controller/follow_joint_trajectory
```

Action 类型：`control_msgs/action/FollowJointTrajectory`

#### 前置：切换控制器

```bash
ros2 control switch_controllers \
  --controller-manager /revo3_right/controller_manager \
  --deactivate joint_forward_mit_controller \
  --activate joint_traj_pos_controller
```

#### 发送目标：全手展开（2 秒内到达）

```bash
ros2 action send_goal \
  /revo3_right/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - right_little_MPR_joint
    - right_little_MCP_joint
    - right_little_PIP_joint
    - right_little_DIP_joint
    - right_ring_MPR_joint
    - right_ring_MCP_joint
    - right_ring_PIP_joint
    - right_ring_DIP_joint
    - right_middle_MPR_joint
    - right_middle_MCP_joint
    - right_middle_PIP_joint
    - right_middle_DIP_joint
    - right_index_MPR_joint
    - right_index_MCP_joint
    - right_index_PIP_joint
    - right_index_DIP_joint
    - right_thumb_MCP_joint
    - right_thumb_PIP_joint
    - right_thumb_DIP_joint
    - right_thumb_CMP_joint
    - right_thumb_CMR_joint
  points:
    - positions: [0.31 , 1.0  , 0.60 , 0.10 ,
     0.20 , 0.95 , 0.30 , 0.25 , 0.12 , 
     0.90 , 0.30 , 0.25 , -0.05, 1.20 , 
    0.25 , -0.00, 0.00 , 0.00 , 1.78 , 
    0.15, 0.30]
      time_from_start: {sec: 1, nanosec: 0}
"

ros2 action send_goal \
  /revo3_left/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - left_little_MPR_joint
    - left_little_MCP_joint
    - left_little_PIP_joint
    - left_little_DIP_joint
    - left_ring_MPR_joint
    - left_ring_MCP_joint
    - left_ring_PIP_joint
    - left_ring_DIP_joint
    - left_middle_MPR_joint
    - left_middle_MCP_joint
    - left_middle_PIP_joint
    - left_middle_DIP_joint
    - left_index_MPR_joint
    - left_index_MCP_joint
    - left_index_PIP_joint
    - left_index_DIP_joint
    - left_thumb_MCP_joint
    - left_thumb_PIP_joint
    - left_thumb_DIP_joint
    - left_thumb_CMP_joint
    - left_thumb_CMR_joint
  points:
    - positions: [0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0]
      time_from_start: {sec: 3, nanosec: 0}
"

ros2 action send_goal \
  /revo3_left/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - left_little_MPR_joint
    - left_little_MCP_joint
    - left_little_PIP_joint
    - left_little_DIP_joint
    - left_ring_MPR_joint
    - left_ring_MCP_joint
    - left_ring_PIP_joint
    - left_ring_DIP_joint
    - left_middle_MPR_joint
    - left_middle_MCP_joint
    - left_middle_PIP_joint
    - left_middle_DIP_joint
    - left_index_MPR_joint
    - left_index_MCP_joint
    - left_index_PIP_joint
    - left_index_DIP_joint
    - left_thumb_MCP_joint
    - left_thumb_PIP_joint
    - left_thumb_DIP_joint
    - left_thumb_CMP_joint
    - left_thumb_CMR_joint
  points:
    - positions: [0.0, 1.4, 1.4, 1.4,
                  0.0, 1.4, 1.4, 1.4,
                  0.0, 1.4, 1.4, 1.4,
                  0.0, 1.4, 1.4, 1.4,
                  0.0, 0.0, 0.0, 0.0,
                  0.0]
      time_from_start: {sec: 3, nanosec: 0}
"
```

#### 发送目标：全手握拳（3 秒内到达）

```bash
ros2 action send_goal \
  /revo3_right/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - right_little_MPR_joint
    - right_little_MCP_joint
    - right_little_PIP_joint
    - right_little_DIP_joint
    - right_ring_MPR_joint
    - right_ring_MCP_joint
    - right_ring_PIP_joint
    - right_ring_DIP_joint
    - right_middle_MPR_joint
    - right_middle_MCP_joint
    - right_middle_PIP_joint
    - right_middle_DIP_joint
    - right_index_MPR_joint
    - right_index_MCP_joint
    - right_index_PIP_joint
    - right_index_DIP_joint
    - right_thumb_MCP_joint
    - right_thumb_PIP_joint
    - right_thumb_DIP_joint
    - right_thumb_CMP_joint
    - right_thumb_CMR_joint
  points:
    - positions: [0.18,0.80,0.50,0.05,
                  0.25,0.50,0.55,0.20,
                  0.00,0.50,0.45,0.15,
                  -0.2,0.85,0.50,0.10,
                  0.40,0.40,0.15,1.55,1.35]
      time_from_start: {sec: 1, nanosec: 0}
"
```

#### 发送目标：多段轨迹（展开 → 握拳 → 展开）

```bash
ros2 action send_goal \
  /revo3_right/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - right_little_MPR_joint
    - right_little_MCP_joint
    - right_little_PIP_joint
    - right_little_DIP_joint
    - right_ring_MPR_joint
    - right_ring_MCP_joint
    - right_ring_PIP_joint
    - right_ring_DIP_joint
    - right_middle_MPR_joint
    - right_middle_MCP_joint
    - right_middle_PIP_joint
    - right_middle_DIP_joint
    - right_index_MPR_joint
    - right_index_MCP_joint
    - right_index_PIP_joint
    - right_index_DIP_joint
    - right_thumb_MCP_joint
    - right_thumb_PIP_joint
    - right_thumb_DIP_joint
    - right_thumb_CMP_joint
    - right_thumb_CMR_joint
  points:
    - positions: [0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0]
      time_from_start: {sec: 2, nanosec: 0}
    - positions: [0.0, 1.2, 1.2, 1.2,
                  0.0, 1.2, 1.2, 1.2,
                  0.0, 1.2, 1.2, 1.2,
                  0.0, 1.2, 1.2, 1.2,
                  0.8, 0.8, 0.5, 0.3,
                  0.0]
      time_from_start: {sec: 5, nanosec: 0}
    - positions: [0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0]
      time_from_start: {sec: 8, nanosec: 0}
"
```

#### 只控制食指（partial goal，需 `allow_partial_joints_goal: true` ）

> 当前配置 `allow_partial_joints_goal: false`，以下命令需先修改 yaml 重启后方可使用。

```bash
ros2 action send_goal \
  /revo3_right/joint_traj_pos_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names:
    - right_index_MCP_joint
    - right_index_PIP_joint
    - right_index_DIP_joint
  points:
    - positions: [1.2, 1.2, 1.2]
      time_from_start: {sec: 2, nanosec: 0}
"
```

#### 查看 Action 状态

```bash
# 列出 Action Server
ros2 action list

# 查看 Action 类型
ros2 action info /revo3_right/joint_traj_pos_controller/follow_joint_trajectory
```

---

## 常见问题

**Q: 发了命令但手不动**

按顺序排查：

1. **命名空间是否匹配**：只启动了右手时，必须用 `/revo3_right/...`，不能用 `/revo3_left/...`。
2. **控制器是否 active**：`ros2 control list_controllers --controller-manager /revo3_right/controller_manager`
3. **是否用了正确的控制器/话题**：
   - 默认 MIT：`/revo3_right/joint_forward_mit_controller/commands`（`Revo3MITCommand`）
   - 位置直通：需先切换到 `joint_forward_pos_controller`，话题为 `.../joint_forward_pos_controller/commands`（`Float64MultiArray`）
4. **MIT 命令是否持续发布**：`command_timeout_sec=0.25`，`--once` 会很快超时；实体手测试用 `--rate 20`。
5. **kp/kd 是否为零**：硬件走 MIT 模态，`kp=0` 时无力矩。使用 MIT 控制器时请显式填 `kp/kd`；位置/轨迹控制器依赖驱动内置回退刚度（`kp=1.0`, `kd=0.1`）。
6. **关节反馈是否变化**：`ros2 topic echo /revo3_right/revo3_joint_state/joint_states`，若 position 不变则命令未到达硬件。

**Q: RViz 只能看到 TF，看不到手模型**

默认 `use_namespace:=true` 时，`robot_state_publisher` 把 URDF 发布到 `/revo3_{left|right}/robot_description`，而旧版 RViz 配置订阅的是全局 `/robot_description`（无发布者），因此只有 TF、没有 mesh。

临时修复（不重启 launch）：在 RViz 左侧 **RobotModel → Description Topic** 改为：

```text
/revo3_right/robot_description   # 右手
/revo3_left/robot_description    # 左手
```

永久修复：更新本仓库后重新 `colcon build` 并重启 launch；launch 会自动写入正确的 `robot_description` 话题。

**Q: `switch_controllers` 一直等待 service**

launch 未运行，或 `--controller-manager` 路径与 `hand_side` 不一致。

**Q: 控制器激活失败，提示 "Not existing" 接口**

确认 `revo3_controllers.yaml` 中的关节名与 `revo3.ros2_control.xacro` 一致。
固定关节（`fixed` 类型，如 `*_tip_Link`）不会生成 ros2_control 接口，不能作为控制关节。

**Q: Modbus 连接超时**

- 检查串口权限：`sudo usermod -aG dialout $USER`（重新登录生效）
- 检查波特率是否为 `5000000`（5 Mbps）
- 设置 `auto_detect: true` 让驱动自动扫描

**Q: `could not enable FIFO RT scheduling` 警告**

非致命警告，不影响功能。如需实时调度，参考：
https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
