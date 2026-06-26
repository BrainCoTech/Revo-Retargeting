# MANUS Revo2 重定向

这个包用于把 MANUS 手套数据重定向到 Revo2 灵巧手。当前推荐真机链路是 `controller_backend:=ros2_control`：retarget 节点只发布目标关节角度，`revo2_driver` 内的 `revo2_pid_controller` 作为 ros2_control controller 直接读取硬件 state interface 并写 velocity command interface。

推荐链路是：

```text
MANUS SDK / manus_ros2
  -> manus_ros2_msgs/ManusGlove
  -> manus_revo2_retarget
  -> sensor_msgs/JointState target (rad)
  -> revo2_driver revo2_pid_controller
  -> Revo2 hardware velocity command interface (rad/s)
```

旧的 `python_topic` 后端仍然保留：`revo2_teleop_controller` 做 PD，输出 `Float64MultiArray` 到 `joint_forward_vel_controller`。它适合回退和对比，但不再是推荐真机路径。

## 控制后端

retarget 和控制后端使用这些配置文件：

```bash
config/retarget.yaml
config/teleop_controller.yaml
revo2_driver/config/revo2_controllers.yaml
```

`retarget.yaml` 负责把 MANUS 手套数据转换成 Revo2 目标关节角度，单位是 rad。

`controller_backend:=ros2_control` 时，`revo2_pid_controller` 在 ros2_control controller manager 内部完成 PD 速度闭环。输入是 `/revo2_<side>/revo2_pid_controller/target_joint_states`，输出是 `<joint>/velocity` command interface。

`controller_backend:=python_topic` 时，`teleop_controller.yaml` 负责把“目标关节角度 + Revo2 当前反馈角度”转换成速度 topic，单位是 rad/s：

```text
manus_revo2_retarget_node  # 只算目标关节角度 target
revo2_teleop_controller    # 做 PD 速度闭环，发布速度命令
```

旧后端下 Revo2 硬件侧使用 `joint_forward_vel_controller`。如果传 `use_split_controller:=false`，就回到更早的单节点路径：`manus_revo2_retarget_node` 一个节点同时负责 retarget 和速度命令发布。

默认 retarget 参数为：

```yaml
skip_calibration: true
use_default_revo3_calibration: true
algorithm: joint_thumb
```

当前默认使用 `joint_thumb` 拇指重定向算法。四指仍沿用 MANUS ergonomics 到 Revo2 近端关节的映射；拇指不再让 IK 自由决定两个主动关节，而是按 MANUS 拇指关节语义直接映射：

```text
ThumbMCPSpread -> Revo2 thumb_metacarpal_joint
ThumbMCPStretch / ThumbPIPStretch / ThumbDIPStretch -> Revo2 thumb_proximal_joint
```

这样做的目标是减少拇指 IK 多解，避免四指弯曲或四指参考中心变化时牵着 `thumb_meta` 乱跑。保留的 `revo3_thumb` 和 MuJoCo debug 接口主要用于诊断目标点和可视化，不作为当前默认拇指控制策略。

当前调好的效果直接保存在默认 `retarget.yaml` 中，正常启动不需要额外指定 `control_config`。

## Conda 环境

推荐使用和 MANUS retargeting 调试时一致的 conda 环境：

```bash
cd /path/to/Revo-Retargeting
conda create -n manusglove python=3.10
conda activate manusglove
python -m pip install --upgrade pip
python -m pip install rospkg catkin_pkg colcon-common-extensions
python -m pip install -r requirements.txt
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

如果环境已经存在，直接激活即可：

```bash
conda activate manusglove
```

注意 build 和 launch 要使用同一个 Python 环境。推荐每个新终端按下面顺序初始化：

```bash
conda activate manusglove
source /opt/ros/humble/setup.bash
cd /path/to/Revo-Retargeting
source install/setup.bash
```

## 构建

在 workspace 根目录执行：

```bash
cd /path/to/Revo-Retargeting
conda activate manusglove
python -m colcon build --packages-select manus_ros2_msgs manus_ros2 revo2_description revo2_driver manus_revo2_retarget --symlink-install
source install/setup.bash
```

如果 `manus_ros2` 和本地依赖已经构建过，也可以只构建 retarget 本包：

```bash
conda activate manusglove
python -m colcon build --packages-select manus_revo2_retarget
```

## 启动

### Hex 手套一键跑通 Revo2 PID

Hex 手套路径复用 `hex_glove_driver`，把 Windows Hex 上位机 UDP 数据转换成 `/manus_glove_0` 和 `/manus_glove_1`，后半段仍使用推荐的 `revo2_pid_controller`：

```text
Windows Hex 上位机
  -> hex_glove_udp_node
  -> /manus_glove_0 或 /manus_glove_1
  -> manus_revo2_retarget target-only
  -> /revo2_<side>/revo2_pid_controller/target_joint_states
  -> revo2_pid_controller
  -> Revo2
```

先构建 Hex bridge、Revo2 retarget 和 Revo2 driver：

```bash
cd /path/to/Revo-Retargeting
python -m colcon build --packages-up-to hex_glove_driver manus_revo2_retarget revo2_driver --symlink-install
source install/setup.bash
```

Windows 上启动 Hex 上位机，连接手套、完成校准并开启数据广播。然后在 Windows 上用 `ipconfig` 查 IPv4 地址，把下面命令里的地址换成实际地址：

```bash
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP>
```

这个 launch 会自动完成：

```text
启动 hex_glove_udp_node
  -> 启动 Revo2 driver
  -> 尝试自动切到 revo2_pid_controller
  -> 启动 target-only retarget
```

默认不会启动 MANUS publisher，也不会启动旧的 Python topic controller。关键默认值是：

```text
launch_manus_publisher=false
controller_backend=ros2_control
launch_plot=false
```

如果只想先检查 Hex 手套数据，不启动 Revo2：

```bash
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP> \
  launch_revo2_pipeline:=false
```

检查 Hex 数据：

```bash
ros2 topic hz /manus_glove_1
ros2 topic echo /manus_glove_1 --once
ros2 topic echo /hex_glove/raw_angles --once
ros2 topic echo /hex_glove/raw_positions --once
```

检查 Revo2 PID 链路：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

应看到 `revo2_pid_controller active`，并且 `target_joint_states` 有输出。左手把 `hand_mode:=right` 换成 `hand_mode:=left`，并检查 `/manus_glove_0` 和 `/revo2_left/revo2_pid_controller/target_joint_states`。

### 一键跑通（推荐）

右手真机 + MANUS 遥操作推荐显式使用 ros2_control 后端；不需要再单独执行 `ros2 launch revo2_driver ...`：

这个 launch 会依次完成四件事，并自动拉起 target/actual/error 图表：

```text
启动 Revo2 driver
  -> 尝试自动切到 revo2_pid_controller
  -> 启动 MANUS publisher + target-only retarget
  -> 启动 retarget plot monitor
```

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control
```

常用参数：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  switch_delay:=16.0 \
  retarget_delay:=18.0 \
  plot_delay:=19.0
```

`update_rate` 是 Revo2 driver 对硬件 read/write 的频率，不是 MANUS 发布频率。一键真机流程默认使用 `20 Hz`，比 `revo2_driver` 单独启动的默认 `500 Hz` 更适合当前 Modbus 右手链路；如果已经验证串口通讯足够稳定，再按现场情况调高。
`switch_delay` / `retarget_delay` / `plot_delay` 默认值已经按慢速 Modbus 启动留出余量；如果固定端口且启动很快，可以按现场情况再调小。

如果不想自动弹出 plot：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  launch_plot:=false
```

### Controller 默认逻辑

`revo2_driver` 单独启动时默认激活 `joint_forward_pos_controller`。这是有意保留的安全默认值：硬件 bring-up 阶段可以直接测试 position command，不依赖 MANUS 手套和 retarget target stream。

MANUS 遥操作的推荐链路使用 `revo2_pid_controller`，所以 `real_hand_pipeline_launch.py` 会尝试把 controller 从默认的 position controller 切到 pid controller。如果自动切换失败，可以手动查看：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers \
  -c /revo2_right/controller_manager
```

遥操作时理想状态是：

```text
revo2_joint_state            active
joint_forward_pos_controller inactive
revo2_pid_controller         active
joint_forward_vel_controller inactive
```

如果当前还是 `joint_forward_pos_controller active`，只 deactivate 当前 active 的 position controller，再 activate pid controller：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate revo2_pid_controller \
  --activate-asap \
  --strict
```

不要在 `--strict` 模式下 deactivate 已经 inactive 的 controller。比如 `joint_forward_vel_controller` 已经 inactive 时，不需要写进 `--deactivate`。

手动切换成功后，可以用 `switch_controllers:=false` 重新启动，避免重复运行自动切换脚本：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  switch_controllers:=false \
  launch_plot:=false
```

如果需要临时回退到旧的 Python topic 控制后端：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=python_topic
```

如果需要临时回退到更早的单节点控制路径：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  use_split_controller:=false
```

只画指定关节：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  plot_joints:=thumb_prox,thumb_meta,index \
  plot_window:=8
```

左手或双手：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py hand_mode:=left
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py hand_mode:=both
```

如果正在直接修改源码里的 YAML，而还没有重新 build，可以显式指定源码配置：

```bash
cd /path/to/Revo-Retargeting
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  control_config:=src/brainco_capabilities/manus_revo2_retarget/config/retarget.yaml
```

也可以只传包内配置文件名，launch 会从已安装的 `manus_revo2_retarget/config/` 下解析：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  control_config:=retarget.yaml
```

### 手动排查流程

需要单独排查每一环时，建议按下面顺序拆开跑。每个新终端都先初始化环境：

```bash
conda activate retarget_revo2
source /opt/ros/humble/setup.bash
cd /path/to/Revo-Retargeting
source install/setup.bash
```

#### 1. 启动 MANUS publisher

如果要单独确认 MANUS 右手是否在线，先启动：

```bash
ros2 run manus_ros2 manus_data_publisher
```

保持这个终端运行。另开终端检查右手数据：

```bash
ros2 node list | grep manus_data_publisher
ros2 topic info /manus_glove_0
ros2 topic echo /manus_glove_0 --once
```

右手正常时，`/manus_glove_0` 或 `/manus_glove_1` 中会有一只手的 `side: Right`。topic 编号不一定等于左右手，retarget 会按消息里的 `side` 字段分流。

#### 2. 启动 Revo2 hand driver

启动右手 Revo2 driver：

```bash
ros2 launch revo2_driver revo2_system.launch.py hand_side:=right
```

把右手切到 ros2_control PID controller：

```bash
ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --deactivate joint_forward_vel_controller \
  --activate revo2_pid_controller \
  --activate-asap \
  --strict
```

确认状态：

```bash
ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_joint_state/joint_states --once
```

应看到 `revo2_pid_controller active`，`joint_forward_pos_controller inactive`，`joint_forward_vel_controller inactive`。

如果要回退旧的 `python_topic` 后端，再切到 velocity topic controller：

```bash
ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --deactivate revo2_pid_controller \
  --activate joint_forward_vel_controller \
  --activate-asap \
  --strict
```

旧后端确认状态：

```bash
ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_joint_state/joint_states --once
ros2 topic info /revo2_right/joint_forward_vel_controller/commands
```

应看到 `joint_forward_vel_controller active`，`joint_forward_pos_controller inactive`。`/revo2_right/joint_forward_vel_controller/commands` 应有 `Subscription count: 1`，说明 velocity controller 正在等待速度命令。

#### 3. 启动 retarget

如果 MANUS publisher 和 Revo2 driver 已经单独启动，ros2_control 后端只需要启动 target-only retarget：

```bash
ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=right \
  launch_manus_publisher:=false \
  controller_backend:=ros2_control
```

这个 launch 会启动：

```text
manus_revo2_retarget       # MANUS -> target JointState
```

检查输出：

```bash
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
ros2 control list_controllers -c /revo2_right/controller_manager
```

`target_joint_states` 有输出，且 `revo2_pid_controller active` 时，新链路已经闭合。

如果使用旧的 Python topic 后端，启动 retarget + Python controller：

```bash
ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=right \
  launch_manus_publisher:=false \
  controller_backend:=python_topic
```

这个 launch 会启动两件事：

```text
manus_revo2_retarget       # MANUS -> target JointState
revo2_teleop_controller    # target + feedback -> rad/s velocity command
```

检查输出：

```bash
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
ros2 topic info /revo2_right/joint_forward_vel_controller/commands
```

旧后端下，`target_joint_states` 有输出，且 command topic 是 `Publisher count: 1` / `Subscription count: 1` 时，链路已经闭合。

#### 4. 旧后端完全单独启动 Python controller

日常不需要单独启动 Python controller；`pipeline_launch.py` 在 `controller_backend:=python_topic` 时会自动启动它。只有想把旧后端的 retarget 和 Python controller 也拆开排查时，才单独运行 `revo2_teleop_controller`。

新 ros2_control 后端的 `revo2_pid_controller` 不是普通 ROS node，不用 `ros2 run` 启动；它由 controller manager 加载和切换。

先只启动 target-only retarget：

```bash
ros2 run manus_revo2_retarget manus_revo2_retarget_node \
  --hand-mode right \
  --control-config retarget.yaml \
  --target-only \
  --right-target-joint-state-topic /revo2_right/revo2_pid_controller/target_joint_states
```

确认 retarget 有目标输出：

```bash
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

再单独启动 controller：

```bash
ros2 run manus_revo2_retarget revo2_teleop_controller \
  --ros-args \
  --params-file install/manus_revo2_retarget/share/manus_revo2_retarget/config/teleop_controller.yaml \
  -p hand_mode:=right
```

如果正在改源码里的 controller YAML 且还没重新 build，可以把 `--params-file` 换成：

```text
src/brainco_capabilities/manus_revo2_retarget/config/teleop_controller.yaml
```

注意不要同时用 `pipeline_launch.py` 和手动 `ros2 run revo2_teleop_controller` 启动两个 controller，否则同一个 velocity command topic 上会出现多个 publisher。

#### 快速定位

```text
Waiting for right target JointState
  -> 检查 MANUS publisher 是否在发 side: Right，以及 retarget 是否有 target publisher。

Waiting for right Revo2 feedback JointState
  -> 检查 /revo2_right/revo2_joint_state/joint_states 是否有 publisher。

revo2_pid_controller 不存在
  -> 检查 revo2_driver 是否重新 build/source，或者用 ros2 control load_controller -c /revo2_right/controller_manager --set-state inactive revo2_pid_controller 手动加载。

revo2_pid_controller inactive
  -> 检查是否仍有 joint_forward_pos_controller 或 joint_forward_vel_controller active，占用了 velocity command interface。

commands 是 Publisher count: 1 / Subscription count: 0
  -> 旧 Python controller 在发命令，但 Revo2 velocity controller 没在线或 namespace 不匹配。

commands 是 Publisher count: 0 / Subscription count: 1
  -> Revo2 velocity controller 在线，但 revo2_teleop_controller 没启动。
```

### 查看 target / actual / error

启动 retarget 后，可以开一个新终端画实时曲线；这个脚本只订阅 topic，不参与控制，也不需要重新 build：

```bash
python3 src/brainco_capabilities/manus_revo2_retarget/tools/revo2_retarget_plot.py --side right
```

图表分三行：

```text
position: target vs actual, rad
error:    target - actual, rad
velocity: command velocity, rad/s
```

只看拇指和食指：

```bash
python3 src/brainco_capabilities/manus_revo2_retarget/tools/revo2_retarget_plot.py \
  --side right \
  --joints thumb_prox,thumb_meta,index \
  --window 8
```

`hand_mode:=both` 会启动 `manus_revo2_retarget_left` 和 `manus_revo2_retarget_right` 两个独立进程；每个进程只初始化并计算对应手的 retargeting。

如果 `manus_ros2` 的 `manus_data_publisher` 已经由别的 launch 启动，可以关闭本 launch 里的 MANUS publisher：

```bash
ros2 launch manus_revo2_retarget pipeline_launch.py hand_mode:=right launch_manus_publisher:=false
```

### MuJoCo 可视化诊断

`mujoco_manus_overlay_viewer` 用来同时看 Revo2 模型、MANUS 原始关键点和 retarget 内部拇指目标点。它只订阅 topic，不参与控制，适合判断问题来自 MANUS 数据、目标生成，还是 Revo2 关节解算/跟踪。

启动 retarget 后另开终端：

```bash
ros2 run manus_revo2_retarget mujoco_manus_overlay_viewer --ros-args \
  -p hand_mode:=right
```

默认会订阅：

```text
/manus_glove_0
/manus_glove_1
/revo2_right/revo2_pid_controller/target_joint_states
/revo2_right/retarget/debug/thumb_ik_target
```

如果想看真手反馈而不是 retarget 目标，可以指定 joint topic：

```bash
ros2 run manus_revo2_retarget mujoco_manus_overlay_viewer --ros-args \
  -p hand_mode:=right \
  -p joint_topic:=/revo2_right/revo2_joint_state/joint_states
```

## 启动前检查

从顶层菜单选择 `HAND_TYPE=revo2` 并启动手部遥操作时，需要确认当前使用的是哪条控制后端。推荐新链路是 `revo2_pid_controller`；旧的 `python_topic` 链路才需要 `joint_forward_vel_controller`。`quick_start_*` 只负责本体硬件启动，`boot_teleop_arm.sh` 只负责臂部 MIT/Tracker。

手动排查或单独启动 Revo2 时，真机运行前需要确认对应手的目标 controller 是 active。以右手 ros2_control 后端为例：

```bash
ros2 control list_controllers -c /revo2_right/controller_manager
```

如果当前激活的是 position controller，需要切到 `revo2_pid_controller`：

```bash
ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --deactivate joint_forward_vel_controller \
  --activate revo2_pid_controller \
  --activate-asap \
  --strict
```

左手把 namespace 换成 `/revo2_left/controller_manager`。

## MANUS 官方手套标定

MANUS 官方手套标定用于校正手套本身，输出/加载 `.mcal` 文件。这个由 `manus_ros2` 里的 `manus_calibration_tool` 负责调用 MANUS SDK 完成。

### Quick Start

第一次拿到仓库时，可以先按下面的最短流程跑右手标定：

```bash
conda activate manusglove
source /opt/ros/humble/setup.bash
cd /path/to/Revo-Retargeting
python -m colcon build --packages-select manus_ros2 manus_revo2_retarget --symlink-install
source install/setup.bash

ros2 run manus_ros2 manus_calibration_tool --list
ros2 run manus_ros2 manus_calibration_tool --side right --overwrite
ros2 run manus_ros2 manus_data_publisher
```

左手把 `--side right` 换成 `--side left`。双手都要使用时，左右手各标定一次。

标定时先停止 `retarget launch` 和 `manus_data_publisher`。工具会在终端里提示每一步动作，准备好后按 Enter。

### 什么时候需要重新做 MANUS 标定

下面几种情况建议重新做一次官方 MANUS 标定：

- 换了使用者，手型明显不同。
- 手套重新佩戴后，指尖位置明显漂移。
- 拇指/无名指对指关系整体偏掉，不是单个关节小 offset 能解决。
- `.mcal` 文件丢失，或者 `manus_ros2` 启动日志提示没有加载到 calibration。

### 标定前准备

标定时建议先停止本包的 retarget launch 和 `manus_data_publisher`，避免多个 SDK client 同时占用手套。然后在新终端初始化环境：

```bash
conda activate manusglove
source /opt/ros/humble/setup.bash
cd /path/to/Revo-Retargeting
```

如果还没有构建过 `manus_ros2`，先构建并加载 workspace：

```bash
python -m colcon build --packages-select manus_ros2 manus_revo2_retarget --symlink-install
source install/setup.bash
```

如果已经构建过，直接加载 workspace：

```bash
source install/setup.bash
```

### 查看已连接手套

先确认 MANUS SDK 能看到手套：

```bash
ros2 run manus_ros2 manus_calibration_tool --list
```

如果有多只同侧手套，或者想固定使用某一只，可以记下输出里的 `glove id`，后续用 `--glove-id <id>` 指定。

### 运行标定

右手标定：

```bash
ros2 run manus_ros2 manus_calibration_tool --side right --overwrite
```

左手标定：

```bash
ros2 run manus_ros2 manus_calibration_tool --side left --overwrite
```

工具会连接 MANUS SDK，选择对应手套，然后打印每一步官方标定动作的 `title` 和 `description`。每一步准备好后按回车，按终端提示保持动作直到该步骤结束。

常见动作大概包括：

```text
1. 手放平，手指并拢，拇指向外
2. 抬手并握拳
3. 用指尖反复触碰掌根/掌心区域
```

具体动作以工具运行时 MANUS SDK 打印出来的提示为准。

标定完成后，工具会把当前手套的 calibration 保存为：

```text
<calibration_directory>/<glove_family>/Calibration_<side>.mcal
```

默认情况下，开发 workspace 会优先保存到源码包的 `manus_ros2/calibrations/` 下，例如：

```text
src/brainco_capabilities/manus_ros2/calibrations/metaglovepro/Calibration_right.mcal
src/brainco_capabilities/manus_ros2/calibrations/metaglove/Calibration_left.mcal
```

### 自定义标定目录

如果不想把 `.mcal` 放在源码目录，也可以指定目录：

```bash
ros2 run manus_ros2 manus_calibration_tool \
  --side right \
  --calibration-directory ~/Documents/manus-calibrations \
  --overwrite
```

读取时也要给 `manus_data_publisher` 指定同一个目录：

```bash
ros2 run manus_ros2 manus_data_publisher \
  --ros-args -p calibration_directory:=~/Documents/manus-calibrations
```

如果 retarget launch 仍然自动启动 `manus_data_publisher`，它会使用默认标定目录。需要自定义目录时，推荐手动启动 `manus_data_publisher`，再启动 retarget launch 时关闭内部 publisher：

```bash
ros2 launch manus_revo2_retarget pipeline_launch.py \
  hand_mode:=right \
  launch_manus_publisher:=false
```

### 验证 `.mcal` 是否加载成功

单独启动 MANUS publisher：

```bash
conda activate manusglove
source /opt/ros/humble/setup.bash
cd /path/to/Revo-Retargeting
source install/setup.bash

ros2 run manus_ros2 manus_data_publisher
```

如果 `.mcal` 放在非默认目录，可以显式指定：

```bash
ros2 run manus_ros2 manus_data_publisher \
  --ros-args -p calibration_directory:=/path/to/manus-calibrations
```

看到类似下面的日志，说明官方 MANUS calibration 已经被 `manus_ros2` 加载：

```text
Successfully loaded calibration for metaglovepro right glove ... from .../metaglovepro/Calibration_right.mcal
Successfully loaded calibration for metaglove left glove ... from .../metaglove/Calibration_left.mcal
```

然后确认手套话题有数据：

```bash
ros2 topic list | grep manus_glove
ros2 topic echo /manus_glove_0 --once
```

## 调参入口

retarget 映射参数在 `config/retarget.yaml`，控制参数在 `config/teleop_controller.yaml`。

`teleop_controller.yaml` 里的主要参数：

```yaml
pd_velocity:
  velocity_kp: 2.4
  velocity_kd: 0.0
  velocity_deadband: 0.013962634
  velocity_max: 2.094395102
  velocity_slew_rate: 1.0

  thumb_velocity_deadband: 0.020943951
  thumb_velocity_kp_scale: 2.4
  thumb_velocity_brake_zone: 0.209439510
  thumb_velocity_brake_max: 0.628318531

  ring_velocity_deadband: 0.020943951
  ring_velocity_kp_scale: 1.5

target_filter:
  alpha: 0.45
  fast_alpha: 0.9
  fast_threshold: 0.095993109
```

这些值都是 ROS 单位：目标和反馈是 rad，速度命令是 rad/s。`target_filter.*` 和 `thumb_velocity_deadband` 作用在 controller 层，只决定目标平滑和误差小于多少时不继续追。

`joint_thumb` 的主要拇指映射参数在 `retarget.yaml` 的 `revo3_thumb.right` 下：

```yaml
thumb_meta_range_deg: 30.0
thumb_meta_zero_deg: 0.0
thumb_prox_zero_deg: 10.0
thumb_prox_range_deg: 90.0
thumb_prox_mcp_weight: 0.45
thumb_prox_pip_weight: 0.35
thumb_prox_dip_weight: 0.20
```

`thumb_meta_range_deg` 越小，MANUS 较小的 `ThumbMCPSpread` 就越容易打满 Revo2 `thumb_metacarpal_joint`。如果中立位偏内摆，优先小幅增加 `thumb_meta_zero_deg`。`thumb_prox_zero_deg` / `thumb_prox_range_deg` 控制拇指弯曲映射，当前值用于让张开时被动 DIP 看起来更直。

如果拇指像“刹车点头”一样晃动，优先小幅降低 `thumb_velocity_kp_scale`，或把 `velocity_kd` 从 `0.01` 试到 `0.015`。`Kd` 不宜过大，MANUS 目标噪声会被放大。

如果某个关节存在稳定的反馈零点偏差，可以增加固定反馈修正：

```yaml
feedback:
  position_offsets: [0, 0, 0, 0, 0, 0]
  position_scales: [1, 1, 1, 1, 1, 1]
```

关节顺序为：

```text
thumb_proximal, thumb_metacarpal, index_proximal,
middle_proximal, ring_proximal, pinky_proximal
```
