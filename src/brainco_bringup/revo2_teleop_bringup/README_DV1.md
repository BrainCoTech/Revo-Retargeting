# 新 SDK → adapter 内部 FK → Revo2（左手测试）

适用 SDK：`brainco_revohuman_sdk` 的 `sdk-encoder-fk-viewer` 分支（核对版本 67531b1）。
回退点：本仓库 `0579772` 是旧 HumanDex 上游四指方案。

## 位置模式测试（feat/revo2-position-teleop）

`dv1_sdk.launch.py controller_backend:=position` 复用现有
`joint_forward_pos_controller`。默认的 `ros2_control` 选项仍是速度 PID。
位置模式的数据流为：

```text
adapter → HandKinematics → retarget（target-only）
  → /revo2_left/retarget/target_joint_states（JointState，rad）
  → revo2_teleop_controller（output_mode=position）
  → /revo2_left/joint_forward_pos_controller/commands（Float64MultiArray，rad）
  → BraincoHandHardware → SDK positions_and_speeds
```

六个角度顺序为拇指弯曲、拇指侧摆、食指、中指、无名指、小指。
手套 adapter、FK、四指标定和 retarget 的 IK 不因位置模式而改变。

### 构建和启动

以下构建复用已安装的消息包，输出到独立目录。adapter 和 FK 一起安装，
避免其他 overlay 的旧 `revohuman_kinematics` 遮蔽 `joint_fk`：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/dv1_sdk_test/local_setup.bash
colcon --log-base log/position_test build \
  --build-base build/position_test --install-base install/position_test \
  --symlink-install \
  --packages-select revo2_driver revo2_hand_retarget revo2_teleop_bringup \
    revohuman_kinematics hand_input_adapters \
  --allow-overriding revo2_driver revo2_hand_retarget revo2_teleop_bringup \
    revohuman_kinematics hand_input_adapters \
  --cmake-args -DBUILD_TESTING=ON -DENABLE_CANFD=OFF
```

先在原终端 Ctrl+C 停止上一套遥操作和占用手套串口的 viewer，然后启动左手：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
source /home/jiimmy/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo2
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/dv1_sdk_test/local_setup.bash
source install/position_test/local_setup.bash
export ROS_DOMAIN_ID=25

ros2 launch revo2_teleop_bringup dv1_sdk.launch.py \
  hand_mode:=left controller_backend:=position \
  port:=/dev/ttyACM0 \
  sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk \
  adapter_config:="$PWD/dv1_left_thumb_tip.yaml" \
  revo2_protocol_config_file:="$PWD/revo2_left_position.yaml" \
  launch_retarget:=true launch_revo2_driver:=true
```

`revo2_left_position.yaml` 指定机器人串口 `/dev/ttyUSB0`、从站 126。
这份配置是 **position + normalized**，目前只支持 Modbus。启动时通过有明确
错误返回的寄存器读取确认 normalized 模式，读取每个电机的最小/最大角度及最大速度，
指令和反馈按同一份范围换算。范围无效或首次反馈失败就拒绝激活，不用 URDF 范围兜底。
这些是固件配置范围，并非实测碰撞、接触或机械限位。代码不修改固件行程或电流保护。
该配置会拒绝激活 velocity 命令接口；切回速度模式须停掉驱动并换回速度 YAML。

### 调参和检查

- `revo2_left_position.yaml` 的 `position_speed_normalized: 1000.0` 是直接传 SDK 的
  **1～1000** 速度值，当前为固件配置的满速；不是旧的 `velocity_percentage: 100`。
- `revo2_hand_retarget/config/teleop_controller.yaml` 的 `position.rate_limit: 3.0`
  限制目标推进速度为 3.0 rad/s（约 172°/s），高于此前读取的固件 130～160°/s 上限。
  `position.max_lead: 0.25` 允许目标领先实际位置最多 0.25 rad，以适配 20 Hz 硬件反馈。
  可复制这份 YAML 后通过 `teleop_controller_config:=/绝对路径.yaml` 调整。
- 第一次输出从新鲜反馈开始。目标中断超过 0.3 s 后锁定一次反馈位置；反馈也过期时
  停止更新位置命令，恢复后从反馈重新开始。retarget 自身还有 0.3 s 输入超时。
  ForwardCommandController 会保留最后一条命令，反馈丢失或输出节点退出不等于硬件急停；
  它仍可能完成最后一小段目标运动，不能把“停止发布”理解为立即停止。

在同样 source 了环境并设置 ROS_DOMAIN_ID 的另一终端检查：

```bash
ros2 control list_controllers -c /revo2_left/controller_manager
ros2 topic echo /revo2_left/retarget/target_joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /revo2_left/joint_forward_pos_controller/commands std_msgs/msg/Float64MultiArray --once
ros2 topic echo /revo2_left/revo2_joint_state/joint_states sensor_msgs/msg/JointState --once
```

应看到 pos controller 为 active，速度 PID 和 forward velocity 为 inactive。
驱动调试日志 `mode=position_velocity` 表示发送位置目标及执行速度；`mode=speed`
才是仅发送速度。位置配置不会使 firmware 限位之外的姿态变得可达。

### 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/dv1_sdk_test/local_setup.bash
source install/position_test/local_setup.bash
python3 -m pytest -q src/brainco_capabilities/revo2_hand_retarget/checks/test_position_*.py
ctest --test-dir build/position_test/revo2_driver -R '^test_normalized_motor_limits$' --output-on-failure
python3 src/brainco_capabilities/revo2_hand_retarget/checks/position_mock_integration.py
```

模拟验证只启动 GenericSystem，在 localhost ROS domain 87 检查目标转换、实际
forward controller、反馈跟随和超时保持，不打开真机串口。真机执行效果需另行测试。

## 数据和坐标定义

```text
SDK tools/ros2_joint_state_pub.py
  /humandex_left/joint_states (21 个 URDF 关节，弧度)
    → humandex_hand_adapter，input_mode=dv1_joint_states
      → 校验 / 可选 EMA → DV1 FK → 坐标转换 + 四指弯曲量
        → /hand_kinematics/left
          → Revo2 retarget → 控制器
```

**FK 运行在 adapter 内部，没有额外 FK 节点，也不等待外部 PoseArray。**
FK 复用 `revohuman_kinematics/joint_fk.py` 和 `core.py`。SDK 已经完成
`wrap(deg - 180°)` 到弧度的转换；adapter 不再次减零位或翻关节符号。

每帧额外发布 `/humandex_left/fk_joint_states` 和 `/humandex_left/eef_pose`，
两者与 HandKinematics 保留同一个输入时间戳。前者是实际用于 FK 的角度，
后者是 **DV1 原生掌心坐标**，顺序为食指、中指、无名指、小指、拇指。
默认参考点是 DIP_Link 原点，姿态为该 link 姿态；`tip_offsets_m` 可设置五组
DIP 局部 XYZ 偏移，单位米。仅改变最后一个 DIP 角度时，零偏移点的位置不动。

配置中的 `left_palm_rpy_rad`、`left_palm_translation_m` 将原生点转换为
HandKinematics 点：`p_out = Rz(yaw) Ry(pitch) Rx(roll) p_native + translation`。
目前采用单位变换作为模型对照起点：两模型在零姿态下指向大体一致，但拇指零姿态、
尺寸和可达范围不同。**这不是已完成真机验证的工作空间映射**。Revo2 retarget
现有的 1.13 倍缩放、拇指 IK 和输出调节继续生效。

四指控制使用 DIP/PIP/MCP 分别归一化后的等权平均，不使用 MPR。
FK 仍使用全部 21 个关节。左手范围默认为临时的 0～90°；必须通过实际姿态核对。
右手提供由旧记录转换的单独配置，不能替代左手测量。

## 构建

```bash
cd ~/Brainco/Code/Revo-Retargeting
source /home/jiimmy/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo2
source /opt/ros/humble/setup.bash
source install/setup.bash
python -m colcon --log-base log/dv1_sdk_test build --base-paths src \
  --packages-select hand_teleop_msgs manus_ros2_msgs revohuman_msgs \
    revohuman_kinematics hand_input_adapters revo2_hand_retarget revo2_teleop_bringup \
  --build-base build/dv1_sdk_test --install-base install/dv1_sdk_test \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
    -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
```

`install/setup.bash` 提供已安装的 Revo2 驱动等依赖。SDK ROS 脚本用系统
`/usr/bin/python3` 运行，需能 `import serial, yaml`；FK/retarget 使用当前环境。

## 1. 启动左手 SDK + adapter，先检查数据

停止旧手套串口读取程序、旧 adapter 和旧真机控制程序。若串口不是 ttyACM0，修改 port。
新终端的公共环境如下（后续终端也需要）：

```bash
cd ~/Brainco/Code/Revo-Retargeting
source /home/jiimmy/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo2
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/dv1_sdk_test/local_setup.bash
export ROS_DOMAIN_ID=25
```

```bash
ros2 launch revo2_teleop_bringup dv1_sdk.launch.py \
  hand_mode:=left port:=/dev/ttyACM0 \
  sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk
```

默认只启动 SDK 和 adapter，不启动 retarget 或真机驱动。如果新 SDK 发布程序已经
单独在运行，添加 `launch_sdk:=false`，保持它的域和手别一致。

```bash
ros2 topic echo /humandex_left/eef_pose geometry_msgs/msg/PoseArray --once
ros2 topic echo /hand_kinematics/left hand_teleop_msgs/msg/HandKinematics --once
```

源话题需 `frame_id=left_palm_link` 且具有恰好 21 个左手关节；无效路 NaN、
缺失/重复关节、时间戳倒退或超过 0.5 秒的帧会丢弃。默认 EMA alpha=0.2，
超过 0.5 秒的有效帧间隔后重置滤波。SDK 发布时打主机时间戳，因此该接口只能
检测消息层的新鲜度，不能还原固件采样时刻或额外的编码器状态掩码。

## 2. 采集左手四指范围

保持 SDK + adapter 运行，在另一个已加载公共环境的终端：

```bash
ros2 run hand_input_adapters calibrate_dv1_fingers --side left \
  --output "$PWD/dv1_left_measured.yaml"
```

按提示分别张开四指、握拳，每个姿态按 Enter 后保持两秒。
至少 30 帧、各弯曲关节变化至少 10°，抖动过大时拒绝保存。
它只写本地 YAML，不修改固件零位、URDF 或 FK 角度。
重新佩戴或固件标定改变后应重新核对端点。

停止步骤 1 的 launch，再启动目标检查：

```bash
ros2 launch revo2_teleop_bringup dv1_sdk.launch.py \
  hand_mode:=left port:=/dev/ttyACM0 \
  sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk \
  adapter_config:="$PWD/dv1_left_measured.yaml" \
  launch_retarget:=true
```

## 3. 检查四指和拇指目标

```bash
ros2 topic echo /revo2_left/revo2_pid_controller/target_joint_states sensor_msgs/msg/JointState --once
python "$(ros2 pkg prefix revo2_teleop_bringup)/share/revo2_teleop_bringup/tools/inspect_dv1_thumb.py" \
  --side left --csv /tmp/dv1_left_thumb.csv
```

慢慢张开、握拳核对四指方向；拇指依次并拢、外展、靠近小指根部，各保持几秒。
监测器显示原生 FK 点、转换后点、IK 目标、两路目标角度，以及目标角度在 Revo2
模型中的末端误差。该误差包含输出偏置/缩放/滤波影响，**不是实际真机误差，也不是
IK 求解器内部残差**；IK/指令按不超过 50ms 的时间差近似配对。
若持续限位或误差很大，保留 CSV 排查坐标和工作空间；尤其检查 SDK 的 ID 20。

## 4. 真机

目标检查通过后，停止步骤 2 的 launch，重启时加 `launch_revo2_driver:=true`：

```bash
ros2 launch revo2_teleop_bringup dv1_sdk.launch.py \
  hand_mode:=left port:=/dev/ttyACM0 \
  sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk \
  adapter_config:="$PWD/dv1_left_measured.yaml" \
  launch_retarget:=true launch_revo2_driver:=true
```

这将启动整个左手的控制，包含拇指。启动后约 16 秒切换控制器，18 秒启动 retarget。

`port` 是手套端口，不是 Revo2 端口。Revo2 默认使用驱动里的
`/dev/revo2_hand_left` 别名；若尚未建立别名，可复制驱动的
`config/protocol_modbus_left.yaml` 为本地文件，将 `hardware.port` 改成已确认的
Revo2 串口，并在上面的启动命令添加
`revo2_protocol_config_file:=/绝对路径/revo2_left_local.yaml`。
该参数只传给当前选中侧的机器人驱动，不修改手套端口或默认驱动配置。
重启整条链路前务必停止之前的 SDK + adapter launch，避免串口独占冲突或重复发布。

## 离线验证

```bash
export DV1_SDK_PATH=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk
export PYTHONPATH="$PWD/src/brainco_capabilities/hand_input_adapters:$PWD/src/brainco_capabilities/revohuman_kinematics:$PYTHONPATH"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  src/brainco_capabilities/revohuman_kinematics/checks/test_joint_fk.py \
  src/brainco_capabilities/hand_input_adapters/checks \
  src/brainco_capabilities/hand_input_adapters/test
```

包含左右手与 SDK MuJoCo 的五指位姿对照、DIP 局部偏移、无效帧、角度跨界、
标定范围和左手消息转换。ROS 通信测试需单独设置 `ROS_DOMAIN_ID=126`、
`DV1_ROS_GRAPH_TEST=1`，禁止在真机域运行合成数据测试。

本次验证：17 项计算/消息回归测试和 2 项隔离 ROS 通信测试通过。
MuJoCo 对照覆盖左右手各 21 组关节角，位置/旋转矩阵容差 1e-9。
安装后的 launch 已用合成左手数据验证到 Revo2 六关节目标输出，未连接真机。
另用三组合成拇指姿态回放现有 IK，输出可变化，但内部位置残差约 65～83mm。
因此数据接入和 FK 算法已验证，左手实际零姿态、四指范围和拇指工作空间仍需实测。
