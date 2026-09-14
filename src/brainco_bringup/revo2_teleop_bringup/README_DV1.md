# 新 SDK → adapter 内部 FK → Revo2（左手测试）

适用 SDK：`brainco_revohuman_sdk` 的 `sdk-encoder-fk-viewer` 分支（核对版本 67531b1）。
回退点：本仓库 `0579772` 是旧 HumanDex 上游四指方案。

## 数据和坐标定义

```text
SDK tools/ros2_joint_state_pub.py
  /humandex_left/joint_states (21 个 URDF 关节，弧度)
    → humandex_hand_adapter，input_mode=dv1_joint_states
      → 校验 / 可选 EMA → DV1 FK → 坐标转换（Revo2 消费端另算四指弯曲量）
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

## 1. 分别启动左手 SDK 和 adapter，先检查数据

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

先在独立终端加载 ROS 环境，再启动上游 SDK（此终端保持运行）：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  "$HOME/code/tele-retarget/brainco_revohuman_sdk/tools/ros2_joint_state_pub.py" \
  --hand left --port /dev/ttyACM0
```

在已加载 workspace 环境的另一个终端启动监听和 FK：

```bash
ros2 launch hand_input_adapters dv1_input.launch.py \
  hand_mode:=left \
  urdf_path:=$HOME/code/tele-retarget/brainco_revohuman_sdk/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf
```

SDK 路径和串口按本机位置修改，各终端必须使用相同 ROS_DOMAIN_ID。
输入入口不启动 SDK 或机器人。旧 `dv1_sdk.launch.py` 保留为 Revo2 组合入口，
同样只监听外部数据；已移除 `sdk_path`、`port`、`sdk_python`、`launch_sdk` 参数，
改为显式传入 `urdf_path`。

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
ros2 run revo2_hand_retarget calibrate_dv1_fingers --side left \
  --output "$PWD/flexion_dv1_left_measured.yaml"
```

按提示分别张开四指、握拳，每个姿态按 Enter 后保持两秒。
至少 30 帧、各弯曲关节变化至少 10°，抖动过大时拒绝保存。
它只写本地 YAML，不修改固件零位、URDF 或 FK 角度。
重新佩戴或固件标定改变后应重新核对端点。

停止步骤 1 的 launch，再启动目标检查：

```bash
ros2 launch revo2_teleop_bringup dv1_sdk.launch.py \
  hand_mode:=left \
  urdf_path:=$HOME/code/tele-retarget/brainco_revohuman_sdk/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf \
  finger_flexion_config:="$PWD/flexion_dv1_left_measured.yaml" \
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
  hand_mode:=left \
  urdf_path:=$HOME/code/tele-retarget/brainco_revohuman_sdk/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf \
  finger_flexion_config:="$PWD/flexion_dv1_left_measured.yaml" \
  launch_retarget:=true launch_revo2_driver:=true
```

这将启动整个左手的控制，包含拇指。启动后约 16 秒切换控制器，18 秒启动 retarget。

SDK 的 `--port` 是手套端口。Revo2 默认使用驱动里的
`/dev/revo2_hand_left` 别名；若尚未建立别名，可复制驱动的
`config/protocol_modbus_left.yaml` 为本地文件，将 `hardware.port` 改成已确认的
Revo2 串口，并在上面的启动命令添加
`revo2_protocol_config_file:=/绝对路径/revo2_left_local.yaml`。
该参数只传给当前选中侧的机器人驱动，不修改手套端口或默认驱动配置。
重启适配或控制链路前停止旧 adapter launch，SDK 采集可保持运行。不要同时启动两个适配器。

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

标定工具现输出 Revo2 flexion 配置；使用 `finger_flexion_config:=...` 加载。adapter 配置只管理采集/FK 与坐标。详见 [迁移说明](../../../docs/revo3_architecture_migration.md)。
