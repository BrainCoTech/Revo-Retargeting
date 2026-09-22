# RevoHuman SDK → adapter 内部 FK → Revo2

适用 SDK：`brainco_revohuman_sdk` 的 `feat/tracker-world-frame-and-bringup` 分支。
接口依据 [Issue #13](https://github.com/HAOTianGa03/brainco_revohuman_sdk/issues/13) 核对提交 `2683152`（2026-09-20）。以下以左手为例；本次适配不代表新版 SDK 已完成真机或 ROS 端到端验证。

## 数据和坐标定义

```text
SDK revohuman_bringup / revohuman.launch.py mode:=left
  /revohuman/left/joint_states (21 个无手侧前缀的关节名，弧度)
    → humandex_hand_adapter，input_mode=dv1_joint_states
      → SDK 名称适配 / 校验 / 可选 EMA → DV1 FK → 坐标转换
        → /hand_kinematics/left
          → Revo2 retarget → 控制器
```

**FK 运行在 adapter 内部，没有额外 FK 节点，也不等待外部 PoseArray。**
FK 复用 `revohuman_kinematics/joint_fk.py` 和 `core.py`。SDK 已经按 joint_map
完成符号、偏移和弧度转换；adapter 仅适配名称，不再次减零位、翻符号或换算角度。

`joint_state_layout` 显式区分三种输入，均按关节名读取，不依赖数组顺序：

| 布局 | 默认源话题 | 关节名 | 默认源 `frame_id` |
| --- | --- | --- | --- |
| `sdk_single`（单手入口默认） | `/revohuman/{side}/joint_states` | 21 个无手侧前缀名称，如 `index_DIP_joint` | `revohuman_left` / `revohuman_right` |
| `sdk_pair` | `/revohuman/pair/joint_states` | 双手共 42 个，带 `left_` / `right_` 前缀 | `revohuman_pair` |
| `legacy` | `/humandex_{side}/joint_states` | 21 个，带所选侧前缀 | `left_palm_link` / `right_palm_link` |

adapter 以 BEST_EFFORT / VOLATILE 订阅 SDK 数据。`source_frame_id` 可覆盖源帧校验值，
它与内部 FK 使用的 `{side}_palm_link` 独立；不能通过更改源帧标识改变 FK 坐标。

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

## SDK 准备（独立工作区）

SDK 和本仓库分别构建、分别启动。SDK 已有官方 `revohuman_msgs`；本仓库保留的旧同名包
只提供 `RawFrame`。不要把 SDK ROS 包放入本仓库 `src`，也不要在同一终端叠加 source 两个工作区。

在 ROS 2 Humble / Python 3.10 的新终端准备 SDK checkout，确认处于上述分支。
`2683152` 是本次依据的接口版本；`main` / `develop` 不提供相同的 bringup 入口。

```bash
source /opt/ros/humble/setup.bash
SDK_ROOT="$HOME/code/tele-retarget/brainco_revohuman_sdk"
cd "$SDK_ROOT"
git branch --show-current
git rev-parse --short HEAD
python3 -m pip install -e .
mkdir -p ~/ros2_revohuman_ws
cd ~/ros2_revohuman_ws
colcon build --base-paths "$SDK_ROOT/ros2" --symlink-install
source install/setup.bash
```

原生 `revohuman` Python 库必须安装到运行 SDK ROS 节点的 Python 环境；`colcon build`
不会替代 `pip install -e .`。三个 ROS 包为 `revohuman_msgs`、`revohuman_ros`、
`revohuman_bringup`；即使关闭触觉也要构建官方消息包。只发关节角无需安装 recording、camera
extra 或 tracker 的 libsurvive。

首次使用先扫描设备，将 `usb_topology_id` 填入注册表，左右手由人声明：

```bash
python3 -c "from revohuman import GloveManager; [print(d.usb_topology_id, d.port, d.registered) for d in GloveManager().scan()]"
```

参考 SDK 的 `examples/glove_sets.example.yaml` 创建 `~/.config/revohuman/glove_sets.yaml`，
保留已有有效注册内容。下面的 hub 是占位符，须替换为扫描结果；仅使用一只手时只登记该侧。

```yaml
sets:
  bench:
    left:
      hub: "3-2"
    right:
      hub: "3-3"
```

不要按 `/dev/ttyACM*` 编号或 USB serial 猜手别。`REVOHUMAN_REGISTRY` 可指定注册表路径，
`set_name:=bench` 选择组。确认当前用户能读写设备，再运行 `python3 -m revohuman doctor`。
`doctor --probe` 会打开串口，仅在采集进程停止后使用。

## 构建本仓库

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

`install/setup.bash` 提供已安装的 Revo2 驱动等依赖。FK/retarget 使用当前环境，
SDK 则使用其独立工作区和 Python 环境。

## 1. 分别启动左手 SDK 和 adapter，先检查数据

停止旧手套串口读取程序、旧 adapter 和旧真机控制程序。
本仓库各终端的公共环境如下；SDK 终端使用下一段的独立环境：

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
source ~/ros2_revohuman_ws/install/setup.bash
export ROS_DOMAIN_ID=25
ros2 launch revohuman_bringup revohuman.launch.py mode:=left set_name:=bench \
  tracker_mode:=off publish_tactile:=off camera_mode:=off
```

在已加载 workspace 环境的另一个终端启动监听和 FK：

```bash
ros2 launch hand_input_adapters dv1_input.launch.py \
  hand_mode:=left \
  urdf_path:=$HOME/code/tele-retarget/brainco_revohuman_sdk/description/urdf/Revo_Human_DV1_URDF_Bimanual.urdf
```

SDK 路径和 registry 组名按本机配置修改，各终端必须使用相同 ROS_DOMAIN_ID。
`tracker_mode:=off` 允许不接 tracker，`publish_tactile:=off` 避免纯关节采集受触觉门影响。
可用 `rate_hz:=200.0` 改速率，浮点参数要带小数。输入入口不启动 SDK 或机器人。
`dv1_sdk.launch.py` 是 Revo2 组合入口，同样只监听外部数据，显式接收 `urdf_path`。

先检查 SDK 话题及其 QoS，再检查 adapter 输出：

```bash
ros2 topic info /revohuman/left/joint_states -v
ros2 topic echo /revohuman/left/joint_states --once --qos-reliability best_effort
ros2 topic echo /humandex_left/eef_pose geometry_msgs/msg/PoseArray --once
ros2 topic echo /hand_kinematics/left hand_teleop_msgs/msg/HandKinematics --once
```

单手默认校验 `frame_id=revohuman_left` 及 21 个无手侧前缀的 SDK 关节名。
无效路 NaN、缺失/重复关节、时间戳倒退或超过 0.5 秒的帧会丢弃。默认 EMA alpha=0.2，
超过 0.5 秒的有效帧间隔后重置滤波。新版 SDK 的 `header.stamp` 是采样时间
（`sample_wall_ns` / CLOCK_REALTIME），adapter 原样保留，不重新打到达时间戳。

### 双手与旧输入

SDK 用 `mode:=pair` 发布 42 关节时，每侧 adapter 显式传 `joint_state_layout:=sdk_pair`；
`hand_mode:=left` / `right` 决定从同一条 pair 消息提取哪只手。SDK 的 pair 模式需要两只手，
缺一侧时不发布关节消息。不要同时启动 pair 与单手采集，也不要并行启动旧 raw driver。

Revo3 的 `bash scripts/teleop.sh both input_source:=dv1` 默认使用 `sdk_pair`。
如果上游分别运行 `mode:=left` 和 `mode:=right`，则使用
`bash scripts/teleop.sh both input_source:=dv1 joint_state_layout:=sdk_single`。
单侧 launch 用 `source_frame_id:=...` 匹配 SDK 自定义源帧；teleop 脚本对应
`left_source_frame_id:=...` / `right_source_frame_id:=...`。

旧发布者仍可显式选择 `joint_state_layout:=legacy joint_topic:=/humandex_left/joint_states`，
其输入需满足上表的旧名称和帧契约。这两个参数及 `source_frame_id` 同样可用于 Revo2 的
`dv1_sdk.launch.py`；不会启动旧采集进程。

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

SDK 通过 registry 定位手套。Revo2 默认使用驱动里的
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

以下为旧 `sdk-encoder-fk-viewer` 分支（67531b1）的历史验证记录，不作为本次新版 SDK 的验证结果：
17 项计算/消息回归测试和 2 项隔离 ROS 通信测试通过。
MuJoCo 对照覆盖左右手各 21 组关节角，位置/旋转矩阵容差 1e-9。
安装后的 launch 已用合成左手数据验证到 Revo2 六关节目标输出，未连接真机。
另用三组合成拇指姿态回放现有 IK，输出可变化，但内部位置残差约 65～83mm。
新版 SDK 的 ROS 通信、采样时间和真机表现仍需在目标环境验证；左手实际零姿态、四指范围和拇指工作空间仍需实测。

标定工具现输出 Revo2 flexion 配置；使用 `finger_flexion_config:=...` 加载。adapter 配置只管理采集/FK 与坐标。详见 [迁移说明](../../../docs/revo3_architecture_migration.md)。
