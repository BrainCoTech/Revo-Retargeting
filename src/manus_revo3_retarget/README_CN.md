# MANUS → Revo3 重定向与在线校准

[English](README.md)

本包接收 MANUS 手套数据，用 C++ 节点计算 Revo3 关节目标，再以默认 200 Hz 发布经过线性插值的 MIT 指令。拇指 IK 使用 Pinocchio 和 `revo3_description` 中的 URDF。本文说明侧摆方向、四指屈曲和对指校准；录制与曲线工具的完整选项见[英文说明](README.md#launch-and-record-mcap)。

## 构建和启动

在仓库根目录执行，先初始化驱动子模块。首次构建使用 `--packages-up-to`，同时构建工作空间内依赖：

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to manus_ros2 revo3_driver manus_revo3_retarget
source install/setup.bash
```

驱动、模型和 Revo3 SDK 由 `src/brainco_revo3_ros2` 子模块提供。主仓库更新后，先执行 `git submodule sync --recursive` 和 `git submodule update --init --recursive`，同步主仓库固定的驱动 SHA。维护者跟进 `develop` 及修复分支审阅状态见[驱动子模块版本](../../README_CN.md#驱动子模块版本)。

更新本功能后，需要重新构建 `revo3_driver` 并重启手部驱动，使退出校准时的零增益释放生效。旧驱动会把显式的零增益替换成非零默认值。

以下以右手为例，在两个终端中分别执行；每个终端都先进入仓库并 source 工作空间：

```bash
# 终端 1：启动右手驱动。
./scripts/start_driver.sh right
```

```bash
# 终端 2：启动 MANUS、右手重定向节点和校准面板。
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right launch_tuning_panel:=true
```

双手使用 `./scripts/start_driver.sh both` 和 `hand_mode:=both`；左右两路重定向各在独立进程中运行。`start_driver.sh` 和 pipeline 的默认手侧均为双手，单手使用时显式写出 `left` 或 `right`。

如果 MANUS 发布节点已经运行，在 pipeline 命令中加 `launch_manus_publisher:=false`。如果重定向节点也已经运行，只打开面板：

```bash
ros2 run manus_revo3_retarget retarget_tuning_panel --side right --operator alice
```

面板使用 Tk，需要图形桌面；面板、MANUS 和驱动应使用相同的 `ROS_DOMAIN_ID`。同一设备只启动一份驱动和 MANUS 发布节点。

## 三阶段校准

直接启动节点或 pipeline 时默认关闭输出。面板连接后先关闭输出，选择当前手，再点击“启用当前手输出”。

| 面板阶段 | `retarget_calibration_mode` | 关节目标 |
| --- | --- | --- |
| 侧摆 / MPR | `spread_only` | 仅四指 MPR；四指屈曲和拇指目标强制为 0° |
| 四指屈曲 | `four_finger_only` | 仅四指 MCP/PIP/DIP；MPR 和拇指目标强制为 0° |
| 拇指 / 对指 | `full` | 全部关节，保留前两阶段的参数 |

前两阶段在最终输出补偿后执行置零，因此非活动关节的 offset 不会改变其零目标。阶段切换、换手和重新使能会清除旧插值目标；使能后等待新手套帧才发送运动指令。校准阶段的零角度目标会让非活动关节朝零位运动，退出释放则通过零增益撤去主动保持力矩。

## 侧摆方向与对指校准

参数中的 `<side>` 为 `left/right`，四指 `<finger>` 为 `index/middle/ring/little`，分别是食指、中指、无名指和小指。

| 现象 | 调整方法 | 基础配置文件 |
| --- | --- | --- |
| 某根手指侧摆方向相反 | 将 `physical_<side>_<finger>_MPR_joint_scale` 的当前值取负，保持绝对值 | `config/spread_retarget.yaml` |
| 侧摆方向正确，并拢位置偏 | 调该指 `physical_<side>_<finger>_MPR_joint_offset_deg` | 同上 |
| 侧摆幅度太大或太小 | 保留 MPR scale 的符号，调绝对值 | 同上 |
| 某根四指弯曲幅度不合适 | 调 `physical_<side>_<finger>_<joint>_joint_scale`，其中 `<joint>` 为 MCP/PIP/DIP | `config/four_finger_retarget.yaml` |
| 四指某关节存在固定角度偏移 | 调对应 `physical_<side>_<finger>_<joint>_joint_offset_deg` | 同上 |
| 对指时拇指掌根姿态偏 | 逐个调 `physical_<side>_thumb_CMP_joint_offset_deg`、`physical_<side>_thumb_CMR_joint_offset_deg` | `config/thumb_retarget.yaml` |
| 拇指弯曲不足或过度 | 调 `physical_<side>_thumb_<joint>_joint_scale` 或 `..._offset_deg`，其中 `<joint>` 为 MCP/PIP/DIP | 同上 |

最终补偿关系为：

```text
q_target_rad = scale × q_mapped_rad + offset_deg × π / 180
```

`q_mapped` 已经过重定向算法和 legacy 映射处理。拇指 IK 内部也有 legacy 补偿和关节限位截断；上式执行后，节点没有再按 URDF 限位截断最终输出。调参时小步修改，并观察指令与实际反馈角度。

松开滑块、在输入框按 Enter 或点击 `+` / `−` 都会应用参数。scale 每步 `0.05`，offset 每步 `0.5°`。面板中 MPR scale 范围为 `-2.5..2.5`、offset 为 `±40°`；四指屈曲和拇指 scale 为 `0..2.5`、offset 为 `±30°`。这些数值是控件范围，机械运动范围需结合实际关节检查。

### 建议校准顺序

1. 确认 MANUS 左右手对应正确，反复做伸直、并拢、张开和弯曲动作，输入应稳定、可重复。
2. 进入侧摆阶段，四指保持伸直，逐根检查张开、收拢方向。反向时只将当前最终 MPR scale 取负：例如 `1.0` 改为 `-1.0`，已有 `-1.2` 则改为 `1.2`。随后调并拢 offset 和张开幅度。
3. 进入四指屈曲阶段，检查伸直、半握和握拳。每次只调一个 MCP/PIP/DIP 参数，调整幅度后复查伸直姿态。
4. 进入拇指阶段，依次检查拇食、拇中、拇无名、拇小指接触。先调拇指 CMP/CMR offset，再调拇指屈曲。只有某根四指接触偏时，再检查该指 MPR 和屈曲参数。offset 应增加还是减小，按该只手的实际运动判断。
5. 每次修改后，复查四组对指和张手姿态；完成后导出当前手的 YAML，重启显式加载，读回参数并重复这些动作。

拇指阶段开放全部关节输出，当前算法没有自动拟合拇指与四指接触的约束。常规校准先使用表中的最终输出参数；原有 legacy 映射保持不动，避免在多层同时取反。

面板还保留 `legacy_<side>_physical_middle_spread_dynamic`，以及拇指几何参数 `legacy_<side>_physical_thumb_reach_scale`、`..._ik_position_scale`，IK 任务权重 `..._pip_ik_scale`、`..._dip_ik_scale` 和 EMA 平滑权重 `..._ema_prev`、`..._ema_cur`。这些参数分别影响原有侧摆规则、拇指目标几何、IK 权重和平滑，建议完成逐关节补偿后再单独调整。

## 保存、历史记录与加载

填写遥操员 ID 和可选标定名，对每只手分别点击“Export Override YAML”。ID 和标定名可用字母、数字、下划线和横线。默认保存结构：

```text
~/.brainco/retarget_calibration/
  library/left/<operator>_<name>_<timestamp>.yaml
  library/left/latest.json
  library/right/...
  <operator>_revo3_left.yaml
  <operator>_revo3_right.yaml
```

每次导出保存时间戳快照，更新该侧 latest 指针和遥操员兼容副本；被替换文件保留 `.bak`。导出包括面板调节参数，不含输出使能、临时校准阶段、MIT 增益及基础配置中其他 legacy 映射参数。历史记录“Set as latest”更新保存的选择及当前遥操员兼容副本，运行中的节点参数保持当前值。

可以用面板的 `--calibration-dir` 指定目录、`--operator` 指定 ID。未指定时，面板读取 `$ROBOT_CONFIG` 或 `~/.brainco/robot_config/robot_config.json` 中的 `teleop_operator` 和 `retarget_calibration_dir`；没有 ID 时使用当前系统用户名。

pipeline 启动时不自动解析 `latest.json`，应显式传入导出的 YAML 或兼容副本：

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=both launch_tuning_panel:=true \
  left_calibration_config:="$HOME/.brainco/retarget_calibration/alice_revo3_left.yaml" \
  right_calibration_config:="$HOME/.brainco/retarget_calibration/alice_revo3_right.yaml"
```

### 最小右手覆盖文件

假设读到右食指最终 MPR scale 为 `1.0`，确认需反向，且并拢 offset 保持 `0.0°`，可将下列内容保存为自己的 `revo3_manus_right.yaml`：

```yaml
manus_revo3_retarget:
  ros__parameters:
    physical_right_index_MPR_joint_scale: -1.0
    physical_right_index_MPR_joint_offset_deg: 0.0
```

只写经过检查、需要覆盖的参数。加载示例：

```bash
ros2 launch manus_revo3_retarget pipeline_launch.py \
  hand_mode:=right launch_tuning_panel:=true \
  right_calibration_config:=/绝对路径/revo3_manus_right.yaml
```

### 配置加载优先级

配置依次加载，后面的同名参数覆盖前面：

```text
control_config → thumb_retarget_config → four_finger_retarget_config
→ spread_retarget_config → 可选 retarget_config → 校准覆盖文件
```

`calibration_config` 非空时，其内容应用到所有启动的手，此时忽略 `left_calibration_config` 和 `right_calibration_config`。未传共同文件时，各手只加载自己的独立覆盖文件。最后，launch 的运行字段 `hand_mode`、`start_enabled`、`use_revo3_namespace`、`command_topic_suffix`、`retarget_target_topic_suffix` 和 `mit_command_publish_hz` 覆盖文件中的同名值。

## 检查参数、输出和反馈

先确认实际节点名。单手 `hand_mode:=left/right` 使用 `/manus_revo3_retarget`；双手使用 `/manus_revo3_retarget_left` 和 `/manus_revo3_retarget_right`。面板默认连接带侧别后缀的节点，单手启动时自动回退到无后缀节点；也可用 `--node`、`--left-node`、`--right-node` 显式指定。

```bash
ros2 node list
# 右手单手启动：
ros2 param get /manus_revo3_retarget physical_right_index_MPR_joint_scale
ros2 param get /manus_revo3_retarget physical_right_index_MPR_joint_offset_deg
# 双手启动时查看右手：
ros2 param get /manus_revo3_retarget_right physical_right_index_MPR_joint_scale
```

默认命名空间下，右手的目标、插值后指令与实际反馈分别查看：

```bash
ros2 topic echo /revo3_right/joint_forward_mit_controller/retarget_targets --once
ros2 topic echo /revo3_right/joint_forward_mit_controller/commands --once
ros2 topic echo /revo3_right/revo3_joint_state/joint_states --once
ros2 launch manus_revo3_retarget command_state_viewer.launch.py hand_mode:=right
```

MIT 消息按 `joint_names`、JointState 按 `name` 对应关节后再比较 `position`，角度单位为弧度。`retarget_targets` 是插值前目标，`commands` 是定时器输出，`joint_states` 是驱动反馈。左手将话题中的 `right` 改成 `left`。

## 关闭输出和退出释放

面板关闭输出时，节点先为 21 个关节发送零 `kp`、零 `kd`、零速度和零力矩，再停止常规发布。关闭窗口、Ctrl+C、SIGTERM 走相同释放流程；曾连接的节点未确认释放请求时，窗口会显示错误并保留以便重试。节点在正常 ROS 退出前也发送释放。

由 `launch_tuning_panel:=true` 启动的面板退出后，pipeline 及其 MANUS 发布节点同时停止；独立启动的手部驱动继续运行。释放撤去主动保持力矩，机械摩擦仍存在，手不会被命令回到零姿态。节点为每只手最多等待 4 秒的可靠 DDS 确认；面板正常关闭输出或退出时，为释放服务调用预留 10 秒，覆盖节点同时管理双手的情况。DDS 确认只反映消息交付，电机状态仍需看实际反馈；强制 `kill -9`、通信中断或同时关闭驱动时无法保证完成释放。

不用面板做普通遥操时，`scripts/teleop.sh` 和 `run_pipeline_record_mcap.sh` 会显式传 `start_enabled:=true`。直接 launch 可添加该参数自动输出，但它与 `launch_tuning_panel:=true` 不能同时使用。运行时通过服务启停，`start_enabled` 只在启动时读取：

```bash
# 双手启动时的右手节点：
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: true}'
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: false}'
```

单手启动时将服务前缀改为 `/manus_revo3_retarget/enable_retarget`，双手左手改为 `/manus_revo3_retarget_left/enable_retarget`。

## 验证

构建并 source 后运行现有测试：

```bash
python3 -m pytest src/manus_revo3_retarget/test
REVO3_CALIBRATION_INTEGRATION=1 python3 -m pytest \
  src/manus_revo3_retarget/test/test_staged_calibration_ros.py
```

第二条是显式开启的 ROS 集成测试，使用独立测试命名空间和合成手套数据；全部输入输出都重映射，不向硬件控制话题发指令。面板交互测试还需要 `DISPLAY`。测试通过后，仍应在真实 MANUS 和手部反馈上检查侧摆方向、四指屈曲、四组对指、保存重载及退出释放。

录制 MCAP、五次多项式关节测试和曲线查看器的完整参数分别见[录制说明](README.md#launch-and-record-mcap)、[关节测试](README.md#quintic-joint-test)、[曲线查看器](README.md#commandstate-time-series-viewer)。
