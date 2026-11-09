# MANUS → Revo3 Vector 重定向

已安装依赖并构建过工作区时，按下面两个终端启动右手。所有命令都在
**Revo-Retargeting 仓库根目录**执行。首次使用先看后面的[首次准备与构建](#首次准备与构建)。

## 日常使用：启动右手

先连接 MANUS 手套和 Revo3 右手，完成 MANUS SDK 自身的手套标定。
Vector 启动后还会采集骨长；这两项标定分别进行。

**终端 1：启动并保持运行。**

```bash
conda activate revo_teleop
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=11
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/teleop.sh right retarget_backend:=vector
```

这条命令启动右手驱动、一个 MANUS 发布器和 Vector 后端。首次采集 120 帧做骨长
标定，保持自然、稳定的手形；有匹配缓存时先用 30 帧复核。完成后输出仍为禁用状态。

**终端 2：查看状态，准备好后启用输出。**

```bash
conda activate revo_teleop
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=11
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /manus_revo3_retarget/status std_msgs/msg/String --once
```

`ROS_DOMAIN_ID=11` 是示例，所有相关终端必须使用同一个值。检查状态中的
`phase` 为 `READY`、`ready_to_arm` 为 `true`；尚未就绪时可重复上面的查看命令，
根据 `reason` 和标定进度处理。就绪后在终端 2 执行：

```bash
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: true}'
```

确认服务返回 `success: true` 后开始跟随手套。状态中的 `enabled` 可用于确认输出是否启用。
默认直接发布每个有效求解帧，不插值。若需要 200 Hz 插值、速度／加速度限制和
2 秒启动缓入，启动时加上 `interpolate:=true`：

```bash
bash scripts/teleop.sh right retarget_backend:=vector interpolate:=true
```

每次只运行一种启动方式；同一只手只能有一个 MIT 命令发布器。
`teleop.sh` 默认使用 legacy 后端，因此使用本包时需显式传入 `retarget_backend:=vector`。
切换后端或启动参数前，先停止原来的管线。

## 暂停、退出和重新标定

在终端 2 暂停输出，保留已完成的骨长标定：

```bash
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: false}'
```

恢复时先查看状态，达到 `READY` 后再次调用 `{data: true}`。
手套输入断流或过期也会停止输出；输入恢复后不会自动恢复跟随，需要重新显式启用。

结束使用时，在终端 1 按 `Ctrl+C`，脚本会清理本次启动的驱动、MANUS 发布器和后端。

需要重新采集 Vector 骨长时，在终端 2 执行：

```bash
ros2 service call /manus_revo3_retarget/recalibrate std_srvs/srv/Trigger '{}'
```

此操作先暂停输出，再跳过旧缓存重新采集完整标定帧。等待 `READY` 后重新启用。
它不执行 MANUS SDK 的手套标定；手套自身标定与连接方法见
[首次安装的手套标定步骤](../../docs/vector_setup_CN.md#4-连接和手套标定)。

## 换成左手或双手

将终端 1 的启动命令替换为下表对应命令，环境初始化步骤相同。双手的状态和服务
按手侧独立，需要分别确认就绪、分别启用。

| 模式 | 启动命令 | 状态／服务前缀 |
| --- | --- | --- |
| 右手 | `bash scripts/teleop.sh right retarget_backend:=vector` | `/manus_revo3_retarget` |
| 左手 | `bash scripts/teleop.sh left retarget_backend:=vector` | `/manus_revo3_retarget` |
| 双手 | `bash scripts/teleop.sh both retarget_backend:=vector` | 左：`/manus_revo3_retarget_left`；右：`/manus_revo3_retarget_right` |

上面的操作命令统一使用 `<前缀>/status`、`<前缀>/enable_retarget` 和
`<前缀>/recalibrate`。例如双手启用前，分别检查左右状态；就绪后执行：

```bash
ros2 service call /manus_revo3_retarget_left/enable_retarget std_srvs/srv/SetBool '{data: true}'
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: true}'
```

## 首次准备与构建

系统依赖、SDK、驱动版本、Python 环境和完整构建步骤见
[Vector 首次安装](../../docs/vector_setup_CN.md)。已有 `revo_teleop` 环境可直接复用。
首次安装完成后回到本页的日常使用步骤。

<details>
<summary>增量构建与运行依赖</summary>

已有正常构建的工作区，在上述 conda 和 ROS 环境中先 `source install/setup.bash`，
再执行：

```bash
python -m colcon build --packages-select manus_revo3_vector_retarget --symlink-install
source install/setup.bash
```

本包通过安装的 `retarget` 入口运行，保留 colcon 构建时的 Python 解释器，避免系统
`ros2` 命令的解释器跳出 conda 环境。运行依赖为 Python 3.10、NumPy 1.24–1.x、
SciPy 1.10–1.15.x、MuJoCo 3.3.7–3.x。MuJoCo 用于正运动学，无窗口运行也需要它。

</details>

## 可选参数与独立验证

<details>
<summary>启动参数、缓存与复用已有进程</summary>

以下参数可追加在 `bash scripts/teleop.sh right retarget_backend:=vector` 后，
例如 `calibration_frames:=120`。

| 参数 | 默认值／用途 |
| --- | --- |
| `left_profile`、`right_profile` | 不指定时使用对应手侧的 `config/<side>_hardware_may3.yaml` |
| `left_glove_id`、`right_glove_id` | 不指定时按消息 `side` 选择；可指定手套 ID |
| `cache_dir` | `$XDG_CACHE_HOME/manus_revo3_vector_retarget`；未设置 `XDG_CACHE_HOME` 时为 `~/.cache/manus_revo3_vector_retarget` |
| `calibration_frames` | `120`，完整骨长标定帧数，至少 `30` |
| `normal_sign` | `-1`，可选 `-1` 或 `1` |
| `numeric_threads` | `1`，数值库线程数；`0` 表示不覆盖线程环境变量 |
| `interpolate` | `false`；设为 `true` 启用 200 Hz 插值和 2 秒启动缓入 |
| `max_velocity_rad_s` | 默认使用 profile；限制 MIT 速度前馈，插值模式下也限制位置变化速度 |
| `max_acceleration_rad_s2` | 默认使用 profile，仅在插值模式下限制加速度 |
| `thumb_offset_deg` | 默认使用 profile 的 `thumb_direction_offset.angle_deg`（左右手 30°）；越大拇指 MCP 越低，见下文“拇指方向补偿” |

直接模式不限制位置变化速度，profile 的速度上限仅限制 MIT 的速度前馈字段。
骨长缓存按手套 ID、手侧、法向符号和骨架版本区分；匹配缓存经 30 帧复核后使用，
复核失败则重新采集完整标定。状态中可查看 `cache_path`、`cache_used` 和 `cache_error`。
旧后端的 `*_calibration_config`、`input_config` 和调参服务不用于 Vector。

若已有驱动和 MANUS 发布器，可复用它们：

```bash
START_REVO3_DRIVER=0 START_MANUS_PUBLISHER=0 \
  bash scripts/teleop.sh both retarget_backend:=vector
```

只复用其中一个时，只设置对应的环境变量。脚本退出只清理本次启动的进程。

</details>

<details>
<summary>独立启动、mock 驱动与软件测试</summary>

以下命令同样在仓库根目录运行，先按日常启动步骤加载 conda、ROS 和工作区，
所有相关终端使用相同 `ROS_DOMAIN_ID`。

```bash
# 仅启动后端和 MANUS，不启动驱动
ros2 launch manus_revo3_vector_retarget pipeline_launch.py hand_mode:=both
# 单侧原生入口，服务前缀为 /manus_revo3_vector_retarget_right
ros2 launch manus_revo3_vector_retarget pipeline.launch.py hand_side:=right
```

使用 mock 驱动时，两个终端分别执行：

```bash
# 终端 1：mock 驱动
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true
# 终端 2：复用驱动，启动 MANUS 和 Vector
START_REVO3_DRIVER=0 bash scripts/teleop.sh right retarget_backend:=vector
```

仍需检查对应状态，再调用启用服务。纯软件回归与合成 ROS 消息测试：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src/manus_revo3_vector_retarget \
  python -m pytest src/manus_revo3_vector_retarget/tests checks/test_revo3_teleop_script.py -q
python src/manus_revo3_vector_retarget/tests/ros_hardware_smoke.py --hand-side right
python src/manus_revo3_vector_retarget/tests/ros_hardware_smoke.py --hand-side left
```

ROS smoke 测试使用本机独立 DDS domain 和合成手套／JointState 消息，不启动硬件驱动。
离线 `evaluate`、`simulate`、`view` 及仅观察的 `live` 命令可通过以下入口查看：

```bash
ros2 run manus_revo3_vector_retarget retarget --help
```

这些软件检查不能验证真机方向、零位和跟踪效果。

</details>

## 拇指方向补偿

Revo3 拇指在 MCP=0 时，CMP/CMR 只能让第一节朝侧面伸（垂直于四指），能让拇指往四指方向倾的只有 MCP。
人手拇指自然往四指方向倾约 40°，所以只按方向贴合时，求解器会把机器手 MCP 放在 40–50°，看起来像“下不来”。

profile 的 `thumb_direction_offset` 先把人手拇指三段方向绕掌心坐标轴 `axis_palm`（lateral, distal, normal）旋转 `angle_deg`，再参与方向和 reach 优化；捏合用的指尖相对向量不受影响。
右手的轴取 `[1, 0, -1]`，即横向轴和负法向轴的角平分线；正角度让机器手拇指 MCP 降低。
这根轴是选定的，不是拟合的。用 2026-11-09 的右手手套数据回放，它和自由拟合的轴效果基本相同；只绕横向轴时 MCP 中位数仍是 50°，只绕法向轴时捏合时 MCP 仍在 46° 上下。

| 补偿角 | 拇指 MCP 中位数（捏合 <35 mm 时） | 最近指尖距离误差 <35 / 35–70 / >70 mm |
| --- | --- | --- |
| 0° | 45°（50°） | 9 / 5 / 24 mm |
| 20° | 38°（46°） | 6 / 16 / 37 mm |
| 30°（右手默认） | 34°（39°） | 5 / 23 / 43 mm |
| 40° | 28°（32°） | 5 / 31 / 50 mm |

角度越大，MCP 越低，但不捏合时拇指会比人手更往侧面张开。可以临时覆盖：

```bash
bash scripts/teleop.sh right retarget_backend:=vector thumb_offset_deg:=40
```

左手 profile 用轴 `[-1, 0, -1]`、30°（左手掌心坐标和右手不同，右手的轴在左手上无效）。2026-11-09 实测：放松时 MCP 约 38°→33°，对掌时仍在 50° 上限，效果有限。profile 里没有这一节时不做补偿。

## 模型和硬件坐标

<details>
<summary>算法来源与坐标细节</summary>

本包移植自 `revoarm_teleoperation` 的 `f0e23030`，复用全手向量优化、骨长标定、
缓存、MIT 命令和启停服务，接入本仓库的 `revo_teleop` 环境与 `scripts/teleop.sh`。
来源与适配范围见 [UPSTREAM.md](UPSTREAM.md)。

求解目标是指骨单位方向、指根到指尖向量，以及拇指和其余四指的相对向量；
接近捏合时平滑提高指尖相对向量的权重。输入使用完整的 MANUS 25 节点骨架，
直接订阅 `/manus_glove_0` 和 `/manus_glove_1`，按消息 `side` 选手，话题编号不代表手侧。
当前通用 `HandKinematics` 的 MANUS adapter 只提供部分关键点，因此本后端不经过它；
`input_source:=dv1|humandex|external` 使用 legacy 后端。

本包直接读取驱动子模块 `src/brainco_revo3_ros2` 中的
`revo3_description/urdf/revo3_<side>_may3.urdf`。资源在源码中按仓库布局查找，
安装后通过 ament 查找；本包未引入新版手模型、驱动 SDK 或整机状态机。
左右侧 profile 分别绑定对应 URDF 的 SHA256，使用相同的 21 个关节名和 May3 限位。

左右手的食指、中指、无名指和小指侧摆（MPR）硬件正方向，都按操作者实物反馈设为朝向拇指
（右手 2026-11-05、左手 2026-11-09 观察到四指全反后修正）。
May3 模型两侧的正方向都是远离拇指，因此这八个关节的 `scale=-1`，即 `q_ros = -q_model`，
反馈也通过相同映射转换回模型坐标。其余关节保持 `q_ros = q_model`，偏移均为零。
左右手四指侧摆方向修正后都已在实物上确认（左手 2026-11-09）；零位和整体跟踪效果尚未验证。

MIT 默认增益沿用 `control.yaml`：普通关节 `kp=0.2, kd=0.02`，
MPR `kp=0.7, kd=0.08`。两个 May3 CAD 文件并非精确数学镜像，各侧独立计算
正运动学（FK）和 Jacobian。

输出话题为 `/revo3_<side>/joint_forward_mit_controller/commands`，
反馈为 `/revo3_<side>/revo3_joint_state/joint_states`，沿用现有 MIT 消息类型。

</details>
