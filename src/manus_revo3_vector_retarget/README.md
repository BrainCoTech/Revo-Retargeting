# MANUS → Revo3 Vector retargeting

本包移植自 `revoarm_teleoperation` 的 `f0e23030`，复用全手向量优化、骨长标定、
缓存、MIT 命令和启停服务，接入 Revo-Retargeting 的 `revo_teleop` 环境与
`scripts/teleop.sh`。来源与适配范围见 [UPSTREAM.md](UPSTREAM.md)。

求解目标是指骨单位方向、指根到指尖向量，以及拇指和其余四指的相对向量；
接近捏合时平滑提高指尖相对向量的权重。输入使用完整的 MANUS 25 节点骨架，
直接订阅 `/manus_glove_0` 和 `/manus_glove_1`，按消息 `side` 选手，话题编号不代表手侧。
当前通用 `HandKinematics` 的 MANUS adapter 只提供部分关键点，因此本后端不经过它；
`input_source:=dv1|humandex|external` 仍使用原有 legacy 后端。

## 构建

从 Revo-Retargeting 根目录执行；MANUS SDK 和系统依赖沿用主仓库 README。

```bash
bash scripts/setup_revo_conda.sh
conda activate revo_teleop
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
python -m colcon build --base-paths src --symlink-install \
  --packages-up-to manus_revo3_vector_retarget revo3_driver \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
               -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
source install/setup.bash
```

已有正常构建的工作区可先 `source install/setup.bash`，再用
`python -m colcon build --packages-select manus_revo3_vector_retarget --symlink-install`
增量构建。本包通过安装的 `retarget` 入口运行，保留 colcon 构建时的 Python 解释器，
避免系统 `ros2` 命令的解释器跳出 conda 环境。无需额外 Vector venv。
运行依赖为 Python 3.10、NumPy 1.24–1.x、SciPy 1.10–1.15.x、MuJoCo 3.3.7–3.x。
MuJoCo 用于正运动学；运行无窗口后端也需要它。

## 启动

```bash
# 启动驱动、一个 MANUS 发布器，以及所选手侧的 Vector 后端
bash scripts/teleop.sh right retarget_backend:=vector
bash scripts/teleop.sh left retarget_backend:=vector
bash scripts/teleop.sh both retarget_backend:=vector

# 复用已经运行的驱动和 MANUS 发布器
START_REVO3_DRIVER=0 START_MANUS_PUBLISHER=0 \
  bash scripts/teleop.sh both retarget_backend:=vector
```

每次只运行其中一种启动方式。同一只手只能有一个 MIT 命令发布器。
未传 `retarget_backend` 时继续使用 legacy；切换后端前停止旧管线。
退出 `teleop.sh` 会清理本次启动的子进程。

Vector 启动后先采集 120 帧做骨长标定，保持自然、稳定的手形。
有匹配缓存时使用 30 帧复核。**标定完成后输出仍处于禁用状态，需要调用服务启用。**

| hand mode | 节点／服务前缀 |
| --- | --- |
| `left` 或 `right` | `/manus_revo3_retarget` |
| `both` | `/manus_revo3_retarget_left`、`/manus_revo3_retarget_right` |

```bash
# 单手；双手时将前缀分别替换为上表的左右手名称
ros2 topic echo /manus_revo3_retarget/status std_msgs/msg/String --once
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: true}'
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: false}'
ros2 service call /manus_revo3_retarget/recalibrate std_srvs/srv/Trigger '{}'
```

`status` 包含 `phase`、`ready_to_arm`、`enabled`、`reason` 和标定进度。
输入过期后停止输出；输入恢复不会自动重新启用。
暂停或重标定后需要再次显式启用。所有进程需要相同 `ROS_DOMAIN_ID`。

默认每个有效求解帧发布一次 MIT 命令。添加 `interpolate:=true` 可使用 200 Hz
插值、速度／加速度限制和 2 秒启动缓入。插值之外的直接模式不限制位置变化速度；
profile 的速度上限仅限制 MIT 的速度前馈字段。

可传入的参数：`left_profile`、`right_profile`、`left_glove_id`、`right_glove_id`、
`cache_dir`、`calibration_frames`（默认 120）、`normal_sign`（默认 -1）、
`numeric_threads`（默认 1）、`interpolate`（默认 false）、
`max_velocity_rad_s`、`max_acceleration_rad_s2`。
旧后端的 `*_calibration_config`、`input_config` 和调参服务不用于 Vector。

## 模型和硬件坐标

驱动子模块继续使用 `brainco_revo3_ros2@34528e75`，本包直接读取其
`revo3_description/urdf/revo3_<side>_may3.urdf`。不引入新版手模型、驱动 SDK 或整机状态机。
资源在源码中按本仓库布局查找，安装后通过 ament 查找。

左右侧的 `config/<side>_hardware_may3.yaml` 分别绑定对应 URDF 的 SHA256，
使用相同的 21 个关节名和 May3 限位。ROS 输出为 `q_ros = q_model`，沿用本仓库
`physical_*_joint_scale=1`、零偏移的配置。MIT 默认增益沿用 `control.yaml`：
普通关节 `kp=0.2, kd=0.02`，MPR `kp=0.7, kd=0.08`。
两个 May3 CAD 文件不是精确数学镜像，各侧独立计算 FK 和 Jacobian。
这些是软件坐标配置，未做真机方向、零位或跟踪效果验证。

输出话题为 `/revo3_<side>/joint_forward_mit_controller/commands`，
反馈为 `/revo3_<side>/revo3_joint_state/joint_states`，沿用现有 MIT 消息类型。

## 独立入口和 mock 验证

```bash
# 仅启动后端和 MANUS；不启动驱动
ros2 launch manus_revo3_vector_retarget pipeline_launch.py hand_mode:=both
# 单侧原生入口：服务前缀为 /manus_revo3_vector_retarget_right
ros2 launch manus_revo3_vector_retarget pipeline.launch.py hand_side:=right
```

使用 mock 驱动时单独启动它，再复用：

```bash
ros2 launch revo3_driver revo3_system.launch.py hand_side:=right if_sim:=true
# 另一终端，已加载同一工作区
START_REVO3_DRIVER=0 bash scripts/teleop.sh right retarget_backend:=vector
```

纯软件回归与合成 ROS 消息测试：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=src/manus_revo3_vector_retarget \
  python -m pytest src/manus_revo3_vector_retarget/tests checks/test_revo3_teleop_script.py -q
python src/manus_revo3_vector_retarget/tests/ros_hardware_smoke.py --hand-side right
python src/manus_revo3_vector_retarget/tests/ros_hardware_smoke.py --hand-side left
```

ROS smoke 测试使用本机独立 DDS domain 和合成手套／JointState 消息，不启动硬件驱动。
离线 `evaluate`、`simulate`、`view` 及仅观察的 `live` 命令可通过
`ros2 run manus_revo3_vector_retarget retarget --help` 查看。
