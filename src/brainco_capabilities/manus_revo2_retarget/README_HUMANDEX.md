# HumanDex Revo2 重定向

HumanDex 数据采集、关节角发布和指尖 FK 都由上级仓库负责。本仓库不会启动或配置
HumanDex，只监听 `/humandex_<side>/joint_states` 和 `/humandex_eef_pose`。

## 当前数据流

```text
HumanDex MCU（左/右）
  |-> /humandex_<side>/tactile                         触觉观测，暂不参与控制
  `-> /humandex_<side>/joint_states
      -> humandex_urdf/joint_state_mux
          |-> /joint_states                            合并关节角，观测支路
          `-> /humandex_eef_pose                       掌根局部指尖 FK
              -> glove_input_adapter/humandex_pose_adapter
                  |-> 四指 PIP -> ergonomics
                  `-> 拇指指尖 pose
                      -> manus_revo2_retarget
                          |-> 四指 PIP 目标
                          `-> revo3_thumb IK
                              -> /revo2_<side>/revo2_pid_controller/target_joint_states
                                  -> revo2_pid_controller -> Revo2 hardware

Revo2 hardware
  -> revo2_joint_state broadcaster
  -> /revo2_<side>/revo2_joint_state/joint_states      状态观测
```

`/humandex_eef_pose` 已经由 URDF 和关节角做 FK。左手位姿相对 `left_palm_Link`，右手位姿
相对 `right_palm_Link`，不需要 Ommo SDK、世界坐标转换或额外掌根追踪。

默认控制固定为：四指只读取各自的 `PIP`，经 ergonomics 送入四指重定向；拇指读取
`/humandex_eef_pose` 中的掌根局部指尖位置，使用 `revo3_thumb` IK。标准启动不再选择
`CMR/CMP` 绝对拇指映射。

PID 在 ros2_control 内直接读取 Revo2 state interface 做位置误差闭环；`revo2_joint_state` topic
是同一硬件状态的观测输出，不是 PID 的 topic 反馈输入。如果上游停止，目标 topic 随之停止更新，
Revo2 PID 在目标超过默认 `0.3 s` 未更新后将速度命令归零。

## 1. 验证外部 HumanDex 输入

先由外部 HumanDex workspace 启动采集与 FK，再在本仓库环境中只读检查输入：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 topic hz /humandex_eef_pose
ros2 topic echo /humandex_eef_pose --once
```

默认每只手的 FK 顺序固定为：

```text
index, middle, ring, little, thumb
```

单手消息包含 5 个 pose；双手消息包含左手 5 个，再接右手 5 个。默认
`header.frame_id` 是 `hand_base_link_local`。

## 2. 验证 PIP + 拇指 IK 目标，不启动 Revo2

右手示例：

```bash
ros2 launch manus_revo2_retarget humandex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  launch_driver:=false \
  switch_controllers:=false \
  retarget_delay:=0

ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

该模式只监听已经存在的 HumanDex topics，启动 PIP adapter 和拇指 IK retarget，
但不启动 HumanDex 或 Revo2 硬件驱动。

## 3. 录制 HumanDex 原始数据

先启动 BrainCo-HumanDex 采集节点，然后在本仓库运行：

```bash
./src/brainco_capabilities/manus_revo2_retarget/tools/record_humandex_raw.sh \
  --side left \
  --duration 60 \
  --domain 11
```

右手把 `--side left` 改为 `--side right`，双手用 `--side both`。脚本会激活
`retarget_revo3`，只录制 HumanDex 来源的话题：`/humandex_<side>/joint_states`、
`/humandex_<side>/tactile`、`/humandex_eef_pose` 和 `/joint_states`。不需要启动
`glove_input_adapter`、retarget 或 Revo2 controller。录制时长从 rosbag2 确认进入
`Recording` 状态后开始计算；`recording_timing.txt` 会同时记录请求时长、计时器墙钟时长和
最终 bag 时间戳跨度。录完后把脚本最后打印的目录路径交给分析端。

## 4. 启动 Revo2 真机

```bash
ros2 launch manus_revo2_retarget humandex_real_hand_pipeline_launch.py \
  hand_mode:=right
```

双手模式：

```bash
ros2 launch manus_revo2_retarget humandex_real_hand_pipeline_launch.py \
  hand_mode:=both
```

`hand_mode` 必须与 `humandex_urdf` 实际启动的手一致，因为 `/humandex_eef_pose` 是紧凑数组：
单手始终为 5 个 pose，双手为 10 个 pose。
