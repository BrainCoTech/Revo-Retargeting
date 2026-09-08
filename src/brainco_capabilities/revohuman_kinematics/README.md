# RevoHuman encoder → FK → Revo2 input

使用用户提供的 DV1 双手模型。原始 driver 保留全部 21 路数据，本包直接发布
`/hand_kinematics/{left|right}`，接现有 Revo2 重定向；不需要旧 HumanDex 采集进程或适配器。
另外发布 `/revohuman/{side}/joint_states`（21 个 URDF 关节角，rad）用于检查。

## 构建、启动

仓库根目录，新终端，ROS Humble / Python 3.10：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select revohuman_msgs revohuman_driver hand_teleop_msgs revohuman_kinematics
source install/setup.bash
ros2 launch revohuman_kinematics input.launch.py \
  hand_mode:=left left_port:=/dev/ttyACM0 \
  sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk
```

串口按实物填写。双手使用 `hand_mode:=both left_port:=... right_port:=...`。
这里只启动输入层，不启动 Revo2。已有 driver 时使用 `launch_driver:=false`。
两个节点不做双手 SOF 对时；每手独立保留原始 ROS 时间戳。

```bash
ros2 topic echo /hand_kinematics/left --once
ros2 topic echo /revohuman/left/joint_states --once
```

重定向包及其依赖已构建/source 后，另一个终端启动目标输出：

```bash
ros2 launch revo2_hand_retarget pipeline_launch.py hand_mode:=left controller_backend:=ros2_control
ros2 topic echo /revo2_left/revo2_pid_controller/target_joint_states --once
```

统一启动（需构建 bringup 及其依赖；默认会连接 Revo2 真机）：

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=revohuman_revo2 hand_mode:=left \
  revohuman_left_port:=/dev/ttyACM0 \
  revohuman_sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk
```

## YAML 标定与 tip 微调

复制 `config/kinematics.yaml` 到自己的配置路径，用 `config_file:=/绝对路径/kinematics.yaml`
传入输入层（统一启动用 `revohuman_config_file:=...`）。修改后重启节点生效。

- 模型为 `urdf/revohuman_dv1_kinematics.urdf`，仅保留运动学，来源见 `urdf/SOURCE.md`。
- SDK ID 0–3、4–7、8–11、12–15 分别对应食/中/无名/小指的 DIP/PIP/MCP/MPR。
  拇指 16–20 对应 DIP/PIP/MCP/CMR/CMP；左右手编号相同。
- 每路按 `q = rad(sign * wrap(raw_deg - zero_deg) + offset_deg)` 转换。
  零位/方向默认 0/+1，仅为初始参数，未做真机标定；不能套用另一版 URDF 的标定。
- 四指开合仅使用各自 DIP/PIP/MCP 三路，按开闭角归一化、加权平均并限幅。
  默认 0° 张开、90° 闭合，输出范围 1.4661 rad，可按手侧/手指覆盖。
  MPR 不参与开合计算，但仍用于完整五指 FK，因此要求全部 21 路有效。
- `tip_offsets_m.thumb: [0, 0, 0]` 表示先用 **DIP_Link 原点作为末端**。
  日后量出的偏移填在 DIP_Link 局部坐标系中，单位米；它随 DIP 关节一起转动。
  零偏移下 DIP 最末关节的转动不会改变 tip 位置，这是当前选择的几何结果。
- `palm_translation_m` / `palm_rpy_rad` 可对输出掌心坐标做统一位置/方向标定。
  默认不变换，坐标取各自 DV1 palm，双手模型的 0.27 m 摆放间距不进入输出。
- 左右手 YAML 分开配置，可独立标定。
- 帧时间沿用 driver 的主机接收时间；丢弃无效、离线、重复编码器帧和过期/乱序帧。
  `max_age_sec` 默认 0.5 秒。无数据时不补发，后级可触发已有输入超时逻辑。

当前默认 Revo2 pose_thumb 参数原先按其它输入调试；DV1 的零位、掌心对齐、拇指尺度
仍需实测调整。软件输出目标不代表已完成真机跟随标定。

## 测试

```bash
python3 -m pytest src/brainco_capabilities/revohuman_kinematics/checks -q
```

需要 source 构建环境。测试覆盖 FK、三个弯曲编码器、侧摆隔离、tip 偏移、数据有效性
和真实 ROS 消息传输；不打开串口或驱动电机。

下游算法验证（需 retarget_revo2 环境与 revo2_hand_retarget import 路径）：

```bash
python -m pytest src/brainco_capabilities/revohuman_kinematics/checks/test_retarget.py -q
```
