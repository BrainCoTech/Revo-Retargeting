# HumanDex / RevoHuman 手套连接检查与分步骤启动

适用：本仓库 `feat/humandex-support` 分支、ROS 2 Humble，先检查手套输入，
再运行不连接 Revo2 真机的重定向。下面的本机路径按需替换。

## 1. 当前设备状态与缺少的组件

2026-09-05 本机检查结果：

- USB 串口已识别：`/dev/ttyACM0`。
- 当前 by-id 路径：`/dev/serial/by-id/usb-BrainCo_RevoHuman_v0.20_BCRH1L001Q2600001-if00`。
- 当前用户 `jiimmy` 已属于 `dialout` 组，有串口访问权限。
- 当时未发现 HumanDex 采集进程；当前 `ROS_DOMAIN_ID=25` 中只发现
  `/parameter_events`、`/rosout`，没有手套话题。
- 本仓库包含 HumanDex **输入适配器**，不包含读取该手套串口的采集驱动。
  录包脚本引用的外部 `../BrainCo-HumanDex` 工程在本机预期目录不存在。
- 已取得 SDK：`/home/jiimmy/Brainco/Code/brainco_revohuman_sdk` 的 `origin/feature`
  （检查提交 `a688153`）。它包含纯 Python 采集程序和 README，但不包含 ROS、URDF 或 FK。
- 用 feature 分支临时副本实测：固件 `0.20.2`，产品当前报告 `ENCODER_ONLY`，
  3 秒收到 302 帧（约 100.4 Hz），302 帧编码器序号均不同且 21 路全部有效，
  `valid_mask=0x1fffff`、`offline_mask=0x0`。测试结束已停止流并关闭串口。

已确认“操作系统识别设备”和“成功接收有效手套数据”，尚未接通 ROS 发布层。
`ENCODER_ONLY` 是本次检测状态，不能据此断言设备永久不支持触觉。
重新插拔后复查设备路径。SDK README 提醒 USB serial descriptor 可能固定，
多设备时不能仅凭这个字符串识别唯一设备或左右手；本次设备信息寄存器里的序列号为空。

## 2. 检查物理连接

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM0
id -nG
udevadm info --query=property --name=/dev/ttyACM0
```

应能看到 `BrainCo_RevoHuman`。串口权限通常是 `root:dialout`；当前用户已在该组，
无需执行 `chmod 777`。不要用 `cat /dev/ttyACM0` 的输出判断关节数据是否有效：
原始数据需要设备协议解析，而且直接读串口可能与采集程序竞争。

## 3. 启动 SDK 原始采集：已实测

SDK 自带 `README.md`。检查时工作目录在 `main`，以下首先切到用户指定的 feature 分支。
本次诊断使用临时副本，没有改变 SDK 工作目录的分支。分支切换前先查看本地修改。

```bash
cd /home/jiimmy/Brainco/Code/brainco_revohuman_sdk
git status --short --branch
git switch feature
source /home/jiimmy/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo2
python -c 'import serial; print(serial.__version__)'
```

本机 `pyserial 3.5` 已安装。只有最后一步提示缺少 `serial` 时，才需要
`python -m pip install pyserial`。原始采集不需要 source ROS 或编译 colcon。

单次读取 21 路编码器角度并退出：

```bash
python 0.read_sensors.py \
  --port /dev/serial/by-id/usb-BrainCo_RevoHuman_v0.20_BCRH1L001Q2600001-if00
```

连续显示：移动手指观察角度变化；按 Ctrl+C 停止，脚本会尝试发送 STOP 并关闭串口。

```bash
python 0.read_sensors.py \
  --port /dev/serial/by-id/usb-BrainCo_RevoHuman_v0.20_BCRH1L001Q2600001-if00 \
  --stream --target-rate 100 --display-interval 0.2
```

自动运行 5 秒后退出：

```bash
python 0.read_sensors.py --port /dev/ttyACM0 \
  --stream --target-rate 100 --duration 5 --display-interval 1
```

**一键查看原始数据**（完成上述 feature 分支切换后，本机可直接执行）：

```bash
/home/jiimmy/miniforge3/envs/retarget_revo2/bin/python /home/jiimmy/Brainco/Code/brainco_revohuman_sdk/0.read_sensors.py --port /dev/serial/by-id/usb-BrainCo_RevoHuman_v0.20_BCRH1L001Q2600001-if00 --stream --target-rate 100
```

同一串口一次只能被一个程序占用；切换到别的查看程序或未来的 ROS 驱动前，先退出它。
这里输出的是编码器角度，单位 **degree**；不要直接当成本仓库需要的关节弧度角。
连接检查不需要运行 `2.calibrate.py` 或 `3.ota_upgrade.py`：前者保存零位，后者写固件。
如果看见数字但怀疑是旧值，检查 SDK 帧的 `encoder.valid_mask`、`offline_mask` 和 `sequence`，
不能只凭数值非零判断连接正常。

### 原始采集到 ROS 之间仍缺什么

这个 feature 分支明确不包含 ROS 节点、URDF 或安装工程，所以没有
`ros2 launch <SDK包名> ...` 入口，运行上面的采集脚本也不会新增 ROS topic。
要接现有 HumanDex 适配器，还需要上游的 ROS 采集/FK 工程，或实现相应桥接层：

1. 取得 21 个编码器索引与左右手关节的准确对应、符号、零位和角度处理规则。
2. 将 SDK 有效帧转换为带语义关节名的 `JointState`，按设备标定规则处理角度并转换为 rad。
3. 取得手套运动学模型并计算 5 个指尖的掌心局部坐标，发布 `PoseArray`（m）。
4. 同一 SDK 输入帧派生的两条 ROS 消息使用同一个非零时间戳。

不能把编码器索引随意命名成关节，或用固定坐标伪造 `/humandex_eef_pose`。
SDK 的双手 SOF 同步解决跨设备编码器配对，不会自动生成 ROS 消息或指尖 FK。
第 4–7 节是 **上游 ROS 采集/FK 层就绪以后** 的操作。本仓库适配器期望收到：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/humandex_left/joint_states`、`/humandex_right/joint_states` | 以实际上游发布类型为准 | 配套录包脚本期望的单侧原始关节话题，用于逐层排查 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 适配器直接订阅的关节角 |
| `/humandex_eef_pose` | `geometry_msgs/msg/PoseArray` | 适配器直接订阅的指尖位置 |

单侧原始话题存在并不表示后两个话题已经就绪。适配器的必要输入是后两项。

## 4. 检查话题与有效数据

在新的检查终端执行，domain 必须与采集进程一致。本机示例使用 `25`：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
ros2 node list --no-daemon --spin-time 3
ros2 topic list --no-daemon --spin-time 3 -t
```

收到一帧并退出，最长等待 10 秒：

```bash
timeout 10s ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
timeout 10s ros2 topic echo /humandex_eef_pose geometry_msgs/msg/PoseArray --once
```

检查连续更新，运行一会儿后按 Ctrl+C：

```bash
ros2 topic hz /joint_states
```

另一个已 source ROS 且 domain 相同的终端：

```bash
ros2 topic hz /humandex_eef_pose
```

判断标准：

1. 有实际消息和持续更新的时间戳，不能只看 topic 名称存在。
2. 轻轻弯曲手指时对应 `position` 和指尖坐标随之变化。
3. 左/右侧以关节名 `left_...` / `right_...` 确认；角度为弧度，位置为米。
4. 两路消息的 `header.stamp` 必须完全相同且非零；适配器不做近似时间同步。
5. 单手 `PoseArray` 为 5 个姿态，顺序为食指、中指、无名指、小指、拇指；
   双手为 10 个，先左后右，且必须与 `JointState` 中出现的手侧一致。
6. 指尖坐标需符合上游约定的掌心局部坐标。HumanDex 适配器直接使用坐标数值，
   不会自动把世界坐标转换为掌心坐标，也不会自动把毫米转换为米。

本分支适配器订阅使用 reliable QoS。若上游只提供 best-effort，需对齐 QoS；
`ros2 topic info /joint_states --verbose` 可检查发布者与订阅者的 QoS。
`echo` 可以接收并不保证适配器的 QoS 也兼容。

一条命令完成 ROS 话题发现（仅检查，不启动采集）：

```bash
bash -lc 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=25 && ros2 topic list --no-daemon --spin-time 3 -t'
```

## 5. 最小构建：适配器与重定向，不编译真机驱动

切换分支后的旧 `install/` 不能代表本分支已经构建。以下使用独立构建子目录。
先在一个新终端执行：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
source /home/jiimmy/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo2
source /opt/ros/humble/setup.bash
python -m colcon --log-base log/humandex_check build \
  --base-paths src \
  --build-base build/humandex_check \
  --install-base install/humandex_check \
  --packages-select hand_teleop_msgs manus_ros2_msgs hand_input_adapters revo2_hand_retarget \
  --cmake-args -DPython3_EXECUTABLE="$(command -v python)" -DPYTHON_EXECUTABLE="$(command -v python)"
```

后续每个运行适配器/重定向的终端都执行：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
source /home/jiimmy/miniforge3/etc/profile.d/conda.sh
conda activate retarget_revo2
source /opt/ros/humble/setup.bash
source install/humandex_check/local_setup.bash
export ROS_DOMAIN_ID=25
```

这里的 `retarget_revo2` 是本机存在的 Python 3.10 环境。仓库录包脚本默认的
`retarget_revo3` 在本次检查时不存在，使用录包脚本时应显式传 `--conda-env retarget_revo2`。

## 6. 分步骤启动下游（采集程序需保持运行）

终端 A：执行第 5 步的运行环境命令后，启动适配器。
下面 `both` 自动接收输入消息里存在的左/右侧，不依赖设备串号推测手侧：

```bash
ros2 run hand_input_adapters humandex_hand_adapter \
  --ros-args -p hand_mode:=both \
  -p joint_topic:=/joint_states -p pose_topic:=/humandex_eef_pose
```

首次收到有效配对数据，会打印 `Published first canonical HumanDex ... frame`。
如果一直只有 `waiting for exact-stamp ... pairs`，回到第 4 步排查。

终端 B：执行相同运行环境命令后，启动目标输出层：

```bash
ros2 launch revo2_hand_retarget pipeline_launch.py \
  hand_mode:=both controller_backend:=ros2_control
```

这个 **pipeline_launch.py** 入口只启动 target-only 重定向节点，
不启动 Revo2 驱动，也不要求真实手关节反馈；没有输入的一侧会等待数据。

终端 C：执行相同运行环境命令后，检查已连接那一侧的消息；这里以左侧为例，
右侧将下列路径里的 `left` 改为 `right`：

```bash
timeout 10s ros2 topic echo /hand_kinematics/left hand_teleop_msgs/msg/HandKinematics --once
timeout 10s ros2 topic echo /revo2_left/revo2_pid_controller/target_joint_states sensor_msgs/msg/JointState --once
```

成功时依次看到标准输入和 6 个 Revo2 关节目标。后者单位仍为 rad。

## 7. 已有的一键入口及边界

本仓库已有“适配器 + 重定向”的统一入口：

```bash
ros2 launch revo2_teleop_bringup teleop.launch.py \
  profile:=humandex_revo2 hand_mode:=both \
  launch_revo2_driver:=false switch_controllers:=false
```

使用前提：

- 外部 HumanDex 采集进程已经启动，必要输入话题已有有效数据。
- 当前分支的 `revo2_teleop_bringup` 及其声明依赖已构建、source。
  **第 5 步的最小构建不包含此入口**。
- 当前实现即使关闭驱动启动，仍会查找 `revo2_driver` 包；仅最小构建请使用第 6 步。
- 默认约 18 秒后启动重定向；这段等待不代表没有识别手套。

`launch_input_driver:=true` 对 HumanDex 分支也不会启动外部采集程序。
第 3 步已有经过实测的 SDK 原始数据一键查看命令；本入口则负责 ROS 下游。
两者之间还缺 ROS 采集/FK 层，因此目前没有“手套串口到重定向”的一键全链路命令。

## 8. 与 MuJoCo 的关系

以上命令用于检查手套与重定向话题，不会打开 MuJoCo。
当前 viewer 的 mesh 路径、默认话题和关节名兼容问题需要修复后才能接入新链路。
`if_sim:=true` 使用 ros2_control mock 硬件，也不等于启动 MuJoCo。

参考：

- SDK feature README：在 SDK 目录运行 `git show origin/feature:README.md`；
  切到 feature 后也可直接查看该目录下的 `README.md`。
- [项目 README](../README_CN.md)
- [统一启动层 README](../src/brainco_bringup/revo2_teleop_bringup/README.md)
- [输入适配器 README](../src/brainco_capabilities/hand_input_adapters/README.md)
