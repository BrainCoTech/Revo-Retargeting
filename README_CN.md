# Revo2 Retargeting

这是一个用于 BrainCo Revo2 灵巧手 + MANUS 手套遥操作的 ROS 2 Humble workspace。

English: [README.md](README.md)

当前推荐真机链路是：

```text
MANUS SDK / manus_ros2
  -> manus_ros2_msgs/ManusGlove
  -> manus_revo2_retarget
  -> sensor_msgs/JointState target
  -> revo2_driver revo2_pid_controller
  -> Revo2 hardware velocity command interface
```

## 包结构

```text
src/brainco_capabilities/manus_ros2          MANUS SDK bridge
src/brainco_capabilities/manus_ros2_msgs     MANUS ROS 2 messages
src/brainco_capabilities/manus_revo2_retarget
src/brainco_drivers/revo2_driver             Revo2 ros2_control driver
src/brainco_description/revo2_description    Revo2 hand description
```

更详细的 retargeting、调参和排查说明在：

```text
src/brainco_capabilities/manus_revo2_retarget/README.md
```

## 环境准备

目标环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

创建并激活 Python 环境：

```bash
conda create -n manusglove python=3.10 -y
conda activate manusglove
python -m pip install --upgrade pip
python -m pip install rospkg catkin_pkg colcon-common-extensions
python -m pip install -r requirements.txt
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

安装 `revo2_driver` 需要的 BrainCo Stark SDK：

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

MANUS SDK 的动态库不会提交到仓库。请把官方 MANUS SDK 文件放到：

```text
src/brainco_capabilities/manus_ros2/ManusSDK/include/
src/brainco_capabilities/manus_ros2/ManusSDK/lib/
```

运行时至少需要：

```text
src/brainco_capabilities/manus_ros2/ManusSDK/lib/libManusSDK.so
src/brainco_capabilities/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
```

## 构建

```bash
conda activate manusglove
source /opt/ros/humble/setup.bash
python -m colcon build --symlink-install --packages-select \
  manus_ros2_msgs manus_ros2 \
  revo2_description revo2_driver \
  manus_revo2_retarget
source install/setup.bash
```

## 启动遥操作

推荐右手真机启动：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control
```

不启动 plot 窗口：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  launch_plot:=false
```

左手或双手：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py hand_mode:=left
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py hand_mode:=both
```

## Controller 默认逻辑

`revo2_driver` 默认激活 `joint_forward_pos_controller`。这是有意保留的安全默认值：单独调硬件时，可以直接用 position command 测试手，不依赖 MANUS 手套和 retarget 节点持续发布目标。

MANUS 遥操作推荐使用 `revo2_pid_controller`：

```text
MANUS -> retarget target JointState -> revo2_pid_controller -> velocity command interface
```

真机 pipeline 里有一个辅助脚本会尝试从默认的 position controller 自动切到 `revo2_pid_controller`。如果自动切换失败，先查看当前 controller 状态：

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

如果看到 `joint_forward_pos_controller active`，而 `revo2_pid_controller inactive`，手动切换：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate revo2_pid_controller \
  --activate-asap \
  --strict
```

注意：不要在 `--strict` 模式下 deactivate 已经 inactive 的 controller。比如 `joint_forward_vel_controller` 已经是 inactive 时，不需要再写进 `--deactivate`，否则可能导致切换失败。

手动切换成功后，可以用下面的方式重新启动，避免重复执行自动切换脚本：

```bash
ros2 launch manus_revo2_retarget real_hand_pipeline_launch.py \
  hand_mode:=right \
  controller_backend:=ros2_control \
  switch_controllers:=false \
  launch_plot:=false
```

## 硬件设置

新电脑上先配置一次 Revo2 串口别名和权限：

```bash
cd src/brainco_drivers/revo2_driver/setup
bash bootstrap_revo2.sh
bash check_revo2_setup.sh
cd -
```

如果只接了右手，自动检测可能会提示缺少左手。这种情况下可以按实际串口手动设置右手，例如：

```bash
sudo bash setup_revo2_udev_rules.sh /dev/ttyUSB0 right
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/revo2_hand_right
```

## 常见问题

如果构建时报 BrainCo Stark SDK 缺失，运行：

```bash
bash src/brainco_drivers/revo2_driver/scripts/download_sdk.sh
```

如果构建时报 MANUS SDK 动态库缺失，把官方 MANUS SDK 的 `.so` 文件放到：

```text
src/brainco_capabilities/manus_ros2/ManusSDK/lib/
```

如果启动了但手不动，确认 controller 和 retarget target topic：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

如果 `ros2 control` 出现 `!rclpy.ok()` 或 `xmlrpc.client.Fault`，通常是 ROS 2 CLI daemon 状态异常。可以重启 daemon，或者对这条命令禁用 daemon：

```bash
ros2 daemon stop
ros2 daemon start
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
```
