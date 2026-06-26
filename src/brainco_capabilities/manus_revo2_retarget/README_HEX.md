# Hex Revo2 重定向

这个文档只写 Hex 手套到 Revo2 的链路。MANUS 手套说明见 [README.md](README.md)。

Hex 链路使用 `hex_glove_driver` 接收 Windows Hex 上位机的 UDP 数据，再把数据桥接到 Revo2 retarget 节点。后半段使用 `revo2_pid_controller` 控制 Revo2。

`hex_glove_driver` 的底层参数、topic 和单独运行方式见：[../../brainco_drivers/hex_glove_driver/README.md](../../brainco_drivers/hex_glove_driver/README.md)。

```text
Windows Hex 上位机
  -> hex_glove_udp_node
  -> /manus_glove_0 或 /manus_glove_1
  -> manus_revo2_retarget target-only
  -> /revo2_<side>/revo2_pid_controller/target_joint_states
  -> revo2_pid_controller
  -> Revo2
```

当前 bridge 会发布这些 topic：

```text
/hex_glove/raw_angles       原始角度 JSON
/hex_glove/raw_positions    原始位置 JSON
/manus_glove_0              左手适配后的 glove topic
/manus_glove_1              右手适配后的 glove topic
```

## 环境

每个新终端先初始化同一套环境：

```bash
conda activate manusglove
source /opt/ros/humble/setup.bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
source install/setup.bash
```

如果还没有 build，先构建 Hex bridge、Revo2 retarget 和 Revo2 driver：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
python -m colcon build --packages-up-to \
  hex_glove_driver manus_revo2_retarget revo2_driver \
  --symlink-install
source install/setup.bash
```

## Windows 上位机

先在 Windows 上启动 Hex 上位机：

1. 连接 Hex 手套。
2. 完成手套校准。
3. 开启 UDP 数据广播。
4. 用 `ipconfig` 查 Windows IPv4 地址。

下面命令里的 `<Windows_Hex_IP>` 换成这台 Windows 电脑的 IPv4 地址。

## 只看 Hex 数据

先不启动 Revo2 硬件，只确认 Hex bridge 是否收到数据：

```bash
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP> \
  launch_revo2_pipeline:=false
```

另开一个终端检查 topic：

```bash
ros2 topic hz /manus_glove_1
ros2 topic echo /manus_glove_1 --once
ros2 topic echo /hex_glove/raw_angles --once
ros2 topic echo /hex_glove/raw_positions --once
```

左手检查 `/manus_glove_0`。

## 控制右手 Revo2

确认 Revo2 串口别名已经存在：

```bash
ls -l /dev/revo2_hand_right
```

如果没有，先做 Revo2 串口绑定：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting/src/brainco_drivers/revo2_driver/setup
bash bootstrap_revo2.sh
```

启动完整右手 Hex -> Revo2 PID 链路：

```bash
cd /home/jiimmy/Brainco/Code/Revo-Retargeting
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP>
```

这个 launch 默认会：

```text
启动 hex_glove_udp_node
  -> 启动 Revo2 driver
  -> 尝试切到 revo2_pid_controller
  -> 启动 target-only retarget
```

关键默认值：

```text
launch_hex_bridge=true
launch_revo2_pipeline=true
launch_manus_publisher=false
controller_backend=ros2_control
launch_plot=false
```

## 验证 PID 链路

在另一个终端检查 controller：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers \
  -c /revo2_right/controller_manager
```

理想状态：

```text
revo2_joint_state            active
joint_forward_pos_controller inactive
revo2_pid_controller         active
joint_forward_vel_controller inactive
```

检查 retarget target 是否有输出：

```bash
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

如果 `revo2_pid_controller` 还是 inactive，手动切 controller：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control switch_controllers \
  -c /revo2_right/controller_manager \
  --deactivate joint_forward_pos_controller \
  --activate revo2_pid_controller \
  --activate-asap \
  --strict
```

手动切成功后，可以重新启动时跳过自动切 controller：

```bash
ros2 launch manus_revo2_retarget hex_real_hand_pipeline_launch.py \
  hand_mode:=right \
  hex_server_host:=<Windows_Hex_IP> \
  switch_controllers:=false
```

## 常用参数

```text
hex_server_host              Windows Hex 上位机 IPv4
angles_port                  Hex 角度 JSON UDP 端口，默认 9011
positions_port               Hex 位置 JSON UDP 端口，默认 9013
launch_revo2_pipeline        false 时只启动 Hex bridge
hand_mode                    left / right / both
revo2_coordinate_transform   是否使用 Revo2 坐标变换，默认 true
zero_angles_on_first_frame   是否用第一帧作为角度零点，默认 false
```

## 排查

如果 `/hex_glove/raw_angles` 和 `/hex_glove/raw_positions` 没数据：

```bash
ping <Windows_Hex_IP>
ros2 topic list | grep hex_glove
```

确认 Windows 防火墙没有拦 UDP，Hex 上位机已经开启数据广播，并且 `hex_server_host` 是 Windows 的实际 IPv4。

如果 raw topic 有数据，但 `/manus_glove_1` 没数据，检查 `hex_glove_udp_node` 日志里是否有 JSON 解析错误。

如果 `/manus_glove_1` 有数据，但 Revo2 不动，优先检查：

```bash
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
ros2 topic echo /revo2_right/revo2_pid_controller/target_joint_states --once
```

如果 `ros2 control` 出现 `!rclpy.ok()` 或 `xmlrpc.client.Fault`，重启 ROS2 CLI daemon 或对命令禁用 daemon：

```bash
ros2 daemon stop
ros2 daemon start
ROS2CLI_DISABLE_DAEMON=1 ros2 control list_controllers -c /revo2_right/controller_manager
```
