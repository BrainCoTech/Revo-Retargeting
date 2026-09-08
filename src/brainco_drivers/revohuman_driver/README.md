# RevoHuman ROS 2 driver

原始采集层，使用外部 `revohuman_glove.py` SDK；不复制 SDK、不执行标定或 OTA。
每个节点打开一个明确指定的串口，支持编码器和可选触觉主动流，连接失败后重试。

## 构建与启动

在仓库根目录、新终端中执行（ROS Humble / Python 3.10，需 pyserial）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select revohuman_msgs revohuman_driver
source install/setup.bash
ros2 launch revohuman_driver revohuman.launch.py \
  sdk_path:=/home/jiimmy/Brainco/Code/RevoHuman/brainco_revohuman_sdk \
  port:=/dev/ttyACM0 side:=left target_rate_hz:=100.0
```

串口应按实际设备填写；有稳定 by-id 路径时优先使用，但不能凭 SDK 的固定 USB
serial descriptor 自动判断左右手。先退出其它占用同一串口的 SDK 程序。
`sdk_path` 也可省略，此时 SDK 必须已经在 Python import 路径中。

```bash
ros2 topic echo /revohuman/left/raw --once
ros2 topic hz /revohuman/left/raw
```

右手使用独立串口和 `side:=right`。两个独立节点不执行双手 SOF 锁定或配对。
QoS 为 reliable / volatile / keep-last 10。`timeout_sec` 默认 1 秒；
`reconnect_delay_sec` 默认 2 秒。参数在启动时读取，修改需重启。
SDK 接收超时至少为 `3 / target_rate_hz + 0.5` 秒，因此低采样率时退出可能较慢。
退出或异常时通过 SDK 上下文尝试 STOP 并关闭串口，断连时不会重发缓存数据。

## 数据契约

`/revohuman/{left|right}/raw` 类型为 `revohuman_msgs/msg/RawFrame`：

- `encoder_angles_deg`：SDK 原始索引顺序的 21 路角度，单位度，不是语义关节角。
- `encoder_*_mask`：保留 ready / valid / reconnecting / offline 掩码。
  无效帧仍发布用于诊断；固件会保留旧数值，下游必须检查有效位与离线位。
- `encoder_sequence` / `encoder_device_tick`：判断编码器采样是否更新。
  流序号更新不代表编码器或触觉序号更新。
- `tactile_present` / `tactile_valid`：区分不存在与无效；数值保持设备原生单位。
- `sync_present` / `sync_valid` 和同步字段：保留 SDK 标签，只有 `sync_valid`
  为真才具有 SDK 所声明的同步语义；driver 不主动锁定 SOF。
- `header.stamp`：收到帧后的 ROS 主机时间；不是设备采样时间。
  `host_time_ns` 为 SDK 原始流帧主机时间；设备 tick 原样保留，没有做跨时钟映射。
- `header.frame_id` 仅为设备标识，不表示数据已转换为掌心空间坐标。

此层没有发布 `/joint_states`、`/humandex_eef_pose` 或 `HandKinematics`。
接入现有重定向仍需准确的编码器关节映射、标定和手套运动学/FK 适配层。

## 验证

构建并 source 后运行（系统 Python）：

```bash
python3 -m pytest src/brainco_drivers/revohuman_driver/test -q
```

测试通过真实 SDK 数据类构造输入，验证 ROS 消息转换，以及模拟串口断线、重连和退出清理。
