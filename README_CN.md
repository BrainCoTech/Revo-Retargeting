# Revo Retargeting

用 MANUS 手套控制 Revo3 机器手。当前分支为 `feat/manus-revo3-vector`，下面按 **Vector 右手遥操作**说明。

[English](README.md) · [首次安装](docs/vector_setup_CN.md) · [Vector 参数与原理](src/manus_revo3_vector_retarget/README.md) · [HumanDex / DV1、legacy、Revo2](docs/other_inputs_CN.md)

## 日常使用：启动、标定、启用

已完成首次安装后，每次按下面两个终端操作。给机器手上电并连接 USB，接入 MANUS 接收器、打开并戴好手套。所有命令在本仓库根目录执行，终端保持运行。

### 终端 1：启动右手

```bash
conda activate revo_teleop
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=11
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/teleop.sh right retarget_backend:=vector
```

这条命令同时启动机器手驱动、MANUS 采集和 Vector 重定向。保持自然、稳定的手形，首次采集 120 帧完成骨长标定；有缓存时用 30 帧复核。

**标定完成后，机器手仍不会跟随。需要在终端 2 启用输出。**

### 终端 2：确认就绪并启用

```bash
conda activate revo_teleop
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=11
source /opt/ros/humble/setup.bash
source install/setup.bash

# 查看状态；phase 为 READY 且 ready_to_arm 为 true 后，执行下一条
ros2 topic echo /manus_revo3_retarget/status std_msgs/msg/String --once

# 启用真机跟随
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: true}'
```

`ROS_DOMAIN_ID=11` 是示例，两个终端必须一致。每次只运行一套启动命令；省略 `retarget_backend:=vector` 会使用 legacy。

### 暂停、退出、重新标定

```bash
# 暂停跟随；恢复时再次调用 data: true
ros2 service call /manus_revo3_retarget/enable_retarget std_srvs/srv/SetBool '{data: false}'

# 重做骨长标定；完成后需要再次启用
ros2 service call /manus_revo3_retarget/recalibrate std_srvs/srv/Trigger '{}'
```

退出时在终端 1 按 `Ctrl+C`，脚本会清理本次启动的进程。输入中断会停止输出，输入恢复后也需要重新启用。暂停会停止 MIT 命令发布，机器手仍保持上电。

## 左手和双手

将终端 1 启动命令中的 `right` 换成 `left` 或 `both`。单手服务名相同；双手分别启用：

```bash
ros2 service call /manus_revo3_retarget_left/enable_retarget std_srvs/srv/SetBool '{data: true}'
ros2 service call /manus_revo3_retarget_right/enable_retarget std_srvs/srv/SetBool '{data: true}'
```

## 启动了但不动

先在终端 2 查看 `status`：

| 状态或现象 | 处理 |
| --- | --- |
| `WAITING_INPUT` | 检查手套电源、MANUS 接收器和采集日志 |
| `CALIBRATING` | 保持自然、稳定的手形，等待标定完成 |
| `READY`、`enabled=false` | 调用 `enable_retarget`，传 `data: true` |
| `BLOCKED` 或 `fault` | 查看 `reason`、`controller.reason`；驱动或输入恢复并就绪后，重新启用 |
| 状态话题或服务找不到 | 检查两个终端的工作区和 `ROS_DOMAIN_ID` 是否一致 |
| 找不到 `install/setup.bash` 或 Vector 包 | 按[首次安装](docs/vector_setup_CN.md)构建 |

左右手四指侧摆的硬件正方向都是**朝拇指**，Vector 已在两侧 profile 中对四指 MPR 关节设置 `scale: -1`。改过 profile 后需重启 Vector；详细方向说明与参数见 [Vector 文档](src/manus_revo3_vector_retarget/README.md#模型和硬件坐标)。

左右手拇指 MCP 默认都带 30° 方向补偿，用来缓解对掌时 MCP 降不下来的问题。可以在启动命令末尾加 `thumb_offset_deg:=<角度>` 调整，越大 MCP 越低；原因和取舍见 [拇指方向补偿](src/manus_revo3_vector_retarget/README.md#拇指方向补偿)。

默认输出不做位置插值。需要 200 Hz 插值和 2 秒启动缓入时，在启动命令末尾添加 `interpolate:=true`。
