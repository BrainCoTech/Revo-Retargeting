# revo3_driver Joint State 接口简表

本文只整理 `revo3_driver` 当前对外发布的关节状态接口，以及 SDK motor 顺序和 ROS joint 顺序的差异。`<side>` 取 `left` 或 `right`。

## 状态 Topic

| Topic | 消息类型 | 发布者 | 数据内容 | 备注 |
| --- | --- | --- | --- | --- |
| `/revo3_<side>/revo3_joint_state/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | 有效数据为 `name` / `position` / `velocity` | ROS 消息类型自带 `effort` 字段；当前无 state `effort`，Humble broadcaster 实测填 `NaN`，不要使用 |
| `/revo3_<side>/revo3_joint_state/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | `joint_state_broadcaster` | 每个 joint 的 `position` / `velocity` / `current` / `motor_state` interface | 诊断和 monitor 应优先看这个 topic |

## State Interface

| Interface | ROS 侧单位/含义 | SDK 来源 | 转换 | `joint_states` | `dynamic_joint_states` |
| --- | --- | --- | --- | --- | --- |
| `position` | rad | `revo3_get_motor_status_data().positions[]`，degree | degree -> rad | 有 | 有 |
| `velocity` | rad/s | `revo3_get_motor_status_data().velocities[]`，rpm | rpm -> rad/s | 有 | 有 |
| `current` | A | `revo3_get_motor_status_data().currents[]`，mA | mA -> A | 无 | 有 |
| `motor_state` | raw SDK status bitmask | `revo3_get_motor_status_data().statuses[]`，`uint16_t` | 原值转成 double 承载 | 无 | 有 |

读取策略：

- `revo3_get_motor_status_data()` 读取失败时，本次 `read()` 返回 error。
- ROS 层目前只导出前 21 个手部 motor；SDK 的 `M21/M22` wrist motor 不进入 Revo3 hand joint state。
- 当前硬件 state interface 不导出 `effort`；`/joint_states.effort` 若出现 `NaN`，表示没有 effort 状态反馈，不是 current，也不是 Nm。
- 当前硬件 state interface 不导出 `error`；故障判定应由 monitor/应用基于 `motor_state` 执行。

## Motor State Bitmask

SDK `statuses[]` 为 `uint16_t` bitmask。当前 `dynamic_joint_states` 中的 `motor_state` interface 直接承载这个原始数值。

| Bit | Flag | 含义 | 处理建议 |
| --- | --- | --- | --- |
| 0 | OverCurrent | 持续过流，SDK 文档为 >= 1.5A 持续 50ms | 自动停机，检查负载/阻塞 |
| 1 | OverVoltage | 电压 > 26V | 降低供电 |
| 2 | UnderVoltage | 电压 < 8V | 检查/充电电池 |
| 3 | OverTemperature | 温度 > 110°C | 等待降到 < 90°C |
| 4 | CurrentSpike | 峰值电流达到 2A | 自动停机，检查冲击/阻塞 |
| 8 | Stalled | 电机堵转 | 检查机械阻塞 |
| 11 | Running | 电机正在运行 | 状态位，不算故障 |

Driver 不做 fault 掩码；判断是否有真实故障时应在 monitor/应用侧忽略 bit 11：

```text
has_fault = (motor_state & ~(1 << 11)) != 0
```

其他 bit 当前按 reserved/unknown 处理；如果出现未知 bit，建议保留原始 `motor_state` 数值并结合 SDK/固件版本排查。

## ROS Controller Joint 顺序

所有 controller 的 `joints` 列表、硬件 state buffer、空 `joint_names` 命令数组，都使用下面顺序：

| ROS index | Joint name pattern | 说明 |
| --- | --- | --- |
| 0 | `<side>_little_MPR_joint` | 小指外展 |
| 1 | `<side>_little_MCP_joint` | 小指 MCP |
| 2 | `<side>_little_PIP_joint` | 小指 PIP |
| 3 | `<side>_little_DIP_joint` | 小指 DIP |
| 4 | `<side>_ring_MPR_joint` | 无名指外展 |
| 5 | `<side>_ring_MCP_joint` | 无名指 MCP |
| 6 | `<side>_ring_PIP_joint` | 无名指 PIP |
| 7 | `<side>_ring_DIP_joint` | 无名指 DIP |
| 8 | `<side>_middle_MPR_joint` | 中指外展 |
| 9 | `<side>_middle_MCP_joint` | 中指 MCP |
| 10 | `<side>_middle_PIP_joint` | 中指 PIP |
| 11 | `<side>_middle_DIP_joint` | 中指 DIP |
| 12 | `<side>_index_MPR_joint` | 食指外展 |
| 13 | `<side>_index_MCP_joint` | 食指 MCP |
| 14 | `<side>_index_PIP_joint` | 食指 PIP |
| 15 | `<side>_index_DIP_joint` | 食指 DIP |
| 16 | `<side>_thumb_MCP_joint` | 当前 ROS slot 名 |
| 17 | `<side>_thumb_PIP_joint` | 当前 ROS slot 名 |
| 18 | `<side>_thumb_DIP_joint` | 当前 ROS slot 名 |
| 19 | `<side>_thumb_CMP_joint` | 当前 ROS slot 名 |
| 20 | `<side>_thumb_CMR_joint` | 当前 ROS slot 名 |

## SDK Motor 顺序

SDK raw array 是 motor id 顺序：`M0..M22`。当前 driver 读取 SDK 前 21 个值，并按同一个下标写入 ROS state slot。

| SDK motor id | SDK 文档定义 |
| --- | --- |
| M0 | Pinky Abd |
| M1 | Pinky MCP |
| M2 | Pinky PIP |
| M3 | Pinky DIP |
| M4 | Ring Abd |
| M5 | Ring MCP |
| M6 | Ring PIP |
| M7 | Ring DIP |
| M8 | Middle Abd |
| M9 | Middle MCP |
| M10 | Middle PIP |
| M11 | Middle DIP |
| M12 | Index Abd |
| M13 | Index MCP |
| M14 | Index PIP |
| M15 | Index DIP |
| M16 | Thumb CMC Rotation |
| M17 | Thumb MCP |
| M18 | Thumb IP |
| M19 | Thumb CMC Abd (diff) |
| M20 | Thumb CMC Flex (diff) |
| M21 | Wrist Flex/Ext，不导出到 ROS hand joint |
| M22 | Wrist Abd，不导出到 ROS hand joint |



## 顺序差异和风险点

| 范围 | 关系 | 说明 |
| --- | --- | --- |
| 0-15 | 基本一致 | ROS 的 `MPR` 可视作 SDK 的 `Abd`，四指按 motor id 升序排列。 |
| 16-20 | 拇指语义不一致 | SDK 文档是 `CMC Rotation / MCP / IP / CMC Abd / CMC Flex`；当前 ROS slot 名是 `thumb_MCP / thumb_PIP / thumb_DIP / thumb_CMP / thumb_CMR`。 |
| `REVO3_FINGER_MOTORS` | 与 ROS controller 顺序不同 | 这是 SDK GUI/按手指展示顺序，不要直接用于 `Float64MultiArray` 或空 `joint_names` 的 controller command。 |

工程使用建议：

- 读取 `dynamic_joint_states` 时，以 `joint_names` 和 `interface_values.interface_names` 对齐数据，不要只靠数组下标。
- 发 `Revo3MITCommand` 时优先填写 `joint_names`，controller 会按名字重排；空 `joint_names` 时数组必须严格匹配 ROS controller 顺序。
- 做拇指诊断时同时标注 ROS joint name 和 SDK motor id，避免把 `M16..M20` 的物理含义混淆。
