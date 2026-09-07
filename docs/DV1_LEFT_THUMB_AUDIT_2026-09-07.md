# DV1 左手拇指定位排查（2026-09-07）

累计排查发现：FK 数学对照通过，但输入工作空间与 Revo2 参考点仍未统一；
左手测试使用的 normalized 配置还有位置反馈缩放问题。下面按各次检查记录证据，
包括 adapter offset 试配，以及末节的设备更换、单位模式和固件限位只读检查。

## 范围与证据

- SDK：`sdk-encoder-fk-viewer`，`67531b1`。
- 输入：`tools/ros2_joint_state_pub.py` 默认 identity 换算。
- adapter：`input_mode=dv1_joint_states`，配置为根目录 `dv1_left_measured.yaml`。
- retarget：`pose_thumb`，`config/retarget.yaml`。
- FK、adapter、retarget 实现及 retarget YAML 的源码与 `install/dv1_sdk_test`
  对应文件 SHA256 相同，排查时没有发现该安装目录中的旧实现。
- 数值记录：`/tmp/dv1_left_thumb.csv`，21:09 留存的 19 条记录。
  它们不是本次新采的多姿态数据，也不是机器人实测指尖位置。
- 排查开始时未发现正在运行的 SDK、adapter 或 retarget 进程；不能据此断言
  用户每次手动启动时都加载了同一组参数。

## 1. FK 对照

重新执行 `revohuman_kinematics/checks/test_joint_fk.py`：8 项通过。
其中 SDK MuJoCo 对照使用 SDK 当前 URDF，左右手各 21 组角度（零姿态和 20 组
固定种子的随机角度），检查五个 DIP link 的位置、旋转和四元数；绝对容差
`1e-9`，相对容差为零。另覆盖 DIP 局部偏移与角度跨界滤波。

重新执行 adapter 的 `checks/test_dv1_adapter.py` 与 `test/`：9 项通过。
覆盖实际转换回调、关节名字重排、五指顺序、同帧时间戳、掌心变换与四指映射。

代码路径核对：

1. SDK 发布器复用 viewer 的 `EncoderMap.to_rad`，默认 `q = wrap(deg - 180°)`。
2. adapter 按关节名字取弧度，不再减 180°，不再翻符号。
3. FK 用 `origin × joint_rotation` 累乘，输出相对 `left_palm_link` 的位姿。
4. 默认 EMA alpha=0.2 会改变运动中的角度，但保持静止后收敛；在 200 Hz 下，
   低频等效延迟约为 4 帧，即 20 ms。对照实时数据应使用 `/humandex_left/fk_joint_states`。

结论：未发现 FK 实现、单位或关节排列错误。上述一致性验证的是同一个模型和
同一个参考点，不等于验证该点就是佩戴者的实际拇指最末端。

## 2. adapter 输出还没有完成工作空间适配

`dv1_left_measured.yaml` 的掌心平移、RPY 和五指 tip 偏移均为零。
因此在当前配置下：

```text
p_native = FK(DV1, q, left_thumb_DIP_Link 原点)
p_HandKinematics = p_native
p_IK ≈ 1.13 × p_HandKinematics
```

最后一步的 1.13 是 retargeter 的 `CANONICAL_LANDMARK_SCALE`；当前
`thumb_ik_position_scale=1.0`，四指中心项抵消。动态时还存在末端 EMA。
把 frame_id 改成 `hand_retarget_left` 本身不会建立两个模型之间的坐标变换。

对 Revo2 当前 IK 参考点，在两个主动拇指关节的 URDF 范围内采样
201 × 201 = 40,401 个姿态，使用 `mj_kinematics` 计算可达点：

| 指标 | 结果 |
| --- | --- |
| IK 参考 body | `left_thumb_distal_Link` 原点 |
| 主动关节范围 | proximal 0～1.0472 rad；metacarpal 0～1.5184 rad |
| 可达点 XYZ 下界，mm（网格近似） | (9.137, -84.190, 36.633) |
| 可达点 XYZ 上界，mm（网格近似） | (69.919, -6.698, 75.365) |
| 19 条记录中的目标 Z，mm | 135.423～147.651 |
| 每个目标到可达网格的最近距离，mm | 最小 64.473；中位 69.178；最大 72.382 |
| 记录中的指令 FK 误差，mm | 最小 68.685；中位 73.188；最大 78.503 |

这些范围属于 retarget 内部模型的参考系。网格是可达范围的数值近似，
不是人体空间测量，也不是连续全局最优的解析证明。

再用现有 IK 对每个目标离线执行 320 次迭代，残差仍为 64.473～74.100 mm，
所有结果的 proximal 均为上限 1.0472 rad。因此仅增加迭代或重启不能解决
这组不可达目标。19 条记录的最终 proximal 指令均为 0.890119755 rad。

## 3. 参考点和 Manus 输入约定

- DV1 当前 `thumb_tip` 是 `left_thumb_DIP_Link` 的原点，局部偏移为零。
  只转最后一个 DIP，会转动整个可视网格，但不移动该原点；因此可视化明显弯曲时，
  这个位置目标仍可不变。这是当前末端定义的结果，不是 FK 丢了关节。
- Revo2 IK 使用 `left_thumb_distal_Link` 原点，没有使用 URDF 中的
  `left_thumb_tip`。后者相对前者有 `(0, 0.0265, 0)` m 的固定平移。
  “tip”这个变量名不能保证两边都指真正的指尖。
- Manus adapter 提供 `thumb_tip`、`thumb_dip`、`thumb_pip` 三个位置。
  现有 retargeter 可以利用它们估计并固定 proximal，再解另一个关节。
- DV1 adapter 目前只提供五个 tip 位置，没有后两个拇指 landmark；当前走的是
  两个主动关节一起拟合单个位置的分支。`joint_positions` 里的 `thumb_dip`
  是角度，不能替代 `landmarks_m` 里的同名位置。

因此算法相同也不代表两种手套走相同的约束分支。补充 landmark 前必须核对
解剖对应关系；直接把各个同名 link 拼上去可能得到不同的弯曲角定义。

## 4. 现有输出调节的影响

`retarget.yaml` 是 Manus 调试参数，IK 解还会经过：

```text
q_proximal_out  = clip(0.75 × q_proximal_IK + 6°)
q_metacarpal_out = clip(1.20 × q_metacarpal_IK + 6°)
```

所以 IK proximal 为 60° 上限时，最终目标为约 51°，看目标 topic 不一定
直接看到 60°。对上述离线收敛结果，这层输出调整让模型参考点移动约
7.44～7.73 mm。它是次要影响；本次主要偏差在目标空间中已经存在。

## 修正顺序建议

1. 保留已经对照通过的 DV1 FK 与固件角度换算。
2. 先明确需要追踪 DIP 关节中心还是实际接触点；统一输入与 Revo2 求解点的意义。
   当前要继续使用既有 retargeter，就需要按它的 distal 原点约定构造目标。
3. 在 adapter 中补齐 DV1 到 Revo2 的空间适配：先对齐参考系、参考姿态与尺寸，
   再用并拢、外展、接近小指等实测姿态核对可达性。不能把一次观察到的 Z 差
   直接作为所有姿态通用的平移，也不能保证 5 关节的 DV1 轨迹能被 2 主动关节精确复现。
4. 单独准备 DV1 调试用 control YAML，离线对照关闭输出比例/偏置后的结果；
   核对驱动模型与真机零点之前，不把中性参数直接当成真机最终参数。
5. 将 IK 目标、内部解的 FK 和最终指令的 FK 分开记录。先确认目标进入可达范围，
   再做真机反馈与接触位置检查。

本次没有修改生产链路，也没有向机器人发布动作指令。

## 后续三姿态回放与指尖模式核对

用户补充：食指与拇指尚有距离，移向中指时拇指不再继续；此前 Manus 可以碰到中指。
这不能直接归因于机器人机械行程不足，还需区分当时的手别、输入约定、目标限位与
真机反馈。排查时未发现运行中的 retarget/控制器，本节没有测量实际机器人运动。

对 `/tmp/dv1_left_thumb.qEIupa` 中 21:41 的三份 HandKinematics，用其中关节角
重新计算 SDK URDF FK，与保存的末端点最大误差约 `1.6e-17` m。使用当前
`retarget.yaml` 和实际 `PoseThumbRetargeter`，逐姿态从零初始化并重复输入 100 次：

| 手套记录 | IK 目标 XYZ，mm | IK 原始角度 proximal/metacarpal，° | 最终目标角度，° | 当前参考点误差，mm |
| --- | --- | --- | --- | --- |
| 并拢 | (46.27, -83.19, 140.96) | (53.20, 13.36) | (45.90, 22.03) | 77.34 |
| 外展 | (97.42, -71.25, 116.92) | (35.37, 48.14) | (32.53, 63.76) | 76.33 |
| 接近小指根部 | (69.17, 36.81, 92.05) | (39.14, 87.00) | (35.35, 87.00) | 55.71 |

第三帧在离线回放中已经达到 metacarpal 上限；这些记录不包含专门的食指/中指捏合
姿态，不能把上述数值当成那两个动作当时的实时目标或反馈。

后续应优先统一指尖定义，并对照 Manus 的 tip/DIP/PIP 输入约束。三姿态空间映射
的试配已停止，临时添加但尚未启用/安装的映射代码已撤回。

另验证了将 Revo2 求解点从 distal 原点移到真正 tip 时需要处理的两项：

- 当前 MuJoCo 加载后 `left_thumb_tip` 的 body ID 为 -1（固定 link 被合并），
  不能只把 body 名字换掉；可使用 distal body 上的 URDF 局部偏移求点与雅可比。
- distal 与 proximal 按 1:1 联动。以 `(proximal, metacarpal)=(0.5, 0.8)` rad
  做中心差分，tip 的 proximal-only 雅可比误差为 0.0265 m/rad；将 distal 列
  加到 proximal 列后误差约 `1.0e-11` m/rad。真正 tip 模式必须包含这一贡献。

## 仅调整 adapter offset 的试验

按用户要求，仅另存 `dv1_left_thumb_tip.yaml`：其参数与 `dv1_left_measured.yaml`
完全一致，只有 `tip_offsets_m` 最后三项改为 `[-0.003, 0.009, 0.032]` m。
这是 SDK `left_thumb_DIP_Link.STL` 沿局部 +Z 最前端顶点
`(-2.857, 9.309, 32.445)` mm 的取整初值。已检查 URDF visual 原点为零，
该值可以直接表达在 DIP 局部系中；它代表模型外表面，尚非佩戴者实际指尖实测值。
没有修改 Revo2 求解点、IK、输出比例、掌心变换或四指标定，也没有启动真机。

使用三份记录重新计算，偏移后的点与 SDK MuJoCo 同一 body 上的局部偏移点
在 `1e-9` m 容差内一致；21 个输入关节和另外四指位姿完全相同。
只增加 DIP 30° 时，原点不动，新参考点移动约 15.596 mm。

| 姿态 | 原/新当前 IK 参考点残差，mm | 原/新目标角度 proximal、metacarpal，° |
| --- | --- | --- |
| 并拢 | 77.34 / 105.23 | (45.90, 22.03) / (51.00, 6.00) |
| 外展 | 76.33 / 99.80 | (32.53, 63.76) / (33.49, 45.46) |
| 接近小指根部 | 55.71 / 63.60 | (35.35, 87.00) / (45.13, 87.00) |

这些误差仍以现有 Revo2 distal 原点为求解参考，并非实际指尖接触误差。
此轮验证了 adapter offset 的生效方式；单改 offset 没有解决已记录的目标限位，
不能把这份初值称为已经完成的对中指标定。
完整回放结果：`/tmp/dv1_left_thumb_offset_replay.json`。

## Speed 模式、单位换算与固件限位（22:21～22:23）

用户反馈 offset 改动略有改善，但 IP 弯曲反向，对中指时侧摆仍停住；随后说明
记得使用的是 speed + physical，并确认期间换过 Revo2 手或重新插拔过 USB。

### 运行模式和设备身份

- 22:10 的 launch 日志：原设备序列号末尾 `00004`、固件 `1.0.22.U`；
  `Finger unit mode confirmed normalized`，随后切到 `SPEED-BASED`。
- 左手本地文件 `revo2_left_local.yaml` 显式设置 `finger_unit_mode: normalized`。
  右手 `protocol_modbus_right.yaml` 设置 physical，不代表本次左手加载 physical。
- 22:21 当前 `/dev/ttyUSB0`、slave 126：序列号末尾 `0000F`、固件 `1.0.14.U`。
  直接读寄存器 937 得到 0，即 normalized。不能把这只手的限位当成上一只手的实测值。
- 使用驱动实际加载的 SDK 1.5.1 只读复核：设备身份、单位模式、位置值均与直接
  Modbus 读取一致。当前无控制进程，不能把静态单位模式读取说成正在 speed 运动。

### 当前设备的固件配置值

| 电机 | 最小/最大位置，° | 最大速度，°/s |
| --- | --- | --- |
| Thumb（弯曲） | 0 / 59 | 145 |
| ThumbAux（侧摆） | 0 / 90 | 160 |
| 食/中/无名/小指（各自） | 0 / 81 | 130 |

最大电流均为 1000 mA；ThumbAux 锁定电流读到 100 mA。本轮没有修改这些设置。
这是固件报告的配置范围，不是带负载实测的运动终点，也不能证明 speed 模式没有
额外保护或机械限制。当前静态反馈速度全零、motor_states 全为 idle。

读取脚本：`src/brainco_drivers/revo2_driver/scripts/read_modbus_limits.py`。
仅发送 FC03/FC04，运行前检查串口占用，不切换单位模式、不发送动作。
已通过 CRC 已知向量校验和真实设备读取。
记录：`/tmp/revo2_left_limits_identity_20260907.json`。
寄存器和 normalized 映射定义参考
[BrainCo Revo2 通信协议](https://www.brainco-hz.com/docs/revolimb-hand/revo2/modbus_touch.html)。

### 软件缩放会提前满足位置目标

驱动 `brainco_hand_hardware.cpp` 的 read() 对全部电机使用同一个
`position_state_scale=0.0017453292519943296`，即把 normalized 1000 当作 100°。
协议定义却是 normalized 0～1000 映射到各电机自己的最小/最大位置。
当前 SDK 与寄存器位置均为 `[399,399,49,50,50,49]`，没有替驱动做角度换算。

以当前这只手的配置推算（不含死区、动态超调、真实零点误差）：

| 拇指轴 | retarget 最终目标上限，ROS ° | 当前换算下对应 normalized | 按固件范围换算的物理 ° |
| --- | --- | --- | --- |
| 弯曲 | 约 51 | 约 510 | 约 30.09 |
| 侧摆 | 约 87 | 约 870 | 约 78.30 |

弯曲的 51° 来自 IK 上限约 60° 经 `0.75 × q + 6°` 的现有输出调整；
控制器自身的弯曲目标上限仍约 60°。这些是代码和协议推算，不是已观察到的实际终点。
如果单位比例不一致，PID 可以在真实关节尚未到预期角度时认为已到目标，不能据此
认定固件挡住了完整行程。

正比例缩放本身不会翻转 IP 方向；IP 反向还需对照手套 DIP、retarget 弯曲目标、
机器人反馈的变化方向。此前记录里的 IK 侧摆饱和也不能由底层换算独自解释。

下一轮应先统一 speed + physical 的位置/速度换算并验证静止读数；不能仅把
finger_unit_mode 改成 physical，保留原来的 normalized 缩放。然后在用户观察真机时
低速分轴记录目标、反馈、SDK 速度命令、电流和状态：目标与反馈接近而速度归零指向
软件目标/映射，持续同向速度命令但反馈停住才继续查固件保护或机械限位。
此次没有改变驱动运行配置或固件参数，也没有执行运动扫限位。
