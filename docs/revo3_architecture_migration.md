# Revo3 重定向处理与配置迁移

本次整理将拇指两层补偿合并，删除四指中心缩放、拇指目标位置 EMA 和每帧角度变化限制，将 MANUS/HumanDex 的 aggregate flexion 移至 Revo2，并将 IK 残差按物理尺度归一化。上游 FK 的关节滤波没有修改；输出插值、IK 数值阻尼和每次迭代步长限制保留。

## 唯一的输出校准

拇指求解器输出模型关节角，节点统一执行 `q_cmd = scale * q_model + offset`，最后按 URDF 限位截断。参数统一为 `physical_<side>_<finger>_<JOINT>_joint_scale/offset_deg`。

旧两层变换 `q1=a1*q+b1; q2=a2*q1+b2` 合并为 `a=a2*a1; b=a2*b1+b2`。仓库自带配置已迁移。迁移自定义配置时必须使用**旧默认文件和旧覆盖文件**，按原 launch 加载顺序合并后再转换，不能分别转换再叠加：

```bash
PYTHONPATH=src/manus_revo3_retarget python3 -m manus_revo3_retarget.migrate_calibration \
  /path/to/old/thumb_retarget.yaml /path/to/old/calibration.yaml --output /tmp/new_calibration.yaml
```

转换结果是有效配置快照；旧参数不能直接输入新节点。迁移工具同时转换 IK 权重并删除已移除的参数。工具只用于旧版文件，不能对新版重复转换。

原先拇指内部存在中间截断。合并后仅保留最终限位，饱和区域不保证等价；四指输出现在也按模型限位截断。输出补偿仍位于 IK 后，因此补偿后指尖几何误差与求解器报告的误差不同。若偏移属于执行器编码，应在之后的独立工作中迁移到驱动并同步反馈换算。

## 拇指目标和归一化残差

拇指仅要求 `thumb_tip`；`thumb_pip`、`thumb_dip` 是可选任务。四指关节仍是四指重定向的必需输入，但四指指尖不再是拇指求解的依赖。不再计算四指中心或 `reach_scale`，也不再进行目标位置 EMA 或每帧角度裁剪。

位置、姿态和平滑残差分别除以 `thumb_ik_position_sigma_m`、`thumb_ik_posture_sigma_deg`、`thumb_ik_smooth_sigma_deg`（后两项运行时转换为 rad）。目标函数为：

`E = lambda_tip * ||(FK_tip(q)-tip)/sigma_p||² + optional_position_tasks + sum(lambda_posture*w_i*((q_i-ref_i)/sigma_q)²) + lambda_smooth*||(q-q_prev)/sigma_s||²`。

残差和 Jacobian 同时乘 `sqrt(lambda)/sigma`，权重为非负有限数，尺度为正有限数。各关节姿态权重由 `thumb_ik_posture_<cmp|cmr|mcp|pip|dip>_weight` 配置。负的人手屈曲角截零规则保留，本次没有修改姿态参考的解剖学映射。

默认尺度为 0.01 m、10°、10°，作为参数表达尺度，不宣称是实测传感器误差。默认权重按 `lambda=(旧残差行系数*sigma)²` 转换，保留旧任务之间的相对取舍和数值行系数，而不是把所有 lambda 强行设为 1。tip 权重为 0.0004；默认 posture/smooth 权重约为 0.00030461742。

`thumb_ik_normalized_tolerance` 判断归一化总残差。DEBUG 日志分别报告最终求解姿态的 `tip_error_m`、`posture_rms_rad`、`normalized_residual` 和迭代次数。这些是补偿前模型误差。没有可用姿态参考时 posture RMS 记为 0，不能解读为姿态匹配成功。

## Revo2 flexion 的归属

MANUS/HumanDex adapter 只发送独立关节与空间点。Revo2 通过 `finger_flexion_config`（节点 CLI 为 `--finger-flexion-config`）选择映射，不根据消息 `source` 隐式选择算法。

- `flexion_manus.yaml`：原 MCP/PIP/DIP 权重 0.50/0.35/0.15。
- `flexion_humandex_pip.yaml`：原 +12° 到 -12° 映射到 0..1.4661 rad。
- `flexion_humandex_right_calibrated.yaml`：保留录制的右手三编码器开合端点。
- `flexion_dv1_<side>.yaml`：原 DV1 profile；左手临时端点仍不是实测标定。
- `flexion_dv1_left_measured/rechecked/thumb_tip.yaml`：从仓库根目录旧 adapter 文件拆出的端点。

文件位于 `src/brainco_capabilities/revo2_hand_retarget/config/`。Revo2 bringup 的 MANUS/HumanDex profiles 已指定相应配置，DV1 launch 默认使用对应侧 DV1 配置。使用根目录 measured adapter 文件时，还要显式选择相应 flexion 文件：

```bash
ros2 launch revo2_teleop_bringup dv1_sdk.launch.py \
  hand_mode:=left urdf_path:=/path/to/Revo_Human_DV1_URDF_Bimanual.urdf \
  adapter_config:=/path/to/dv1_left_measured.yaml \
  finger_flexion_config:=flexion_dv1_left_measured.yaml
```

`calibrate_dv1_fingers` 已迁至 `revo2_hand_retarget`，输出 Revo2 映射文件。重启时传给 `finger_flexion_config`，不再传给 adapter。没有指定映射时保留已有 aggregate 字段（供尚未迁移的 Hex/RevoHuman 路径），不会猜测设备；直接启动 MANUS/HumanDex 的 Revo2 消费者必须显式指定配置。

## 指尖与指腹参考点（2026-09-14 更新）

`*_tip` 表示手套／模型指尖。HumanDex 上游以 `p_tip=p_DIP+R_DIP*offset_m` 计算指尖，adapter 不再叠加；Revo3 IK 保持追踪现有 `thumb_tip_Link`。指腹点独立保存，暂不参与优化。

`src/manus_revo3_retarget/config/humandex_fingertips.yaml` 记录本次采用的偏移：来源是 BrainCo-HumanDex 的 `HumanDex_bimanual.urdf` 网格，以右手数据为准，左手仅 Y 取反。BrainCo-HumanDex 的默认 `config/config.yaml` 已按左五指、右五指的顺序写入 `eef_tip_offsets_m`。FK 实现已支持局部偏移，因此本次无需修改 FK 算法。

选点采用沿末节纵向最前端 0.1 mm 表面区域的面积中心，再沿纵向投影到网格表面；这是模型几何参考点。区域厚度在 0.05–0.5 mm 间变化时，选点最多变化约 0.8 mm。它不是实物测量精度，也不代表人手皮肤接触点。直接 DV1 SDK 路径使用另一份 URDF，DIP 转轴也不同，因此不修改其零偏移或试验偏移配置。自定义上游参数文件仍可能覆盖默认值。

`config/revo3_pad_contacts.yaml` 保存用户选定的 Revo3 指腹点，全部相对现有 `tip_Link`，单位 m。双手拇指采用用户分别给出的数值；左手四指按右手镜像并依据 URDF 局部坐标转换。该文件是备用参考记录，不由当前 launch 加载，也没有修改驱动子模块或机器人 URDF。

`scripts/fit_fingertip_contact.py` 保留为未来实物接触点标定工具，不用于本次模型指尖定义。它输出固定接触点拟合记录，不能直接覆盖本次几何指尖。

## 验证范围

自动化检查覆盖补偿合成、残差系数迁移、固定接触点拟合的可观测性、Revo2 端点和 MANUS/PIP 公式对照、adapter 不再输出派生量、C++ 求解器有限输出、输入不依赖四指指尖。实测接触点标定和机器人运动验收仍需独立进行。

### 本次离线检查结果

C++ 节点和求解库通过独立 CMake 构建；CTest 的求解器检查和 Python 检查均通过。三个变更的 Python 包通过 setup.py build；另外运行了 27 项 Python 回归测试，全部通过。colcon 在本机执行期间停留在子进程协调阶段，已中止，未将其报告为完整工作区构建成功。验证使用 /tmp 中的独立构建，运行环境仍需重新构建安装后再重启。

对 5 份配置分别扫描左右拇指 10 个关节，每关节在 URDF 范围内均匀取 301 个输入角，共 3010 点/配置。未饱和区间的最大差异为 2.22e-16 rad。存在差异的采样点均为旧两层输出已经超出模型限位的点，新输出限制在模型范围内；最大变化约 32°。这不是实际轨迹误差或实机运动结果。

| 配置 | 输出变化点数 / 3010 | 旧输出超限点数 |
|---|---:|---:|
| thumb_retarget.yaml | 342 | 342 |
| revo3_left_measured.yaml | 304 | 304 |
| retarget_tuning_left_DV1.yaml | 233 | 233 |
| retarget_tuning_left_DV2.yaml | 233 | 233 |
| retarget_tuning_right_DV1.yaml | 233 | 233 |

本次未启动硬件、未执行实物接触点采样，也未用实测完整运动轨迹评估删除滤波后的动态响应。恢复实机前应先用录制输入验证延迟、关节速度与突变。

### 第二项：指尖接入验证（2026-09-14）

本次指尖变更的 12 项检查全部通过：配置记录与上游默认值一致、左右严格镜像，以及双手十指在 DIP 从 0 转到 0.6 rad 时，DIP 原点不变而指尖移动，输出满足 `p_DIP + R_DIP * offset`。测试直接调用上游实际 FK 发布函数，不启动 ROS 图或硬件。联合运行 Revo3 Python 测试和 HumanDex adapter 测试共 27 项通过；两个仓库 `git diff --check` 通过。

测试 `test/test_fingertip_geometry.py` 可用 `HUMANDEX_ROOT` 指定上游仓库；缺少上游或 ROS 时跳过跨仓库检查。更新安装目录中的 HumanDex 参数文件后重启 FK 才会生效，本次未重启进程或操作硬件。新参考 YAML 随 manus_revo3_retarget 的 config 目录安装，但不会自动作为 ROS 参数加载。

## DV1 启动职责

SDK 串口采集由上游独立启动。本仓库的 `hand_input_adapters/dv1_input.launch.py`
只监听 JointState、计算 FK 并发布 HandKinematics；Revo3 使用
`input_source:=external` 接收数据，并通过 `input_config` 显式加载
`src/manus_revo3_retarget/config/input_humandex.yaml` 的字段映射。
完整命令见 [输入适配器说明](../src/brainco_capabilities/hand_input_adapters/README.md#右手-dv1--revo3)。
旧 Revo2 `dv1_sdk.launch.py` 保留适配器与 Revo2 的组合启动，移除 SDK 采集参数，
必须显式传入 `urdf_path`。切勿同时启动新输入入口和旧入口，以免重复发布。
