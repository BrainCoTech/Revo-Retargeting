# 共享重定向求解器：侧摆、对指与屈伸

本轮只修改 `experiments/revo3_lab/scripts/vector_solver.py` 的公共 `Solver`，所有动作使用同一条 21 自由度优化路径及同一组参数。任务名称只用于取样和评价；没有搓指周期、外部滑动轨迹或专用关节轨迹输入。旧版搓指实验的独立机器人 slide 结果已撤回，不能作为本轮证据。

## 优化目标

保留指尖位置、拇指到各指的相对向量、姿态参考、时间平滑和碰撞项。新增项默认关闭，旧配置输出与冻结版本作数值回归。

- 侧摆：在公共路径中使用已有的可观测侧摆参考，替换旧的投影角参考。根据输入的可观测性调整姿态权重；扣除侧摆后计算 MCP 屈曲参考。
- 指腹：从原碰撞网格提取五指固定 pad 几何。同一规则处理拇指与其余四指，无动作标签。近接时逐渐增加指腹位置和法向对置损失，降低对应的指尖/相对向量权重，避免重复目标强烈竞争。
- 方向：指腹相对位置在另一指的纵向、横向和法向框架中计算，分别设置尺度。输入框架只由另一指末节方向和手掌横向定义，不沿拇指位移旋转；切向目标直接来自骨架相对向量。
- 法向：使用统一的骨架到表面间距校准和小预压。校准会让间距小于 clearance 的正面输入饱和，所以不能宣称该区域完整跟踪法向幅度。反侧接近关闭 pad 权重，回到已有向量损失。原始输入和校准目标分别记录。
- 所有输出仍经同一限位、平滑、速度和加速度约束，并用动力学实际关节状态评价。

这是一套软约束最小二乘目标；增加 loss 不保证可达性、全局最优或无穿透。不得通过降低输出运动幅度而只报告接触率来认定成功。

## 检查与比较口径

DSW 执行 `check_shared_solver.py`：解析 Jacobian 对有限差分、输入切/横/法三个方向互不混淆、公共刚体变换不产生相对信号、旧配置对冻结版本兼容、侧摆分支、反侧接近，以及拒绝不支持的 hybrid+pad 组合。全部数值检查和模拟在注册 DSW 中运行。

`run_shared_evaluation.py` 冻结一次输入，四套配置复用相同骨架。包括 known_angles 合成诊断夹具、两段真实侧摆、两段真实对指和真实屈伸。known_angles 含超过机器人 MCP 屈曲范围的输入，需区分可达子集。真实 MANUS 骨架没有关节或皮肤接触真值；侧摆参考误差和 source-close 接触率是诊断指标。

接触评价门槛固定为输入拇食指 TIP 距离小于 15 mm。空窗口的接触率为 null；断触时长 0 不代表接触成功。预热阶段包含在全程穿透指标中，并另查跟踪阶段。其他手指接触不自动判定为错误动作。

新视频均显示同一帧输入骨架、旧版实际动力学输出和候选实际输出，标明真实/合成来源；指腹评价另提供近景。视频不会用优化目标关节替代实际关节状态。

## 结果

第一轮 run：`20260913T144909Z_shared_evaluation_e1377abb`。14 项回归全部通过，run：`20260913T144842Z_check_shared_solver_13c2520c`。以下为第一轮 `shared` 对 `legacy`，所有动作共用配置；当前配置保留为实验候选，未升级为默认。

| 输入 | 侧摆参考 RMSE（°）旧→新 | PIP/DIP 参考 RMSE（°）旧→新 | 指尖均值误差（mm）旧→新 | 全阶段最大穿透（mm）旧→新 |
|---|---:|---:|---:|---:|

| known_angles | 5.90 → 2.00 | 24.23 → 14.23 | 18.02 → 19.30 | 1.54 → 0.64 |
| side_finger_agility | 11.40 → 10.00 | 21.98 → 10.10 | 23.00 → 24.21 | 1.68 → 1.85 |
| side_hand_mobility | 11.44 → 8.02 | 23.85 → 16.59 | 26.34 → 26.36 | 2.40 → 2.21 |
| pinch_finger_agility | 7.59 → 7.21 | 31.48 → 18.91 | 12.03 → 13.55 | 1.95 → 3.71 |
| pinch_hand_mobility | 16.15 → 12.65 | 28.09 → 23.12 | 15.82 → 16.34 | 2.39 → 2.54 |
| flex_finger_agility | 12.11 → 7.49 | 27.32 → 12.98 | 13.48 → 13.35 | 6.76 → 1.61 |

合成夹具只看 MCP 屈曲在硬件界限内且侧摆可观测的子集，侧摆真值 RMSE 为 **4.26° → 1.66°**。这不是实测人体关节真值。

对指 FA 的 source-close 窗口只有约 **0.266 s**，接触率 **29.3% → 55.6%**；对指 HM 只有 **0.200 s**，接触率 **15.0% → 41.0%**。两者都远不足以认定持续稳定接触。FA 食指 PIP/DIP 实际运动范围约 **48.4°/16.9°**，说明已非只有 MCP 运动，但不能用幅度替代跟踪误差。

`shared` 的六片段 p95 求解耗时为约 **19–124 ms**，部分帧达到 max_nfev；没有证明实时 30 Hz。

实际指腹坐标分析 run：`20260913T145158Z_analyze_shared_ab87d85f`。在 alpha>0.5 的 FA 近接区只有 4 帧，切向绝对 RMSE 8.24 mm；HM 有 13 帧，RMSE 2.54 mm，但相关系数约 -0.92。短窗、局部框架运动和动力学滞后均会影响解释；不得把这些结果称为搓指成功。原始和校准法向分开保留，骨架 TIP 不是人体皮肤接触点。


第二轮 run：`20260913T145246Z_shared_evaluation_69b22ee7`。仅修改共同参数：碰撞权重 300/1000、预压 0.35 mm，以及一组 pad 权重 0.25。六组输入 hash 和重复基线数值均与第一轮一致，两轮共 48 个完整 rollout。

| 第二轮候选 | 全六片段最大穿透（mm） | FA 对指接触率 | HM 对指接触率 | 是否升为默认 |
|---|---:|---:|---:|---|
| collision300 | 6.56 | 42.9% | 28.0% | 否 |
| pad025 | 6.56 | 41.4% | 38.0% | 否 |
| collision1000 | 5.06 | 28.6% | 35.0% | 否 |

更大碰撞权重没有带来一致改善，且 max_nfev 触顶帧增加；固定损失权重的静态解、滤波命令路径和实际接触动力学之间仍有冲突。因此保留第一轮 shared 作为诊断展示，没有挑每个动作各自最好的配置拼接结果，也没有改生产默认。

间距偏置/预压是可调参数，并非测得的人体皮肤厚度。现有录制片段只能支持几何和近接诊断，尚不支持真实切向搓指成功的结论。下一步应继续在同一 Solver 内检查时间项、命令路径和切向误差的竞争；保持统一输入和评价门槛，不以增加独立目标运动解决。


第二轮实际坐标分析 run：`20260913T145636Z_analyze_shared_6d50b378`。例如 FA 对指的 `pad025` 跟踪阶段穿透为 0.47 mm，但预热接近阶段达到 6.56 mm，不能裁掉预热只报告前者。

## 文件与复现

- 配置与状态：`experiments/revo3_lab/configs/shared_batch_01.json`、`shared_batch_02.json`、`shared_iteration_status.json`。
- 本地证据：`artifacts/revo3_lab/shared_solver/` 中的两轮 metrics、analysis、checks 和视频 manifest。下载文件已按 DSW manifest 的 SHA256 校验，当前求解器代码与通过检查的版本一致。
- DSW 注册根：`/mnt/workspace/wensheng/revo3-retargeting-lab`。完整输入、每帧 q_goal/q_mapped/q_target/q_actual、接触表和代码快照保存在上述 run ID 的目录中。
- 使用注册 Python 执行 `run_shared_evaluation.py --config repos/Revo-Retargeting/experiments/revo3_lab/configs/shared_batch_01.json`。所有参数组在同一六输入集合上运行；`analyze_shared_evaluation.py --run-id <ID>` 重建实际几何。

## 视频

三个全景视频均为 **输入骨架 / legacy 实际输出 / shared 实际输出**；全部使用同一 shared 配置，近景只是换相机，不改变轨迹。真实 MANUS 来源和每帧源时间写在画面中。下列视频是阶段性结果，也展示尚未解决的偏差。

[侧摆同源对照](/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab/shared_solver/videos/side/side_finger_agility_with_input.mp4)

[屈伸同源对照](/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab/shared_solver/videos/flex/flex_finger_agility_with_input.mp4)

[对指同源全景](/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab/shared_solver/videos/pinch/pinch_finger_agility_with_input.mp4)

[对指侧面近景](/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab/shared_solver/videos/pinch_side_close/pinch_finger_agility_close_with_input.mp4)
