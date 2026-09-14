# Revo3 统一末端输入

新增链路将输入适配与机器人求解分开：MANUS 的 canonical 掌坐标骨架和 MediaPipe 原生 21 点分别转换为 `EndpointTargets`，然后调用同一 `Solver.solve_endpoints`。最终接口包含时间戳、五指目标位置、有效掩码，以及可选末节方向和方向有效掩码。目标位置单位为米，位于机器人 world 坐标系；五指顺序为拇指、食指、中指、无名指、小指。末端指机器人模型的 tip site；尚未转换为指腹接触坐标系。

MediaPipe 的 DIP→TIP 可以提供方向，不能直接提供完整末端旋转。首版基线关闭方向约束。适配器负责掌坐标、手侧、一次明确的反射和固定尺度；不会补造 CMC。MANUS `palm_positions_m` 必须已经由现有 ingest 完成 canonical 处理，配置中的 `palm_x_sign` 随后只应用一次。不能把未归一化的 world 骨架误作这个 NPZ 接口。

## 配置与运行

以下是注册 DSW `dsw-86hev0txafus51z1hp` 的实验入口；本地入口见文末“本地运行”。本地复用现有 `.venv`，不创建新环境。同步本次代码至注册仓库后，在已配置的 DSW shell 中运行：

```sh
LAB_ROOT=/mnt/workspace/wensheng/revo3-retargeting-lab
LAB_PY=$LAB_ROOT/envs/core-py312/bin/python
LAB_SCRIPTS=$LAB_ROOT/repos/Revo-Retargeting/experiments/revo3_lab/scripts
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_SCRIPTS/run_endpoint_evaluation.py" \
  --config repos/Revo-Retargeting/experiments/revo3_lab/configs/endpoint_baseline.json \
  --max-frames 30
```

删除 `--max-frames` 可处理所选完整区间。基线选用现有 `finger_agility.npz` 的 20.8–21.8 秒，保留原始源帧，不做 30 Hz 插值。`scale: 1.0` 是明确的未校准起点，不代表已测量的人手到机器人比例；评估正式效果前应通过固定校准片段确定比例，冻结配置后再比较算法。此入口不按每帧掌宽调整尺度。

MediaPipe 回放将配置的 `input` 替换为以下内容，其余 solver 配置不变。录制文件须先复制到注册根目录内；相对路径都以该根目录解析，路径越界会拒绝运行。这里的路径是需由使用者替换的示例。

```json
{
  "source": "mediapipe",
  "file": "data/mediapipe/my_recording/frames.jsonl",
  "hand_side": "Right",
  "input_field": "raw",
  "palm_x_sign": 1,
  "scale": 1.0,
  "include_directions": false
}
```

JSONL 直接使用现有采集工具的记录格式，不需要在 DSW 安装 MediaPipe。丢失检测的源帧仍进入求解器，用真实时间戳驱动失跟策略。时间戳必须严格递增；默认大于 `max_gap_s: 1.0` 的间隔会报错，不会默默压缩时间或跨缺失插值。首个选中源时间戳对应模拟时间 0，不额外预推进。每帧先用上一帧已经生效的命令推进到当前相对源时间，记录此时的实际状态，再求解并应用当前帧命令；最后一帧后不额外推进。固定步长模拟向上取整到覆盖当前相对源时间，保存实际模拟时间，因此存在小于一个模型步长的时间误差。求解器首帧滤波初始化步长仍为 1/30 秒，这不改变物理时间原点。没有额外的 CommandLimiter；平滑与速度限制由 solver 负责。

## 兼容性与验证范围

旧 `solve(points, targets)` 及其骨架姿态先验继续保留，现有 `run_shared_evaluation.py` 和旧配置仍用于 legacy 回归。新的 endpoint 入口使用机器人中立姿态正则，不再读取 MANUS CMC/MCP 推导关节先验。两种模式可能得到不同的关节姿态，不能将旧链路的对指接触成绩直接归属于新链路。`observation_mode: legacy` 在新 profile 中只表示不启用旧骨架侧摆观测器；`solve_endpoints` 本身不调用骨架观测。

每个不可变 run 保存配置、代码快照、输入文件 SHA256、逐帧原生骨架和来源记录、末端目标与有效掩码、求解诊断，以及同帧对齐的 `q_command`、`q_actual`、实际指尖与时间戳。末端拟合指标只计算有效目标；另报失跟、求解状态和动力学警告。MANUS 的规范化样本需包含 `timestamps`、`palm_positions_m`、与 ingest 完全一致的 `names`；`source_kind` 必须明确区分真实录制与合成输入。

这套接口能在没有真实 MediaPipe 录制的情况下完成结构和 MANUS 回归验证；真实视觉抖动、遮挡、尺度误差和失跟恢复仍需 MediaPipe 录制验证。本轮不提供渲染、指腹接触验收或硬件效果结论。五指位置或相对向量接近，并不等于真实接触成立。

## 2026-09-14 重构验证

本次在 DSW 独立快照目录 `repos/Revo-Retargeting-endpoint-296784b` 执行，未覆盖原实验仓库。上述运行命令中的 `repos/Revo-Retargeting` 可替换为此目录来复现本次检查：

```sh
LAB_SCRIPTS=$LAB_ROOT/repos/Revo-Retargeting-endpoint-296784b/experiments/revo3_lab/scripts
"$LAB_PY" "$LAB_SCRIPTS/check_endpoint_input.py"
"$LAB_PY" "$LAB_SCRIPTS/check_shared_solver.py"
"$LAB_PY" "$LAB_SCRIPTS/check_endpoint_pipeline.py"
```

- 末端输入与求解：10 项通过，run `20260914T065356Z_check_endpoint_input_b7240258`。
- 原共享 solver 回归：14 项通过，包括与冻结旧实现的数值比较，run `20260914T065213Z_check_shared_solver_575af70d`。
- 完整回放：MANUS 片段 61 帧，以及明确标注的合成 MediaPipe 格式 5 帧，run `20260914T065451Z_check_endpoint_pipeline_def9e4bb`。独立重积分保存命令，确认实际状态与源时间因果对齐；缺失、保持、回中立和恢复状态通过。两次动力学运行均无 MuJoCo 警告。

这是结构与回归验收。MANUS 未校准尺度的片段平均实际指尖目标误差约 18.89 mm（包括从中立开始的过渡），不是接触验收或调优结果。新入口的 pad 接触损失尚未迁移，基线 collision_weight 为 0；仅保持旧入口功能与测试，不承诺新入口具有旧接触任务表现。

用户指定真实录制为 `artifacts/mediapipe_revo3/runs/20260914T064114Z_c0956351`，共 1682 帧，Right、input_mirrored=false。对应配置是 `configs/endpoint_mediapipe_c0956351.json`，保留全部帧，包括准备和被重录取代的片段，不按动作统计成绩。`palm_x_sign: 1` 沿用现有 MediaPipe 右手标准化约定；固定 `scale: 1` 尚未做人体到机器人尺度校准。JSONL SHA256 为 `3cb9c9756d3a3154563b808bb65a42adbe6688852bcd7bd5cdfa8a21b625b132`。

用户明确授权后，关键点 JSONL 和 manifest/summary/timeline 元数据已上传至已注册 DSW，上传后 SHA256 与本地一致；未上传 raw.avi。全量回放 run 为 `20260914T065619Z_endpoint_evaluation_bc38d506`：1682 帧全部保留，1463 帧 tracking、8 帧 hold、211 帧 return_neutral，MuJoCo 警告为 0。有效目标上的实际指尖误差均值 10.91 mm、P95 32.97 mm；相对于适配后的视觉目标计算，不是人手真实几何误差或接触成功率。

输入录制旧 Mapper 的 tracking 为 1437 帧，而新 raw 适配接受了额外 26 帧：它没有套用旧 Mapper 的 landmark_jump 拒绝策略。因此 tracking 数量增加不能视为检测质量提升。固定尺度、位置目标和机器人中立正则尚未调优；真实输入链路已验证，映射保真度、碰撞和揉搓效果仍未验收。

首次回放的不可变 metrics 将 219 个未求解的失跟帧计入了 `solver_not_converged_frames`。报告逻辑随后修正为分别统计 attempted、skipped 和实际未收敛；旧 run 文件不覆写，正确收敛口径以逐帧诊断及独立审计为准。该修正不改变求解或保存的机器人轨迹。

独立审计 `20260914T065810Z_audit_endpoint_recording_4889ef2a` 通过：1463 次实际求解全部收敛；1682 帧原始时间戳全部一致；命令满足关节限位及 4 rad/s 限速；失跟保持和回中立符合配置。独立重积分保存命令与全部实际状态一致，最大模拟时间离散误差 1.998 ms。源录制时长约 166.94 秒，帧间隔中位数约 99.94 ms。评估和审计完整产物存于本地 `artifacts/revo3_lab/endpoint_refactor/`，DSW run 保留为权威记录。

## 本地运行

用户授权本地运行后，求解核心已解除 DSW 注册依赖。`hand_topology.py` 提供无运行时依赖的旧拓扑常量，`Hand` 可直接加载模型，DSW 注册仅由实验入口调用。`transform_official_model` 由 DSW 与本地共用，保留原 PD-v1/v2 转换；末节方向约束直接用 `Hand.distal_directions()`，不读取 DSW 指腹资源。

```bash
# 本地摄像头 → shared solver → MuJoCo实际状态预览
bash tools/mediapipe_revo3/run.sh --camera 0 --hand Right

# 本地完整关键点回放，显示原生输入骨架及 q_actual 实体模型
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/20260914T064114Z_c0956351/frames.jsonl --hand Right

# 全速无窗口回放
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/20260914T064114Z_c0956351/frames.jsonl --hand Right --no-display
```

默认 `--solver shared`，旧独立 IK 用 `--solver baseline`。本地 `.venv` 为 Python 3.11 / MuJoCo 3.3.5；官方 71 个 mesh 已缓存于 `artifacts/mediapipe_revo3/assets`，无需每次联网。结果仍写入独立 local run。相机／视频路径使用 MediaPipe，关键点回放不加载检测器；无窗口回放不初始化 OpenGL。

本地完整 run `20260914T071426Z_f21b362b` 共 1682 帧，与 DSW 同源结果比较：目标和模拟时间完全一致，最大命令差 `3.5805e-5 rad`（约 0.0021°），最大实际关节差 `3.1574e-5 rad`，最大实际指尖坐标差 `4.417e-7 m`。初始 `1e-5 rad` 严格命令阈值有一个元素超过；检查双方相同输入／配置与收敛诊断后，跨平台验收使用 `1e-4 rad` 关节、`1e-6 m` 指尖坐标容差，并保留实际差值，不声称位级一致。

新增本地 `check_shared.py` 五项检查与原 baseline 七项检查通过；末节方向 Jacobian 有限差分误差小于 `3e-10`。实体模型窗口三帧回放通过（`20260914T071628Z_505be228`）。macOS 沙箱不允许 CoreGraphics 时，需从正常本机终端启动窗口；无窗口全量回放已在项目环境验证。真实相机实时操作仍需用户在实际使用中确认检测效果，映射尺度和接触验收边界保持不变。
