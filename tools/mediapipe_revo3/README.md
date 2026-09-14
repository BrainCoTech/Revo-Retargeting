# MediaPipe → Revo3 本地输入

已在当前 Mac 的项目 `.venv` 安装 Python 3.11 运行依赖。直接在仓库根目录运行：

```bash
bash tools/mediapipe_revo3/run.sh --camera 0 --hand Right --record-video --session p01_day01_free_motion
```

默认 `--solver shared`：摄像头／视频／关键点回放 → 统一五指末端目标 → 与 DSW 相同的 `Solver.solve_endpoints()` → 本地 MuJoCo PD 动力学 → 实体模型显示 `q_actual`。全程不需要连接 DSW。`--solver baseline` 保留此前独立的 21 点 IK 和命令姿态预览。

直接查看已有的 1682 帧录制：

```bash
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/20260914T064114Z_c0956351/frames.jsonl --hand Right
```

加 `--no-display` 可全速离线计算；有窗口时按源时间间隔回放。回放只读取关键点，不初始化 MediaPipe 检测器。新默认模型首次使用会下载固定官方版本的碰撞／显示网格并缓存；本机已完成缓存。默认使用 `endpoint_local.json` 的 `bone_scaled`：保留人手各骨段方向，换用机器人中立姿态的指根和骨长构造末端位置，并以 0.1 权重约束末节方向，避免手型尺寸差导致张手时折指。`--scale` 在此模式下缩放机器人骨长，默认 1.0；`--palm-x-sign` 默认 1。`--solver-config` 读取 profile 的 `solver` 与 `max_gap_s`，同时读取 `input.target_mode`，但不覆盖 CLI 的输入、手侧和尺度。`--target-mode` 可显式覆盖目标构造方式。复现旧的腕坐标距离映射（含旧 DSW 对比）请指定 `--solver-config experiments/revo3_lab/configs/endpoint_baseline.json`。

窗口左侧为输入图像及 MediaPipe 原始 21 点，右侧第一个视角正对掌心，第二个从拇指侧看向手掌；下方输入骨架的投影方向与对应模型视角一致。按 **q / Esc** 退出。默认选择 Right；如果要用左手驱动右机器人，改成 `--hand Left`。左右手分类分数不等同于关键点置信度。第一次请正对镜头张开手、再屈伸食指，确认标签与屈伸方向；输入视频本身若已镜像，加 `--input-mirrored`，程序先取消镜像再推理。不会自动猜测或切换镜像。

摄像头打不开：在 macOS「系统设置 → 隐私与安全性 → 相机」允许启动它的终端／Codex 使用相机，关闭占用相机的软件，也可试 `--camera 1`。此版本通过 OpenCV 窗口显示，不需要 `mjpython` 或 ROS。MediaPipe 在 macOS 上即使选 CPU 也可能初始化 OpenGL；若在受限沙箱中遇到 `NSOpenGLPixelFormat`，请从正常本机终端启动。

## 引导录制与动作时间轴

```bash
bash tools/mediapipe_revo3/record_guided.sh --camera 0 --hand Right --session p01_day01_guided
```

自动保存原视频，共 **10 段动作，动作计时合计 30 秒，准备时间不限**。
每段停在 PREPARE 状态，读完提示、准备好后，在预览窗口或启动它的终端按 **Enter（回车）** 才开始当前动作；计时结束后等待下一次回车。
窗口底部显示英文动作指令、PREPARE／RECORD 状态，只有 RECORD 阶段显示倒计时，终端同步显示中文说明。
macOS 可追加 `--guide-voice` 使用系统 `say` 朗读中文；语音取决于系统可用声音，切换阶段时会终止上一条提示，避免积压。
录制中按回车不会提前结束当前动作，也不会预约下一段。准备阶段仍保存预览视频与帧数据，标记为 `prepare`，不计入动作片段。

**重录上一条：** 等待下一条时，在预览窗口按 **R**，从下一帧重新录制刚完成的上一条，完整重新计时；重录结束后仍等待回车开始下一条。可多次重录，首条开始前按 R 无效。
录制过程中按 R 则重新录制当前条。如果使用终端控制，输入 `r` 后按回车。
重录不会清空整次会话：旧尝试仍保留在原视频和 JSONL 中，时间轴将它列入 `superseded_attempts`，不再作为当前动作的选中版本。

| 动作标签 | 时长 | 提示 |
|---|---:|---|
| `open_hold` | 2 秒 | 张手保持，掌心朝镜头 |
| `open_close` | 4 秒 | 握拳、张开两次 |
| `individual_flexion` | 8 秒 | 拇指到小指依次单指屈伸 |
| `spread_close` | 2 秒 | 四指张开并拢一次 |
| `opposition_index` | 2 秒 | 拇指对食指：接近、短暂轻触、分开 |
| `opposition_middle` | 2 秒 | 拇指对中指 |
| `opposition_ring` | 2 秒 | 拇指对无名指 |
| `opposition_little` | 2 秒 | 拇指对小指 |
| `contact_rubbing` | 3 秒 | 拇指与食指指腹轻触揉搓 |
| `gap_sliding` | 3 秒 | 两指不接触，保持约 1–2 cm 间距，小幅相对滑动模拟拧瓶盖 |

每组对指在两秒内做一次接近、短暂轻触、分开，不额外保持。
最后一项为空手动作，不要求拿瓶盖。间距是操作提示，不是检测器测得或保证的距离。
最后一条完成后等待确认：按回车结束保存，按 R 重录最后一条；随时按 **q / Esc** 可保存部分录制，Ctrl-C 标为 interrupted。
动作时间不会因为失跟暂停，所有失败帧仍保留；录制时只需按提示自然运动。

一个 run 目录保存连续 `raw.avi`、`frames.jsonl`，并新增 `timeline.json`：

- 每个 JSONL 帧的 `guide` 字段含 `action_id`、`phase`（prepare／record／review／done，review 为末条完成后等待确认）、`attempt`（动作尝试序号，准备段为 0）、阶段经过时间和剩余时间。
- 时间轴分别索引准备段与动作段，记录回车后首帧的动作起点、计划结束源时间、实际首末帧时间、`frame_start`（含）与 `frame_end_exclusive`（不含）、总帧数和 tracking 帧数。
- `complete`／`partial`／`not_started` 表示该段录制覆盖情况；尚未开始的阶段时间为空，没有帧的阶段不会伪造样本。
- 标签是**提示的动作意图**，不是自动验证的动作、接触或阶段真值。对指的接近／保持／释放没有额外自动细分。
- 后续评估先读 `timeline.json` 的 `segments`，选择 `phase == "record"` 的条目，再按其帧范围或 **`action_id` + `attempt`** 提取独立轨迹；不能仅按动作名筛选，否则会混入作废尝试。最新一次若未完成，保持 `partial`，不会静默回退到作废尝试。
- 保留原始 `timestamp_s`，不要按 AVI 的固定 FPS 代替真实时间。复播 JSONL 时保留动作标签，不重新计时或发出录制提示。旧录制不含 `attempt` 的，继续使用原时间轴的帧范围。

查看完整计划与检查时间轴逻辑（不打开摄像头、不做 IK）：

```bash
bash tools/mediapipe_revo3/record_guided.sh --guide-plan
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/mediapipe_revo3/check_guide.py
```

## 环境与文件位置

只有仓库根目录 `.venv` 一个本地环境；启动脚本始终使用绝对定位到它的 Python，不依赖当前 shell 的 conda 激活状态，不在 base 装包。没有改动根目录通用 `requirements.txt` 或旧 DSW 实验环境。

在新机器首次安装（需 Python 3.11，已验证 macOS arm64）：

```bash
PYTHON_BIN="$HOME/.local/bin/python3.11" bash tools/mediapipe_revo3/setup.sh
bash tools/mediapipe_revo3/run.sh --check
```

也可让 `PYTHON_BIN` 指向已有 Python 3.11。安装脚本不会覆盖错误版本的已有 `.venv`，会提示失败；不要删除现有环境来绕过检查。`requirements.in` 列直接依赖，`requirements.lock` 固定本次实际安装的全部传递依赖。其他 OS/架构的 wheel 可用性尚未逐一验证。仅安装 `opencv-contrib-python`，避免多个包争用 `cv2`。

模型、缓存、录制与报告全部进入已 gitignore 的 `artifacts/mediapipe_revo3/`。Revo3 固定官方提交 `f332a6f0dc944e26b82976b637074b03f7ee8a2c`，XML 校验固定 SHA-256，网格记录并验证缓存哈希。shared 使用完整官方 MJCF、碰撞几何和与 DSW `right_official_pd_v2` 共用的模型转换：21 轴、2 ms 步长、PD 参数和关节／力矩限位一致。模型及资源清单位于 `assets/local_dynamics/`。baseline 仍使用轻量运动学模型与单独 visual 模型，不进行动力学。

## 视频、回放、压力对照

```bash
bash tools/mediapipe_revo3/run.sh --video /absolute/path/hand.mp4 --hand Right --session p01_day02_occlusion
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/RUN_ID/frames.jsonl --no-display
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/RUN_ID/frames.jsonl --noise-mm 2 --dropout 0.1 --seed 17 --no-display
```

回放不重复运行检测器，使用录制的原始世界关键点和源时间戳。请保持 `--hand` 与采集一致；重新检测用 `--video raw.avi`。对已镜像源生成的 raw.avi 重新检测时保留 `--input-mirrored`。逐帧回放显示输入骨架；不会捏造原视频图像。噪声版本另建 run、保留原观察与真正送入 mapper 的点，并标成 `perturbed_mediapipe_replay`。

每次运行独立目录，保存：

- `manifest.json` 与 `code/`：输入来源／hash、参数、模型／代码 hash、环境包版本、关节名及限位、完成或失败状态，以及运行时源码快照。
- `frames.jsonl`：每个源帧的时间、全部检测到的手、原始归一化／世界 21 点、所选手、mapper 实际输入、掌坐标、`q_command_rad`、官方 FK、失败状态和处理耗时。shared 另存 `q_actual_rad`、`qvel_actual_rad_s`、末端目标／有效掩码、实际指尖、模拟时间与求解诊断。无手帧同样保存，不补零伪造观察。
- shared 的 `model.xml`、`model.json`、`solver_profile.json`：运行模型、包含网格哈希的来源收据和 profile 快照；`code/shared_core/` 保存实际共享算法代码。
- `summary.json`：跟踪帧比例、各类状态次数、处理耗时 P50/P95/P99。
- `raw.avi`：仅指定 `--record-video` 时保存解码后的源图像（MJPEG，无音频）。帧号对应 JSONL；AVI 使用固定播放 FPS，真实采集时间以 JSONL 为准，不能将 AVI 的时长当作采集真值。

摄像头串行读取后同步 VIDEO 模式推理，时间戳单调；原始摄像头驱动缓冲时延没有计入处理耗时。文件输入优先采用解码展示时间，不可用时用 FPS 回退。最多处理 N 帧可加 `--max-frames N`；`--no-display` 可用于离线批处理。

## 映射范围

shared 使用五指末端位置及指尖相对向量，不补造 CMC，不读取人手中间关节先验；采用机器人中立姿态正则、时间正则和同一共享优化器。默认低通时间常数 60 ms、速度上限 4 rad/s、单次 limiter dt 最大 100 ms；失跟先保持至上次有效帧后 300 ms，再回中立。源时间严格递增，超过 profile 的 `max_gap_s`（默认 1 秒）报错。首帧模拟时间为 0，每帧先用上一条命令推进，再求当前命令，因此显示／保存的实际状态没有提前使用未来输入。

`--solver baseline` 是此前独立的 **21 点向量 IK 基线**：掌坐标归一化、固定机器人指根并按输入骨方向匹配机器人骨长、五指中间关节和指尖目标、拇指相对其他四指的向量目标。它的速度上限为 3 rad/s，保留原来的 landmark_jump 拒绝策略，只输出运动学命令姿态。

MediaPipe world landmarks 是单目估计的米制坐标，不是真实测量；深度、骨长、掌宽会漂移。shared 有实际动力学状态，但默认 profile 不启用优化器碰撞损失，尚未迁移 pad 接触损失或验收对指／揉搓，不保证接触效果，也不发送硬件命令。baseline 没有 `q_actual`。不同算法的结果分别保存，不能混入旧 MANUS 接触回归表。

## 检查

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/mediapipe_revo3/check.py
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/mediapipe_revo3/check_shared.py
bash tools/mediapipe_revo3/run.sh --check
```

本地 shared 全量验证 run：`20260914T071426Z_f21b362b`，1682 帧全部完成。与 DSW 同源结果逐帧对比，末端目标和模拟时间一致，最大命令差 `3.5805e-5 rad`，最大实际指尖坐标差 `4.417e-7 m`；不是位级完全一致。比较脚本 `compare_dsw.py --local-run <目录> --reference-run <DSW产物目录>` 接收保存的两份结果，不连接云端。报告位于 `validation/shared_dsw_comparison_20260914T071426Z_f21b362b.json`。

单元检查中的合成点只用于不变性、导数与边界测试，不计入真实数据表现。`--check` 用黑帧检查模型加载和实际推理，不占用摄像头。真人采集方案见 [验证方案](../../docs/revo3_mediapipe_validation.md)。

API 依据：[MediaPipe 官方 Python 指南](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)，[官方手部跟踪论文](https://arxiv.org/abs/2006.10214)。
