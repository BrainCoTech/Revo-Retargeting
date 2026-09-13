# MediaPipe → Revo3 本地输入

已在当前 Mac 的项目 `.venv` 安装 Python 3.11 运行依赖。直接在仓库根目录运行：

```bash
bash tools/mediapipe_revo3/run.sh --camera 0 --hand Right --record-video --session p01_day01_free_motion
```

窗口左侧为输入图像及 MediaPipe 原始 21 点，右侧为官方 Revo3 正面／侧面运动学骨架和对应的掌坐标输入。按 **q / Esc** 退出。默认选择 Right；如果要用左手驱动右机器人，改成 `--hand Left`。左右手分类分数不等同于关键点置信度。第一次请正对镜头张开手、再屈伸食指，确认标签与屈伸方向；输入视频本身若已镜像，加 `--input-mirrored`，程序先取消镜像再推理。不会自动猜测或切换镜像。

摄像头打不开：在 macOS「系统设置 → 隐私与安全性 → 相机」允许启动它的终端／Codex 使用相机，关闭占用相机的软件，也可试 `--camera 1`。此版本通过 OpenCV 窗口显示，不需要 `mjpython` 或 ROS。MediaPipe 在 macOS 上即使选 CPU 也可能初始化 OpenGL；若在受限沙箱中遇到 `NSOpenGLPixelFormat`，请从正常本机终端启动。

## 环境与文件位置

只有仓库根目录 `.venv` 一个本地环境；启动脚本始终使用绝对定位到它的 Python，不依赖当前 shell 的 conda 激活状态，不在 base 装包。没有改动根目录通用 `requirements.txt` 或旧 DSW 实验环境。

在新机器首次安装（需 Python 3.11，已验证 macOS arm64）：

```bash
PYTHON_BIN="$HOME/.local/bin/python3.11" bash tools/mediapipe_revo3/setup.sh
bash tools/mediapipe_revo3/run.sh --check
```

也可让 `PYTHON_BIN` 指向已有 Python 3.11。安装脚本不会覆盖错误版本的已有 `.venv`，会提示失败；不要删除现有环境来绕过检查。`requirements.in` 列直接依赖，`requirements.lock` 固定本次实际安装的全部传递依赖。其他 OS/架构的 wheel 可用性尚未逐一验证。仅安装 `opencv-contrib-python`，避免多个包争用 `cv2`。

模型、缓存、录制与报告全部进入已 gitignore 的 `artifacts/mediapipe_revo3/`。模型固定版本并校验 SHA-256；Revo3 固定官方提交 `f332a6f0dc944e26b82976b637074b03f7ee8a2c`。程序仅使用其关节、坐标变换、惯量和限位，移除网格／碰撞／驱动器以生成轻量运动学模型；无需下载几十个网格。

## 视频、回放、压力对照

```bash
bash tools/mediapipe_revo3/run.sh --video /absolute/path/hand.mp4 --hand Right --session p01_day02_occlusion
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/RUN_ID/frames.jsonl --no-display
bash tools/mediapipe_revo3/run.sh --replay artifacts/mediapipe_revo3/runs/RUN_ID/frames.jsonl --noise-mm 2 --dropout 0.1 --seed 17 --no-display
```

回放不重复运行检测器，使用录制的原始世界关键点和源时间戳。请保持 `--hand` 与采集一致；重新检测用 `--video raw.avi`。对已镜像源生成的 raw.avi 重新检测时保留 `--input-mirrored`。逐帧回放显示输入骨架；不会捏造原视频图像。噪声版本另建 run、保留原观察与真正送入 mapper 的点，并标成 `perturbed_mediapipe_replay`。

每次运行独立目录，保存：

- `manifest.json` 与 `code/`：输入来源／hash、参数、模型／代码 hash、环境包版本、关节名及限位、完成或失败状态，以及运行时源码快照。
- `frames.jsonl`：每个源帧的时间、全部检测到的手、原始归一化／世界 21 点、所选手、mapper 实际输入、掌坐标、`q_command_rad`、官方 FK、失败状态和处理耗时。无手帧同样保存，不补零伪造观察。
- `summary.json`：跟踪帧比例、各类状态次数、处理耗时 P50/P95/P99。
- `raw.avi`：仅指定 `--record-video` 时保存解码后的源图像（MJPEG，无音频）。帧号对应 JSONL；AVI 使用固定播放 FPS，真实采集时间以 JSONL 为准，不能将 AVI 的时长当作采集真值。

摄像头串行读取后同步 VIDEO 模式推理，时间戳单调；原始摄像头驱动缓冲时延没有计入处理耗时。文件输入优先采用解码展示时间，不可用时用 FPS 回退。最多处理 N 帧可加 `--max-frames N`；`--no-display` 可用于离线批处理。

## 映射范围

此版本是独立的 **21 点向量 IK 基线**，方便真实输入采集和反馈：掌坐标归一化、固定机器人指根并按输入骨方向匹配机器人骨长、五指中间关节和指尖目标、拇指相对其他四指的向量目标，使用官方 21 轴 FK 和解析 Jacobian。鲁棒最小二乘 + 时间正则，命令低通时间常数 60 ms、速度上限 3 rad/s、单次 limiter dt 最大 100 ms。检测缺失／分类低分／同侧手歧义或几何无效时，先保持至距上次有效帧 300 ms，再平滑回中立。

MediaPipe world landmarks 是单目估计的米制坐标，不是真实测量；深度、骨长、掌宽会漂移。接口保留原始 21 点，未合成 MANUS 缺失的 CMC。没有移植 DSW 的 pad 接触优化、揉搓控制或侧摆避碰；这是 `q_command` 的运动学预览，**没有 q_actual、碰撞保证、真实接触或硬件发送**。若要比较旧 solver，必须先定义带估计掩码的适配契约，不能把本版本结果混入旧 MANUS 回归表。

## 检查

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python tools/mediapipe_revo3/check.py
bash tools/mediapipe_revo3/run.sh --check
```

单元检查中的合成点只用于不变性、导数与边界测试，不计入真实数据表现。`--check` 用黑帧检查模型加载和实际推理，不占用摄像头。真人采集方案见 [验证方案](../../docs/revo3_mediapipe_validation.md)。

API 依据：[MediaPipe 官方 Python 指南](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)，[官方手部跟踪论文](https://arxiv.org/abs/2006.10214)。
