# MANUS → Revo3：CPU DSW 执行与资源管理计划

日期：2026-09-13。状态：CPU DSW 与核心环境已复用；MANUS 指定姿态两秒对指保持及两个原始短片段的指腹接触子任务已通过。完整序列、独立测试和往复搓指尚未验收；硬件能力按用户确认跳过。

## 1. 已确认的资源与执行边界

- 用户已切换到自己重启的无 GPU DSW：`dsw-86hev0txafus51z1hp`，区域 `cn-wulanchabu`。本次实际 SSH 连接成功，用户为 `root`。
- 实测容器 **2 CPU、24 GiB 内存、无 GPU**；CPU cgroup quota/period 为 `200000/100000`，内存限制为 `25769803776` 字节。Ubuntu 24.04.2，系统 Python 3.12.3 位于 `/usr/bin/python3`。不是根据宿主机规格推算。
- `/mnt/workspace` 与 `/mnt/data_nas` 均挂载 `10.0.2.55:/` NFS；固定使用前者。两条路径是同一存储的入口，不是两份备份。NFS `df` 显示的是池容量，**不能当作本项目实际配额或无限可用容量**；云侧配额、保留策略未在控制台核验。
- `/mnt/workspace/wensheng/` 已存在且可写；`wensheng` 在这里是项目命名空间，不要求存在同名 Linux 用户。只在其下新建专用实验根，不复用或更改其余项目环境。
- 已有系统 Python、Git、curl；PATH 中无 uv、Conda、Blender，无 OSMesa 库。存在 Mesa EGL 库；首轮已用该环境成功离屏渲染 4 张动力学轨迹图，完整任务录像尚未验收。
- 历史 8×A100 的公网 SSH 入口本次未连通，现退出当前执行链路。不会新购实例或把其 GPU 视为当前可用资源。
- 首轮使用 MANUS，暂不接 HumanDex。目标仍是对指，以及指定指腹保持接触并相对滑动；CPU 资源不会降低接触验收标准。
- 本地旧样本已经校验迁移到 `artifacts/revo3_lab/`，原 `/tmp/revo3_manus_research/` 已移除，旧研究目录只留下入口 README。全部文件迁移记入 `registry/migrations.jsonl`。

已执行状态以本文件末尾与 `registry`/run manifest 为准；目录树中尚未使用的业务子目录仍属后续计划。

## 2. 唯一执行位置

### 云端

指定唯一项目根目录：

```text
/mnt/workspace/wensheng/revo3-retargeting-lab/
```

已确认父目录权限、NFS 挂载并通过实际写入/fsync/读回检查；云侧配额与停止/删除实例后的存储保留策略仍待独立核实，不能据此清理或停机。要求是支持正常 POSIX 文件操作的磁盘或 NAS；不把 Git、venv 或日志数据库放进 OSS FUSE。若该路径不可用，先修正权限/挂载或明确更新本计划，**不自动改到 `/root`、`/tmp`、其他人的目录或另一台实例重新安装**。

以 `LAB_ROOT` 表示该根目录，所有脚本从同一个 `registry/lab.json` 读取绝对路径，不依赖运行时的当前目录。

```text
LAB_ROOT/
├── registry/
│   ├── lab.json                   # 唯一根目录、主机、挂载与路径约定
│   ├── resources.json             # 实例/存储/环境/工具清单与状态
│   ├── environments.json          # 解释器、依赖锁、安装位置
│   ├── migrations.jsonl           # 旧路径 → 新路径、hash、验证状态
│   └── runs.jsonl                 # 实验索引，只追加
├── repos/
│   ├── Revo-Retargeting/           # 当前为脚本快照；manifest 记录本地 commit 与文件 hash
│   └── brainco-description/       # 官方模型，固定 commit
├── envs/
│   └── core-py312/                # 首期唯一 Python 环境
├── toolchains/
│   ├── uv/                       # 若系统无 uv，则放这里
│   ├── python/                   # 若需管理 Python，仅安装到这里
│   └── blender/                  # FBX 转换工具，必要时下载一个固定版本
├── locks/                        # 依赖锁、工具版本/hash、版本变更记录
├── data/
│   ├── raw/manus_official/v1/     # 原始文件及来源清单
│   ├── raw/opengraph/<revision>/
│   ├── quarantine/               # 重复、语义不明或损坏的数据索引
│   ├── normalized/<dataset_id>/  # 掌坐标系、米、弧度、时间戳
│   ├── annotations/              # 手侧、动作阶段、接触意图、校准
│   └── splits/                   # 固定的调参/验证/最终测试划分
├── models/revo3/<model_id>/       # 从官方模型构建的仿真配置及差异
├── runs/<run_id>/
│   ├── manifest.json             # 输入、代码、模型、环境、seed、命令
│   ├── config.yaml
│   ├── stdout.log
│   ├── metrics.json
│   ├── trajectories.npz
│   ├── contacts.parquet
│   ├── video.mp4
│   └── checkpoints/              # 仅学习实验有
├── reports/                      # 对比报告、失败片段索引
├── exports/                      # 验收通过的标签、网络、配置
├── cache/                        # pip/uv/HF/torch/绘图/渲染缓存
└── tmp/<run_id>/                  # 下载分片、临时转换和渲染文件
```

`<run_id>` 形式为 `20260913T080000Z_vector_0001`；包含 UTC 时间、实验类型和序号，不复用已有 ID。每次运行只新增 run 目录；不为每个候选参数新建环境、克隆仓库或复制完整数据。

首期只使用上述两个源码位置；当前先同步本地实验脚本的带 hash 快照，不将其称为完整 Git 检出。官方模型在 P2 获取时固定 commit。RevoLab/Revo-MjLab 的已核实配置作为实现参考，确需本地取源码时才在 `repos/` 登记；不默认安装其整套训练环境。

### 本机

```text
/Users/woltim/code/Revo-Retargeting/
├── docs/revo3_manus_execution_plan.md   # 本计划，纳入 Git
├── experiments/revo3_lab/              # 新算法、导入、仿真、评测、运维脚本
└── artifacts/revo3_lab/                # 本地唯一数据/报告/同步副本根目录
```

本机负责代码与结果审阅；首期不建立本机 Python/Conda 测试环境。实验代码及其测试、配置模板全部放在 `experiments/revo3_lab/`，云端在对应仓库目录安装同一份代码。原 `src/` 的 Revo2 生产链路不作为 Revo3 仿真试验场。

本地原始数据是已登记的下载留存，云端是校验过的执行副本；这是唯一允许的常规双份数据，二者用相同 SHA256 关联。运行结果以云端 run 为主，本地只同步报告、配置、指标、优选视频和模型，不自动镜像全部中间文件。

## 3. 创建什么软件环境

首期仅创建 `LAB_ROOT/envs/core-py312`，复用系统 Python 3.12.3 作为 venv 基础；不向系统解释器安装实验依赖。当前是 Ubuntu 24.04.2，以实际 wheel 安装、导入与数值 smoke 验证兼容性。

| 内容 | 计划 |
|---|---|
| Python | 系统 3.12.3 + 专用 venv `envs/core-py312`；当前不下载第二份 Python |
| 仿真 | 原生 MuJoCo；第一份候选锁使用 `mujoco==3.3.7`，通过模型加载、动力学、接触和录像检查后冻结 |
| 优化/数据 | 首份输入锁：NumPy 2.2.6、SciPy 1.15.3、PyYAML 6.0.2、PyArrow 20.0.0、h5py 3.13.0；含传递依赖/hash 的实际锁位于 `locks/` |
| 学习 | P4 才安装官方 PyTorch CPU wheel，并更新同一环境的锁；先训练小 MLP，首期不安装 CUDA 依赖 |
| FBX 导入 | 已改用固定 ufbx v0.17.1（commit `6ca5309972f03625e6990f3084ff4c1cc55a09b6`）在 `toolchains/ufbx-0.17.1/` 编译轻量求值器；读取完整动画变换/关键时刻，保留文件 SHA256；不再需要下载 Blender |
| 渲染 | 计算不创建窗口；录像独立进程使用 MuJoCo + OSMesa/CPU。若系统缺 OSMesa，先登记所需系统库及安装动作，数值评测仍可先行 |
| 环境管理 | 固定 uv 0.8.22 官方 wheel，校验 PyPI SHA256，仅提取 uv 到 `toolchains/uv/`；禁止 `sudo pip`、向系统或其他项目环境安装包 |

当前 DSW 直连 PyPI 的大文件下载发生停滞：uv wheel 已由本机从官方源下载并验证 SHA256 后同步；依赖使用阿里云 PyPI 镜像下载，安装前将锁内每个包的 hash 与 PyPI 官方版本元数据逐项核对，验证记录写入 run。首次下载中断同样保留 manifest，不另建环境。

表中核心仿真/数据依赖已在本机型实际通过导入与 smoke；FBX 导入已完成，学习和视频验收仍是后续步骤。安装失败或依赖冲突时，在同一环境内修复，保留旧锁文件、变更原因和安装日志；不生成 `test2`、`env-new` 等试错环境。环境必要的重建也复用同一个登记路径，先确认没有项目作业运行。

启动脚本按进程设置 `TMPDIR`、`XDG_CACHE_HOME`、`UV_CACHE_DIR`、`UV_PYTHON_INSTALL_DIR`、`PIP_CACHE_DIR`、`HF_HOME`、`TORCH_HOME`、`TORCH_EXTENSIONS_DIR`、`MPLCONFIGDIR`、`PYTHONPYCACHEPREFIX` 到根目录对应位置；设置 `PYTHONNOUSERSITE=1`。不改 `HOME` 或全局 shell 配置。Blender 用户配置亦定位到项目 cache。

工具的系统依赖若必须写入项目根以外，须在 `resources.json` 登记实际路径/包名、用途和安装日志；不把系统依赖伪装成“没有外部改动”。不使用默认 Docker 数据目录创建容器环境。

## 4. 2 CPU / 24 GiB 如何使用

- 首期 **1 个 worker**，`OMP_NUM_THREADS`、`OPENBLAS_NUM_THREADS`、`MKL_NUM_THREADS` 等均为 1，避免并发与线程相乘。机器只有 2 核配额，不默认开 8 个任务。
- 数据导入、FK、MuJoCo `mj_step`、接触评测与 vector 调参均在 CPU 上运行。先用 2–3 个候选测实际耗时，再安排后续批次。
- 仿真与录像分开执行，只渲染基线、最优候选和关键失败片段。当前无 OSMesa，数值验证先行；可尝试已有 Mesa EGL 软件渲染，失败则登记后再处理 CPU 渲染依赖。
- 小型个体网络先用同一环境的 PyTorch CPU 版本，前提是已有合格教师数据；是否够快由吞吐和延迟实测决定。
- 多人预训练、多 seed、大规模消融若超过 CPU 预算，输出耗时估算后再核实可用 GPU 资源。当前不预约、创建或迁移到任何 GPU 实例。
- 当前无 GPU，不安装 Isaac Sim/Isaac Lab、MuJoCo-Warp、MJX 或整套 mjlab 训练环境。BrainCo 示例可用作控制和任务配置参考。

## 5. 分阶段执行和退出条件

### P0：连接、资源登记与归档

1. 连接当前 CPU DSW，记录 cgroup CPU/内存配额、系统版本、NFS 挂载、权限、已有工具与渲染库。已完成；GPU 缺失是预期状态。
2. 核验指定云端根路径，填 `registry/lab.json` 和 `resources.json`。CPU/内存配额改变、已有作业繁忙或目录不满足要求时先报告差异；不改变其他项目。
3. 将上一轮本地资料整理到 `artifacts/revo3_lab/`：原始样本到 `data/raw/manus_official/v1/`，核验记录到 `reports/`，临时 HTML 到 `reports/source_snapshots/`。
4. 对源/目标 SHA256 校验后，清除本项目 `/tmp/revo3_manus_research/` 的重复副本；旧研究目录只保留指向新位置的 README。迁移过程写 `migrations.jsonl`。不清理他人目录或其他项目缓存。

交付：唯一位置、资源清单、迁移清单；没有通过这一步就不创建运行环境。

### P1：MANUS 输入可重放

1. 用两个已下载且内容不同的 FBX 建立骨架回放；两个重复 MREC 隔离，不按两个序列使用。
2. 核实手侧、父子骨架、单位、坐标系、实际关键帧时间、旋转顺序和绑定姿态。FBX 局部欧拉角不能直接视为手套人体工学角度。
3. 转成统一的 timestamp、关节/关键点名称、掌坐标系位置与可观测量 mask；原始字段与转换版本保留。
4. 加入 OpenGraphLabs 的一个小 episode，确认字段顺序、左右手、时间与单位，测得转换耗时/体积后再扩到 20 段；首期不下载 depth 和全部视频。
5. 按实际序列/人员/session（若有）划分调参与验证集；保留独立最终测试序列。官方两段短样例只用于接入检查，不能证明泛化。

交付：规范数据、骨架回放、指间距离曲线、数据质量表。未知的坐标/单位不靠猜测进入后续优化。

### P2：准备算法评测模型（硬件能力已确认，跳过能力测试）

2026-09-13 用户明确确认 Revo3 硬件本身符合要求。本轮不再搜索或验收机器人本体的对指/滑动能力，也不将该项作为 P3 前置门槛。下列模型、驱动和接触配置仅服务于 retargeting 软件评测；仿真中的失败归因于输入、映射、模型或控制实现，不据此推翻已确认的硬件能力。

1. 固定官方 `brainco-description` commit：`f332a6f0dc944e26b82976b637074b03f7ee8a2c`，保留原始仓库不改。
2. 在 `models/revo3/<model_id>/` 生成派生配置和差异：21 主动关节、驱动器、PD 控制、关节限制、接触几何、指腹区域、摩擦及速度/力矩限制。
3. 首轮算法聚焦右手拇指—食指，同时记录其余手指的映射误差；不执行本体接触能力搜索。
4. 必须用驱动和 `mj_step` 执行动力学；直接设置 qpos 的画面只算 FK 检查。机构不可达与控制器跟踪失败分别记录。
5. 当前官方模型与产品资料存在拇指 MCP 限位差异；首轮使用版本化模型限位，不为了通过评测静默放宽。

交付：固定版本模型、派生驱动配置和软件跟踪指标。硬件能力状态记为 `confirmed_by_user`，测试状态记为 `skipped_by_user`，不伪造实测结果。

### P3：vector 算法的自动迭代

建立 Revo3 21 轴的混合基线（拇指 IK，其余四指角度）和 vector 实现，在相同输入、模型、驱动和滤波协议下对比。当前本地 Revo2 的六轴输出不能直接作为 Revo3 基线；若无法取得原 Revo3 实现，明确标为新建参考基线。

vector 逐项增加：掌根—指尖关系、拇指—其他指腹相对关系、接触区域/法向、接触后切向目标、时间连续性和非目标碰撞约束。使用带滞回的接触状态，记录所有阈值。

先测 2–3 组候选的 CPU 耗时；每批最多 12 组候选、60 分钟（均为上限，不承诺吞吐），首轮最多 3 批。这是建议的实验预算，不是已承诺算力价格。参数改变用批量搜索，结构性失败由代理分析后修改代码。每轮保留输入、候选配置、最优配置、退化项与失败序列。

验证集连续两批无改善、发现模型漏洞或达到预算即停止该轮并输出归因；不能通过改阈值、删除失败样本或继续观看最终测试集来获得“达标”。固定最终测试集只用于阶段验收。

建议初始验收协议（算法任务验收前冻结，均不是已有实验结果；硬件能力测试已跳过）：

- 目标指腹保持接触 2 秒，随后完成至少 3 个往复周期。
- 接触阶段有效接触时间占比至少 95%，最长中断不超过 100 ms。
- 滑移幅度、方向与法向力范围：依据已有硬件确认资料和控制器设定，写入 config 后冻结；未提供具体数值时仅报告观测量，不宣称接触/搓指任务通过。
- 有效接触同时考虑指定指腹、法向接触力和间隙；避免把仅进入 collision margin 算成功。
- 滑移以接触两表面的相对切向速度积分衡量；单纯指尖运动、整手平移或纯滚动不算完成滑动。
- 对最大穿透、非目标碰撞、关节速度/加速度、执行误差和推理延迟设独立上限；不给严重违规用平均分抵消的机会。
- 对接触求解器容差、时间步长和合理摩擦扰动进行稳定性复验，确认结果不是一个特定数值设置造成。

交付：基线/改进算法对比、固定测试结果、可回放失败案例及最佳配置。

### P4：生成教师数据，再学习

1. 用已通过独立任务评测的离线轨迹优化器生成 `MANUS 输入 → Revo3 q_target`；保存 actual q、接触结果与标签质量，不能把实际反馈当作天然正确标签。
2. 先训练小 MLP；根据接触状态和动作历史的必要性，再比较带短时序输入的小网络。先用 CPU 运行并记录吞吐。
3. 与“基线+个人校准”比较任务质量、延迟和真实录制成本，而不只比较关节 MSE。
4. 对目标人的效果，需要目标人的真实 MANUS 录制。公开样例/合成数据用于启动、覆盖和预训练，不冒充个体适配验证。
5. sEMG-MANUS 在此阶段再引入：优先 15 人推荐 cohort；先解决角度标度、无逐帧时间戳与缺少骨架的问题。不能把它直接当成配对 Revo3 数据。
6. 预训练按留一人/留一场次测试，用达到同等接触任务质量所需的个人数据时长衡量收益。人员、seed 和消融实验先串行；大规模预训练在后续资源阶段重新评估。

交付：标签数据清单、网络权重、输入规范、复现实验命令、个体适配对比。

## 6. 可追溯性与清理规则

每项资源必须登记：ID、用途、创建时间、负责人/本任务、绝对路径或实例 ID、版本/hash、依赖关系、状态及清理方式。失败和中止的实验也写入 run 索引，不留无主 `nohup.out`。

每个 run 最少保存十类信息：身份；输入来源/hash；输入语义；模型版本；校准；算法代码及未提交 patch；环境/seed；仿真协议；评测/划分；输出与状态。开始运行前写 manifest，结束时补结果和校验，不只在成功时记录。

已实现统一入口 `experiments/revo3_lab/scripts/lab.py`，提供 `inventory`、`assets`、`ingest`、`check`、`run`、`report`。`setup`、`archive`、`clean --dry-run` 尚未整合；P0 登记脚本为 `scripts/initialize_lab.py`，核心环境建立脚本为 `scripts/bootstrap_cpu.py`。所有操作解析真实路径，检查归属与符号链接，禁止输出逃出根目录。

- CPU 小样例阶段预算：cache 4 GiB、tmp 2 GiB、run 4 GiB；文件系统报告空闲不足 5 GiB 或触达项目预算时停止新增作业。NFS 池剩余容量不是个人配额，下载/写入失败必须落盘记录，不能自动扩盘或改路径。扩大数据前先核实配额。
- 清理默认只列清单；实际清理仅针对已登记的本项目临时文件、可重建缓存。原始数据、指标、配置、代码快照和最终模型保留。
- 环境修复/重建前确认没有正在使用它的本项目进程；不凭 PID 文件终止无关进程。
- 归档完成必须验证文件数和 hash。NAS 为持续存储但不等于备份；代码、manifest、最佳结果和模型同步到本机项目归档根目录。大规模原始数据通过来源/hash可重新获取，个人录制则需单独登记备份位置。
- 不在 OSS 上运行环境；若采用既有 OSS 归档桶，只登记并使用一个固定 `revo3-retargeting-lab/` 前缀，不创建多个桶。
- 进程退出和 SSH 断开不等于 DSW 停止收费。当前是用户自己的 DSW，仍需确认没有其他作业、归档完整且取得停机授权，才通过实际云控制权限停止并确认状态；不自动停止或删除实例。

## 7. 第一阶段的完成定义

P0–P3 完成时应具备：一个登记的执行根、一个核心环境、MANUS 规范数据、Revo3 可控接触场景、21 轴基线、vector 候选、固定指标/测试集，以及能从 manifest 重跑的对比报告。达不到某个手指组合的要求时，给出可达性或控制/映射失败证据，不将“运行无报错”写成“搓指成功”。

当前 CPU DSW 连接、配额、根目录和 NFS 写入已验证。`envs/core-py312` 已建立，13 个依赖的锁定 hash 已逐项核对官方 PyPI，导入/数值优化/2000 步落球接触 smoke 均通过。成功 run 为 `20260913T081424Z-bootstrap-cpu-3482e2d7`，本地已归档 19 个运行与登记文件。首次官方工具下载中断也保留了独立 run。其后已完成官方 MANUS 转换、Revo3 21 轴模型驱动、基线与 vector 对比。指腹区域/法向与接触意图标注、独立最终测试、接触后切向目标和搓指任务验收仍未完成。

## 8. 已核验参考

- [BrainCo 官方 Revo3 资产](https://github.com/BrainCoTech/brainco-description)
- [MANUS 官方样例](https://www.manus-meta.com/products/metagloves-pro)
- [OpenGraphLabs MANUS 数据](https://huggingface.co/datasets/OpenGraphLabs-Research/manus-egocentric-sample)
- [sEMG-MANUS 作者数据](https://zenodo.org/records/19261324)
- [MuJoCo 3.3.7 官方 Python 包](https://pypi.org/project/mujoco/3.3.7/)
- [PyTorch 官方历史版本与 CUDA wheel](https://pytorch.org/get-started/previous-versions/)
- [CUDA 与驱动兼容规则](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)
- [Isaac Sim 官方要求：A100/H100 不受支持](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [DSW 目录/存储/计费与生命周期](https://www.alibabacloud.com/help/en/pai/create-and-manage-dsw-instances)
- [DSW 挂载 OSS/NAS 与使用限制](https://help.aliyun.com/zh/pai/read-and-write-dataset-data)


## 9. Vector 首轮迭代记录

- 用户确认 Revo3 硬件能力满足要求；所有新 run 均登记 `hardware_capability=confirmed_by_user` 与 `hardware_capability_test=skipped_by_user`。
- 唯一 CPU 环境未变；新增 ufbx C 求值器及官方右手 XML/完整引用 mesh 的固定提交快照，未创建新 Python 环境。
- 官方两段 FBX 完成完整动画求值和米制掌坐标转换（1861/1942 帧）；四指骨长在整段动画中稳定。源文件未标明手侧，掌法向适配依据调参输入显式固定，不把它当作侧别识别。
- `finger_agility` 用于调参，`hand_mobility` 用于验证；独立最终测试为空。OpenGraph 接入/语义核验仍待完成；本阶段属于公开样例软件接入与调参，不等于 P1–P3 全部验收。
- 短回放计时后执行三批，每批 4 个候选（含参考项）、2 段完整输入，30 Hz 输入、2 ms 动力学步长、1 worker；达到首轮三批上限后停止。
- v1 固定根节点的装配碰撞影响控制结果，第二批结束后停止 v1 搜索。第三批以单独的 `right_official_pd_v2` 同时重跑基线/候选，仅排除固定掌座与 5 个直属关节的碰撞；源模型、关节/力矩限位不变，差异与理由入档。
- 数值回归检查共 7 项通过。结果、延迟尾部、退化项、失败片段索引及下一轮事项见本地 `artifacts/revo3_lab/reports/vector_iteration_20260913.md` 和同名 JSON。
- 本轮不导出合格教师数据，不进入 P4；公开输入缺少接触与搓指真值，仍需处理滤波后轨迹的瞬态碰撞和加速度约束。

- EGL 渲染 run：`20260913T084249Z_render_vector_3e22c054` 已成功生成第三批保存的动力学轨迹抽帧；没有安装 OSMesa 或第二套图形环境。

## 10. 指腹对指跟进迭代

- 已用原始末节碰撞 STL 定义拇指/食指的指腹区域、表面中心和法向；接触必须发生在两侧指腹内且具有正法向力。模型运动学、几何及硬件限位保持原版本。
- `finger_agility` 21.3 s 真实姿态冻结后的两秒保持达到 100% 有效接触、零中断；全过程最大穿透 0.131 mm。公开源录制没有两秒保持段，因此该结果只称为冻结输入保持子任务。
- 原始 20.8–21.8 s 与 `hand_mobility` 25.8–26.7 s 两段在 1 倍速下分别达到 97.7%、99.0% 有效接触覆盖率，最大穿透分别为 0.430、0.152 mm；共同配置使用离散制动距离限制器、两帧因果闭合预测，停用产生退化的速度前馈。旧映射在相同限制器下均未通过。
- 五项数值/接触/边界制动检查通过；固定保持目标在时间步长减半及摩擦 ±20% 等三种数值扰动下仍通过。具体子任务门槛与未覆盖项见 `experiments/revo3_lab/configs/pinch_task_v1.json`。
- 保留所有候选及退化记录。选定 run 见 `experiments/revo3_lab/configs/pinch_selected_runs.json`，报告与同步回放见本地 `artifacts/revo3_lab/reports/pinch_iteration_20260913.md`。
- 其余三指在本轮对照中使用相同零指令；这些已查看的公开短片段不是独立最终测试。完整序列的误触发、接触释放、其他指对、完整五指运动和规定三周期往复搓指尚未验收，不导出合格教师标签。

## 11. 其余三指逐一对指与 IK 合成输入

- 用户明确要求中指、无名指、小指分别与拇指对指；没有无名指/小指 MANUS 录制，并授权用 IK 生成缺失输入。硬件能力测试继续跳过。
- 中指沿用真实 `finger_agility` 输入。10.0–11.0 s 与 8.1–9.1 s 分别达到 96.2%、98.8% 有效接触覆盖率，最大穿透 0.288、0.248 mm；两段共用参数。10.5 s 真实姿态冻结的两秒保持为 100%。
- 无名指/小指先在人手骨架上用 IK 求闭合—两秒保持—释放，再交给机器人指腹映射和动力学。骨长与不参与手指点位保持不变；关节约束是合成工程边界，未标定人体解剖学限制。明确标记 `synthetic_ik`，不能当作真实录制验收。
- 无名指合成片段接触覆盖率 98.1%，最大穿透 0.061 mm；小指 96.4%，最大穿透 0.088 mm。二者片段内两秒保持均 100%，末尾均释放。
- 小指最终采用固定指腹区域内横向 +4 mm 的拇指/小指表面目标、目标指姿态约束和非目标碰撞惩罚；保留原模型、指腹区域和验收门槛，未新增碰撞排除。对指优化仅涉及拇指与所选目标指，其余手指零指令但仍参与碰撞评估。
- 中指采用目标前制动、张开距离映射系数 0.2、0.8 mm 目标预压量与四帧因果预测；无名指/小指采用两帧预测。提前接触现象及端到端在线延迟限制写入报告，尚未验证完整序列误触发。
- 九项接触/索引/雅可比/制动回归与三项合成输入检查通过；三组各做三类固定目标数值扰动，共九例均两秒连续接触。结果仅支持本轮孤立指对子任务，不表示完整五指同时运动或搓指完成。
- 选定运行与配置见 `experiments/revo3_lab/configs/partner_selected_runs.json`；报告与同步视频见本地 `artifacts/revo3_lab/reports/partner_iteration_20260913.md`。失败候选、合成输入、源 hash、实际动力学轨迹与逐步接触记录全部入档。
