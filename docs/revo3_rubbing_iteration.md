# 拇指—食指搓动优化

> 已撤销第二轮作为重定向优化结果的推荐。机器人切向目标由独立正弦信号指定，骨架两指尖间距却在约 12–45 mm 变化，不能证明输入切向运动被映射到输出。下述第二轮指标仅保留为机器人激励实验记录。后续优化必须使用同一个公共算法，只调整参数及 loss，并以相同输入对比、回归其他动作；不再为搓指增加专用映射路径。

## 第二轮：指间关节参与的动态压力输入

本轮先核对首轮实际轨迹：拇指/食指 MCP 峰峰值为 2.884°/2.792°，拇指 PIP/DIP 为 0.158°/0.085°，食指 PIP/DIP 为 0.121°/0.061°。首轮确实基本仅由 MCP 完成。

新增 `rubbing_input.py`，在真实 MANUS 21.3 秒姿态的拓扑和骨长上，生成合成动态输入；四个指间铰链作周期屈伸，未参与的骨架点保持不变。`counter` 模式中拇指 PIP 屈曲时 DIP 伸展，减少末端总朝向变化。合成铰链边界是工程约束，不是人体解剖或皮肤接触标定。滑动目标仍为独立的任务空间正弦激励，不能称作真人搓指录制的重定向。

求解新增可选的输入相对姿态项，四个远端关节独立加权；既有对指调用不启用该项。接触位置和法向权重提高，同时 DIP 权重是 PIP 的四倍，避免冗余自由度将动作集中到 PIP/MCP。几何、碰撞排除、关节限位和原接触验收门槛未改变。新增每周期四个远端关节均至少 10° 的运动检查，并报告绝对角度 RMSE 和输入/实际轨迹相关性。

两秒周期候选 `20260913T140416Z_rubbing_9297ddf5`：±4 mm 目标，三周期有效接触率 100%，最长中断 0 ms；各周期真实表面滑移跨度 7.184、7.182、7.178 mm。拇指 PIP/DIP 总峰峰值为 18.897°/19.676°，食指为 13.978°/20.323°；逐周期四关节最小运动幅度 13.686°。最大穿透 0.745 mm、非目标穿透 0.275 mm，原门槛均通过。

四关节输入/实际相关性为 0.986、0.995、0.928、0.999；绝对角度 RMSE 仍为 22.46°、10.22°、21.34°、10.29°。这证明周期和方向基本保留，不证明绝对姿态高保真。法向力均值 0.568 N、峰值 0.748 N，仅为当前软件模型结果。`rubbing_task_accepted` 继续保持 false。

相同两秒周期下，时间步减半及摩擦 ±20% 的三次复验均为 100% 接触，四远端关节逐周期运动门槛全部通过，最大穿透分别为 0.658、0.748、0.741 mm。同动态骨架、同预压的零滑动对照仍有约 1 mm 周期滑移跨度，完整候选为约 7.18 mm；因此不能将屈伸本身引起的滑动计为独立滑动目标的全部贡献。

最终 11 项数值回归通过，包括骨长/未参与点不变、counter 模式实际输入角度变化与姿态目标一致、无接触零滑移及解析雅可比。五个不满足接触或关节运动要求的候选全部保留；另一次因会话中断触发 BrokenPipeError 的不完整实验单独列出。选定、对照及扰动 run 见 `configs/articulated_rubbing_selected_runs.json`，完整指标与视频见 `artifacts/revo3_lab/reports/rubbing_articulated_20260913.md`。

`counter` 的首帧拇指源 DIP 相对原始录制姿态已预屈 21°，随后周期性伸展；它从第一帧起就是合成输入。四个机器人姿态参考为显式输入相对校准，不能解释成原始真人绝对关节角。

运行命令（登记 DSW 内）：

```sh
LAB_ROOT=/mnt/workspace/wensheng/revo3-retargeting-lab
LAB_PY=$LAB_ROOT/envs/core-py312/bin/python
LAB_SCRIPTS=$LAB_ROOT/repos/Revo-Retargeting/experiments/revo3_lab/scripts
"$LAB_PY" "$LAB_SCRIPTS/check_rubbing.py"
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --articulated --curl-pattern counter --neutral-deg 15 30 40 12 --amplitude-mm 4 --period-s 2 --posture-weight 100 --dip-weight-multiplier 4 --preload-mm 3.5 --pad-scale-mm 1 --normal-scale 0.08
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --render-run 20260913T140416Z_rubbing_9297ddf5
```

每个新视频必须显示输入骨架。搓指视频使用保存的逐帧输入点、机器人实际状态和指腹特写；输入帧对应其控制周期末的实际状态，不再反射已变换的输入。连续对指视频也增加了骨架面板；没有真人输入的机器人过桥阶段显示明确标注的冻结端点参考。渲染前校验原始来源哈希，视频同时标明合成输入来源。该要求已写入 `experiments/revo3_lab/AGENTS.md`。

## 首轮记录

2026-09-13：已完成首轮优化。九项回归通过；十组动力学实验全部保留，其中四组候选未通过接触门槛。选定候选三周期连续接触，时间步减半和摩擦 ±20% 复验均通过接触及运动约束。完整真人搓指任务尚未验收。

主目标为拇指与食指指腹之间的连续接触和往复相对滑动。其余手指为次要目标；首轮固定为零指令并监测非目标碰撞，后续再接入次要姿态跟随。

`solve_pads(..., slide_m=0)` 新增沿食指指腹纵向切平面的偏移目标及解析雅可比，默认零偏移保持原对指目标。使用原始碰撞网格、关节限位及既有指腹区域。

`run_rubbing.py` 从已通过的 MANUS 21.3 秒姿态出发，执行 1.5 秒接近、1 秒稳定、2 秒保持和三周期正弦搓动。初始候选为 ±2 mm、周期 2 秒、法向预压 0.35 mm；选定默认预压为 1.8 mm。预压是软位置目标，并非实际网格穿透或已标定的硬件压入量。输入明确标记为冻结真人姿态叠加合成任务空间目标，不能视作真人搓指录制的重定向结果。

滑移以同一个有效接触点处两物体的雅可比乘实际关节速度得到，去除法向分量后，按接触力加权。逐物理步记录有符号纵向速度、切向速度模和横向分量；逐周期报告正负向积分与累计位移跨度。它不会将接触点位置漂移、无接触运动或共同刚体运动算作表面滑移。多接触点平均值是诊断量，不等价于所有接触点都达到同样滑移。

接触、最长中断、穿透、非目标碰撞、关节限制、指令速度和加速度使用既有门槛。记录实际速度与力；由于滑动幅度、法向力工作区间和独立真人输入尚未冻结，`rubbing_task_accepted` 保持 false。求解失败数和延迟同样保留。

`check_rubbing.py` 继承五项既有对指回归，增加非零滑动目标雅可比有限差分、共同刚体运动零滑移、已知相对平移速度、无接触运动零滑移四项检查。九项全部通过，run 为 `20260913T132946Z_check_rubbing_f45a06a7`。

用户已明确授权传输实验代码及运行。选定参数可在已登记 DSW 环境中复现：

```sh
LAB_ROOT=/mnt/workspace/wensheng/revo3-retargeting-lab
LAB_PY=$LAB_ROOT/envs/core-py312/bin/python
LAB_SCRIPTS=$LAB_ROOT/repos/Revo-Retargeting/experiments/revo3_lab/scripts
"$LAB_PY" "$LAB_SCRIPTS/check_rubbing.py"
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --amplitude-mm 0 --preload-mm 1.8
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --amplitude-mm 2 --preload-mm 1.8
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --amplitude-mm 2 --preload-mm 1.8 --timestep-scale 0.5
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --amplitude-mm 2 --preload-mm 1.8 --friction-scale 0.8
"$LAB_PY" "$LAB_SCRIPTS/run_rubbing.py" --amplitude-mm 2 --preload-mm 1.8 --friction-scale 1.2
```

## 结果及选择

初始 ±2 mm 候选接触率 66.3%、最长中断 638 ms；±4 mm 候选为 58.5%、760 ms。失效帧大多没有目标指腹接触，法向角度约 6–8°，主要问题为滑动中离面。保持 ±2 mm 幅度，将预压依次设为 0.7、1.2、1.8 mm，接触率依次为 73.7%、87.0%、100%。所有组使用相同接触及穿透验收门槛，没有修改几何、碰撞排除或关节限位。

选定 `20260913T133311Z_rubbing_66bf8e61`：两秒保持和六秒搓动接触率均为 100%，最长中断 0 ms；三周期的有符号表面滑移跨度为 3.262、3.258、3.254 mm。第一周期含运动启动，正向累计滑移 2.697 mm，负向 3.262 mm；后两周期正负方向均约 3.26 mm。完整回放最大穿透 0.370 mm，非目标最大穿透 0.188 mm；法向力均值 0.224 N、峰值 0.310 N，均为该软件模型的观测值。求解延迟 P95 为 5.42 ms，未验证在线端到端时延。

相同 1.8 mm 预压的零滑动对照，六秒切向累计路程仅 0.041 mm；选定候选为 19.191 mm。该对照用于区分保持时数值漂移与主动往复运动。横向累计路程为 2.064 mm，仍存在方向偏差。

时间步减半及摩擦乘以 0.8、1.2 时，接触率为 100%、100%、99.8%，最长中断分别为 0、0、2 ms。各周期滑移跨度约 3.19–3.33 mm，最大穿透均不超过 0.374 mm。三组独立数值扰动均通过既有约束。

选定及失败 run 清单在 `experiments/revo3_lab/configs/rubbing_selected_runs.json`；逐组指标和视频在 `artifacts/revo3_lab/reports/rubbing_iteration_20260913.md`。视频使用 `q_actual`，不会把 IK 目标当成执行结果。每组均保留 manifest、代码快照、目标/实际轨迹、逐物理步接触表及求解记录。冻结代码位于各 run 的 `code/scripts/`，对应参数位于 `config.json`。

下一步需要接入真人连续搓指输入，验证方向/幅度跟随和接触进入/释放，再接入其他手指的次要姿态跟随。本轮保留 `rubbing_task_accepted=false`，不输出合格教师标签。
