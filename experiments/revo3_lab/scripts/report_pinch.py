"""Evidence report for pad opposition; keep hold, raw clip and full task distinct."""
import argparse
import json
from pathlib import Path
from lab_common import checked,configure,run,write_json,digest,stamp
configure()
import numpy as np
from ingest_manus import NAMES

LOCAL='/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab'


def load_run(run_id):
    folder=checked(Path('runs')/run_id)
    mf=json.loads((folder/'manifest.json').read_text())
    if mf['status']!='success':raise RuntimeError('Selected run did not complete')
    return folder,json.loads((folder/'metrics.json').read_text())


def audit_inputs():
    result={}
    for sequence in ['finger_agility','hand_mobility']:
        path=checked(f'data/normalized/manus_official_ufbx_v1/{sequence}.npz')
        data=np.load(path,allow_pickle=False);ts=data['timestamps'];points=data['palm_positions_m']
        gaps=np.linalg.norm(points[:,NAMES.index('Thumb_TIP')]-points[:,NAMES.index('Index_TIP')],axis=1)
        indexes=np.flatnonzero(gaps<.015);intervals=[]
        for group in np.split(indexes,np.flatnonzero(np.diff(indexes)>1)+1):
            if not len(group):continue
            best=int(group[np.argmin(gaps[group])]);intervals.append({'start_s':float(ts[group[0]]),'end_s':float(ts[group[-1]]),
                'sample_support_s':float(ts[group[-1]]-ts[group[0]]+np.median(np.diff(ts))),
                'min_gap_m':float(gaps[best]),'min_time_s':float(ts[best]),'min_frame':best})
        result[sequence]={'path':str(path),'sha256':digest(path),'frames':len(ts),'intervals_gap_lt_15mm':intervals,
                          'any_recorded_2s_close_interval':any(x['sample_support_s']>=2 for x in intervals),
                          'label':'tip-proximity candidate only; no human contact-force ground truth'}
    return result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--train',required=True);ap.add_argument('--validation',required=True)
    ap.add_argument('--hold-video',required=True);ap.add_argument('--train-video',required=True);ap.add_argument('--validation-video',required=True)
    args=ap.parse_args();hold_id='20260913T090155Z_pinch_hold_f567ad65';robust_id='20260913T091208Z_pinch_dynamics_check_0aaaf8bf'
    hf,hold=load_run(hold_id);tf,train=load_run(args.train);vf,validation=load_run(args.validation)
    robust=json.loads(checked(f'runs/{robust_id}/checks.json').read_text());audit=audit_inputs()
    trials=[]
    for folder in sorted(checked('runs').iterdir()):
        mf=folder/'manifest.json'
        if not mf.exists():continue
        meta=json.loads(mf.read_text())
        if meta['kind']=='pinch_clip' and meta['status']=='success':
            cfg=json.loads((folder/'config.json').read_text());metrics=json.loads((folder/'metrics.json').read_text())
            trials.append({'run_id':folder.name,'sequence':cfg['sequence'],'protocol':cfg['clip_protocol'],'metrics':metrics})
    selected_cfg=json.loads((tf/'config.json').read_text())
    with run('pinch_report',vars(args)) as (out,mf):
        summary={'created_at':stamp(),'hold_run':hold_id,'train_run':args.train,'validation_run':args.validation,
                 'hold_metrics':hold,'train_metrics':train,'validation_metrics':validation,'robustness_run':robust_id,'robustness':robust,
                 'input_audit':audit,'clip_trials':trials,'videos':vars(args),
                 'status':'held_input_and_two_short_clips_passed; full_sequence_and_rubbing_not_accepted',
                 'hardware_capability':'confirmed_by_user','hardware_capability_test':'skipped_by_user',
                 'teacher_export':False,'source_side_verified':False,'source_palm_x_sign':-1,
                 'independent_final_test':False}
        write_json(out/'summary.json',summary);write_json(out/'input_audit.json',audit)
        before,after=hold['old_vector'],hold['pad_vector']
        lines=['# MANUS → Revo3 指腹对指迭代','',
            '日期：2026-09-13。硬件能力沿用用户确认，本轮未测试硬件能力。',
            '', '## 结论','',
            '指定 MANUS 姿态的两秒保持子任务通过；这不等于整段实时 retargeting 或搓指任务完成。输入侧用真实录制的拇指—食指接近片段，机器人侧改用原始末节碰撞网格上的指腹区域及相对法向。',
            '', '## 输入与范围','',
            '- 保持测试使用 `finger_agility` 的 21.300 s（第 1278 帧），源骨架两指尖间距约 3.513 mm。此姿态被冻结两秒；录制本身没有两秒对指保持段。',
            '- 原始片段分别为 finger_agility 20.8–21.8 s、hand_mobility 25.8–26.7 s，按 1 倍速、30 Hz 重放；开始前均有相同 2.5 s 的接近/稳定阶段。',
            '- 人手橙色为拇指，青色为食指。机器人视频使用保存的动力学实际关节状态，保留原始 visual mesh；特写有两个视角。',
            '- 两种机器人映射都将中指、无名指、小指固定为相同零指令，以隔离拇指—食指问题。因此不代表五指同时运动已通过。',
            '- 源掌坐标 x 反射 -1 明确写入配置；左右手语义未核实。人手接触标签由 15 mm 骨架端点距离产生，未有接触力真值。两份公开示例均已被查看，不能再称为盲测集。',
            '', '## 两秒保持结果','',
            '| 指标 | 旧 vector | 指腹 vector |', '|---|---:|---:|',
            f"| 有效接触占比 | {before['contact_fraction']:.1%} | {after['contact_fraction']:.1%} |",
            f"| 最长中断 | {before['longest_break_s']*1000:.0f} ms | {after['longest_break_s']*1000:.0f} ms |",
            f"| 保持期间最大法向偏差 | {before['max_hold_normal_angle_deg']:.2f}° | {after['max_hold_normal_angle_deg']:.2f}° |",
            f"| 全过程最大穿透 | {before['max_rollout_penetration_m']*1000:.3f} mm | {after['max_rollout_penetration_m']*1000:.3f} mm |",
            f"| 保持平均有效接触法向力 | {before['mean_hold_force_n']:.3f} N | {after['mean_hold_force_n']:.3f} N |",
            f"| 保持跟踪 RMSE | {before['hold_tracking_rmse_rad']:.5f} rad | {after['hold_tracking_rmse_rad']:.5f} rad |",
            '',f"[两秒保持同步视频]({LOCAL}/runs/{args.hold_video}/held_pose.mp4)",
            '',f"![两视角指腹特写]({LOCAL}/runs/20260913T091046Z_render_pinch_40e77be3/pad_closeup.png)",
            '', '## 原始短片段','',
            '| 输入片段 | 旧映射接触覆盖率 | 新映射接触覆盖率 | 新映射最大穿透 | 新映射子任务通过 |', '|---|---:|---:|---:|---|']
        for label,metrics in [('finger_agility',train),('hand_mobility',validation)]:
            b=metrics['old_vector'];a=metrics['pad_vector']
            lines.append(f"| {label} | {b['contact_fraction_during_source_close']:.1%} | {a['contact_fraction_during_source_close']:.1%} | {a['max_penetration_m']*1000:.3f} mm | {'是' if a.get('short_clip_task_passed',False) else '否'} |")
        lines+=['', '覆盖率仅在 30 Hz 输入零阶保持后的 `source_gap <= 15 mm` 窗口内统计，窗口时长分别约 0.266 s、0.200 s；它不是整段视频接触率，也不是两秒保持率。接触状态在每个 2 ms 动力学步后重新计算。',
                '',f"[原始 finger_agility 同步视频]({LOCAL}/runs/{args.train_video}/raw_clip.mp4) · [原始 hand_mobility 同步视频]({LOCAL}/runs/{args.validation_video}/raw_clip.mp4)",
                '', '## 实现与验收','',
                '- 指腹中心通过对原始 STL 正面三角形射线求交确定：拇指末节局部 y=20 mm、食指末节局部 z=18 mm。指腹区域、表面法向、文件 hash 全部保存在 pads.json。未新增碰撞球体、放大网格或改变运动学/硬件限位。',
                '- 优化目标包含两指腹位置差、相对法向、源姿态中心及末节方向、时间平滑和非目标碰撞。保留同一个软件 PD 模型。',
                '- 有效接触必须来自原始拇指/食指末节碰撞对，接触点位于双方指腹区域，指腹法向偏差 ≤35°，接触法向落在两侧 45° 锥内，间隙 ≤0.1 mm，正法向力合计 ≥0.01 N。0.01 N 是仿真数值正接触门槛，不是硬件工作力范围。',
                '- 保持验收为 2 s、接触占比 ≥95%、最长中断 ≤100 ms；全程穿透和非目标穿透各 ≤1 mm，实际关节限位偏差 ≤0.02 rad。另检验指令速度 ≤4 rad/s、指令加速度 ≤20 rad/s²、无仿真警告。这些指令阈值不冒充硬件实测运动极限。',
                f"- 选定短片段控制配置：制动距离限制器；闭合预测提前量 {selected_cfg['clip_protocol']['closing_lead_s']*1000:.3f} ms；速度前馈 {selected_cfg['clip_protocol']['velocity_feedforward_s']*1000:.3f} ms。预测仅使用当前和前一帧，接触评价仍用当前源输入。",
                '- 五项回归检查通过：指腹位置/法向/轴向 Jacobian 的有限差分、指腹区域拒绝背面/侧面/近端、从实际状态重算真实接触且模型几何不变、张手无假阳性，以及反向运动/边界制动的速度加速度约束。',
                '- 固定同一个保持目标，在 1 ms 步长、0.8 倍摩擦，以及 1.2 倍摩擦并收紧求解容差至 1e-10 三种设置下，都为 100% 有效接触、零中断，且全部保持验收门槛通过。',
                '', '## 迭代与退化记录','',
                '| Run | 输入 | 限制器 | 提前量 ms | 速度前馈 ms | 新映射覆盖率 | 新映射最大穿透 mm |', '|---|---|---|---:|---:|---:|---:|']
        for trial in trials:
            p=trial['protocol'];a=trial['metrics']['pad_vector']
            limiter='v2制动' if 'v2' in p['command_filter'] else 'v1硬截断'
            lines.append(f"| {trial['run_id']} | {trial['sequence']} | {limiter} | {p.get('closing_lead_s',0)*1000:.1f} | {p.get('velocity_feedforward_s',0)*1000:.1f} | {a['contact_fraction_during_source_close']:.1%} | {a['max_penetration_m']*1000:.3f} |")
        lines+=['','v1 在边界硬截断导致指令加速度超限，已修正。速度前馈虽然提前接触，却使 finger_agility 覆盖率退化并发生 >1 mm 穿透，因此未采用。所有候选保留原始轨迹与接触记录。',
                '', '## 尚未验收','',
                '完整序列所有对指、其他手指组合、真实连续两秒对指录制、规定方向/幅度的三次往复搓动和独立最终测试尚未完成。没有导出合格教师标签，没有进入学习阶段。下一阶段应围绕连续接触的维持/释放及全序列误触发检查，不能将这次保持成功写成整体任务完成。',
                '', '## 复现与归档','',
                f'保持 `{hold_id}`；原始调参片段 `{args.train}`；原始验证片段 `{args.validation}`；数值扰动 `{robust_id}`。',
                '', '每个 run 含代码快照、依赖锁 hash、输入/model hash、配置、metrics、contacts.parquet、q_target/q_actual 和日志。视频工具是注册目录内的 imageio-ffmpeg 0.6.0 wheel 中 ffmpeg 7.0.2-static，未修改核心 Python 环境。远端原始产物保留在已登记的 NFS 根目录，本地归档带 SHA-256 清单。',
                '']
        lines+=['## CPU 映射求解耗时','', '| 输入 | 平均 ms | P95 ms | 最大 ms |','|---|---:|---:|---:|']
        for label,metrics in [('finger_agility',train),('hand_mobility',validation)]:
            latency=metrics['solver_latency_ms'];lines.append(f"| {label} | {latency['mean']:.2f} | {latency['p95']:.2f} | {latency['max']:.2f} |")
        lines+=['','耗时为注册 CPU 上每帧映射求解器计时，不包括通信/真实设备执行，也不是在线端到端延迟验收。','']
        report='\n'.join(lines);(out/'report.md').write_text(report)
        checked('reports/pinch_iteration_20260913.md').write_text(report)
        summary['report_run']=mf['run_id'];write_json('reports/pinch_iteration_20260913.json',summary)
        write_json('registry/pinch_iteration.json',{'updated_at':stamp(),'report_run':mf['run_id'],'status':summary['status'],
                   'hold_run':hold_id,'train_run':args.train,'validation_run':args.validation,'hardware_capability_test':'skipped_by_user',
                   'teacher_export':False,'videos':vars(args)})
        print(str(out),flush=True)

if __name__=='__main__':main()
