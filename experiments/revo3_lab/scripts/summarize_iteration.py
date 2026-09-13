"""Archive an iteration comparison and reproducible worst-case replay indexes."""
import argparse
import json
from pathlib import Path
from lab_common import checked, configure, digest, write_json
configure()
import numpy as np
import pyarrow.parquet as pq
from revo3_model import Hand


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-ids',nargs='+',required=True);args=p.parse_args()
    batches=[]; failures=[]
    for rid in args.run_ids:
        folder=checked(Path('runs')/rid)
        manifest=json.loads((folder/'manifest.json').read_text())
        if manifest['status']!='success':raise RuntimeError('Cannot summarize incomplete run')
        evaluation_hand=Hand(manifest['model']['path'])
        metrics=json.loads((folder/'metrics.json').read_text())
        selection=json.loads((folder/'selection.json').read_text())
        for m in metrics:
            traj=np.load(folder/m['candidate']/m['sequence']/'trajectories.npz',allow_pickle=False)
            m['latency_p99_ms']=float(np.quantile(traj['latency_s'],.99)*1000)
            m['latency_max_ms']=float(traj['latency_s'].max()*1000)
            m['latency_over_30hz_fraction']=float(np.mean(traj['latency_s']>1/30))
            aligned_tips=np.stack([evaluation_hand.fk(q)[0] for q in traj['q_actual']])
            m['actual_tip_error_mean_mm']=float(np.linalg.norm(aligned_tips-traj['target_tips_m'],axis=2).mean()*1000)
            m['actual_tip_sample_alignment']='recomputed from saved post-step q_actual; original mj_step site cache can lag by 2 ms'
            # Use time-separated maxima without removing any frames from metrics.
            err=np.linalg.norm(aligned_tips-traj['target_tips_m'],axis=2).mean(axis=1)*1000
            table=pq.read_table(folder/m['candidate']/m['sequence']/'contacts.parquet').to_pydict()
            for reason,values in [('actual_tip_error_mm',err),('penetration_mm',np.array(table['max_penetration_m'])*1000)]:
                chosen=[]
                for index in np.argsort(values)[::-1]:
                    if all(abs(int(index)-j)>=60 for j in chosen):chosen.append(int(index))
                    if len(chosen)>=3:break
                for i in chosen:
                    failures.append({'run_id':rid,'candidate':m['candidate'],'sequence':m['sequence'],
                                     'criterion':reason,'frame':i,'time_s':float(traj['timestamps'][i]),'value':float(values[i]),
                                     'replay_archive':str(folder/m['candidate']/m['sequence']/'trajectories.npz')})
        batches.append({'run_id':rid,'model_sha256':manifest['model']['sha256'],'metrics':metrics,'selection':selection})
    final=batches[-1]; val=[m for m in final['metrics'] if m['split']=='validation']
    lines=['# Vector 首轮迭代（2026-09-13）','',
           '已完成短序列计时、三批完整软件回放。Revo3 硬件能力直接采用用户确认结论，未进行能力搜索或硬件测试。','',
           '第三批统一使用修正固定掌座直属关节碰撞规则的 v2 模型；下表只在这一相同模型、输入、限位、PD 和滤波条件下比较。','',
           '| 验证集候选 | 指尖 mm ↓ | 指间向量 mm ↓ | 实际指尖 mm ↓ | 跟踪 RMSE rad ↓ | 最大穿透 mm ↓ | P95 / P99 ms ↓ |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for m in val:
        lines.append(f"| {m['candidate']} | {m['tip_error_mean_mm']:.2f} | {m['pair_error_mean_mm']:.2f} | {m['actual_tip_error_mean_mm']:.2f} | {m['tracking_rmse_rad']:.4f} | {m['max_penetration_mm']:.2f} | {m['latency_p95_ms']:.2f} / {m['latency_p99_ms']:.2f} |")
    lines+=['','## 本轮做了什么','',
            '- 两段官方 FBX 以固定 ufbx v0.17.1 完整求值：31.0 s / 1861 帧，32.35 s / 1942 帧；单位依据 FBX 元数据，骨长稳定，保存原始变换与关键时刻。',
            '- 新建 21 轴混合参考基线（拇指 IK、其余指角度）及 vector 实现；比较指尖、指间、时间连续性和碰撞惩罚。它不是用户既有 Revo3 算法。',
            '- 采用 finger_agility 调参、hand_mobility 验证的固定文件划分。公开样例侧别未标明；掌法向反射由调参样例弯指方向确定并显式固定，未宣称识别出人员或手侧。',
            '- FK/碰撞分离雅可比、刚体变换不变性、时间/骨长、输出边界及模型版本一致性共 7 项数值回归检查（含 step 后坐标时间对齐）。',
            '- 发现固定掌座装配接触后中止 v1 参数搜索；第三批只比较共同使用 v2 的基线与候选。v2 仅排除固定掌座与 5 个直属关节的碰撞，不放宽关节或力矩限位；保留非相邻、指间碰撞。',
            '',
            'MuJoCo 的固定根节点属于 world 焊接组，默认父子过滤的例外会产生这类装配碰撞；修正依据见 [MuJoCo 官方说明](https://mujoco.readthedocs.io/en/3.3.5/computation/#collision-detection)。',
            '', '## 实验索引','', '| 批次 | Run | 模型 SHA256 前缀 | 墙钟秒 | 调参集几何优选 |','|---|---|---|---:|---|']
    for i,b in enumerate(batches,1):
        lines.append(f"| {i} | {b['run_id']} | {b['model_sha256'][:12]} | {b['selection']['wall_s']:.1f} | {b['selection']['best_geometry_candidate']} |")
    lines+=['','## 限制与下一轮','',
            '表中的实际指尖误差和失败索引统一从已保存的 q_actual 重新计算 FK，修正旧 mj_step site 缓存最多 2 ms 的时间偏移；原始运行文件保持不变。', '',
            '本轮达到三批上限后停止。几何优选仅使用调参集的「平均指尖误差 + 平均指间向量误差」；这不是接触任务通过判定。P95 延迟不代表每帧实时保证，完整 P99、最大值及 30 Hz 超时比例见 JSON。',
            '',
            '当前仍有穿透、非目标接触和跟踪偏差。collision 惩罚只在姿态求解阶段起作用，滤波后轨迹及动力学执行仍可能发生瞬态碰撞；速度已限幅，加速度尚无硬约束。',
            '',
            '公开样例没有指腹区域、接触阶段或切向滑移真值。官方 tip_Link 是几何目标标记，distal collision 接触和 contact_point_slip_sum 只是诊断；后者对多接触点求和，不能作为任务滑移幅度。其他手指接触可能属于原动作意图，不能凭本轮固定拇食指目标判断全部为错误。',
            '',
            '独立最终测试集仍为空，OpenGraph 小样本接入及手侧/字段语义核验未完成；P1 仅完成官方 FBX 接入部分。下一轮应先补接触标注与模型指腹区域定义，再对滤波后轨迹加入加速度/碰撞约束，最后用固定的独立对指/搓指序列验收。',
            '',
            '不导出合格教师标签，不开始学习式训练，不宣称泛化或搓指成功。失败片段索引保留每候选每输入的 3 个相隔至少 2 秒的误差峰和穿透峰，所有帧均参与统计。','',
            '## 复现','',
            '云端根：/mnt/workspace/wensheng/revo3-retargeting-lab。统一入口 experiments/revo3_lab/scripts/lab.py；使用唯一 envs/core-py312 解释器。',
            '每个 run 保存源代码副本、配置、manifest、锁文件 hash、q_target/q_actual、contacts.parquet、指标与日志；末批实际状态时间戳另保存在 actual_timestamps。', '']
    report=checked('reports/vector_iteration_20260913.md');report.write_text('\n'.join(lines))
    result={'batches':batches,'failure_index_count':len(failures),'stop_reason':'three_batch_initial_iteration_budget',
            'hardware_capability':'confirmed_by_user','hardware_capability_test':'skipped_by_user',
            'task_acceptance':'not_evaluated','report_sha256':digest(report)}
    write_json('reports/vector_iteration_20260913.json',result)
    write_json('reports/vector_failures_20260913.json',failures)
    print(json.dumps({'report':str(report),'validation':[{k:v for k,v in m.items() if k in ['candidate','tip_error_mean_mm','pair_error_mean_mm','actual_tip_error_mean_mm','tracking_rmse_rad','max_penetration_mm','latency_p95_ms','latency_p99_ms','latency_max_ms','latency_over_30hz_fraction']} for m in val]},ensure_ascii=False))


if __name__=='__main__':
    main()
