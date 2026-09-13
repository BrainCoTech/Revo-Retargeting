"""Produce a compact, auditable report from completed vector run metrics."""
import argparse
import json
from lab_common import checked, write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);args=p.parse_args()
    folder=checked('runs')/args.run_id
    manifest=json.loads(checked(folder/'manifest.json').read_text())
    if manifest['status']!='success':
        raise RuntimeError('Report requires completed run; inspect failed manifest directly')
    metrics=json.loads((folder/'metrics.json').read_text())
    selected=json.loads((folder/'selection.json').read_text())
    lines=['# MANUS → Revo3 vector 迭代结果', '', f'Run: `{args.run_id}`。', '',
           'Revo3 硬件能力采用用户已确认结论，本轮未执行硬件能力测试。', '',
           '公开样例无指腹接触/滑动真值；以下是几何映射和受驱动 MuJoCo 软件回放结果，不代表搓指任务验收通过。', '',
           '| 输入 | 候选 | 指尖误差 mm | 指间向量误差 mm | 实际指尖误差 mm | 控制 RMSE rad | 最大穿透 mm | P95 延迟 ms |',
           '|---|---|---:|---:|---:|---:|---:|---:|']
    for m in metrics:
        lines.append(f"| {m['split']}/{m['sequence']} | {m['candidate']} | {m['tip_error_mean_mm']:.2f} | {m['pair_error_mean_mm']:.2f} | {m['actual_tip_error_mean_mm']:.2f} | {m['tracking_rmse_rad']:.3f} | {m['max_penetration_mm']:.2f} | {m['latency_p95_ms']:.2f} |")
    lines += ['',f"仅按调参集几何误差选择：`{selected['best_geometry_candidate']}`。本批耗时 {selected['wall_s']:.1f} 秒。",'',
              '模型：官方固定提交，21 轴、原关节/力矩限制；派生 PD 参数记录在 models/revo3/right_official_pd_v1/model.json。',
              '输入：固定文件级调参/验证划分；60 Hz 原始关键时刻求值，30 Hz 重采样；源掌坐标适配与手部尺度系数全部记录。',
              '控制：所有候选使用同一低通/速度限制，MuJoCo mj_step 运行，保存 q_target 与 q_actual；初始化设为首个目标姿态并进行 0.2 秒稳定。',
              '接触：统计官方 distal collision 几何的接触，不把它冒充已验证的指腹区域；非目标接触、穿透和跟踪误差独立报告。',
              '独立最终测试集为空；本轮结果不得导出为合格教师标签。', '',
              '每个候选/输入目录提供 metrics.json、trajectories.npz、contacts.parquet；run 中保留配置、源码副本、日志、manifest 和输出 SHA256。','']
    report=checked('reports')/(args.run_id+'.md');report.write_text('\n'.join(lines))
    print(report)


if __name__=='__main__':
    main()
