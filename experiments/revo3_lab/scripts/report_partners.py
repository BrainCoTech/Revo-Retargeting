"""Audit immutable pair trajectories and assemble the recorded/synthetic report."""
import json
from pathlib import Path
from lab_common import checked, configure, run, write_json, digest
configure()
import numpy as np
import pyarrow.parquet as pq

LOCAL = '/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab'
FINGERS = {'middle': '中指', 'ring': '无名指', 'little': '小指'}


def read(run_id, name):
    return json.loads(checked(f'runs/{run_id}/{name}').read_text())


def audit_clip(run_id, synthetic):
    path = checked(f'runs/{run_id}/pad_vector/contacts.parquet')
    rows = [r for r in pq.read_table(path).to_pylist() if r['evaluated_clip']]
    dt = float(np.median(np.diff([r['time_s'] for r in rows])))
    missing = longest = 0
    for r in rows:
        missing = missing + 1 if r['source_close'] and not r['valid_pad_contact'] else 0
        longest = max(longest, missing)
    cfg = read(run_id, 'config.json')
    result = {'contacts_sha256': digest(path), 'physics_step_s': dt,
              'longest_break_during_source_close_s': longest * dt,
              'longest_break_passed': longest * dt <= cfg['acceptance']['longest_break_s_max'] + 1e-9,
              'final_source_gap_m': rows[-1]['source_gap_m'],
              'final_valid_contact': rows[-1]['valid_pad_contact']}
    if synthetic:
        # Both retained generators explicitly use .75 s approach, 2 s hold, .75 s release.
        hold = [r for r in rows if .75 <= r['source_time_s'] < 2.75]
        result.update(hold_window_s=[.75, 2.75], hold_duration_s=len(hold) * dt,
                      hold_contact_fraction=float(np.mean([r['valid_pad_contact'] for r in hold])),
                      released_at_end=rows[-1]['source_gap_m'] >= .020 and not rows[-1]['valid_pad_contact'])
    return result


def main():
    selected = json.loads((Path(__file__).resolve().parents[1] / 'configs/partner_selected_runs.json').read_text())
    with run('partner_report', selected) as (out, mf):
        evidence = {}
        for finger, name in FINGERS.items():
            case = selected[finger]
            clips = []
            for rid in case['clips']:
                metrics = read(rid, 'metrics.json')
                audit = audit_clip(rid, case['source_kind'] == 'synthetic_ik')
                assert metrics['pad_vector']['short_clip_task_passed'] and audit['longest_break_passed']
                if case['source_kind'] == 'synthetic_ik':
                    assert abs(audit['hold_duration_s'] - 2.) < 1e-8
                    assert audit['hold_contact_fraction'] == 1. and audit['released_at_end']
                clips.append({'run_id': rid, 'config': read(rid, 'config.json'), 'metrics': metrics, 'audit': audit})
            evidence[finger] = {'source_kind': case['source_kind'], 'clips': clips}
            if case.get('hold'):
                evidence[finger]['hold'] = read(case['hold'], 'metrics.json')
                assert evidence[finger]['hold']['pad_vector']['held_input_task_passed']
            if case.get('input'):
                evidence[finger]['synthesis'] = read(case['input'], 'synthesis.json')
            vid = read(case['video'], 'video.json')
            video_path = checked(f"runs/{case['video']}/raw_clip.mp4")
            assert digest(video_path) == vid['sha256']
            assert read(case['video'], 'manifest.json')['status'] == 'success'
            evidence[finger]['video'] = vid

        checks = {key: read(selected[key], 'checks.json')
                  for key in ['regression_checks', 'synthetic_checks', 'dynamics_checks']}
        for key in ['regression_checks', 'synthetic_checks']:
            assert checks[key]['failures'] == 0 and checks[key]['errors'] == 0
        assert all(v['held_input_task_passed'] for cases in checks['dynamics_checks'].values() for v in cases.values())

        candidates = []
        for folder in sorted(checked('runs').iterdir()):
            path = folder / 'manifest.json'
            if folder.name < '20260913T092600Z' or not path.exists() or folder == out:
                continue
            meta = json.loads(path.read_text())
            if meta['kind'] in {'pinch_hold', 'pinch_clip', 'partner_initialization', 'partner_anchor_tuning', 'synthetic_partner_input'}:
                candidates.append({'run_id': folder.name, 'kind': meta['kind'], 'status': meta['status'],
                                   'metrics': json.loads((folder / 'metrics.json').read_text()) if (folder / 'metrics.json').exists() else None})
        report = {'report_run': mf['run_id'], 'selected': selected, 'evidence': evidence, 'checks': checks,
                  'candidates': candidates, 'hardware_capability_test': 'skipped_by_user',
                  'scope': 'Three isolated opposition subtasks; no full-sequence, five-finger simultaneous, rubbing or teacher-data acceptance'}
        write_json(out / 'report.json', report)
        write_json('reports/partner_iteration_20260913.json', report)
        write_json('registry/partner_iteration.json', {'report_run': mf['run_id'], 'selected': selected})

        lines = ['# 中指、无名指、小指对指迭代 · 2026-09-13', '',
                 '三组在各自选定输入下通过本轮对指子任务。中指使用真实 MANUS 公开录制；无名指、小指按用户要求由人手骨架 IK 合成输入，再经 robot pad-vector IK 映射并执行 MuJoCo 动力学。没有进行 Revo3 硬件能力测试。', '',
                 '## 结果', '',
                 '| 组合 | 输入 | 有效接触覆盖率 | 最长未接触段 | 最大穿透 | 旧 vector 覆盖率 |',
                 '|---|---|---:|---:|---:|---:|']
        for finger, name in FINGERS.items():
            for clip in evidence[finger]['clips']:
                v = clip['metrics']['pad_vector']; old = clip['metrics']['old_vector']; cfg = clip['config']
                source = ('真实 finger_agility ' if finger == 'middle' else 'IK 合成 ') + '–'.join(f'{x:g}' for x in cfg['source_window_s']) + ' s'
                lines.append(f"| 拇指—{name} | {source} | {100*v['contact_fraction_during_source_close']:.1f}% | {1000*clip['audit']['longest_break_during_source_close_s']:.0f} ms | {1000*v['max_penetration_m']:.3f} mm | {100*old['contact_fraction_during_source_close']:.1f}% |")
        lines += ['', '覆盖率分母为源指尖距离 ≤15 mm 的标注窗口，按 2 ms 物理步统计；这只是几何接近标签，源录制没有接触力真值。输入按原时间播放；IK 合成片段为 0.75 s 闭合、2 s 保持、0.75 s 张开。', '',
                  '中指将真实 10.5 s 姿态冻结后的两秒保持为 100%；无名指、小指在合成片段内部的两秒保持均为 100%，末尾均已解除有效指腹接触。中指另测的两个短片段本身没有两秒保持段。', '',
                  '有效接触要求两侧原始末节 mesh 的接触点位于指定指腹区域、法向夹角 ≤35°、接触方向在 45° 法向锥内、间隙 ≤0.1 mm 且正法向力 ≥0.01 N。该力阈值是仿真数值下限，不是硬件工作力范围。覆盖率 ≥95%、最长中断 ≤100 ms、最大穿透/非目标穿透 ≤1 mm、指令速度 ≤4 rad/s、加速度 ≤20 rad/s²、实际关节越界 ≤0.02 rad，且无仿真告警；本轮没有放宽这些门槛。', '',
                  f"![三组实际动力学接触姿态]({LOCAL}/runs/{selected['summary_render']}/three_pairs.png)", '',
                  '## IK 和映射改动', '',
                  '- 源人手 IK 固定拇指 CMC 和目标指 MCP，优化两指 9 个旋转参数；逐根保留观测骨长，其他手指点位不变。合成关节边界是工程设定，未作人体关节标定或人手碰撞验证。合成输入不能替代真实 MANUS 泛化测试。',
                  '- 机器人求解器显式选择拇指及目标手指的 9 个自由度，加入指腹位置/法向、源方向、时间连续性与非目标碰撞项；三根不参与的机器人手指使用零指令，仍参与碰撞与越界评估。',
                  '- 中指使用目标前制动、张开距离映射系数 0.2、0.8 mm 目标预压量、4 帧因果闭合预测；预测只读当前和前一帧。两个片段采用相同设置，出现约 14/110 ms 提前接触，完整序列误触发尚未验证。',
                  '- 无名指增加非目标碰撞惩罚，使用 2 帧闭合预测和目标前制动。',
                  '- 小指在固定指腹区域内比较 9 组接触目标位置。选中拇指和小指横向各 +4 mm，保留纵向 20/18 mm；加入目标指姿态项与较强碰撞惩罚。目标位置/法向来自原 mesh 表面，未改变几何、扩大接触区域或新增碰撞排除。',
                  '- 小指初版源姿态仅张开到 19.2 mm，未跨过 20 mm 释放阈值；用源骨架 IK 求到 30 mm 开口后，再完整验证闭合—保持—释放。', '']
        for finger in ['ring', 'little']:
            syn = evidence[finger]['synthesis']
            lines.append(f"{FINGERS[finger]}合成输入的最大骨长误差：{syn['max_bone_length_error_m']:.3g} m；完整 IK 参数、边界、源帧与 hash 见 `runs/{selected[finger]['input']}/synthesis.json`。")
        lines += ['', '## 验证与 CPU 耗时', '',
                  f"{checks['regression_checks']['tests']} 项接触/雅可比/关节索引/制动回归检查、{checks['synthetic_checks']['tests']} 项合成骨架与来源检查均通过。三组各做步长减半、摩擦 -20%、摩擦 +20% 且收紧求解容差，共 9 个固定目标复验，全部两秒接触 100%、零中断。此处复验固定映射目标的数值稳定性，没有搜索硬件能力。", '',
                  '| 输入 | IK 均值 | P95 | 最大 |', '|---|---:|---:|---:|']
        for finger, name in FINGERS.items():
            for clip in evidence[finger]['clips']:
                lat = clip['metrics']['solver_latency_ms']
                lines.append(f"| {name} {'–'.join(str(x) for x in clip['config']['source_window_s'])} s | {lat['mean']:.2f} ms | {lat['p95']:.2f} ms | {lat['max']:.2f} ms |")
        lines += ['', '这些是登记 CPU 环境中单帧求解耗时；轨迹先离线求解，再以 30 Hz 输入、500 Hz 物理步回放。尾延迟仍有超出 33.3 ms 的帧，尚未验证在线端到端调度。', '',
                  '## 留存的退化与适用范围', '',
                  '初始通用指腹中心设置在无名指/小指上受指根非目标接触影响。只提高小指碰撞权重、多初值求解或纵向移动目标均不足以通过；横向目标位置才改善了实际动力学接触。中指短片段先后经历接触覆盖率约 44%、68%、85%、94.5% 的候选，最后采用上述共同参数。失败配置、轨迹、碰撞记录和代码快照全部留存于本轮归档；JSON 附全部候选清单。', '',
                  '本轮属于逐一对指的仿真映射优化。真实录制已用于调参和复看，不是独立最终测试；无名指、小指只有合成数据结果。完整五指同时运动、连续切换时互扰、完整序列误触发、真实无名指/小指录制、接触后切向运动及三周期搓指尚未验收，不导出合格教师标签。现有食指默认配置及入口保留，回归检查覆盖其接触路径。', '',
                  '## 回放与复现', '']
        for finger, name in FINGERS.items():
            lines.append(f"- [{name}：源骨架 / 旧 vector / 新 pad-vector 同步回放]({LOCAL}/runs/{selected[finger]['video']}/raw_clip.mp4)")
        lines += ['', '所有视频使用保存的 `q_actual`，经 ffmpeg 完整解码检查；画面重放本身不是重新执行动力学。确切运行 ID、最终配置、视频与检查 ID 见 `experiments/revo3_lab/configs/partner_selected_runs.json`。', '',
                  '在已登记 DSW 环境中，以下命令从对应 run 的冻结配置复现：', '', '```sh',
                  'LAB_ROOT=/mnt/workspace/wensheng/revo3-retargeting-lab',
                  'LAB_PY=$LAB_ROOT/envs/core-py312/bin/python',
                  'LAB_SCRIPTS=$LAB_ROOT/repos/Revo-Retargeting/experiments/revo3_lab/scripts']
        for finger in FINGERS:
            for rid in selected[finger]['clips']:
                lines.append(f'"$LAB_PY" "$LAB_SCRIPTS/run_pinch_clip.py" --config "$LAB_ROOT/runs/{rid}/config.json" --lead-s {selected[finger]["closing_lead_s"]}')
        lines += ['```', '', '模型、源输入、核心环境和代码由每个 run 的 manifest 记录；本地归档保留生成的合成输入、基线与完整结果。原始官方输入和模型仍位于登记根目录，版本/hash 沿用既有登记。', '']
        text = '\n'.join(lines)
        (out / 'report.md').write_text(text)
        checked('reports/partner_iteration_20260913.md').write_text(text)
        print(json.dumps({'report_run': mf['run_id'], 'candidates': len(candidates),
                          'audits': {f: [c['audit'] for c in e['clips']] for f, e in evidence.items()}}), flush=True)


if __name__ == '__main__':
    main()
