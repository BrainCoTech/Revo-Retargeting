"""Build a provenance-aware catalog or print a frozen/current-code replay command.

Read-only against existing runs. Building creates a new immutable catalog run.
"""
import argparse
import json
from pathlib import Path
import shlex
from lab_common import checked, configure, run, write_json, digest, ROOT


def selected_cases():
    base = Path(__file__).resolve().parents[1]/'configs'
    ix = json.loads((base/'pinch_selected_runs.json').read_text())
    ps = json.loads((base/'partner_selected_runs.json').read_text())
    result = {'index_hold': ix['hold_run'], 'index_agility': ix['finger_agility_clip_run'],
              'index_mobility': ix['hand_mobility_clip_run']}
    for f in ['middle', 'ring', 'little']:
        for i,rid in enumerate(ps[f]['clips']): result[f+'_clip_'+str(i+1)] = rid
        if 'hold' in ps[f]: result[f+'_hold'] = ps[f]['hold']
    return result


def replay(rid, current_code=False):
    root = checked('runs/'+rid)
    meta = json.loads((root/'manifest.json').read_text())
    kind = meta['kind']
    if kind in ['side_swing','opposition_sequence']:
        scripts = checked('repos/Revo-Retargeting/experiments/revo3_lab/scripts') if current_code else root/'code/scripts'
        script = 'run_side_swing.py' if kind=='side_swing' else 'run_opposition_sequence.py'
        return 'PYTHONNOUSERSITE=1 '+shlex.join([str(ROOT/'envs/core-py312/bin/python'),str(scripts/script),*meta['command'][1:]])
    if kind not in ['pinch_clip', 'pinch_hold']:
        raise ValueError('Replay supports pair, side-swing and continuous-opposition runs')
    cfg = json.loads((root/'config.json').read_text())
    scripts = checked('repos/Revo-Retargeting/experiments/revo3_lab/scripts') if current_code else root/'code/scripts'
    # Registered venv Python is a symlink to the provisioned interpreter.
    script = scripts/('run_pinch_clip.py' if kind=='pinch_clip' else 'run_pinch.py')
    cmd = [str(ROOT/'envs/core-py312/bin/python'), str(script)]
    if not current_code:
        # Earliest clip scripts had no CLI/config override. Their code/config
        # snapshot plus the original argv is the reproducible interface.
        argv = list(meta['command'][1:])
        if '--config' in argv:
            argv[argv.index('--config')+1] = str(root/'config.json')
        elif '--config' in script.read_text():
            argv += ['--config', str(root/'config.json')]
        cmd += argv
    else:
        cmd += ['--config', str(root/'config.json')]
    if current_code and kind=='pinch_clip':
        p = cfg['clip_protocol']
        cmd += ['--lead-s', str(p.get('closing_lead_s',0.)), '--velocity-feedforward-s', str(p.get('velocity_feedforward_s',0.))]
    return 'PYTHONNOUSERSITE=1 '+shlex.join(cmd)


def build():
    selected = selected_cases()
    cases = []
    for folder in sorted(checked('runs').iterdir()):
        path = folder/'manifest.json'
        if not path.exists(): continue
        mf = json.loads(path.read_text())
        if mf['kind'] not in ['pinch_clip', 'pinch_hold', 'synthetic_partner_input', 'vector', 'opposition_sequence', 'side_swing']: continue
        cfg = json.loads((folder/'config.json').read_text()) if (folder/'config.json').exists() else mf.get('config',{})
        metrics = json.loads((folder/'metrics.json').read_text()) if (folder/'metrics.json').exists() else {}
        # Full vector batches store per-sequence metric rows, unlike pair runs.
        if not isinstance(metrics, dict): metrics = {'vector_rows': metrics}
        pv = metrics.get('pad_vector', {})
        aliases = [k for k,v in selected.items() if v==folder.name]
        source = cfg.get('normalized_source')
        if not source and cfg.get('sequence'): source = f"data/normalized/manus_official_ufbx_v1/{cfg['sequence']}.npz"
        if mf['kind']=='synthetic_partner_input': source = str(folder/'source.npz')
        files = [p for p in folder.rglob('*.npz') if 'code' not in p.parts]
        record = {'run_id':folder.name, 'aliases':aliases, 'kind':mf['kind'], 'status':mf['status'],
            'selected':bool(aliases), 'finger':cfg.get('target_finger','index' if mf['kind'].startswith('pinch_') else None),
            'source_kind':cfg.get('source_kind','recorded_manus' if mf['kind'].startswith('pinch_') else 'see_manifest'),
            'sequence':cfg.get('sequence'), 'source_window_s':cfg.get('source_window_s'),
            'source_pose_s':cfg.get('source_pose_s'),
            'temporal_protocol':'frozen_pose' if mf['kind']=='pinch_hold' else 'chronological_clip' if mf['kind']=='pinch_clip' else mf['kind'],
            'use':'selected_regression' if aliases else 'candidate_or_fixture_archive',
            'independent_final_test':False, 'teacher_qualified':False,
            'task_passed':pv.get('short_clip_task_passed',pv.get('held_input_task_passed',metrics.get('passed'))),
            'contact_fraction':pv.get('contact_fraction_during_source_close',pv.get('contact_fraction')),
            'config':str((folder/'config.json').relative_to(ROOT)) if (folder/'config.json').exists() else None,
            'manifest_sha256':digest(path),
            'trajectories':[{'path':str(p.relative_to(ROOT)), 'sha256':digest(p)} for p in files]}
        if source:
            p = checked(source)
            record['source'] = {'path':str(p.relative_to(ROOT)), 'sha256':digest(p) if p.exists() else None}
        if mf['kind']=='side_swing':
            record['source_kind']='mixed_recorded_and_synthetic'
            record['candidate_results']={case:{candidate:{k:v for k,v in values.items() if k in
                ['software_gates_passed','gates','max_penetration_m','observable_mapping_max_error_deg',
                 'max_flexion_correction_deg','max_side_correction_deg','lateral_ground_truth_available']}
                for candidate,values in candidates.items()} for case,candidates in metrics.items()}
        if mf['status']=='success' and mf['kind'] in ['pinch_clip','pinch_hold','side_swing','opposition_sequence']:
            record['reproduce_frozen_code'] = replay(folder.name)
            record['regress_current_code'] = replay(folder.name, True)
        cases.append(record)
    with run('trajectory_catalog', {'schema_version':1}) as (out, mf):
        catalog = {'schema_version':1, 'root':str(ROOT), 'selected_aliases':selected, 'cases':cases,
                   'raw_dataset':'data/raw/manus_official/v1',
                   'normalized_dataset':'data/normalized/manus_official_ufbx_v1',
                   'split':'data/splits/manus_official_v1.json',
                   'policy':'Recorded tuning, synthetic fixtures, frozen poses, and composite demos are distinct; no independent final set yet.'}
        write_json(out/'catalog.json', catalog)
        write_json('registry/trajectory_catalog.json', catalog)
        print(json.dumps({'cases':len(cases),'selected':len(selected),'catalog':str(out/'catalog.json')}))


def main():
    configure()
    ap = argparse.ArgumentParser()
    ap.add_argument('action', choices=['build','list','command'])
    ap.add_argument('--case')
    ap.add_argument('--current-code', action='store_true')
    args = ap.parse_args()
    if args.action=='build': build()
    elif args.action=='list':
        for alias,rid in selected_cases().items(): print(alias, rid)
    else:
        if not args.case: ap.error('command requires --case ALIAS_OR_RUN_ID')
        print(replay(selected_cases().get(args.case,args.case), args.current_code))


if __name__=='__main__': main()
