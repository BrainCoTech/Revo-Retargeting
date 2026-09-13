"""Package registered manifests/reports and selected replay artifacts without bulk mirroring."""
import argparse
import json
from pathlib import Path
import tarfile
from lab_common import checked, configure, digest, stamp, storage_check, write_json, append_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--final-run',required=True);p.add_argument('--render-run',required=True)
    args=p.parse_args();configure();storage_check()
    final=checked(Path('runs')/args.final_run)
    selection=json.loads((final/'selection.json').read_text())
    resources=json.loads(checked('registry/resources.json').read_text())
    resources['vector_iteration_result']={'updated_at':stamp(),'status':'three_batches_completed_task_not_accepted',
        'final_run':args.final_run,'hardware_capability':'confirmed_by_user','hardware_capability_test':'skipped_by_user',
        'model_ids':['right_official_pd_v1','right_official_pd_v2'],'ufbx':'toolchains/ufbx-0.17.1',
        'renderer':'existing EGL successfully rendered four saved dynamic states','render_run':args.render_run,
        'outside_root_installs':[],'next':'labelled contact task input, pad regions, trajectory constraints'}
    write_json('registry/resources.json',resources)
    write_json('registry/vector_iteration.json',resources['vector_iteration_result'])
    report=checked('reports/vector_iteration_20260913.md')
    extra='\n## 动力学轨迹抽帧\n\n已有 EGL 离屏渲染通过，4 张图来自第三批 vector_collision_4 的 q_actual；图像本身为已执行动力学轨迹的姿态回放。\n'
    if '## 动力学轨迹抽帧' not in report.read_text():report.write_text(report.read_text()+extra)
    overview=json.loads(checked('reports/vector_iteration_20260913.json').read_text())
    overview['report_sha256']=digest(report)
    overview['render_run']=args.render_run
    write_json('reports/vector_iteration_20260913.json',overview)
    keep=set()
    def add(path):
        path=checked(path)
        if path.is_file():keep.add(path)
    for p in checked('registry').rglob('*.json'):add(p)
    for p in checked('registry').glob('*.jsonl'):add(p)
    for p in checked('reports').glob('vector_*20260913.*'):add(p)
    for p in checked('data/normalized/manus_official_ufbx_v1').glob('*.json'):add(p)
    add('data/splits/manus_official_v1.json')
    for p in checked('models/revo3').rglob('*'):
        if p.suffix in ['.xml','.json','.diff']:add(p)
    recognized={'prepare_assets','ingest_manus','check_vector','vector','render_vector'}
    for folder in checked('runs').iterdir():
        mf=folder/'manifest.json'
        if not mf.exists():continue
        meta=json.loads(mf.read_text())
        if meta.get('kind') not in recognized:continue
        for p in folder.rglob('*'):
            if p.is_file() and (p.suffix in ['.json','.log','.py','.c','.md'] or p.name in ['source.diff']):add(p)
    for candidate in {'hybrid_reference',selection['best_geometry_candidate'],'vector_collision_4'}:
        for p in (final/candidate).rglob('*'):
            if p.suffix in ['.npz','.parquet']:add(p)
    for p in checked(Path('runs')/args.render_run).glob('*.png'):add(p)
    source=checked('repos/Revo-Retargeting/experiments/revo3_lab')
    previous=json.loads(checked('registry/source_snapshot.json').read_text())
    source_snapshot={'created_at':stamp(),'kind':'vector_iteration_final','parent_snapshot':previous,
                     'files_sha256':{str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts}}
    write_json('registry/source_snapshots/vector_iteration_20260913.json',source_snapshot)
    add('registry/source_snapshots/vector_iteration_20260913.json')
    records=[{'path':str(p.relative_to(checked('.'))),'bytes':p.stat().st_size,'sha256':digest(p)} for p in sorted(keep)]
    index=checked('exports/vector_iteration_20260913_review_index.json')
    write_json(index,{'created_at':stamp(),'files':records,'policy':'selected trajectories only; remaining intermediate data retained remotely'})
    archive=checked('exports/vector_iteration_20260913_review.tar.gz')
    if archive.exists():raise FileExistsError('Do not overwrite an existing iteration archive')
    with tarfile.open(archive,'w:gz') as tar:
        for path in sorted(keep|{index}):tar.add(path,arcname=str(path.relative_to(checked('.'))),recursive=False)
    receipt={'created_at':stamp(),'path':str(archive),'sha256':digest(archive),'bytes':archive.stat().st_size,'files':len(keep)+1}
    append_json('registry/transfers.jsonl',receipt)
    print(json.dumps(receipt))


if __name__=='__main__':main()
