"""Preserve every candidate from this opposition iteration and hash the transfer."""
import json
import tarfile
from lab_common import checked, configure, storage_check, digest, write_json, append_json, stamp


def main():
    configure(); storage_check()
    base = 'partner_iteration_20260913'
    index = checked(f'exports/{base}_review_index.json')
    archive = checked(f'exports/{base}_review.tar.gz')
    if index.exists() or archive.exists():
        raise FileExistsError('Immutable archive already exists; use a new version')
    kinds = {'pinch_hold', 'pinch_clip', 'render_pinch', 'check_partners', 'synthetic_partner_input',
             'check_synthetic_inputs', 'partner_initialization', 'partner_anchor_tuning',
             'partner_dynamics_check', 'render_partners', 'partner_report'}
    keep = set()
    for folder in checked('runs').iterdir():
        mf = folder / 'manifest.json'
        if folder.name >= '20260913T092600Z' and mf.exists() and json.loads(mf.read_text())['kind'] in kinds:
            meta = json.loads(mf.read_text())
            if meta['status'] == 'running':
                raise RuntimeError(f'Cannot archive unfinished run: {folder.name}')
            keep.update(p for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    for pattern in [f'reports/{base}.*', 'registry/partner_iteration.json', 'registry/runs.jsonl',
                    'registry/video_tool.json', 'locks/core-py312-linux-x86_64.txt',
                    'repos/Revo-Retargeting/docs/revo3_manus_execution_plan.md']:
        keep.update(p for p in checked('.').glob(pattern) if p.is_file())
    records = [{'path': str(p.relative_to(checked('.'))), 'bytes': p.stat().st_size, 'sha256': digest(p)}
               for p in sorted(keep)]
    write_json(index, {'created_at': stamp(), 'files': records,
                       'policy': 'All candidates including failures retained; generated inputs included; raw authoritative runs remain remotely'})
    with tarfile.open(archive, 'w:gz') as tar:
        for path in sorted(keep | {index}):
            tar.add(path, arcname=str(path.relative_to(checked('.'))), recursive=False)
    receipt = {'created_at': stamp(), 'path': str(archive), 'sha256': digest(archive),
               'bytes': archive.stat().st_size, 'files': len(keep) + 1}
    append_json('registry/transfers.jsonl', receipt)
    write_json(f'exports/{base}_receipt.json', receipt)
    print(json.dumps(receipt), flush=True)


if __name__ == '__main__':
    main()
