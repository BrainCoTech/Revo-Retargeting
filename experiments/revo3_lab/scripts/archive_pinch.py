"""Selected immutable pinch experiment artifacts and cryptographic transfer inventory."""
import json
from pathlib import Path
import tarfile
from lab_common import checked,configure,storage_check,digest,write_json,append_json,stamp


def main():
    configure();storage_check();keep=set()
    kinds={'pinch_hold','pinch_clip','render_pinch','check_pinch','pinch_dynamics_check','pinch_report','video_tool'}
    for folder in checked('runs').iterdir():
        mf=folder/'manifest.json'
        if mf.exists() and json.loads(mf.read_text())['kind'] in kinds:
            keep.update(p for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    for pattern in ['reports/pinch_iteration_20260913.*','registry/pinch_iteration.json','registry/video_tool.json','registry/runs.jsonl','locks/core-py312-linux-x86_64.txt']:
        keep.update(p for p in checked('.').glob(pattern) if p.is_file())
    records=[{'path':str(p.relative_to(checked('.'))),'bytes':p.stat().st_size,'sha256':digest(p)} for p in sorted(keep)]
    index=checked('exports/pinch_iteration_20260913_review_index.json')
    if index.exists():raise FileExistsError('Archive already exists; use a new version')
    write_json(index,{'created_at':stamp(),'files':records,'policy':'all pinch candidates including failures retained; authoritative raw runs remain remotely'})
    archive=checked('exports/pinch_iteration_20260913_review.tar.gz')
    with tarfile.open(archive,'w:gz') as tar:
        for p in sorted(keep|{index}):tar.add(p,arcname=str(p.relative_to(checked('.'))),recursive=False)
    receipt={'created_at':stamp(),'path':str(archive),'sha256':digest(archive),'bytes':archive.stat().st_size,'files':len(keep)+1}
    append_json('registry/transfers.jsonl',receipt);write_json('exports/pinch_iteration_20260913_receipt.json',receipt)
    print(json.dumps(receipt),flush=True)

if __name__=='__main__':main()
