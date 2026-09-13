#!/usr/bin/env python3
"""Single entry point for registered offline vector experiments."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from lab_common import checked, configure


def main():
    p=argparse.ArgumentParser()
    p.add_argument('command',choices=['inventory','assets','ingest','check','run','report'])
    args,rest=p.parse_known_args()
    configure()
    scripts=Path(__file__).resolve().parent
    if args.command=='inventory':
        for rel in ['registry/lab.json','registry/environments.json','registry/vector_assets.json','reports/latest_vector.json']:
            path=checked(rel)
            if path.exists():
                value=json.loads(path.read_text())
                if rel.endswith('vector_assets.json'):
                    value={k:v for k,v in value.items() if k!='files'}
                if rel.endswith('latest_vector.json'):
                    value={k:v for k,v in value.items() if k!='metrics'}
                print(json.dumps({rel:value},ensure_ascii=False,indent=2))
        return
    script={'assets':'prepare_assets.py','ingest':'ingest_manus.py','check':'check_vector.py',
            'run':'evaluate_vector.py','report':'report_vector.py'}[args.command]
    subprocess.run([sys.executable,str(scripts/script),*rest],check=True)


if __name__=='__main__':
    main()
