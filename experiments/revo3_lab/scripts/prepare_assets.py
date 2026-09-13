"""Fetch a fixed official right-hand asset snapshot and a small FBX evaluator."""
import concurrent.futures
from pathlib import Path
import posixpath
import urllib.request
import xml.etree.ElementTree as ET
from lab_common import checked, digest, run, write_json

MODEL_COMMIT = 'f332a6f0dc944e26b82976b637074b03f7ee8a2c'
UFBX_COMMIT = '6ca5309972f03625e6990f3084ff4c1cc55a09b6'


def download(url, path):
    path = checked(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        temp = checked(path.with_suffix(path.suffix + '.partial'))
        try:
            with urllib.request.urlopen(url, timeout=90) as r, temp.open('wb') as f:
                while block := r.read(1024 * 1024):
                    f.write(block)
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
    return {'path': str(path), 'source': url, 'sha256': digest(path), 'bytes': path.stat().st_size}


def main():
    with run('prepare_assets') as (out, manifest):
        base = f'https://raw.githubusercontent.com/BrainCoTech/brainco-description/{MODEL_COMMIT}/'
        folder = checked('repos/brainco-description')
        xml_rel = 'revo3_system/mjcf/revo3_right.xml'
        records = [download(base + xml_rel, folder / xml_rel)]
        tree = ET.parse(folder / xml_rel)
        paths = [posixpath.normpath(posixpath.join('revo3_system/mjcf', m.attrib['file']))
                 for m in tree.findall('.//mesh') if 'file' in m.attrib]
        if any(not p.startswith('revo3_system/meshes/') for p in paths):
            raise RuntimeError('Unexpected mesh path')
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            for result in pool.map(lambda p: download(base + p, folder / p), sorted(set(paths))):
                records.append(result)
                print('asset', len(records), Path(result['path']).name, flush=True)
        for name in ['ufbx.c', 'ufbx.h', 'LICENSE']:
            records.append(download(f'https://raw.githubusercontent.com/ufbx/ufbx/{UFBX_COMMIT}/{name}',
                                    checked('toolchains/ufbx-0.17.1') / name))
        receipt = {'model_commit': MODEL_COMMIT, 'ufbx_commit': UFBX_COMMIT,
                   'model_checkout': 'unmodified right-hand XML and all referenced meshes; source snapshot, not full git clone',
                   'fbx_evaluator': 'ufbx v0.17.1 C API; replaces planned Blender with a smaller animation evaluator',
                   'files': records}
        write_json(out / 'assets.json', receipt)
        write_json('registry/vector_assets.json', receipt)
        manifest['asset_receipt'] = str(out / 'assets.json')


if __name__ == '__main__':
    main()
