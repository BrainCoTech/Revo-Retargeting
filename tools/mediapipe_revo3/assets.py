"""Download pinned assets atomically; never write outside local artifacts."""
import hashlib
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[2]
ROOT = REPO / 'artifacts/mediapipe_revo3'
COMMIT = 'f332a6f0dc944e26b82976b637074b03f7ee8a2c'
ASSETS = {
    'revo3_right.xml': (
        f'https://raw.githubusercontent.com/BrainCoTech/brainco-description/{COMMIT}/revo3_system/mjcf/revo3_right.xml',
        'b83f91212daec690d9a7d6589206be05a4ac51edb2750a8c3d5e7e4408ec90a7'),
    'hand_landmarker.task': (
        'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task',
        'fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1'),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare():
    folder = ROOT / 'assets'
    folder.mkdir(parents=True, exist_ok=True)
    for name, (url, expected) in ASSETS.items():
        dest = folder / name
        if not dest.exists():
            tmp = dest.with_suffix('.download')
            try:
                with urllib.request.urlopen(url, timeout=60) as response:
                    tmp.write_bytes(response.read())
                if expected and sha(tmp) != expected:
                    raise RuntimeError(f'Asset checksum mismatch: {name}')
                tmp.replace(dest)
            finally:
                tmp.unlink(missing_ok=True)
        if expected and sha(dest) != expected:
            raise RuntimeError(f'Asset checksum mismatch: {dest}')
        print(f'{name}: {sha(dest)}')
    # Kinematic-only preview. Keep official transforms, inertias, axes and limits;
    # omit meshes, contacts and actuators. No dynamics/contact claim is made.
    root = ET.parse(folder / 'revo3_right.xml').getroot()
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag in {'geom', 'asset', 'actuator', 'sensor', 'contact', 'keyframe'}:
                parent.remove(child)
    ET.ElementTree(root).write(folder / 'kinematics.xml', encoding='unicode')
    return folder


if __name__ == '__main__':
    prepare()
