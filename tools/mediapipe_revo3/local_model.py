"""Pinned official Revo3 dynamics using the same transformation as DSW PD v2."""
import json
from pathlib import Path
import sys
import urllib.request
import xml.etree.ElementTree as ET

try:
    from . import assets
except ImportError:
    import assets

SCRIPTS = assets.REPO / 'experiments/revo3_lab/scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
from revo3_model import transform_official_model
import mujoco


def _download(url, dest, expected=None):
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        tmp = dest.with_suffix(dest.suffix + '.download')
        try:
            with urllib.request.urlopen(url, timeout=60) as response:
                tmp.write_bytes(response.read())
            if expected and assets.sha(tmp) != expected:
                raise RuntimeError(f'Asset checksum mismatch: {dest}')
            tmp.replace(dest)
        finally:
            tmp.unlink(missing_ok=True)
    if expected and assets.sha(dest) != expected:
        raise RuntimeError(f'Asset checksum mismatch: {dest}')


def prepare_local_model() -> Path:
    """Cache all official meshes and return a full-contact PD-v2 scene path.

    The XML is checksum-pinned. Meshes come from the same immutable commit;
    their first-use hashes are recorded and checked on subsequent builds.
    """
    folder = assets.ROOT / 'assets'
    source = folder / 'revo3_right.xml'
    source_url, source_hash = assets.ASSETS[source.name]
    _download(source_url, source, source_hash)
    output = folder / 'local_dynamics'
    output.mkdir(parents=True, exist_ok=True)
    receipt_path = output / 'model.json'
    previous = json.loads(receipt_path.read_text()) if receipt_path.exists() else {}
    if previous and previous.get('official_commit') != assets.COMMIT:
        raise RuntimeError('Local dynamics cache uses a different official commit')
    hashes = {}
    paths = {}
    for mesh in ET.parse(source).getroot().findall('.//asset/mesh'):
        relative = mesh.attrib['file'].removeprefix('../')
        if Path(relative).is_absolute() or '..' in Path(relative).parts:
            raise ValueError(f'Unexpected mesh path: {relative}')
        dest = folder / relative
        url = f'https://raw.githubusercontent.com/BrainCoTech/brainco-description/{assets.COMMIT}/revo3_system/{relative}'
        _download(url, dest, previous.get('mesh_sha256', {}).get(relative))
        hashes[relative] = assets.sha(dest)
        paths[(source.parent / mesh.attrib['file']).as_posix()] = dest.resolve()
    root, exclusions = transform_official_model(
        source, 'right_official_pd_v2', lambda p: paths[p.as_posix()])
    scene = output / 'scene.xml'
    scene.write_text(ET.tostring(root, encoding='unicode'))
    model = mujoco.MjModel.from_xml_path(str(scene))
    if (model.nq, model.nv, model.nu) != (21, 21, 21):
        raise RuntimeError('Expected 21 independent controlled hinge joints')
    receipt = {
        'id': 'right_official_pd_v2', 'official_commit': assets.COMMIT,
        'source_url': assets.ASSETS[source.name][0],
        'source_sha256': assets.sha(source), 'derived_sha256': assets.sha(scene),
        'mesh_sha256': hashes, 'adjacent_fixed_base_exclusions': exclusions,
        'joint_names': [model.joint(i).name for i in range(model.njnt)],
        'joint_limits_rad': model.jnt_range.tolist(),
        'torque_limits_nm': model.actuator_forcerange.tolist(),
        'timestep_s': .002, 'integrator': 'implicitfast', 'gravity': [0, 0, 0],
        'kp': 3., 'kv': .08, 'damping': .03, 'armature': .00005,
        'friction': [.8, .005, .0001],
        'controller_status': 'software comparison controller; not identified from hardware',
        'landmarks': 'official tip_Link origins; distal collision contacts are proxies, not verified fingerpad contacts',
    }
    receipt_path.write_text(json.dumps(receipt, indent=2, allow_nan=False) + '\n')
    return scene


if __name__ == '__main__':
    print(prepare_local_model())
