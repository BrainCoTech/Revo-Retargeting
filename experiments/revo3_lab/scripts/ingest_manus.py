"""Evaluate official MANUS FBX key times, retain raw transforms, normalize landmarks."""
import json
from pathlib import Path
import subprocess
from lab_common import checked, configure, digest, run, write_json
configure()
import numpy as np

FINGERS = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
NAMES = ['Hand'] + ['Thumb_' + j for j in ['CMC', 'MCP', 'DIP', 'TIP']] + [
    f + '_' + j for f in FINGERS[1:] for j in ['CMC', 'MCP', 'PIP', 'DIP', 'TIP']]


def normalize(world, names):
    idx = {name: names.index(name) for name in NAMES}
    wrist = world[:, idx['Hand']]
    z = world[:, idx['Middle_MCP']] - wrist
    z /= np.linalg.norm(z, axis=-1, keepdims=True)
    y = world[:, idx['Index_MCP']] - world[:, idx['Pinky_MCP']]
    y -= np.sum(y*z, axis=-1, keepdims=True)*z
    y /= np.linalg.norm(y, axis=-1, keepdims=True)
    x = np.cross(y, z)
    basis = np.stack([x, y, z], axis=-1)
    palm = np.einsum('fji,fkj->fki', basis, world - wrist[:, None])
    return palm, basis


def main():
    with run('ingest_manus') as (out, manifest):
        tool = checked('toolchains/ufbx-0.17.1')
        binary = tool / 'export_fbx'
        source = Path(__file__).resolve().parents[1] / 'tools/export_fbx.c'
        cmd = ['cc', '-O2', '-std=c99', '-I', str(tool), str(source), str(tool/'ufbx.c'), '-lm', '-o', str(binary)]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        (out/'compile.log').write_text(result.stdout + result.stderr)
        result.check_returncode()
        manifest['exporter'] = {'command': cmd, 'sha256': digest(binary)}
        raw_manifest = json.loads(checked('data/raw/manus_official/v1/manifest.json').read_text())
        qualities = []
        for item in raw_manifest:
            if not item['name'].endswith('.fbx'):
                continue
            source_file = checked('data/raw/manus_official/v1') / item['name']
            if digest(source_file) != item['sha256']:
                raise ValueError('MANUS source hash mismatch')
            raw = out / (source_file.stem + '.jsonl')
            with raw.open('w') as f:
                subprocess.run([str(binary), str(source_file)], stdout=f, check=True, timeout=60)
            with raw.open() as f:
                meta = json.loads(next(f))
                frames = [json.loads(line) for line in f]
            names = [x['name'] for x in meta['nodes']]
            indexes = [names.index(n) for n in NAMES]
            matrices = np.array([x['world_matrices'] for x in frames]).reshape(len(frames), -1, 3, 4)
            times = np.array([x['time'] for x in frames])
            # Index landmarks after selecting translation: mixed advanced/scalar
            # indexing here would move the landmark axis ahead of the time axis.
            world = matrices[:, :, :, 3][:, indexes, :] * meta['unit_meters']
            if world.shape != (len(times), len(NAMES), 3):
                raise ValueError('Expected [time, landmark, xyz] coordinates')
            if not np.all(np.diff(times) > 0) or not np.isfinite(world).all():
                raise ValueError('Invalid times/coordinates')
            parents = []
            for name in NAMES:
                node = meta['nodes'][names.index(name)]
                parent = next((n['name'] for n in meta['nodes'] if n['id'] == node['parent_id']), '')
                parents.append(NAMES.index(parent) if parent in NAMES else -1)
            bone_lengths = np.stack([np.linalg.norm(world[:, i]-world[:, p], axis=1)
                                     for i, p in enumerate(parents) if p >= 0], axis=1)
            if bone_lengths.min() < .002 or bone_lengths.max() > .2:
                raise ValueError('Unexpected bone length; do not guess unit correction')
            palm, basis = normalize(world, NAMES)
            distances = np.stack([np.linalg.norm(palm[:, NAMES.index(f+'_TIP')] - palm[:, 4], axis=1)
                                  for f in FINGERS[1:]], axis=1)
            # Canonical x is cross(radial, distal). Record curl sign, do not silently reflect anatomy.
            curl_x = np.concatenate([palm[:, NAMES.index(f+'_TIP'), 0] - palm[:, NAMES.index(f+'_MCP'), 0]
                                     for f in FINGERS[1:]])
            quality = {'name': source_file.stem, 'source': item, 'frames': len(times),
                       'duration_s': float(times[-1]-times[0]), 'declared_fps': meta['fps'],
                       'median_key_dt_s': float(np.median(np.diff(times))), 'unit_meters': meta['unit_meters'],
                       'coordinate_frame': 'origin Hand; z to Middle_MCP; y Pinky_MCP to Index_MCP orthogonalized; x=cross(y,z)',
                       'source_hand_side': 'unspecified in source; kinematic curl sign recorded for mapping review',
                       'curl_x_quantiles_m': np.quantile(curl_x, [.01,.5,.99]).tolist(),
                       'bone_length_min_m': float(bone_lengths.min()), 'bone_length_max_m': float(bone_lengths.max()),
                       'max_bone_length_std_m': float(bone_lengths.std(axis=0).max()),
                       'tip_distance_min_m': distances.min(axis=0).tolist(),
                       'contact_intent': 'unlabelled; tip proximity is not a contact annotation',
                       'raw_evaluated_transforms': str(raw), 'raw_evaluated_transforms_sha256': digest(raw)}
            folder = checked('data/normalized/manus_official_ufbx_v1')
            folder.mkdir(parents=True, exist_ok=True)
            dest = folder/(source_file.stem+'.npz')
            arrays = dict(timestamps=times, names=np.array(NAMES), parent=np.array(parents),
                          world_positions_m=world, palm_positions_m=palm, palm_basis=basis,
                          observable_mask=np.ones((len(times),len(NAMES)), dtype=bool),
                          tip_distances_m=distances, world_matrices=matrices)
            if dest.exists():
                with np.load(dest,allow_pickle=False) as existing:
                    if set(existing.files) != set(arrays) or any(not np.array_equal(existing[k],v) for k,v in arrays.items()):
                        raise RuntimeError(f'Refusing to mutate normalized data version: {dest}')
            else:
                np.savez_compressed(dest, **arrays)
            quality['normalized_sha256'] = digest(dest)
            if not (folder/(source_file.stem+'.json')).exists():
                write_json(folder/(source_file.stem+'.json'), quality)
            qualities.append(quality)
            print(json.dumps(quality), flush=True)
        write_json(out/'quality.json', qualities)
        manifest['input_quality'] = qualities
        split = {'schema_version': 1, 'purpose': 'integration and tuning only; two official samples do not establish generalization',
                 'tune': ['finger_agility'], 'validation': ['hand_mobility'], 'final_test': [],
                 'final_test_status': 'requires independent labelled sequence; never reuse tuning/validation as final',
                 'dataset': 'manus_official_ufbx_v1', 'sequence_hashes': {q['name']: q['normalized_sha256'] for q in qualities}}
        path = checked('data/splits/manus_official_v1.json')
        if path.exists() and json.loads(path.read_text()) != split:
            raise RuntimeError('Refusing to mutate frozen data split')
        write_json(path, split)


if __name__ == '__main__':
    main()
