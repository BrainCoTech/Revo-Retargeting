"""Read saved shared-solver dynamics; report geometric proxies, not skin slip."""
import argparse
import json
import re
from lab_common import checked, configure, digest, run, write_json
configure()
import numpy as np
import pyarrow.parquet as pq
from revo3_model import Hand
from pinch_geometry import AllPadGeometry
from vector_solver import pair_observation
from side_swing import SIDE_DOFS, lateral_observation


def verified(path, manifest, root):
    expected = manifest['outputs_sha256'].get(str(path.relative_to(root)))
    if expected is None or digest(path) != expected:
        raise ValueError('Missing or changed recorded output: '+str(path))
    return path


def axis_stats(actual, reference, mask):
    a, r = actual[mask], reference[mask]
    if not len(a):
        return dict(samples=0, rmse_mm=None, actual_range_mm=None,
                    reference_range_mm=None, correlation=None)
    return dict(samples=len(a), rmse_mm=float(np.sqrt(np.mean((a-r)**2))*1000),
                actual_range_mm=float(np.ptp(a)*1000), reference_range_mm=float(np.ptp(r)*1000),
                correlation=float(np.corrcoef(a, r)[0, 1])
                if len(a)>1 and min(float(np.std(a)), float(np.std(r)))>1e-8 else None)


def truth_stats(actual, truth, mask):
    error = np.degrees(actual-truth)
    per_finger = []
    for k in range(4):
        values = error[mask[:, k], k]
        per_finger.append(dict(samples=len(values), rmse_deg=float(np.sqrt(np.mean(values**2))) if len(values) else None))
    return dict(samples=int(mask.sum()),
                rmse_deg=float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None,
                per_finger=per_finger)


def known_truth(root, case, source, record, hand):
    if case != 'known_angles':
        return None
    # Shared input originally saved side truth only. Recover flex truth from
    # its hash-verified original fixture, never infer it from actual outputs.
    original = checked(record['source_path'])
    if digest(original) != record['source_sha256']:
        raise ValueError('Original known-angle fixture checksum changed')
    with np.load(original, allow_pickle=False) as fixture:
        required = {'times', 'palm_positions_m', 'side_truth_rad', 'flex_truth_rad'}
        if not required.issubset(fixture.files):
            raise ValueError('Known-angle fixture lacks explicit truth: '+str(required-set(fixture.files)))
        indices = np.searchsorted(fixture['times'], source['times'])
        if np.any(indices >= len(fixture['times'])):
            raise ValueError('Known-angle input times outside original fixture')
        np.testing.assert_array_equal(fixture['times'][indices], source['times'])
        np.testing.assert_array_equal(fixture['palm_positions_m'][indices], source['palm_positions_m'])
        truth = fixture['side_truth_rad'][indices]
        np.testing.assert_array_equal(truth, source['side_truth_rad'])
        flex = fixture['flex_truth_rad'][indices]
        if flex.ndim == 1:
            flex = np.repeat(flex[:, None], 4, axis=1)
        if flex.shape != truth.shape or not np.isfinite(flex).all():
            raise ValueError('Invalid flexion truth shape or values')
    flex_ids = SIDE_DOFS+1
    reachable = (flex >= hand.lo[flex_ids]-1e-8) & (flex <= hand.hi[flex_ids]+1e-8)
    observable = np.asarray([lateral_observation(p)[2] >= .3 for p in source['palm_positions_m']])
    return truth, flex, reachable, observable, dict(
        original_path=str(original), original_sha256=digest(original),
        explicit_truth_fields=['side_truth_rad', 'flex_truth_rad'],
        flex_joint_names=[hand.names[j] for j in flex_ids],
        flex_hardware_bounds_deg=np.degrees(np.stack([hand.lo[flex_ids], hand.hi[flex_ids]], axis=1)).tolist(),
        reachability_definition='Per-finger MCP flex truth within hardware joint bounds; not full-hand collision feasibility.',
        observability_definition='Independent input-only lateral_observation confidence >= 0.3; not Solver hysteresis state.')


def phase_contacts(path):
    rows = pq.read_table(path).to_pydict()
    phases = np.asarray(rows['phase'])
    result = {}
    for name, mask in [('warmup', phases != 'tracking'), ('tracking', phases == 'tracking')]:
        entry = {'physics_samples': int(mask.sum())}
        for key in ['max_penetration_m', 'non_target_penetration_m', 'joint_limit_violation_rad']:
            values = np.asarray(rows[key])[mask] if key in rows else np.empty(0)
            entry[key] = float(values.max()) if len(values) else None
        result[name] = entry
    return result


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    if not re.fullmatch('[A-Za-z0-9_]+', args.run_id):
        raise ValueError('Invalid run id')
    root = checked('runs/'+args.run_id)
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest['status'] != 'success':
        raise ValueError('Analyze only completed evaluations')
    cfg = json.loads(verified(root/'config.json', manifest, root).read_text())
    model = checked(manifest['model']['path'])
    if digest(model) != manifest['model']['sha256']:
        raise ValueError('Model checksum changed')
    hand = Hand(model); pads = AllPadGeometry(hand)
    summary = []
    with run('analyze_shared', vars(args)) as (out, analysis_manifest):
        analysis_manifest.update(source_run=args.run_id, source_manifest_sha256=digest(root/'manifest.json'),
                                 model=manifest['model'], analysis='Saved actual-state geometry; no simulation or optimization rerun')
        for source_folder in sorted((root/'inputs').iterdir()):
            if not source_folder.is_dir(): continue
            case = source_folder.name
            record = json.loads(verified(source_folder/'source.json', manifest, root).read_text())
            source_path = verified(source_folder/'input.npz', manifest, root)
            if digest(source_path) != record['exact_input_sha256']:
                raise ValueError('Frozen source checksum mismatch')
            with np.load(source_path, allow_pickle=False) as z:
                source = {k: z[k] for k in z.files}
            known = known_truth(root, case, source, record, hand)
            for candidate in cfg['candidates']:
                name = candidate['name']; folder = root/name/case
                trajectory_path = verified(folder/'trajectory.npz', manifest, root)
                details_path = verified(folder/'solver_details.json', manifest, root)
                contacts_path = verified(folder/'contacts.parquet', manifest, root)
                details = json.loads(details_path.read_text())
                with np.load(trajectory_path, allow_pickle=False) as z:
                    tr = {k: z[k] for k in z.files}
                np.testing.assert_array_equal(tr['times'], source['times'])
                np.testing.assert_array_equal(tr['input_palm_m'], source['palm_positions_m'])
                count = len(tr['times'])
                if len(details) != count or tr['q_actual'].shape != (count, 21):
                    raise ValueError('Saved frame count mismatch')
                raw = np.array([pair_observation(p, target, hand)[0]
                                for p, target in zip(tr['input_palm_m'], tr['target_tips_m'])])
                activation = np.asarray([d['pair_activation'] for d in details])
                target_available = all('target_pair_coordinates_m' in d for d in details)
                if any('target_pair_coordinates_m' in d for d in details) != target_available:
                    raise ValueError('Partial target-coordinate diagnostics')
                target = np.asarray([d['target_pair_coordinates_m'] for d in details]) if target_available else None
                if target_available:
                    logged_raw = np.asarray([d['input_pair_coordinates_m'] for d in details])
                    np.testing.assert_allclose(raw, logged_raw, atol=1e-10, rtol=1e-9)
                actual = []
                for q in tr['q_actual']:
                    hand.fk(q)
                    p, n, axes, *_ = pads.evaluate()
                    tangent = axes[1:]-n[1:]*np.sum(axes[1:]*n[1:], axis=1)[:, None]
                    lengths = np.linalg.norm(tangent, axis=1)
                    if np.any(lengths < 1e-8): raise ValueError('Degenerate robot pad tangent')
                    tangent /= lengths[:, None]
                    frame = np.stack([tangent, np.cross(n[1:], tangent), n[1:]], axis=1)
                    actual.append(np.einsum('nik,nk->ni', frame, p[0]-p[1:]))
                actual = np.asarray(actual)
                pair_metrics = {}
                for k, finger in enumerate(['index', 'middle', 'ring', 'little']):
                    mask = activation[:, k] > .5
                    pair_metrics[finger] = dict(near_frames=int(mask.sum()), total_frames=count,
                        raw={axis: axis_stats(actual[:, k, j], raw[:, k, j], mask)
                             for j, axis in enumerate(['longitudinal', 'transverse', 'normal'])},
                        target={axis: axis_stats(actual[:, k, j], target[:, k, j], mask)
                                for j, axis in enumerate(['longitudinal', 'transverse', 'normal'])}
                        if target_available else None)
                result = dict(candidate=name, case=case, pairs=pair_metrics,
                    near_mask='Saved candidate pair_activation > 0.5; candidates without pad activation have zero near samples.',
                    coordinates_note='Skeleton tip-vector proxy versus robot fixed-pad-center vector, each in its partner moving frame. Not human skin contact or simulated surface slip.',
                    error_note='Absolute local-coordinate RMSE; ranges and correlation use the same selected frames without demeaning RMSE or fitting gain/lag.',
                    contact_scope_note='Existing contact parquet uses thumb/index as target. Its non_target_penetration includes other thumb partners and is not a pair-wise pad-contact verdict.',
                    target_available=target_available, phases=phase_contacts(contacts_path))
                extras = {}
                if known is not None:
                    truth, flex, reachable, observable, info = known
                    result['known_angles'] = dict(**info, groups={label: truth_stats(tr['q_actual'][:, SIDE_DOFS], truth, mask)
                        for label, mask in [('all', np.ones_like(reachable)), ('flex_reachable', reachable),
                                            ('observable', observable), ('flex_reachable_and_observable', reachable & observable),
                                            ('flex_unreachable', ~reachable)]})
                    extras.update(side_truth_rad=truth, flex_truth_rad=flex,
                                  flex_reachable=reachable, independent_side_observable=observable)
                dest = out/name/case; dest.mkdir(parents=True)
                np.savez_compressed(dest/'coordinates.npz', times=tr['times'], actual_pair_coordinates_m=actual,
                                    raw_input_pair_coordinates_m=raw, pair_activation=activation,
                                    **({'target_pair_coordinates_m': target} if target_available else {}), **extras)
                write_json(dest/'metrics.json', result); summary.append(result)
                print(name, case, 'analyzed', count, 'frames', flush=True)
        write_json(out/'metrics.json', summary)
        print(str(out), flush=True)


if __name__ == '__main__':
    main()
