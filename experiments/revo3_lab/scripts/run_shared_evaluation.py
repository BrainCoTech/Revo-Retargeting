"""Frozen cross-motion evaluation of the single public vector Solver (software only)."""
import argparse
import json
import re
import time
from lab_common import checked, configure, digest, run, write_json
configure()
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from ingest_manus import NAMES, FINGERS
from revo3_model import Hand, prepare_model
from vector_solver import Solver, finger_angles
from side_swing import lateral_observation, SIDE_DOFS
from command_limiter import CommandLimiter

FPS = 30
CASES = {
    'side_finger_agility': ('finger_agility', 24., 27.),
    'side_hand_mobility': ('hand_mobility', 18., 21.),
    'pinch_finger_agility': ('finger_agility', 20.8, 21.8),
    'pinch_hand_mobility': ('hand_mobility', 25.8, 26.7),
    'flex_finger_agility': ('finger_agility', 2., 6.),
}


def frozen_inputs(out, cfg, names, max_frames):
    sources = {}
    for seq in ['finger_agility', 'hand_mobility']:
        path = checked('data/normalized/manus_official_ufbx_v1/' + seq + '.npz')
        sources[seq] = (path, np.load(path, allow_pickle=False))
        np.testing.assert_array_equal(sources[seq][1]['names'], NAMES)
    for name in names:
        truth = np.empty((0, 4))
        if name == 'known_angles':
            path = checked('runs/20260913T105508Z_side_swing_8437bb56/known_angles/input.npz')
            data = np.load(path, allow_pickle=False)
            ts, points, truth = data['times'], data['palm_positions_m'], data['side_truth_rad']
            parent = sources['finger_agility'][1]['parent']
            record = {'source_kind': 'synthetic_known_angles', 'already_reflected': True,
                      'anatomical_validity': False, 'angle_truth_available': True}
        else:
            seq, start, end = CASES[name]
            path, data = sources[seq]
            ts = np.arange(start, end + 1e-8, 1/FPS)
            original = data['palm_positions_m']
            points = np.stack([np.interp(ts, data['timestamps'], original[:, i, j])
                               for i in range(25) for j in range(3)], axis=1).reshape(-1, 25, 3)
            points[:, :, 0] *= cfg['source_palm_x_sign']
            parent = data['parent']
            record = {'source_kind': 'recorded_manus', 'sequence': seq, 'window_s': [start, end],
                      'angle_truth_available': False, 'purpose': 'diagnostic; no motion-specific solver'}
        if max_frames:
            ts, points = ts[:max_frames], points[:max_frames]
            truth = truth[:max_frames]
        if len(ts) == 0 or not np.isfinite(points).all():
            raise ValueError('Empty/nonfinite input')
        folder = out/'inputs'/name
        folder.mkdir(parents=True)
        np.savez_compressed(folder/'input.npz', times=ts, palm_positions_m=points, parent=parent,
                            names=np.array(NAMES), side_truth_rad=truth)
        record.update(source_path=str(path), source_sha256=digest(path),
                      exact_input_sha256=digest(folder/'input.npz'), frames=len(ts),
                      reflection_applied_to_recording=cfg['source_palm_x_sign'], resampling='linear, 30 Hz')
        write_json(folder/'source.json', record)
        yield name, ts, points, truth, record


def basic_contact(hand, data):
    force = np.zeros(6)
    pair = False
    non = False
    pen = nonpen = 0.
    for i, contact in enumerate(data.contact):
        depth = max(0., -float(contact.dist)); pen = max(pen, depth)
        target = {hand.distal_geoms.get(int(contact.geom1), -1),
                  hand.distal_geoms.get(int(contact.geom2), -1)} == {0, 1}
        mujoco.mj_contactForce(hand.model, data, i, force)
        touching = contact.dist <= 0 and force[0] > 0
        pair |= target and touching
        non |= not target and touching
        if not target: nonpen = max(nonpen, depth)
    return {'distal_pair_contact': bool(pair), 'non_target_contact': bool(non),
            'max_penetration_m': pen, 'non_target_penetration_m': nonpen}


def rollout(hand, cfg, candidate, ts, points, folder, contact_cfg):
    width = float(np.median(np.linalg.norm(points[:, 6]-points[:, 21], axis=1)))
    if width <= 1e-8: raise ValueError('Degenerate palm width')
    scale = hand.robot_width/width
    targets = points[:, [NAMES.index(f+'_TIP') for f in FINGERS]]*scale@hand.basis.T+hand.wrist
    options = {**cfg['common'], **candidate}
    if options.get('kind') != 'vector': raise ValueError('Every candidate must use the 21-axis vector solver')
    solver = Solver(hand, options, 1/FPS)
    limiter = CommandLimiter(hand.q0, hand.lo, hand.hi, dt=1/FPS,
        speed=cfg.get('limiter', {}).get('speed', 4.), acceleration=cfg.get('limiter', {}).get('acceleration', 20.),
        brake_at_target=True)
    model = hand.model; data = mujoco.MjData(model)
    data.qpos[:] = hand.q0; data.ctrl[:] = hand.q0; mujoco.mj_forward(model, data)
    rows = []; goals = []; mapped = []; commands = []; actual = []; velocities = []; actual_tips = []; latencies = []; details = []
    all_commands = [hand.q0.copy()]
    # First frame repeated for 1.5 s approach and 1 s settling; no robot trajectory is prescribed.
    warm = 75
    for frame in range(warm+len(ts)):
        i = max(0, frame-warm)
        before = time.perf_counter(); q_mapped, detail = solver.solve(points[i], targets[i])
        elapsed = time.perf_counter()-before
        goal = np.asarray(detail['ik_solution'], dtype=float)
        if goal.shape != (21,) or not np.isfinite(goal).all(): raise ValueError('Invalid Solver IK diagnostic')
        command = limiter.update(q_mapped); data.ctrl[:] = command; all_commands.append(command)
        close = np.linalg.norm(points[i, 4]-points[i, 9]) < cfg.get('source_close_m', .015)
        while data.time < (frame+1)/FPS-1e-10:
            mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or np.max(abs(data.qvel)) > 1e4:
                raise RuntimeError('Unstable dynamics')
            # Refresh contacts and body/site transforms together, matching run_pinch.
            mujoco.mj_forward(model, data)
            row = basic_contact(hand, data)
            if contact_cfg:
                from run_pinch import contact_state
                row.update(contact_state(hand, data, contact_cfg))
            row.update(time_s=float(data.time), frame=frame-warm,
                       phase='approach' if frame < 45 else 'settle' if frame < warm else 'tracking',
                       source_close=bool(close), evaluated_contact=bool(frame >= warm and close))
            rows.append(row)
        if frame >= warm:
            goals.append(goal); mapped.append(q_mapped.copy()); commands.append(command); actual.append(data.qpos.copy()); velocities.append(data.qvel.copy())
            actual_tips.append(data.site_xpos[hand.tip_ids].copy()); latencies.append(elapsed); details.append(detail)
        if frame % 60 == 0: print(candidate['name'], folder.name, frame, '/', warm+len(ts), flush=True)
    goals, mapped, commands, actual, velocities, actual_tips = map(np.asarray, [goals, mapped, commands, actual, velocities, actual_tips])
    refs = np.array([finger_angles(p, hand) for p in points])
    lateral = [lateral_observation(p) for p in points]
    side_ref = np.array([v[1] for v in lateral]); confidence = np.array([v[2] for v in lateral])
    observable = confidence >= .3
    # Evaluation reference and mask depend only on input, never the candidate.
    solver_side = {}
    if all('side_reference_rad' in d and 'side_observable' in d for d in details):
        solver_side = {
            'solver_side_reference_rad': np.asarray([d['side_reference_rad'] for d in details]),
            'solver_side_observable': np.asarray([d['side_observable'] for d in details], dtype=bool),
        }
    source_relative = targets[:, 1:]-targets[:, :1]
    actual_relative = actual_tips[:, 1:]-actual_tips[:, :1]
    np.savez_compressed(folder/'trajectory.npz', times=ts, actual_times=ts+1/FPS,
        q_goal=goals, q_mapped=mapped, q_target=commands, q_actual=actual, qvel_actual=velocities, target_tips_m=targets, actual_tips_m=actual_tips,
        posture_reference=refs, side_reference_rad=side_ref, side_confidence=confidence, side_observable=observable,
        source_relative_world_m=source_relative, actual_relative_world_m=actual_relative,
        input_palm_m=points, joint_names=np.array(hand.names), latency_s=latencies, **solver_side)
    pq.write_table(pa.Table.from_pylist(rows), folder/'contacts.parquet')
    write_json(folder/'solver_details.json', details)
    tiperr = np.linalg.norm(actual_tips-targets, axis=-1)*1000
    pairerr = np.linalg.norm(actual_relative-source_relative, axis=-1)*1000
    velocity = np.diff(all_commands, axis=0)*FPS; acceleration = np.diff(velocity, axis=0)*FPS
    flex_ids = np.array([7, 8, 11, 12, 15, 16, 19, 20])
    key = 'valid_pad_contact' if contact_cfg else 'distal_pair_contact'
    evaluated = [r for r in rows if r['evaluated_contact']]
    longest = streak = 0
    for r in rows:
        streak = streak+1 if r['evaluated_contact'] and not r[key] else 0
        longest = max(longest, streak)
    side_error = actual[:, SIDE_DOFS]-side_ref
    metrics = {'scale': scale, 'frames': len(ts), 'actual_tip_mean_mm': float(tiperr.mean()),
        'actual_tip_p95_mm': float(np.quantile(tiperr, .95)), 'actual_pair_mean_mm': float(pairerr.mean()),
        'actual_pair_per_finger_mm': pairerr.mean(axis=0).tolist(),
        'observable_side_reference_rmse_deg': float(np.degrees(np.sqrt(np.mean(side_error[observable]**2)))) if observable.any() else None,
        'side_observable_fraction': float(observable.mean()),
        'pip_dip_actual_range_deg': np.degrees(np.ptp(actual[:, flex_ids], axis=0)).tolist(),
        'pip_dip_reference_rmse_deg': float(np.degrees(np.sqrt(np.mean((actual[:, flex_ids]-refs[:, flex_ids])**2)))),
        'command_speed_max_rad_s': float(abs(velocity).max()),
        'command_acceleration_max_rad_s2': float(abs(acceleration).max()) if acceleration.size else 0.,
        'source_close_evaluated_s': len(evaluated)*model.opt.timestep,
        'source_close_contact_fraction': sum(r[key] for r in evaluated)/len(evaluated) if evaluated else None,
        'source_close_longest_break_s': longest*model.opt.timestep,
        'contact_definition': key, 'source_close_is_contact_truth': False,
        'non_target_contact_note': 'Non-thumb/index contacts only; other finger opposition can be intended, so this is not an automatic wrong-action classification.',
        'all_phases_non_target_contact_fraction': float(np.mean([r['non_target_contact'] for r in rows])),
        'all_phases_max_non_target_penetration_m': max(r['non_target_penetration_m'] for r in rows),
        'all_phases_max_penetration_m': max(r['max_penetration_m'] for r in rows),
        'all_phases_max_joint_limit_violation_rad': max(r.get('joint_limit_violation_rad', 0.) for r in rows),
        'actual_command_tracking_rmse_rad': float(np.sqrt(np.mean((actual-commands)**2))),
        'simulation_warning_counts': [int(w.number) for w in data.warning],
        'solver_not_converged_frames': sum(not x['converged'] for x in details),
        'latency_p95_ms': float(np.quantile(latencies, .95)*1000),
        'relative_vector_note': 'Saved XYZ skeleton/robot tip vectors are geometric diagnostics, not skin sliding.'}
    return metrics, actual, observable


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--config', required=True)
    ap.add_argument('--cases'); ap.add_argument('--max-frames', type=int, default=0)
    args = ap.parse_args(); cfg = json.loads(checked(args.config).read_text())
    names = args.cases.split(',') if args.cases else ['known_angles', *CASES]
    if set(names)-{'known_angles', *CASES}: raise ValueError('Unknown case')
    cnames = [x['name'] for x in cfg['candidates']]
    if len(set(cnames)) != len(cnames) or not all(re.fullmatch('[a-z0-9_]+', n) for n in cnames): raise ValueError('Invalid candidate names')
    if args.max_frames < 0: raise ValueError('max-frames must be nonnegative')
    contact_cfg = cfg.get('contact_geometry')
    if contact_cfg is None:
        contact_cfg = json.loads(checked(cfg.get('contact_config',
            'repos/Revo-Retargeting/experiments/revo3_lab/configs/pinch_task_v1.json')).read_text())
    with run('shared_evaluation', {'config': cfg, **vars(args)}) as (out, manifest):
        model = prepare_model(cfg.get('model_id', 'right_official_pd_v2'))
        if contact_cfg:
            from pinch_geometry import PadHand
            hand = PadHand(model, contact_cfg)
        else: hand = Hand(model)
        assert len(hand.names) == 21
        manifest.update(model={'path': str(model), 'sha256': digest(model)},
            mapping='vector_solver.Solver for all candidates/cases; all 21 axes',
            warmup='1.5 s approach + 1 s settle using repeated first source frame; included in collision metrics')
        write_json(out/'config.json', cfg)
        summary = []
        for case, ts, points, truth, record in frozen_inputs(out, cfg, names, args.max_frames):
            for candidate in cfg['candidates']:
                folder = out/candidate['name']/case; folder.mkdir(parents=True)
                metrics, actual, observable = rollout(hand, cfg, candidate, ts, points, folder, contact_cfg)
                if len(truth):
                    error = actual[:, SIDE_DOFS]-truth
                    metrics['observable_known_side_truth_rmse_deg'] = float(np.degrees(np.sqrt(np.mean(error[observable]**2)))) if observable.any() else None
                metrics.update(candidate=candidate['name'], case=case, **record)
                write_json(folder/'metrics.json', metrics); summary.append(metrics)
                write_json(out/'metrics.json', summary)
                print(json.dumps(metrics), flush=True)


if __name__ == '__main__': main()
