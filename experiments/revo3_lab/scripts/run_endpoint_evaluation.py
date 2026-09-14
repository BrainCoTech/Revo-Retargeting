"""Source-frame-aligned endpoint evaluation; registered DSW runtime only."""
import argparse
from collections import Counter
import json
import time
from lab_common import checked, configure, digest, run, write_json
configure()
import mujoco
import numpy as np
from endpoint_input import from_manus, from_mediapipe, iter_mediapipe_jsonl
from ingest_manus import NAMES
from revo3_model import Hand, prepare_model
from vector_solver import Solver


def json_safe(value):
    if isinstance(value, dict): return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list): return [json_safe(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def validate(cfg):
    inp = cfg['input']
    if inp['source'] == 'manus' and (not isinstance(inp.get('source_kind'), str) or not inp['source_kind'].strip()):
        raise ValueError('MANUS requires explicit nonempty source_kind provenance')
    if inp['source'] not in ('manus', 'mediapipe'):
        raise ValueError('input.source must be manus or mediapipe')
    if inp['palm_x_sign'] not in (-1, 1):
        raise ValueError('Explicit palm_x_sign must be -1 or 1')
    if not np.isfinite(inp['scale']) or inp['scale'] <= 0:
        raise ValueError('scale must be a fixed finite positive number')
    if inp['hand_side'] not in ('Right', 'Left'):
        raise ValueError('hand_side must be Right or Left')
    gap = cfg.get('max_gap_s', 1.)
    if not np.isfinite(gap) or gap < 1/30:
        raise ValueError('max_gap_s must be finite and at least 1/30')
    solver = cfg['solver']
    if solver.get('kind') != 'vector' or solver.get('pad_pair_weight', 0) != 0:
        raise ValueError('Endpoint profile requires vector solver without pad losses')
    if solver.get('observation_mode', 'legacy') != 'legacy':
        raise ValueError('Endpoint profile must not enable skeleton observation')
    return checked(inp['file'])


def frames(path, inp, hand):
    common = dict(basis=hand.basis, wrist=hand.wrist, scale=inp['scale'],
                  include_directions=inp.get('include_directions', False),
                  target_mode=inp.get('target_mode', 'wrist_scaled'),
                  robot_rest_chains_m=hand.rest_chains)
    if inp['source'] == 'mediapipe':
        for ts, world, row in iter_mediapipe_jsonl(path, hand_side=inp['hand_side'],
                                                 input_field=inp.get('input_field', 'raw')):
            target = from_mediapipe(world, ts, hand_side=inp['hand_side'],
                                   palm_x_sign=inp['palm_x_sign'], **common)
            target.source_kind = inp.get('source_kind',
                'mediapipe_mapper_replay' if inp.get('input_field') == 'mapper' else 'mediapipe_visual_estimate')
            yield ts, world, row, target
        return
    with np.load(path, allow_pickle=False) as data:
        if not np.array_equal(data['names'], np.asarray(NAMES)):
            raise ValueError('MANUS names/order differ from ingest_manus.NAMES')
        ts, points = data['timestamps'], data['palm_positions_m']
        if ts.ndim != 1 or not len(ts) or points.shape != (len(ts), 25, 3):
            raise ValueError('Invalid normalized MANUS arrays')
        if not np.isfinite(ts).all() or np.any(np.diff(ts) <= 0):
            raise ValueError('MANUS timestamps must be finite and strictly increasing')
        start, end = inp.get('start_s', float(ts[0])), inp.get('end_s', float(ts[-1]))
        if not np.isfinite([start, end]).all() or start > end:
            raise ValueError('Invalid MANUS time window')
        for i in np.flatnonzero((ts >= start) & (ts <= end)):
            raw = points[i].copy()
            canonical = raw.copy(); canonical[:, 0] *= inp['palm_x_sign']
            source = inp['source_kind']
            target = from_manus(canonical, float(ts[i]), source_kind=source, **common)
            yield float(ts[i]), raw, {'source_frame_index': int(i), 'source_kind': source}, target


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--max-frames', type=int, default=0)
    args = parser.parse_args()
    if args.max_frames < 0:
        raise ValueError('max-frames must be nonnegative')
    cfg = json.loads(checked(args.config).read_text())
    path = validate(cfg)
    with run('endpoint_evaluation', {'config': cfg, **vars(args)}) as (out, manifest):
        model_path = prepare_model(cfg.get('model_id', 'right_official_pd_v2'))
        hand = Hand(model_path)
        model = hand.model; data = mujoco.MjData(model)
        data.qpos[:] = hand.q0; data.ctrl[:] = hand.q0
        mujoco.mj_forward(model, data)
        solver = Solver(hand, cfg['solver'], 1/30)
        manifest.update(input={'path': str(path), 'sha256': digest(path),
                               'resampling': 'none; every source frame retained',
                               'snapshot_nonfinite_encoding': 'null; source file hash identifies original bytes'},
                        model={'path': str(model_path), 'sha256': digest(model_path)},
                        mapping='vector_solver.Solver.solve_endpoints', warmup='none',
                        time_convention='q_actual sampled before applying current source-frame q_command; no final extrapolation',
                        simulation_time_origin='first selected source timestamp',
                        contact_validated=False)
        write_json(out/'config.json', cfg)
        values = {k: [] for k in ('times', 'simulation_times', 'target_tips_m', 'target_valid',
                                  'q_command', 'q_actual', 'qvel_actual', 'actual_tips_m', 'latency_s')}
        details = []; last = None; origin = None; count = 0
        snapshot = out/'source_frames.jsonl'
        with snapshot.open('w') as stream:
            for ts, native, source_row, target in frames(path, cfg['input'], hand):
                if args.max_frames and count >= args.max_frames:
                    break
                dt = None if last is None else ts-last
                if not np.isfinite(ts) or (dt is not None and not 0 < dt <= cfg.get('max_gap_s', 1.)):
                    raise ValueError(f'Invalid source time or gap at frame {count}: dt={dt}')
                if origin is None:
                    origin = ts
                    manifest['source_time_origin_s'] = float(origin)
                last = ts
                elapsed = ts-origin
                record = dict(frame=count, timestamp_s=ts, source_kind=target.source_kind,
                              native_points=None if native is None else np.asarray(native).tolist(),
                              source_row=source_row, target_positions_m=target.positions_m.tolist(),
                              target_valid=target.valid.tolist(),
                              target_directions=None if target.directions is None else target.directions.tolist(),
                              direction_valid=None if target.direction_valid is None else target.direction_valid.tolist())
                # Preserve input rows, including missing detections, before solving.
                stream.write(json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False)+'\n'); stream.flush()
                # Advance the preceding interval with its already active command.
                while data.time < elapsed-1e-10:
                    mujoco.mj_step(model, data)
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or np.max(abs(data.qvel)) > 1e4:
                        raise RuntimeError('Unstable dynamics')
                mujoco.mj_forward(model, data)
                before = time.perf_counter()
                command, detail = solver.solve_endpoints(target)
                latency = time.perf_counter()-before
                if command.shape != hand.q0.shape or not np.isfinite(command).all():
                    raise RuntimeError('Invalid endpoint command')
                # q_actual describes arrival at this timestamp; q_command starts here.
                data.ctrl[:] = command
                row = (ts, float(data.time), target.positions_m.copy(), target.valid.copy(),
                       command.copy(), data.qpos.copy(), data.qvel.copy(),
                       data.site_xpos[hand.tip_ids].copy(), latency)
                for key, value in zip(values, row): values[key].append(value)
                details.append(detail); count += 1
                if count % 60 == 0: print(f'endpoint frames: {count}', flush=True)
        if not count: raise ValueError('No source frames selected')
        arrays = {k: np.asarray(v) for k, v in values.items()}
        np.savez_compressed(out/'trajectory.npz', **arrays, joint_names=np.asarray(hand.names))
        write_json(out/'solver_details.json', details)
        valid = arrays['target_valid']
        error = np.linalg.norm(arrays['actual_tips_m']-arrays['target_tips_m'], axis=-1)[valid]*1000
        metrics = dict(frames=count, valid_endpoint_fraction=float(valid.mean()),
                       missing_frames=int(np.sum(~valid.any(axis=1))),
                       actual_tip_mean_mm=float(error.mean()) if error.size else None,
                       actual_tip_p95_mm=float(np.quantile(error, .95)) if error.size else None,
                       solver_attempted_frames=sum(d.get('nfev', 0) > 0 for d in details),
                       solver_skipped_frames=sum(d.get('nfev', 0) == 0 for d in details),
                       solver_not_converged_frames=sum(d.get('nfev', 0) > 0 and not d.get('converged', False) for d in details),
                       solver_status_counts=dict(Counter(str(d.get('status', 'unspecified')) for d in details)),
                       simulation_warning_counts=[int(w.number) for w in data.warning],
                       source_snapshot_sha256=digest(snapshot), contact_validated=False,
                       note='Endpoint fit under explicit fixed scale; not contact or anatomical ground truth.')
        write_json(out/'metrics.json', metrics)
        print(json.dumps(metrics), flush=True)


if __name__ == '__main__':
    main()
