"""Compose selected mapped clips, re-simulate without resets, and render actual states.

The neutral bridges are generated robot commands, not recorded human motion.
No online partner detector or generalization claim is made by this demonstration.
"""
import json
import os
import subprocess
from pathlib import Path
from lab_common import checked, configure, run, write_json, digest

configure()
os.environ['MUJOCO_GL'] = 'egl'
os.environ['EGL_PLATFORM'] = 'surfaceless'
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from command_limiter import CommandLimiter
from pinch_geometry import PadHand
from run_pinch import source_data, contact_state
from render_pinch import Views, caption, png
from render_side_swing import human_points


def main():
    config_dir = Path(__file__).resolve().parents[1] / 'configs'
    index = json.loads((config_dir / 'pinch_selected_runs.json').read_text())
    partners = json.loads((config_dir / 'partner_selected_runs.json').read_text())
    selected = {'index': index['finger_agility_clip_run'],
                **{f: partners[f]['clips'][0] for f in ['middle', 'ring', 'little']}}
    protocol = {'selected': selected, 'fps': 30, 'approach_s': 1.5,
                'settle_s': 1., 'release_s': 1.5, 'neutral_settle_s': 1.,
                'input_kind': 'composite recorded and synthetic mapped clips with generated neutral bridges',
                'continuous_physics': True, 'online_partner_selection': False}
    with run('opposition_sequence', protocol) as (out, mf):
        cases = []; source_skeletons = {}
        for finger, rid in selected.items():
            root = checked('runs/' + rid)
            cfg = json.loads((root/'config.json').read_text())
            tr_path = root/'pad_vector/trajectory.npz'
            tr = np.load(tr_path, allow_pickle=False)
            hand = PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"), cfg)
            data, points, src = source_data(cfg)
            original_manifest=json.loads((root/'manifest.json').read_text())
            if original_manifest['status']!='success' or digest(src)!=original_manifest['source']['sha256']:
                raise ValueError('Selected source rollout or MANUS input hash is invalid')
            if 'source_kind' not in cfg:
                official=checked(f"data/normalized/manus_official_ufbx_v1/{cfg['sequence']}.npz")
                if src!=official:
                    raise ValueError('Unclassified input source; recorded/synthetic provenance required')
                cfg['source_kind']='recorded_manus'
            source_skeletons[finger]=(data['timestamps'].copy(),points.copy(),data['parent'].copy())
            ts = tr['source_clip_times']
            ix = [int(np.argmin(abs(data['timestamps']-t))) for t in ts]
            close = np.linalg.norm(points[ix, 4]-points[ix, hand.source_tip_index], axis=1) <= .015
            cases.append((finger, cfg, hand, tr['mapped_goals'].copy(), ts.copy(), close))
            mf.setdefault('sources', []).append({'finger': finger, 'run_id': rid,
                'config_sha256': digest(root/'config.json'), 'trajectory_sha256': digest(tr_path),
                'normalized_source': str(src), 'source_sha256': digest(src),
                'source_kind': cfg.get('source_kind', 'recorded_manus')})
        model = cases[0][2].model
        assert all(c[1]['model_id'] == cases[0][1]['model_id'] for c in cases)
        d = mujoco.MjData(model)
        mujoco.mj_forward(model, d)
        h0 = cases[0][2]
        limiter = CommandLimiter(h0.q0, h0.lo, h0.hi)
        schedule = []
        for ci, (_, cfg, hand, goals, ts, close) in enumerate(cases):
            for i in range(75):
                u = min(1., i/45)
                blend = 10*u**3-15*u**4+6*u**5
                schedule.append((ci, 'approach', hand.q0+blend*(goals[0]-hand.q0), None, False))
            schedule.extend((ci, 'clip', q, float(t), bool(c)) for q,t,c in zip(goals,ts,close))
            for i in range(75):
                u = min(1., (i+1)/45)
                blend = 10*u**3-15*u**4+6*u**5
                schedule.append((ci, 'release', goals[-1]+blend*(hand.q0-goals[-1]), None, False))
        rows, commands, actual, velocities, frame_info = [], [], [], [], []
        for fi, (ci, phase, goal, source_t, close) in enumerate(schedule):
            finger, cfg, hand, _, _, _ = cases[ci]
            limiter.brake_at_target = cfg.get('command_brake_at_target', False)
            command = limiter.update(goal)
            d.ctrl[:] = command
            while d.time < (fi+1)/30-1e-9:
                mujoco.mj_step(model, d)
                mujoco.mj_forward(model, d)
                if not np.isfinite(d.qpos).all():
                    raise RuntimeError('Non-finite continuous physics state')
                row = contact_state(hand, d, cfg)
                row.update(time_s=float(d.time), finger=finger, phase=phase,
                           source_time_s=source_t, source_close=close)
                rows.append(row)
            all_contacts = [contact_state(c[2], d, c[1])['valid_pad_contact'] for c in cases]
            frame_info.append({'finger': finger, 'phase': phase, 'source_time_s': source_t,
                               'valid_contacts': all_contacts,
                               'source_kind': cfg.get('source_kind', 'recorded_manus')})
            commands.append(command.copy())
            actual.append(d.qpos.copy())
            velocities.append(d.qvel.copy())
        commands, actual = np.array(commands), np.array(actual)
        metrics = {'continuous_physics': True, 'state_resets': 0, 'frames': len(schedule),
                   'duration_s': len(schedule)/30, 'pairs': {},
                   'max_command_speed_rad_s': float(abs(np.diff(commands,axis=0)*30).max()),
                   'max_command_acceleration_rad_s2': float(abs(np.diff(commands,n=2,axis=0)*900).max()),
                   'warnings': [int(w.number) for w in d.warning]}
        for ci, (finger,cfg,hand,_,_,_) in enumerate(cases):
            own = [r for r in rows if r['finger']==finger]
            contact = [r for r in own if r['phase']=='clip' and r['source_close']]
            streak = longest = 0
            for r in own:
                streak = streak+1 if r['phase']=='clip' and r['source_close'] and not r['valid_pad_contact'] else 0
                longest = max(longest, streak)
            final_frames = [r for r in frame_info if r['finger']==finger][-9:]
            values = {'contact_fraction': float(np.mean([r['valid_pad_contact'] for r in contact])) if contact else None,
                      'source_close_s': len(contact)*model.opt.timestep,
                      'longest_break_s': longest*model.opt.timestep,
                      'max_penetration_m': max(r['max_penetration_m'] for r in own),
                      'max_non_target_penetration_m': max(r['non_target_penetration_m'] for r in own),
                      'max_joint_limit_violation_rad': max(r['joint_limit_violation_rad'] for r in own),
                      'released_all_pairs_final_300ms': not any(any(r['valid_contacts']) for r in final_frames)}
            a = cfg['acceptance']
            values['gates'] = {'contact': bool(contact) and values['contact_fraction']>=a['contact_fraction_min'],
                'continuity': values['longest_break_s']<=a['longest_break_s_max'],
                'penetration': values['max_penetration_m']<=a['penetration_m_max'],
                'non_target_penetration': values['max_non_target_penetration_m']<=a['non_target_penetration_m_max'],
                'joint_limits': values['max_joint_limit_violation_rad']<=a['joint_limit_violation_rad_max'],
                'release': values['released_all_pairs_final_300ms']}
            metrics['pairs'][finger] = values
        metrics['passed'] = all(all(v['gates'].values()) for v in metrics['pairs'].values()) and metrics['max_command_speed_rad_s']<=4+1e-8 and metrics['max_command_acceleration_rad_s2']<=20+1e-8 and not any(metrics['warnings'])
        write_json(out/'config.json', protocol)
        write_json(out/'metrics.json', metrics)
        write_json(out/'frames.json', frame_info)
        np.savez_compressed(out/'trajectory.npz', times=(np.arange(len(schedule))+1)/30,
                            q_target=commands, q_actual=actual, qvel_actual=np.array(velocities))
        pq.write_table(pa.Table.from_pylist(rows), out/'contacts.parquet')
        print(json.dumps(metrics), flush=True)
        view = Views(h0, None)
        ffmpeg = checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg')
        video = out/'four_finger_continuous.mp4'
        with (out/'encoder.log').open('w') as log:
            proc = subprocess.Popen([str(ffmpeg), '-y', '-hide_banner', '-f', 'rawvideo',
                '-pixel_format', 'rgb24', '-video_size', '1920x640', '-framerate', '30', '-i', '-',
                '-an', '-c:v', 'libx264', '-threads', '1', '-crf', '18', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(video)], stdin=subprocess.PIPE, stderr=log)
            try:
                saved = set()
                for i, (q, info) in enumerate(zip(actual, frame_info)):
                    canvas = np.full((640,1920,3), [18,24,35], dtype=np.uint8)
                    source_ts,source_ps,source_parent=source_skeletons[info['finger']]
                    case=cases[list(selected).index(info['finger'])]
                    source_t=info['source_time_s']
                    if source_t is None:
                        source_t=float(case[4][0 if info['phase']=='approach' else -1])
                    source_index=int(np.argmin(abs(source_ts-source_t)))
                    view.camera.azimuth=150
                    canvas[80:560,:640]=human_points(view,source_ps[source_index],source_parent)
                    for col, az in enumerate([150,240],1):
                        view.camera.azimuth = az
                        canvas[80:560,col*640:(col+1)*640] = view.robot(q)
                    caption(canvas, 'THUMB / '+info['finger'].upper()+' / '+info['phase'].upper(), 24, 16, scale=3)
                    caption(canvas, 'INPUT SKELETON / ACTUAL DYNAMICS / SECOND VIEW', 24, 52)
                    label = ('RECORDED MANUS' if info['source_kind']=='recorded_manus' else 'SYNTHETIC HUMAN IK') if info['phase']=='clip' else 'GENERATED ROBOT TRANSITION'
                    caption(canvas, label, 24, 582)
                    caption(canvas,f'SOURCE {source_ts[source_index]:.2f} S',1304,582)
                    caption(canvas,'SYNCHRONIZED INPUT' if info['phase']=='clip' else 'HELD REFERENCE - NO HUMAN BRIDGE INPUT',1304,610)
                    caption(canvas, f'TIME {(i+1)/30:.2f} S / 1X', 24, 610)
                    contact = info['valid_contacts'][list(selected).index(info['finger'])]
                    caption(canvas, 'VALID PAD CONTACT' if contact else 'NO VALID PAD CONTACT', 664, 582,
                            (93,226,153) if contact else (248,159,105))
                    caption(canvas, 'SAVED ACTUAL JOINT STATES', 664, 610)
                    proc.stdin.write(canvas.tobytes())
                    if info['phase']=='clip' and contact and info['finger'] not in saved:
                        png(out/(info['finger']+'.png'), canvas)
                        saved.add(info['finger'])
                    if i%150==0: print(f'render {i}/{len(actual)}', flush=True)
                proc.stdin.close()
                if proc.wait()!=0: raise RuntimeError('Encoder failed')
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                view.close()
        with (out/'decode_check.log').open('w') as log:
            subprocess.run([str(ffmpeg), '-v', 'error', '-i', str(video), '-f', 'null', '-'],
                           check=True, stderr=log)
        write_json(out/'video.json', {'frames':len(actual), 'fps':30, 'sha256':digest(video),
                    'encoder_sha256':digest(ffmpeg), 'full_decode_passed':True,'input_landmarks':25,
                    'input_during_generated_bridges':'held endpoint reference explicitly labeled; no recorded bridge input'})
        write_json('reports/latest_opposition_sequence.json', {'run_id':mf['run_id'], 'metrics':metrics})


if __name__ == '__main__':
    main()
