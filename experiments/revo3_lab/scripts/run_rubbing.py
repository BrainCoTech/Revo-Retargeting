"""Frozen MANUS opposition plus synthetic index-longitudinal rubbing targets.

Surface slip is measured from velocities of both bodies at the SAME contact
point, not from migrating contact points or fingertip displacement.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path
from lab_common import checked, configure, run, write_json, digest
configure()
if '--render-run' in sys.argv:
    os.environ['MUJOCO_GL'] = 'egl'
    os.environ['EGL_PLATFORM'] = 'surfaceless'
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from pinch_geometry import PadHand, solve_pads
from run_pinch import source_data, old_targets, contact_state
from command_limiter import CommandLimiter


def relative_velocity(model, data, bodies, point):
    jac = []
    for body in bodies:
        jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, jr, point, body)
        jac.append(jp)
    return (jac[0]-jac[1]) @ data.qvel


def surface_slip(hand, data, cfg):
    """Force-weighted velocity over valid patch contacts; no contact => zero."""
    m = hand.model
    ns = [data.xmat[b].reshape(3, 3) @ n for b, n in zip(hand.pad_body_ids, hand.pad_normals)]
    angle = np.degrees(np.arccos(np.clip(-ns[0] @ ns[1], -1, 1)))
    acc = cfg['acceptance']
    if angle > acc['normal_angle_deg_max']:
        return 0., 0., 0.
    axis = data.xmat[hand.pad_body_ids[1]].reshape(3, 3) @ hand.pad_axis[1]
    weight = signed = magnitude = transverse = 0.
    for i, c in enumerate(data.contact):
        if {int(c.geom1), int(c.geom2)} != set(hand.pad_geom_ids):
            continue
        if c.dist > acc['gap_m_max'] or not all(hand.point_in_pad(f, c.pos, data) for f in hand.pad_fingers):
            continue
        normal = c.frame[:3] if int(c.geom1) == hand.pad_geom_ids[0] else -c.frame[:3]
        cone = np.cos(np.radians(acc['contact_normal_cone_deg']))
        if ns[0] @ normal < cone or ns[1] @ -normal < cone:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(m, data, i, force)
        w = max(0., float(force[0]))
        tangent = axis-normal*(axis @ normal)
        length = np.linalg.norm(tangent)
        if length < 1e-8:
            continue
        tangent /= length
        v = relative_velocity(m, data, hand.pad_body_ids, c.pos)
        vt = v-normal*(v @ normal)
        s = float(vt @ tangent)
        signed += w*s
        magnitude += w*np.linalg.norm(vt)
        transverse += w*np.linalg.norm(vt-s*tangent)
        weight += w
    return (signed/weight, magnitude/weight, transverse/weight) if weight >= acc['normal_force_n_min'] else (0., 0., 0.)


def longest_break(rows, dt):
    longest = streak = 0
    for row in rows:
        streak = 0 if row['valid_pad_contact'] else streak+1
        longest = max(longest, streak)
    return longest*dt


def evaluate(hand, goals, offsets, cfg, folder, input_points=None, posture=None, parent=None):
    p = cfg['rubbing']; fps = cfg['protocol']['fps']; dt = hand.model.opt.timestep
    start = p['approach_s']+p['settle_s']; rub_start = start+p['hold_s']
    end = rub_start+p['period_s']*p['cycles']
    times = np.arange(0, end-1e-9, 1/fps)
    data = mujoco.MjData(hand.model); mujoco.mj_forward(hand.model, data)
    limiter = CommandLimiter(hand.q0, hand.lo, hand.hi)
    commands = []; actual = []; velocities = []; rows = []; targets = []; input_indices = []
    for t in times:
        idx = max(0, min(len(goals)-1, round((t-rub_start)*fps)))
        desired = goals[idx].copy()
        if t < p['approach_s']:
            x = t/p['approach_s']; blend = 10*x**3-15*x**4+6*x**5
            desired = hand.q0+blend*(goals[0]-hand.q0)
        command = limiter.update(desired); data.ctrl[:] = command
        while data.time < t+1/fps-1e-9:
            mujoco.mj_step(hand.model, data); mujoco.mj_forward(hand.model, data)
            if not np.isfinite(data.qpos).all():
                raise RuntimeError('Nonfinite physics')
            row = contact_state(hand, data, cfg)
            slip, absolute, transverse = surface_slip(hand, data, cfg)
            row.update(time_s=float(data.time), phase='approach' if t < start-1e-8 else 'hold' if t < rub_start-1e-8 else 'rub',
                       signed_slip_m_s=float(slip), slip_speed_m_s=float(absolute), transverse_slip_m_s=float(transverse),
                       target_offset_m=float(offsets[idx]))
            rows.append(row)
        commands.append(command); actual.append(data.qpos.copy()); velocities.append(data.qvel.copy()); targets.append(desired)
        input_indices.append(idx)
    hold = [r for r in rows if r['phase']=='hold']; rub = [r for r in rows if r['phase']=='rub']
    cycles = []
    for k in range(p['cycles']):
        rr = [r for r in rub if rub_start+k*p['period_s'] < r['time_s'] <= rub_start+(k+1)*p['period_s']+1e-9]
        signed = np.array([r['signed_slip_m_s'] for r in rr])*dt
        integrated = np.r_[0., np.cumsum(signed)]
        cycles.append({'cycle':k+1, 'contact_fraction':float(np.mean([r['valid_pad_contact'] for r in rr])),
                       'longest_break_s':longest_break(rr, dt),
                       'positive_slip_mm':float(signed[signed>0].sum()*1000),
                       'negative_slip_mm':float(-signed[signed<0].sum()*1000),
                       'signed_excursion_mm':float(np.ptp(integrated)*1000)})
    command_speed = np.diff(np.vstack([hand.q0, commands]), axis=0)*fps
    acceleration = np.diff(np.vstack([np.zeros(21), command_speed]), axis=0)*fps
    a = cfg['acceptance']
    metrics = {'hold_contact_fraction':float(np.mean([r['valid_pad_contact'] for r in hold])),
               'rub_contact_fraction':float(np.mean([r['valid_pad_contact'] for r in rub])),
               'hold_longest_break_s':longest_break(hold, dt), 'rub_longest_break_s':longest_break(rub, dt), 'cycles':cycles,
               'absolute_surface_slip_mm':sum(r['slip_speed_m_s'] for r in rub)*dt*1000,
               'transverse_surface_slip_mm':sum(r['transverse_slip_m_s'] for r in rub)*dt*1000,
               'max_penetration_mm':max(r['max_penetration_m'] for r in rows)*1000,
               'max_non_target_penetration_mm':max(r['non_target_penetration_m'] for r in rows)*1000,
               'max_joint_limit_violation_rad':max(r['joint_limit_violation_rad'] for r in rows),
               'max_command_speed_rad_s':float(abs(command_speed).max()),
               'max_command_acceleration_rad_s2':float(abs(acceleration).max()),
               'max_actual_speed_rad_s':float(np.max(np.abs(velocities))),
               'rub_force_n_mean':float(np.mean([r['normal_force_n'] for r in rub])),
               'rub_force_n_max':max(r['normal_force_n'] for r in rub),
               'warnings':[int(w.number) for w in data.warning]}
    metrics['gates'] = {
        'contact':min(metrics['hold_contact_fraction'], metrics['rub_contact_fraction'])>=a['contact_fraction_min'],
        'continuity':max(metrics['hold_longest_break_s'], metrics['rub_longest_break_s'])<=a['longest_break_s_max'],
        'penetration':metrics['max_penetration_mm']<=1000*a['penetration_m_max'],
        'non_target_collision':metrics['max_non_target_penetration_mm']<=1000*a['non_target_penetration_m_max'],
        'joint_limits':metrics['max_joint_limit_violation_rad']<=a['joint_limit_violation_rad_max'],
        'command_speed':metrics['max_command_speed_rad_s']<=cfg['protocol']['max_command_speed_rad_s']+1e-8,
        'command_acceleration':metrics['max_command_acceleration_rad_s2']<=cfg['protocol']['max_command_acceleration_rad_s2']+1e-8,
        'simulation_warnings':not any(metrics['warnings'])}
    metrics['contact_motion_constraints_passed'] = all(metrics['gates'].values())
    rub_frames=times>=rub_start-1e-9
    q_rub=np.asarray(actual)[rub_frames]
    metrics['joint_excursion_deg']={hand.model.joint(j).name:float(np.degrees(np.ptp(q_rub[:,j]))) for j in range(21)}
    if posture is not None:
        dofs=cfg['solver']['rubbing_posture_dofs']
        ref=posture[np.asarray(input_indices)[rub_frames]][:,dofs]
        metrics['distal_tracking_rmse_deg']=np.degrees(np.sqrt(np.mean((q_rub[:,dofs]-ref)**2,axis=0))).tolist()
        metrics['distal_tracking_correlation']=[float(np.corrcoef(q_rub[:,j],ref[:,k])[0,1])
            if np.std(q_rub[:,j])>1e-10 and np.std(ref[:,k])>1e-10 else None for k,j in enumerate(dofs)]
        for k,cycle in enumerate(cycles):
            # These start-of-control-frame bounds select saved end states in
            # (cycle_start, cycle_end], matching the physics-step cycle above.
            selected=(times>=rub_start+k*p['period_s']-1e-9)&(times<rub_start+(k+1)*p['period_s']-1e-9)
            cycle['distal_excursion_deg']=np.degrees(np.ptp(np.asarray(actual)[selected][:,dofs],axis=0)).tolist()
        metrics['articulation_gate']=bool(min(min(c['distal_excursion_deg']) for c in cycles)>=cfg['rubbing']['min_distal_excursion_deg'])
        metrics['articulated_contact_constraints_passed']=metrics['articulation_gate'] and metrics['contact_motion_constraints_passed']
    metrics['rubbing_task_accepted'] = False
    metrics['scope'] = cfg['scope']+'; no real rubbing-input or teacher-label acceptance.'
    folder.mkdir()
    extras={}
    if input_points is not None:
        extras=dict(input_points=input_points[input_indices],input_indices=input_indices,parent=parent,
                    input_time_s=np.asarray(input_indices)/fps)
    if posture is not None: extras['posture_target']=posture[input_indices]
    np.savez_compressed(folder/'trajectory.npz', times=times+1/fps, q_target=commands, q_actual=actual, qvel_actual=velocities, mapped_goals=targets, **extras)
    pq.write_table(pa.Table.from_pylist(rows), folder/'contacts.parquet')
    write_json(folder/'metrics.json', metrics)
    return metrics


def render_saved(run_id):
    """Render saved actual dynamics, never use IK targets as executed motion."""
    import subprocess
    from render_pinch import Views, caption, png
    from render_side_swing import human_points
    source = checked('runs')/run_id
    original_manifest=json.loads((source/'manifest.json').read_text())
    if original_manifest['status']!='success': raise ValueError('Only completed dynamics can be rendered')
    cfg = json.loads((source/'config.json').read_text())
    h = PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"), cfg)
    data, source_points, source_path = source_data(cfg)
    if digest(source_path)!=original_manifest['source']['sha256']:
        raise ValueError('Original MANUS source hash mismatch')
    trajectory = np.load(source/'pad_slide/trajectory.npz', allow_pickle=False)
    trajectory_hash=digest(source/'pad_slide/trajectory.npz')
    if trajectory_hash!=original_manifest['outputs_sha256']['pad_slide/trajectory.npz']:
        raise ValueError('Saved trajectory hash mismatch')
    pose_frame=int(original_manifest['source']['frame'])
    articulated='curl_deg' in cfg['rubbing']
    if articulated:
        if not {'input_points','input_indices','input_time_s','parent'}.issubset(trajectory.files):
            raise ValueError('Articulated replay requires exact saved input, not a frozen fallback')
        if digest(source/'input.npz')!=original_manifest['input_sha256']:
            raise ValueError('Saved synthetic input hash mismatch')
        recorded_input=np.load(source/'input.npz',allow_pickle=False)
        np.testing.assert_array_equal(trajectory['input_points'],recorded_input['palm_positions_m'][trajectory['input_indices']])
        np.testing.assert_array_equal(trajectory['parent'],recorded_input['parent'])
    saved_input=trajectory['input_points'] if 'input_points' in trajectory.files else np.tile(source_points[pose_frame],(len(trajectory['times']),1,1))
    parent=trajectory['parent'] if 'parent' in trajectory.files else data['parent']
    if len(saved_input)!=len(trajectory['times']): raise ValueError('Input/actual frame count mismatch')
    rows = pq.read_table(source/'pad_slide/contacts.parquet').to_pydict()
    sample_times = np.asarray(rows['time_s'])
    views = Views(h, data)
    ffmpeg = checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg')
    with run('render_rubbing', {'source_run':run_id}) as (out, mf):
        mf.update(source_run=run_id, rendering='saved q_actual and exact mapper input; synthetic task excitation',
            trajectory_sha256=trajectory_hash,source_sha256=digest(source_path),
            source_seed_frame=pose_frame,source_seed_time_s=float(data['timestamps'][pose_frame]),
            synthetic_input_sha256=original_manifest.get('input_sha256'),encoder_sha256=digest(ffmpeg),
            input_frame_alignment='input command frame paired with actual state at end of that control interval',
            source_kind=cfg['rubbing']['source_kind'])
        output = out/'rubbing_with_input.mp4'
        with (out/'encoder.log').open('w') as log:
            proc = subprocess.Popen([str(ffmpeg),'-y','-hide_banner','-f','rawvideo','-pixel_format','rgb24',
                '-video_size','1920x640','-framerate','30','-i','-','-an','-c:v','libx264','-threads','1',
                '-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)], stdin=subprocess.PIPE, stderr=log)
            try:
                for i, t in enumerate(trajectory['times']):
                    j = int(np.argmin(abs(sample_times-t)))
                    canvas = np.full((640,1920,3), [18,24,35], dtype=np.uint8)
                    views.camera.lookat[:] = [.035,.008,.11]; views.camera.distance = .36
                    views.camera.azimuth = 240; views.camera.elevation = 15
                    canvas[80:560,:640] = human_points(views,saved_input[i],parent)
                    canvas[80:560,640:1280] = views.robot(trajectory['q_actual'][i])
                    pos,_,_ = h.pad_fk(trajectory['q_actual'][i])
                    views.camera.lookat[:] = pos.mean(axis=0); views.camera.distance = .12
                    views.camera.azimuth = 150; views.camera.elevation = 5
                    canvas[80:560,1280:] = views.robot(trajectory['q_actual'][i])
                    for x,label in zip([24,664,1304], ['MANUS-TOPOLOGY INPUT','THUMB-INDEX RUBBING','PAD CLOSEUP']):
                        caption(canvas,label,x,18,scale=3)
                    caption(canvas,'SYNTHETIC BONE-PRESERVING MOTION' if articulated else 'FROZEN RECORDED MANUS POSE',24,52)
                    caption(canvas,'SYNTHETIC SLIDE TARGET / ACTUAL DYNAMICS',664,52)
                    caption(canvas,f'TIME {t:.2f} S  '+rows['phase'][j],24,584)
                    input_t=float(trajectory['input_time_s'][i]) if articulated else float(data['timestamps'][pose_frame])
                    caption(canvas,f'SYNTHETIC {input_t:.2f} S / 1X' if articulated else f'HELD SOURCE POSE {input_t:.2f} S',24,614)
                    contact = rows['valid_pad_contact'][j]
                    caption(canvas,'VALID PAD CONTACT' if contact else 'NO VALID PAD CONTACT',664,584,
                            (93,226,153) if contact else (248,159,105))
                    caption(canvas,f'TARGET {rows["target_offset_m"][j]*1000:+.2f} MM',664,614)
                    caption(canvas,f'CONTACT SLIP {rows["signed_slip_m_s"][j]*1000:+.2f} MM/S',1304,584)
                    distal=np.degrees(trajectory['q_actual'][i,[3,4,7,8]])
                    caption(canvas,'TH PIP/DIP '+f'{distal[0]:.1f}/{distal[1]:.1f}  IX '+f'{distal[2]:.1f}/{distal[3]:.1f}',1304,614)
                    proc.stdin.write(canvas.tobytes())
                    if abs(t-7.)<.017: png(out/'overview_t07.png',canvas)
                    rub_start=sum(cfg['rubbing'][k] for k in ['approach_s','settle_s','hold_s'])
                    for fraction in [0.,.25,.5,.75]:
                        if abs(t-(rub_start+fraction*cfg['rubbing']['period_s']+1/30))<.017:
                            png(out/f'phase_{round(100*fraction):03d}.png',canvas)
                proc.stdin.close()
                if proc.wait()!=0: raise RuntimeError('Video encoding failed')
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                views.close()
        with (out/'decode_check.log').open('w') as log:
            subprocess.run([str(ffmpeg),'-hide_banner','-i',str(output),'-f','null','-'],check=True,stdout=subprocess.DEVNULL,stderr=log)
        write_json(out/'video.json', {'frames':len(trajectory['times']),'fps':30,'sha256':digest(output),
            'input_landmarks':25,'source_kind':cfg['rubbing']['source_kind'],'full_decode_passed':True,'encoder_sha256':digest(ffmpeg)})
        print(str(out),flush=True)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--amplitude-mm', type=float, default=2.)
    ap.add_argument('--preload-mm', type=float, default=1.8); ap.add_argument('--period-s', type=float, default=2.)
    ap.add_argument('--timestep-scale', type=float, default=1.); ap.add_argument('--friction-scale', type=float, default=1.)
    ap.add_argument('--render-run')
    ap.add_argument('--articulated',action='store_true')
    ap.add_argument('--curl-deg',type=float,default=15.)
    ap.add_argument('--posture-weight',type=float,default=3.)
    ap.add_argument('--neutral-deg',type=float,nargs=4,help='TH PIP/DIP, IX PIP/DIP reference; defaults depend on curl pattern')
    ap.add_argument('--pad-scale-mm',type=float,default=2.)
    ap.add_argument('--normal-scale',type=float,default=.2)
    ap.add_argument('--curl-pattern',choices=['parallel','counter'],default='parallel')
    ap.add_argument('--dip-weight-multiplier',type=float,default=1.)
    args = ap.parse_args()
    if args.render_run:
        render_saved(args.render_run)
        return
    from rubbing_input import default_neutral
    if args.neutral_deg is None: args.neutral_deg=default_neutral(args.curl_pattern)
    if args.amplitude_mm < 0 or min(args.preload_mm, args.period_s, args.timestep_scale, args.friction_scale)<=0:
        ap.error('Amplitude must be nonnegative; other scales must be positive')
    if min(args.pad_scale_mm,args.normal_scale,args.dip_weight_multiplier)<=0: ap.error('Residual scales and DIP multiplier must be positive')
    if not np.isfinite([args.amplitude_mm,args.preload_mm,args.period_s,args.timestep_scale,args.friction_scale,args.curl_deg,args.posture_weight,args.pad_scale_mm,args.normal_scale,args.dip_weight_multiplier,*args.neutral_deg]).all():
        ap.error('All parameters must be finite')
    cfg = json.loads((Path(__file__).resolve().parents[1]/'configs/pinch_task_v1.json').read_text())
    cfg['solver']['preload_m'] = args.preload_mm/1000
    cfg['solver'].update(pad_position_scale_m=args.pad_scale_mm/1000,opposed_normal_scale=args.normal_scale)
    cfg['rubbing'] = dict(amplitude_m=args.amplitude_mm/1000, period_s=args.period_s, cycles=3,
                          approach_s=1.5, settle_s=1., hold_s=2., direction='index pad longitudinal tangent',
                          source_kind='frozen_recorded_pose_plus_synthetic_task_space_sine',
                          other_fingers='secondary: neutral commands, collision monitored',
                          timestep_scale=args.timestep_scale, friction_scale=args.friction_scale)
    cfg['scope'] = 'First rubbing optimization; generated task excitation, not recorded human rubbing'
    if args.articulated:
        if args.curl_deg<=0 or args.posture_weight<0: ap.error('Curl must be positive and posture weight nonnegative')
        cfg['solver'].update(rubbing_posture_dofs=[3,4,7,8],
            rubbing_posture_weight=(args.posture_weight*np.array([1.,args.dip_weight_multiplier,1.,args.dip_weight_multiplier])).tolist())
        cfg['rubbing'].update(source_kind='bone_preserving_synthetic_MANUS_topology',curl_deg=args.curl_deg,
            neutral_deg=args.neutral_deg,min_distal_excursion_deg=10.,curl_pattern=args.curl_pattern)
        cfg['scope']='Articulated synthetic stress input plus explicit task-space slide; no recorded human rubbing claim'
    with run('rubbing', cfg) as (out, mf):
        hand = PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"), cfg)
        hand.model.opt.timestep *= args.timestep_scale
        hand.model.geom_friction[:] *= args.friction_scale
        data, points, path = source_data(cfg); old, old_path = old_targets(data, cfg)
        frame = int(np.argmin(abs(data['timestamps']-cfg['source_pose_s'])))
        prior = old['q_target'][np.argmin(abs(old['timestamps']-cfg['source_pose_s']))].copy()
        prior[hand.inactive_dofs] = 0
        goal, fit = solve_pads(hand, points[frame], prior, cfg)
        mf.update(source={'path':str(path), 'sha256':digest(path), 'frame':frame}, baseline={'path':str(old_path),'sha256':digest(old_path)})
        write_json(out/'config.json', cfg); write_json(out/'pads.json', hand.pads)
        # Solve each frame chronologically; retain optimizer diagnostics, including failures.
        ts = np.arange(0, args.period_s*3-1e-9, 1/30)
        offsets = args.amplitude_mm/1000*np.sin(2*np.pi*ts/args.period_s)
        input_points=np.tile(points[frame],(len(ts),1,1));posture=None
        if args.articulated:
            from rubbing_input import articulated_input,posture_targets
            input_points,deltas,details=articulated_input(points[frame],data['parent'],ts,args.period_s,args.curl_deg,args.curl_pattern)
            posture=posture_targets(hand,goal,deltas,args.neutral_deg)
            write_json(out/'input_synthesis.json',details)
            np.savez_compressed(out/'input.npz',times=ts,palm_positions_m=input_points,parent=data['parent'],
                names=data['names'],synthetic_joint_deltas=deltas,posture_target=posture,
                slide_target_m=offsets,source_frame=frame,source_time_s=data['timestamps'][frame])
            mf['input_sha256']=digest(out/'input.npz')
        goals = []; fits = []; latencies = []; previous = goal
        for i, offset in enumerate(offsets):
            before = time.perf_counter()
            q, fit = solve_pads(hand, input_points[i], previous, cfg, slide_m=float(offset),
                                posture_target=None if posture is None else posture[i])
            latencies.append(time.perf_counter()-before); goals.append(q); fits.append(fit); previous = q
            if i%60==0: print('IK', i, len(offsets), flush=True)
        write_json(out/'fits.json', fits)
        metrics = evaluate(hand, np.array(goals), offsets, cfg, out/'pad_slide', input_points, posture,data['parent'])
        metrics['solver_latency_ms_p95'] = float(np.percentile(latencies,95)*1000)
        metrics['solver_failure_count'] = sum(not f['success'] for f in fits)
        write_json(out/'metrics.json', metrics)
        write_json('reports/latest_rubbing.json', {'run_id':mf['run_id'], 'metrics':metrics})
        print(json.dumps(metrics), flush=True)


if __name__=='__main__':
    main()
