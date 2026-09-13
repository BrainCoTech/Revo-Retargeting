"""MANUS pad-opposition mapping and explicit held-input task, evaluated by mj_step."""
import argparse
import json
from pathlib import Path
import time
from lab_common import checked,configure,run,write_json,digest
configure()
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from pinch_geometry import PadHand,solve_pads


def source_data(config):
    path=checked(config.get('normalized_source',f"data/normalized/manus_official_ufbx_v1/{config['sequence']}.npz"))
    data=np.load(path,allow_pickle=False)
    points=data['palm_positions_m'].copy();points[:,:,0]*=config['source_palm_x_sign']
    return data,points,path


def old_targets(data,config):
    path=checked(config['legacy_baseline_npz']) if 'legacy_baseline_npz' in config else checked('runs/20260913T084011Z_vector_c5ab0b5e/vector_balanced')/config['sequence']/'trajectories.npz'
    old=np.load(path,allow_pickle=False)
    return old,path


def contact_state(hand,d,cfg):
    m=hand.model;acc=cfg['acceptance']
    ns=[d.xmat[b].reshape(3,3)@n for b,n in zip(hand.pad_body_ids,hand.pad_normals)]
    pp=[d.xpos[b]+d.xmat[b].reshape(3,3)@p for b,p in zip(hand.pad_body_ids,hand.pad_positions)]
    angle=float(np.degrees(np.arccos(np.clip(-ns[0]@ns[1],-1,1))))
    force=np.zeros(6);normal_force=0.;gap=1.;valid=False;pen=0.;non_pen=0.;point=np.full(3,np.nan)
    closest_patch=False;pair_contact=False
    for i,c in enumerate(d.contact):
        penetration=max(0.,-float(c.dist));pen=max(pen,penetration)
        pair={int(c.geom1),int(c.geom2)}==set(hand.pad_geom_ids)
        if not pair:non_pen=max(non_pen,penetration);continue
        pair_contact=True;gap=min(gap,float(c.dist))
        mujoco.mj_contactForce(m,d,i,force)
        inside=hand.point_in_pad('thumb',c.pos,d) and hand.point_in_pad(hand.target_finger,c.pos,d)
        direction=c.frame[:3] if int(c.geom1)==hand.pad_geom_ids[0] else -c.frame[:3]
        oriented=(ns[0]@direction>=np.cos(np.radians(acc['contact_normal_cone_deg'])) and
                  ns[1]@-direction>=np.cos(np.radians(acc['contact_normal_cone_deg'])))
        if inside:closest_patch=True
        if inside and oriented and c.dist<=acc['gap_m_max'] and angle<=acc['normal_angle_deg_max']:
            normal_force+=max(0.,float(force[0]));point=c.pos.copy()
    valid=normal_force>=acc['normal_force_n_min']
    limit=max(0.,float(np.max(hand.lo-d.qpos)),float(np.max(d.qpos-hand.hi)))
    return {'valid_pad_contact':bool(valid),'distal_pair_contact':bool(pair_contact),'contact_inside_both_patches':bool(closest_patch),
            'normal_force_n':normal_force,'contact_gap_m':gap,'pad_center_gap_m':float(np.linalg.norm(pp[0]-pp[1])),
            'normal_angle_deg':angle,'max_penetration_m':pen,'non_target_penetration_m':non_pen,
            'joint_limit_violation_rad':limit,'contact_point_x':None if not valid else float(point[0]),
            'contact_point_y':None if not valid else float(point[1]),'contact_point_z':None if not valid else float(point[2])}


def rollout(hand,goal,config,folder):
    p=config['protocol'];m=hand.model;d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    duration=p['approach_s']+p['settle_s']+p['hold_s']
    frames=np.arange(0,duration-1e-9,1/p['fps'])
    commands=[];actual=[];states=[];vel=[];all_rows=[]
    for i,t in enumerate(frames):
        phase=min(1.,t/p['approach_s']);blend=10*phase**3-15*phase**4+6*phase**5
        q=hand.q0+blend*(goal-hand.q0);d.ctrl[:]=q
        while d.time<t+1/p['fps']-1e-9:
            mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
            if not np.isfinite(d.qpos).all():raise RuntimeError('Non-finite physics state')
            row=contact_state(hand,d,config);row['time_s']=float(d.time)
            row['evaluated_hold']=p['approach_s']+p['settle_s']<d.time<=duration+1e-8
            all_rows.append(row)
        commands.append(q);actual.append(d.qpos.copy());vel.append(d.qvel.copy());states.append(all_rows[-1])
    commands=np.array(commands);actual=np.array(actual)
    speed=np.diff(commands,axis=0)*p['fps'];accel=np.diff(speed,axis=0)*p['fps']
    hold=[r for r in all_rows if r['evaluated_hold']]
    longest=0;streak=0
    for row in hold:
        streak=0 if row['valid_pad_contact'] else streak+1;longest=max(longest,streak)
    a=config['acceptance']
    values={'contact_fraction':sum(r['valid_pad_contact'] for r in hold)/len(hold),
            'evaluated_hold_s':len(hold)*m.opt.timestep,'longest_break_s':longest*m.opt.timestep,
            'max_hold_normal_angle_deg':max(r['normal_angle_deg'] for r in hold),
            'max_hold_penetration_m':max(r['max_penetration_m'] for r in hold),
            'max_hold_non_target_penetration_m':max(r['non_target_penetration_m'] for r in hold),
            'max_rollout_penetration_m':max(r['max_penetration_m'] for r in all_rows),
            'max_rollout_non_target_penetration_m':max(r['non_target_penetration_m'] for r in all_rows),
            'max_joint_limit_violation_rad':max(r['joint_limit_violation_rad'] for r in all_rows),
            'mean_hold_force_n':float(np.mean([r['normal_force_n'] for r in hold])),
            'max_hold_force_n':max(r['normal_force_n'] for r in hold),
            'max_command_speed_rad_s':float(abs(speed).max()),'max_command_acceleration_rad_s2':float(abs(accel).max()),
            'warnings':[int(w.number) for w in d.warning], 'hold_tracking_rmse_rad':float(np.sqrt(np.mean((actual[frames>=p['approach_s']+p['settle_s']]-commands[frames>=p['approach_s']+p['settle_s']])**2)))}
    gates={'hold_duration':values['evaluated_hold_s']>=a['required_hold_s']-1e-8,
           'contact_fraction':values['contact_fraction']>=a['contact_fraction_min'],
           'contact_continuity':values['longest_break_s']<=a['longest_break_s_max'],
           'penetration':values['max_rollout_penetration_m']<=a['penetration_m_max'],
           'non_target_penetration':values['max_rollout_non_target_penetration_m']<=a['non_target_penetration_m_max'],
           'joint_limits':values['max_joint_limit_violation_rad']<=a['joint_limit_violation_rad_max'],
           'command_speed':values['max_command_speed_rad_s']<=p['max_command_speed_rad_s'],
           'command_acceleration':values['max_command_acceleration_rad_s2']<=p['max_command_acceleration_rad_s2'],
           'simulation_warnings':not any(values['warnings'])}
    values.update(gates=gates,held_input_task_passed=all(gates.values()),target_finger=hand.target_finger,
                  label=config.get('evaluation_label','explicitly frozen MANUS pose; not a real recorded 2-second hold and not a hardware capability test'))
    folder.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(folder/'trajectory.npz',times=frames+1/p['fps'],q_target=commands,q_actual=actual,qvel_actual=np.array(vel),
                        goal=goal,frame_valid_contact=np.array([r['valid_pad_contact'] for r in states]))
    pq.write_table(pa.Table.from_pylist(all_rows),folder/'contacts.parquet')
    write_json(folder/'metrics.json',values)
    return values


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',default='repos/Revo-Retargeting/experiments/revo3_lab/configs/pinch_task_v1.json');args=p.parse_args()
    cfg=json.loads(checked(args.config).read_text())
    with run('pinch_hold',cfg) as (out,manifest):
        model=checked(f"models/revo3/{cfg['model_id']}/scene.xml")
        h=PadHand(model,cfg);data,points,source=source_data(cfg);old,old_path=old_targets(data,cfg)
        frame=int(np.argmin(abs(data['timestamps']-cfg['source_pose_s'])))
        old_frame=int(np.argmin(abs(old['timestamps']-cfg['source_pose_s'])))
        prior=old['q_target'][old_frame].copy();prior[h.inactive_dofs]=0
        before=time.perf_counter();goal,fit=solve_pads(h,points[frame],prior,cfg)
        fit['wall_s']=time.perf_counter()-before
        write_json(out/'pads.json',h.pads);write_json(out/'fit.json',fit);write_json(out/'config.json',cfg)
        manifest.update(source={'path':str(source),'sha256':digest(source),'frame':frame,'time_s':float(data['timestamps'][frame]),'contact_label':cfg['source_annotation']},
                        model={'path':str(model),'sha256':digest(model)},baseline={'path':str(old_path),'sha256':digest(old_path)},
                        target_finger=h.target_finger,source_annotation=cfg['source_annotation'])
        print('PADS',json.dumps(h.pads),flush=True);print('FIT',json.dumps(fit),flush=True)
        results={}
        for label,q in [('old_vector',prior),('pad_vector',goal)]:
            results[label]=rollout(h,q,cfg,out/label)
            print(label,json.dumps(results[label]),flush=True)
        write_json(out/'metrics.json',results)
        write_json('reports/latest_pinch.json',{'run_id':manifest['run_id'],'metrics':results,'fit':fit})

if __name__=='__main__':main()
