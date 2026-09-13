"""Serial, manifested software comparison. Never runs a hardware capability test."""
import argparse
import json
import re
from collections import Counter
from pathlib import Path
import time
from lab_common import checked, configure, digest, run, write_json
configure()
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from ingest_manus import NAMES, FINGERS
from revo3_model import Hand, prepare_model
from vector_solver import Solver


def resample(data, fps, max_frames):
    times = data['timestamps']
    ts = np.arange(times[0],times[-1]+1e-9,1/fps)
    if max_frames:
        ts = ts[:max_frames]
    p = data['palm_positions_m']
    points = np.stack([np.interp(ts,times,p[:,i,j]) for i in range(len(NAMES)) for j in range(3)],axis=1)
    return ts-ts[0],points.reshape(-1,len(NAMES),3)


def dynamic_eval(hand, qs, targets, times):
    m,d = hand.model,mujoco.MjData(hand.model)
    d.qpos[:] = qs[0]
    d.ctrl[:] = qs[0]
    mujoco.mj_forward(m,d)
    for _ in range(100):
        mujoco.mj_step(m,d)
    d.time = 0.
    actual, actual_tips, contacts = [],[],[]
    max_penetration = 0.; non_target_steps = 0; any_steps = 0
    target_steps = 0; physics_steps = 0; slip = 0.; max_force = 0.
    pair_counts = Counter()
    force = np.zeros(6)
    j1,j2,jr = [np.zeros((3,21)) for _ in range(3)]
    for frame,(q,t) in enumerate(zip(qs,times)):
        # At frame i report the state after holding this command for one input period.
        d.ctrl[:] = q
        end = t + (times[1]-times[0])
        frame_pair = False
        frame_force = 0.; frame_pen = 0.; frame_slip = 0.; frame_non = False
        while d.time < end-1e-10:
            mujoco.mj_step(m,d)
            if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all() or np.max(np.abs(d.qvel)) > 1e4:
                raise RuntimeError('Unstable numerical integration')
            physics_steps += 1
            pair_contact = False; non_target = False
            any_contact = False
            for i in range(d.ncon):
                c = d.contact[i]
                pen = max(0.,-float(c.dist))
                frame_pen = max(frame_pen,pen); max_penetration = max(max_penetration,pen)
                mujoco.mj_contactForce(m,d,i,force)
                if c.dist > 0 or force[0] <= 0:
                    continue
                any_contact = True
                pair_name = '|'.join(sorted([m.geom(int(c.geom1)).name, m.geom(int(c.geom2)).name]))
                pair_counts[pair_name] += 1
                fingers = {hand.distal_geoms.get(int(c.geom1),-1),hand.distal_geoms.get(int(c.geom2),-1)}
                if fingers == {0,1}:
                    pair_contact = True; frame_pair = True
                    max_force=max(max_force,float(force[0])); frame_force=max(frame_force,float(force[0]))
                    mujoco.mj_jac(m,d,j1,jr,c.pos,int(m.geom_bodyid[c.geom1]))
                    mujoco.mj_jac(m,d,j2,jr,c.pos,int(m.geom_bodyid[c.geom2]))
                    relative = (j1-j2)@d.qvel
                    normal = c.frame[:3]
                    tangential = relative-normal*(relative@normal)
                    # Multiple simultaneous contacts: average point speed within this step below.
                    frame_slip += np.linalg.norm(tangential)*m.opt.timestep
                else:
                    non_target = True; frame_non = True
            target_steps += pair_contact; non_target_steps += non_target; any_steps += any_contact
        # Sum over contact points is diagnostic only; no sliding task success is inferred.
        slip += frame_slip
        contacts.append({'frame':frame,'time_s':float(t),'thumb_index_distal_contact':frame_pair,
                         'max_normal_force_n':frame_force,'max_penetration_m':frame_pen,
                         'contact_point_slip_sum_m':frame_slip,'non_target_contact':frame_non})
        mujoco.mj_kinematics(m,d)  # Align saved sites with post-step qpos, not the pre-integration cache.
        actual.append(d.qpos.copy()); actual_tips.append(d.site_xpos[hand.tip_ids].copy())
    actual,actual_tips=np.array(actual),np.array(actual_tips)
    return actual,actual_tips,contacts,{
        'physics_steps':physics_steps,'max_penetration_mm':1000*max_penetration,
        'thumb_index_distal_contact_fraction':target_steps/max(physics_steps,1),
        'any_contact_fraction':any_steps/max(physics_steps,1),'non_target_contact_fraction':non_target_steps/max(physics_steps,1),
        'max_thumb_index_normal_force_n':max_force,'contact_point_slip_sum_mm':slip*1000,
        'tracking_rmse_rad':float(np.sqrt(np.mean((actual-qs)**2))),
        'tracking_max_rad':float(np.max(np.abs(actual-qs))),
        'joint_limit_violation_max_rad':float(max(0.,np.max(hand.lo-actual),np.max(actual-hand.hi))),
        'simulation_warning_counts': [int(w.number) for w in d.warning],
        'contact_pair_counts': dict(pair_counts.most_common())}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--max-frames',type=int,default=0)
    parser.add_argument('--fps',type=int,default=30)
    args=parser.parse_args()
    config_path=checked(args.config)
    cfg=json.loads(config_path.read_text())
    if any(not re.fullmatch(r'[a-z0-9_]+',c['name']) for c in cfg['candidates']):
        raise ValueError('Invalid candidate name')
    if len({c['name'] for c in cfg['candidates']}) != len(cfg['candidates']):
        raise ValueError('Duplicate candidate name')
    if len(cfg['candidates'])>12:
        raise ValueError('At most 12 candidates per batch')
    with run('vector',{'config':cfg,'fps':args.fps,'max_frames':args.max_frames}) as (out,manifest):
        model_path=prepare_model(cfg.get('model_id','right_official_pd_v1'))
        hand=Hand(model_path)
        split_path=checked('data/splits/manus_official_v1.json')
        split=json.loads(split_path.read_text())
        manifest.update(model={'path':str(model_path),'sha256':digest(model_path)},
                        split={'path':str(split_path),'sha256':digest(split_path)},
                        scope='software vector integration/tuning; unlabelled MANUS samples; hardware test skipped; final test absent')
        write_json(out/'config.json',cfg)
        summaries=[]; start=time.perf_counter()
        for candidate in cfg['candidates']:
            for split_name in ['tune','validation']:
                for sequence in split[split_name]:
                    if time.perf_counter()-start > cfg['batch_wall_limit_s']:
                        raise TimeoutError('Batch budget exhausted')
                    data_path=checked('data/normalized')/split['dataset']/(sequence+'.npz')
                    if digest(data_path)!=split['sequence_hashes'][sequence]:
                        raise RuntimeError('Frozen input checksum mismatch')
                    data=np.load(data_path,allow_pickle=False)
                    ts,points=resample(data,args.fps,args.max_frames)
                    # Explicit frame adaptation fixed in config; never infer source side identity.
                    points[:,:,0]*=cfg['source_palm_x_sign']
                    human_width=float(np.median(np.linalg.norm(points[:,NAMES.index('Index_MCP')]-points[:,NAMES.index('Pinky_MCP')],axis=1)))
                    scale=hand.robot_width/human_width
                    targets=points[:,[NAMES.index(f+'_TIP') for f in FINGERS]]*scale
                    targets=targets@hand.basis.T+hand.wrist
                    solver=Solver(hand,{**cfg['common'],**candidate},1/args.fps)
                    qs=[]; tips=[]; latencies=[]; details=[]
                    for i,(p,target) in enumerate(zip(points,targets)):
                        before=time.perf_counter()
                        q,detail=solver.solve(p,target)
                        latencies.append(time.perf_counter()-before)
                        qs.append(q); tips.append(hand.fk(q)[0]); details.append(detail)
                        if i%300==0:
                            print(candidate['name'],sequence,'frame',i,'/',len(ts),flush=True)
                        if time.perf_counter()-start > cfg['batch_wall_limit_s']:
                            raise TimeoutError('Batch budget exhausted during optimization')
                    qs,tips=np.array(qs),np.array(tips)
                    actual,actual_tips,contacts,dynamics=dynamic_eval(hand,qs,targets,ts)
                    folder=out/candidate['name']/sequence
                    folder.mkdir(parents=True)
                    np.savez_compressed(folder/'trajectories.npz',timestamps=ts,actual_timestamps=ts+1/args.fps,q_target=qs,q_actual=actual,
                                        target_tips_m=targets,fk_tips_m=tips,actual_tips_m=actual_tips,
                                        input_palm_m=points,latency_s=np.array(latencies),joint_names=np.array(hand.names))
                    pq.write_table(pa.Table.from_pylist(contacts),folder/'contacts.parquet')
                    tip_error=np.linalg.norm(tips-targets,axis=2)*1000
                    pair_error=np.linalg.norm((tips[:,1:]-tips[:,:1])-(targets[:,1:]-targets[:,:1]),axis=2)*1000
                    speeds=np.diff(qs,axis=0)*args.fps
                    acceleration=np.diff(speeds,axis=0)*args.fps
                    metrics={'candidate':candidate['name'],'sequence':sequence,'split':split_name,'frames':len(ts),
                             'source_sha256':digest(data_path),'scale':scale,'tip_error_mean_mm':float(tip_error.mean()),
                             'tip_error_p95_mm':float(np.quantile(tip_error,.95)),
                             'pair_error_mean_mm':float(pair_error.mean()),'pair_error_per_finger_mm':pair_error.mean(axis=0).tolist(),
                             'actual_tip_error_mean_mm':float(np.linalg.norm(actual_tips-targets,axis=2).mean()*1000),
                             'command_speed_max_rad_s':float(np.abs(speeds).max()),
                             'command_acceleration_max_rad_s2':float(np.abs(acceleration).max()),
                             'latency_median_ms':float(np.median(latencies)*1000),'latency_p95_ms':float(np.quantile(latencies,.95)*1000),
                             'optimization_wall_s':float(sum(latencies)),
                             'solver_not_converged_frames':sum(not d['converged'] for d in details),
                             'close_proxy_frames':sum(d['close_proxy'] for d in details),
                             'contact_task_acceptance':'not_evaluated_unlabelled_input_and_unverified_pad_region',**dynamics}
                    write_json(folder/'metrics.json',metrics)
                    summaries.append(metrics)
                    write_json(out/'metrics.json',summaries)
                    print(json.dumps(metrics),flush=True)
        scores={c['name']:float(np.mean([s['tip_error_mean_mm']+s['pair_error_mean_mm'] for s in summaries
                                       if s['candidate']==c['name'] and s['split']=='tune'])) for c in cfg['candidates']}
        best=min(scores,key=scores.get)
        selection={'best_geometry_candidate':best,'selection_split':'tune','score_definition':'mean tip error + mean relative tip vector error, mm',
                   'scores':scores,'hardware_capability_test':'skipped_by_user',
                   'task_acceptance':'not_evaluated','final_test':'not_available',
                   'wall_s':time.perf_counter()-start}
        write_json(out/'selection.json',selection)
        write_json('reports/latest_vector.json',{'run_id':manifest['run_id'],'selection':selection,'metrics':summaries})
        manifest['selection']=selection


if __name__=='__main__':
    main()
