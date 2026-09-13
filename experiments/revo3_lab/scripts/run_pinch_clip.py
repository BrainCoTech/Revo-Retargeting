"""Offline chronological clip evaluation with causal inputs and 1x playback timing."""
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
from run_pinch import source_data,old_targets,contact_state
from command_limiter import CommandLimiter


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--lead-s',type=float,default=0.)
    ap.add_argument('--config')
    ap.add_argument('--velocity-feedforward-s',type=float,default=0.)
    ap.add_argument('--sequence');ap.add_argument('--start-s',type=float);ap.add_argument('--end-s',type=float);ap.add_argument('--pose-s',type=float)
    args=ap.parse_args()
    config_path=checked(args.config) if args.config else Path(__file__).resolve().parents[1]/'configs/pinch_task_v1.json'
    cfg=json.loads(config_path.read_text())
    if args.sequence:cfg['sequence']=args.sequence
    if args.start_s is not None:cfg['source_window_s'][0]=args.start_s
    if args.end_s is not None:cfg['source_window_s'][1]=args.end_s
    if args.pose_s is not None:cfg['source_pose_s']=args.pose_s
    cfg['clip_protocol']={'source_interval_s':cfg['source_window_s'],'warmup_approach_s':1.5,'warmup_settle_s':1.0,
                          'close_source_gap_m':.015,'release_source_gap_m':.020,'fps':30,
                          'closing_lead_s':args.lead_s,'contact_trigger':'closing gap extrapolated using current and previous frames only; release uses current gap',
                          'velocity_feedforward_s':args.velocity_feedforward_s,
                          'feedforward_reason':'shared software PD steady-velocity compensation; nominal (joint damping + actuator kv)/kp = (0.03+0.08)/3 seconds; no hardware identification',
                          'command_filter':'v2 shared causal limiter with discrete braking distance: 4 rad/s, 20 rad/s2',
                          'brake_at_target':cfg.get('command_brake_at_target',False),
                          'evaluation':'proximity-labeled interval; input has no contact-force ground truth; no 2s-hold claim'}
    with run('pinch_clip',cfg) as (out,mf):
        h=PadHand(checked(f"models/revo3/{cfg['model_id']}/scene.xml"),cfg)
        data,points,path=source_data(cfg);old,old_path=old_targets(data,cfg)
        ts=np.arange(cfg['source_window_s'][0],cfg['source_window_s'][1]+1e-8,1/30)
        selected=np.array([np.argmin(abs(data['timestamps']-t)) for t in ts]);ps=points[selected]
        baseline=np.array([old['q_target'][np.argmin(abs(old['timestamps']-t))] for t in ts]);baseline[:,h.inactive_dofs]=0
        goals=[];fits=[];previous=baseline[0];active=False;source_close=[];gaps=[];latency=[]
        for p in ps:
            gap=float(np.linalg.norm(p[4]-p[h.source_tip_index]));gaps.append(gap)
            closing_rate=min(0.,(gap-gaps[-2])*30) if len(gaps)>1 else 0.
            predicted_gap=max(0.,gap+args.lead_s*closing_rate)
            active=(gap<cfg['clip_protocol']['release_source_gap_m']) if active else predicted_gap<cfg['clip_protocol']['close_source_gap_m']
            source_close.append(gap<=.015)
            before=time.perf_counter();q,fit=solve_pads(h,p,previous,cfg,close=active);latency.append(time.perf_counter()-before)
            goals.append(q);fits.append(fit);previous=q
        write_json(out/'config.json',cfg);write_json(out/'pads.json',h.pads);write_json(out/'fits.json',fits)
        mf.update(source={'path':str(path),'sha256':digest(path),'frames':selected.tolist()},baseline={'path':str(old_path),'sha256':digest(old_path)})
        results={}
        for label,targets in [('old_vector',baseline),('pad_vector',np.array(goals))]:
            d=mujoco.MjData(h.model);mujoco.mj_forward(h.model,d)
            limiter=CommandLimiter(h.q0,h.lo,h.hi,brake_at_target=cfg.get('command_brake_at_target',False));rows=[];commands=[];actual=[];vel=[];valids=[];source_times=[]
            duration=2.5+ts[-1]-ts[0]+1/30
            frame_times=np.arange(0,duration-1e-8,1/30)
            for t in frame_times:
                source_t=ts[0]+max(0,t-2.5);idx=min(len(ts)-1,max(0,round((source_t-ts[0])*30)))
                if t<2.5:
                    phase=min(1.,t/1.5);blend=10*phase**3-15*phase**4+6*phase**5
                    desired=h.q0+blend*(targets[0]-h.q0)
                else:
                    slope=np.clip((targets[idx]-targets[max(0,idx-1)])*30,-4,4)
                    desired=np.clip(targets[idx]+args.velocity_feedforward_s*slope,h.lo,h.hi)
                command=limiter.update(desired);d.ctrl[:]=command
                while d.time<t+1/30-1e-9:
                    mujoco.mj_step(h.model,d);mujoco.mj_forward(h.model,d)
                    row=contact_state(h,d,cfg);row.update(time_s=float(d.time),source_time_s=float(source_t),evaluated_clip=t>=2.5-1e-8,
                                                        source_close=bool(source_close[idx]) if t>=2.5-1e-8 else False,source_gap_m=gaps[idx])
                    rows.append(row)
                commands.append(command.copy());actual.append(d.qpos.copy());vel.append(d.qvel.copy());valids.append(rows[-1]['valid_pad_contact']);source_times.append(source_t)
            clip=[r for r in rows if r['evaluated_clip']];contact=[r for r in clip if r['source_close']]
            first_source=next((r['time_s'] for r in clip if r['source_close']),None)
            first_robot=next((r['time_s'] for r in clip if r['valid_pad_contact']),None)
            speeds=np.diff(commands,axis=0)*30;accels=np.diff(speeds,axis=0)*30
            values={'input_close_s':len(contact)*h.model.opt.timestep,
                    'contact_fraction_during_source_close':float(np.mean([r['valid_pad_contact'] for r in contact])) if contact else None,
                    'total_valid_contact_s':sum(r['valid_pad_contact'] for r in clip)*h.model.opt.timestep,
                    'contact_onset_delay_s':None if first_source is None or first_robot is None else first_robot-first_source,
                    'max_penetration_m':max(r['max_penetration_m'] for r in rows),
                    'max_non_target_penetration_m':max(r['non_target_penetration_m'] for r in rows),
                    'max_command_speed_rad_s':float(abs(speeds).max()),'max_command_acceleration_rad_s2':float(abs(accels).max()),
                    'max_joint_limit_violation_rad':max(r['joint_limit_violation_rad'] for r in rows),
                    'warnings':[int(w.number) for w in d.warning],'source_stretched':False,'recorded_2s_hold_available':False}
            values['source_kind']=cfg.get('source_kind','recorded_manus')
            values['final_source_gap_m']=clip[-1]['source_gap_m']
            values['final_valid_contact']=clip[-1]['valid_pad_contact']
            if cfg.get('synthetic_hold_window_s'):
                start,end=cfg['synthetic_hold_window_s'];hold=[r for r in clip if start<=r['source_time_s']<end]
                values['synthetic_hold_s']=len(hold)*h.model.opt.timestep
                values['synthetic_hold_contact_fraction']=float(np.mean([r['valid_pad_contact'] for r in hold]))
            values['target_finger']=h.target_finger
            values['short_clip_contact_fraction_passed']=bool(contact) and values['contact_fraction_during_source_close']>=cfg['acceptance']['contact_fraction_min']
            values['gates']={'contact_fraction':values['short_clip_contact_fraction_passed'],
                            'command_speed':values['max_command_speed_rad_s']<=4+1e-8,
                            'command_acceleration':values['max_command_acceleration_rad_s2']<=20+1e-8,
                            'penetration':values['max_penetration_m']<=cfg['acceptance']['penetration_m_max'],
                            'non_target_penetration':values['max_non_target_penetration_m']<=cfg['acceptance']['non_target_penetration_m_max'],
                            'joint_limits':values['max_joint_limit_violation_rad']<=cfg['acceptance']['joint_limit_violation_rad_max'],
                            'simulation_warnings':not any(values['warnings'])}
            if cfg.get('source_kind')=='synthetic_ik':
                values['gates']['released_at_end']=values['final_source_gap_m']>=cfg['clip_protocol']['release_source_gap_m'] and not values['final_valid_contact']
            values['short_clip_task_passed']=all(values['gates'].values())
            folder=out/label;folder.mkdir()
            np.savez_compressed(folder/'trajectory.npz',times=frame_times+1/30,source_times=np.array(source_times),q_target=np.array(commands),q_actual=np.array(actual),qvel_actual=np.array(vel),frame_valid_contact=np.array(valids),mapped_goals=targets,source_clip_times=ts)
            pq.write_table(pa.Table.from_pylist(rows),folder/'contacts.parquet');write_json(folder/'metrics.json',values)
            results[label]=values;print(label,json.dumps(values),flush=True)
        results['solver_latency_ms']={'mean':float(np.mean(latency)*1000),'p95':float(np.percentile(latency,95)*1000),'max':float(np.max(latency)*1000)}
        write_json(out/'metrics.json',results);write_json('reports/latest_pinch_clip.json',{'run_id':mf['run_id'],'metrics':results})

if __name__=='__main__':main()
