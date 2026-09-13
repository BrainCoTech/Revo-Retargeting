"""Lateral mapping ablations on known-angle fixtures and recorded stress clips."""
import json
import argparse
import os
import subprocess
from pathlib import Path
from lab_common import checked, configure, run, write_json, digest
configure()
os.environ['MUJOCO_GL']='egl'
os.environ['EGL_PLATFORM']='surfaceless'
import mujoco
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from revo3_model import Hand
from vector_solver import finger_angles
from command_limiter import CommandLimiter
from side_swing import SIDE_DOFS, SideMapper, lateral_observation, project_flexion_clearance, retreat_clearance
from render_pinch import Views, caption, png
from render_side_swing import human_points


def synthetic(template):
    """Fixed segment lengths; labelled mathematical fixture, not human recording."""
    ts=np.arange(0,18,1/30);points=[];truth=[];flexion=[]
    rng=np.random.default_rng(0)
    for t in ts:
        stage=min(int(t//4),3)
        levels=[0.,60.,120.,90.]
        phase=(t%4)/.75
        u=np.clip(phase,0,1);blend=10*u**3-15*u**4+6*u**5
        flex=np.radians((levels[max(0,stage-1)]*(1-blend)+levels[stage]*blend) if stage else 0.)
        side=np.radians(12)*np.sin(np.pi*t) if t<16 else 0.
        if t>=16: flex=np.radians(90)*max(0.,1-(t-16))
        p=template.copy()
        for k in range(4):
            j=5+5*k
            lengths=[np.linalg.norm(template[j+i+1]-template[j+i]) for i in range(4)]
            p[j]=p[j+1]-np.array([0.,0.,lengths[0]])
            direction=np.array([np.sin(flex),-np.sin(side)*np.cos(flex),np.cos(side)*np.cos(flex)])
            if 12.75<=t<16:
                direction+=rng.normal(0,.0005,3)
                direction/=np.linalg.norm(direction)
            for n in range(1,4):p[j+n+1]=p[j+n]+lengths[n]*direction
        points.append(p);truth.append([side]*4);flexion.append(flex)
    return ts,np.array(points),np.array(truth),np.array(flexion)


def numeric_checks():
    points=np.zeros((25,3));results=[]
    for flex in [0,45,70,110,120]:
        for side in [-12,0,12]:
            p=points.copy()
            a,b=np.radians([side,flex])
            for k in range(4):
                j=5+5*k;p[j+1]=[0,0,.03]
                p[j+2]=p[j+1]+.04*np.array([np.sin(b),-np.sin(a)*np.cos(b),np.cos(a)*np.cos(b)])
            _,angle,conf=lateral_observation(p)
            np.testing.assert_allclose(angle,a,atol=1e-12)
            assert np.all(conf>.3)
            results.append({'side_deg':side,'flex_deg':flex,'max_error_rad':float(abs(angle-a).max())})
    mapper=SideMapper(np.full(4,-.2618),np.full(4,.2618))
    mapper.update(p)
    missing=np.full((25,3),np.nan)
    for _ in range(9):q,diag=mapper.update(missing)
    assert np.all(q==0) and not diag['observable'].any()
    return {'passed':True,'known_angle_cases':results,'missing_after_timeout_returns_neutral':True}


def rollout(h,points,kind):
    mapper=SideMapper(h.lo[SIDE_DOFS],h.hi[SIDE_DOFS],'observable' if kind in ['collision','retreat'] else kind)
    limiter=CommandLimiter(h.q0,h.lo,h.hi,brake_at_target=True)
    d=mujoco.MjData(h.model);mujoco.mj_forward(h.model,d)
    target=[];command=[];actual=[];diagnostics=[];rows=[]
    for fi,p in enumerate(points):
        q=finger_angles(p,h);q[:5]=h.q0[:5]
        side,diag=mapper.update(p);q[SIDE_DOFS]=side
        if kind=='collision':
            q,projection=project_flexion_clearance(h,q);diag['projection']=projection
        elif kind=='retreat':
            q,projection=retreat_clearance(h,q);diag['projection']=projection
        cmd=limiter.update(q);d.ctrl[:]=cmd
        while d.time<(fi+1)/30-1e-9:
            mujoco.mj_step(h.model,d);mujoco.mj_forward(h.model,d)
            if not np.isfinite(d.qpos).all():raise RuntimeError('Nonfinite dynamics')
            deepest=max(d.contact,key=lambda c:-c.dist,default=None)
            rows.append({'time_s':float(d.time),'penetration_m':max(0.,-float(deepest.dist)) if deepest is not None else 0.,
                'geom1':h.model.geom(int(deepest.geom1)).name if deepest is not None else None,
                'geom2':h.model.geom(int(deepest.geom2)).name if deepest is not None else None,
                'joint_limit_violation_rad':float(max(0.,np.max(h.lo-d.qpos),np.max(d.qpos-h.hi)))})
        target.append(q);command.append(cmd);actual.append(d.qpos.copy());diagnostics.append(diag)
    return np.array(target),np.array(command),np.array(actual),diagnostics,rows,[int(w.number) for w in d.warning]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--candidate',action='append',choices=['legacy','wrapped','observable','collision','retreat'])
    parser.add_argument('--no-render',action='store_true')
    args=parser.parse_args()
    model=checked('models/revo3/right_official_pd_v2/scene.xml')
    h=Hand(model)
    protocol={'source_palm_x_sign':-1,'model_id':'right_official_pd_v2','fps':30,
        'candidates':args.candidate or ['legacy','wrapped','observable'],'observability_enter':.30,'observability_exit':.20,
        'missing_timeout_s':.25,'known_observable_error_max_deg':.1,'singular_target_step_max_deg':1.,
        'penetration_max_m':.001,'joint_limit_violation_max_rad':.02,'max_command_speed_rad_s':4.,
        'max_command_acceleration_rad_s2':20.,'scope':'isolated four-finger side mapping; thumb neutral; not hardware capability testing',
        'real_clips':'diagnostic stress windows selected for large projected-angle variation; no lateral-angle truth'}
    with run('side_swing',protocol) as (out,mf):
        mf['model']={'path':str(model),'sha256':digest(model)}
        checks=numeric_checks();write_json(out/'checks.json',checks)
        sources=[];parent=None
        for seq,window in [('finger_agility',[24.,27.]),('hand_mobility',[18.,21.])]:
            path=checked('data/normalized/manus_official_ufbx_v1/'+seq+'.npz')
            data=np.load(path,allow_pickle=False);p=data['palm_positions_m'].copy();p[:,:,0]*=-1
            if parent is None:parent=data['parent'].copy()
            else:np.testing.assert_array_equal(parent,data['parent'])
            ts=np.arange(window[0],window[1]+1e-8,1/30)
            ids=[int(np.argmin(abs(data['timestamps']-t))) for t in ts]
            sources.append((seq,ts,p[ids],None,None))
            mf.setdefault('sources',[]).append({'sequence':seq,'path':str(path),'sha256':digest(path),'frames':ids})
            if seq=='finger_agility':
                syn=synthetic(p[0]);sources.insert(0,('known_angles',*syn))
                mf['synthetic_template']={'path':str(path),'sha256':digest(path),'frame':0,
                    'construction':'fixed segment lengths; idealized 2-axis finger rotation and seeded noise at 90-degree flexion',
                    'human_anatomical_validity':False}
        results={};render_cases=[]
        for name,ts,points,truth,flex in sources:
            folder=out/name;folder.mkdir();np.savez_compressed(folder/'input.npz',times=ts,palm_positions_m=points,
                side_truth_rad=truth if truth is not None else np.empty((0,4)),flex_truth_rad=flex if flex is not None else np.empty(0))
            videos={};results[name]={}
            for kind in protocol['candidates']:
                target,command,actual,diags,rows,warnings=rollout(h,points,kind)
                dest=folder/kind;dest.mkdir()
                conf=np.array([x['confidence'] for x in diags]);raw=np.array([x['raw'] for x in diags])
                m={'source_kind':'synthetic_known_angles' if truth is not None else 'recorded_manus',
                    'lateral_ground_truth_available':truth is not None,
                    'lateral_tracking_rmse_deg':float(np.degrees(np.sqrt(np.mean((actual[:,SIDE_DOFS]-command[:,SIDE_DOFS])**2)))),
                    'max_lateral_target_step_deg':float(np.degrees(abs(np.diff(target[:,SIDE_DOFS],axis=0)).max())),
                    'command_at_lateral_limit_fraction':float(np.mean(abs(command[:,SIDE_DOFS])>=.2618-1e-5)),
                    'source_projection_unobservable_fraction':float(np.mean(conf<.2)),
                    'raw_source_outside_robot_range_fraction':float(np.mean(abs(raw)>.2618)),
                    'max_penetration_m':max(r['penetration_m'] for r in rows),
                    'max_joint_limit_violation_rad':max(r['joint_limit_violation_rad'] for r in rows),
                    'max_command_speed_rad_s':float(abs(np.diff(command,axis=0)*30).max()),
                    'max_command_acceleration_rad_s2':float(abs(np.diff(command,n=2,axis=0)*900).max()),'warnings':warnings}
                m['gates']={'penetration':m['max_penetration_m']<=.001,'joint_limits':m['max_joint_limit_violation_rad']<=.02,
                    'speed':m['max_command_speed_rad_s']<=4+1e-8,'acceleration':m['max_command_acceleration_rad_s2']<=20+1e-8,'warnings':not any(warnings)}
                if truth is not None:
                    # Hysteresis recovery band is reported separately; oracle
                    # comparison uses comfortably observable configurations.
                    mask=abs(np.cos(flex))>=.35
                    m['observable_mapping_max_error_deg']=float(np.degrees(abs(target[mask][:,SIDE_DOFS]-truth[mask]).max()))
                    singular=(ts>=13)&(ts<15.9)
                    m['singular_max_target_step_deg']=float(np.degrees(abs(np.diff(target[singular][:,SIDE_DOFS],axis=0)).max()))
                    m['gates']['observable_mapping']=m['observable_mapping_max_error_deg']<=.1
                    m['gates']['singular_stability']=m['singular_max_target_step_deg']<=1.
                    lengths=[]
                    for k in range(4):
                        j=5+5*k
                        lengths.extend([np.ptp(np.linalg.norm(points[:,j+n+1]-points[:,j+n],axis=1)) for n in range(4)])
                    m['synthetic_max_bone_length_change_m']=float(max(lengths))
                    assert max(lengths)<1e-12
                if kind in ['collision','retreat']:
                    m['max_flexion_correction_deg']=max(x['projection']['max_flexion_correction_deg'] for x in diags)
                    m['projection_converged_fraction']=float(np.mean([x['projection']['converged'] for x in diags]))
                    write_json(dest/'projection.json',[x['projection'] for x in diags])
                if kind=='retreat':
                    m['max_side_correction_deg']=max(x['projection']['max_side_correction_deg'] for x in diags)
                    m['retreat_frame_fraction']=float(np.mean([x['projection']['flex_scale']<1 or x['projection']['side_scale']<1 for x in diags]))
                m['software_gates_passed']=all(m['gates'].values())
                results[name][kind]=m;videos[kind]=actual
                np.savez_compressed(dest/'trajectory.npz',times=ts,q_goal=target,q_target=command,q_actual=actual,source_confidence=conf)
                pq.write_table(pa.Table.from_pylist(rows),dest/'contacts.parquet');write_json(dest/'metrics.json',m)
                print(name,kind,json.dumps(m),flush=True)
            render_cases.append((name,ts,videos,points))
        write_json(out/'config.json',protocol);write_json(out/'metrics.json',results)
        if args.no_render:
            write_json('reports/latest_side_swing.json',{'run_id':mf['run_id'],'metrics':results})
            return
        view=Views(h,None);view.camera.azimuth=150
        panel_rows=(len(protocol['candidates'])+2)//2
        ffmpeg=checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg');video=out/'side_swing_comparison.mp4'
        with (out/'encoder.log').open('w') as log:
            proc=subprocess.Popen([str(ffmpeg),'-y','-hide_banner','-f','rawvideo','-pixel_format','rgb24',
                '-video_size',f'1280x{640*panel_rows}','-framerate','30','-i','-','-an','-c:v','libx264','-threads','1',
                '-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE,stderr=log)
            count=0
            try:
                for name,ts,videos,points in render_cases:
                    for i,t in enumerate(ts):
                        canvas=np.full((640*panel_rows,1280,3),[18,24,35],dtype=np.uint8)
                        panels=[human_points(view,points[i],parent)]+[view.robot(videos[kind][i]) for kind in protocol['candidates']]
                        for panel,(label,rgb) in enumerate(zip(['INPUT SKELETON']+protocol['candidates'],panels)):
                            x=(panel%2)*640;y=(panel//2)*640
                            canvas[y+80:y+560,x:x+640]=rgb
                            caption(canvas,label.upper(),x+24,y+20,scale=3)
                            caption(canvas,name.replace('_',' ').upper(),x+24,y+52)
                            caption(canvas,f'SOURCE {t:.2f} S / 1X',x+24,y+582)
                            caption(canvas,'SYNTHETIC ANGLE FIXTURE' if name=='known_angles' else 'RECORDED / NO ANGLE TRUTH',x+24,y+610)
                        proc.stdin.write(canvas.tobytes());count+=1
                        if name=='known_angles' and i in [60,180,300,420]:png(out/f'frame_{i}.png',canvas)
                        if count%150==0:print(f'render {count}',flush=True)
                proc.stdin.close()
                if proc.wait()!=0:raise RuntimeError('Encoder failed')
            finally:
                if proc.poll() is None:proc.kill();proc.wait()
                view.close()
        with (out/'decode_check.log').open('w') as log:
            subprocess.run([str(ffmpeg),'-v','error','-i',str(video),'-f','null','-'],check=True,stderr=log)
        write_json(out/'video.json',{'frames':count,'fps':30,'sha256':digest(video),'full_decode_passed':True,
            'input_landmarks':25,'width':1280,'height':640*panel_rows,'input_frame_alignment':'same saved input frame as robot command'})
        write_json('reports/latest_side_swing.json',{'run_id':mf['run_id'],'metrics':results})


if __name__=='__main__':main()
