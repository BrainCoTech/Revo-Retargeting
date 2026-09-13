"""Compare saved side-swing rollouts; every case is a separate simulation."""
import argparse
import json
import os
import subprocess
from lab_common import checked,configure,run,write_json,digest
configure()
os.environ['MUJOCO_GL']='egl'
os.environ['EGL_PLATFORM']='surfaceless'
import numpy as np
from revo3_model import Hand
from render_pinch import Views,caption,png,connector,sphere

COLORS=[(1.,.68,.15,1.),(.1,.8,1.,1.),(.35,.9,.45,1.),(.75,.55,1.,1.),(1.,.4,.65,1.)]


def human_points(view,points,parent):
    """Render the exact mapper input, already reflected into its palm frame."""
    width=np.linalg.norm(points[6]-points[21])
    if not np.isfinite(points).all() or width<=1e-8:
        raise ValueError('Input skeleton is invalid; do not fabricate missing landmarks')
    p=(points-points[0])*(view.h.robot_width/width)@view.h.basis.T+view.h.wrist
    view.renderer.update_scene(view.data,camera=view.camera,scene_option=view.opts)
    view.renderer.scene.ngeom=0
    for i,ancestor in enumerate(parent):
        color=(.9,.93,.97,1.) if i==0 else COLORS[0 if i<=4 else 1+(i-5)//5]
        if ancestor>=0:connector(view.renderer.scene,p[ancestor],p[i],color,.0012)
        sphere(view.renderer.scene,p[i],color,.0022)
    return view.renderer.render().copy()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--baseline',required=True);ap.add_argument('--run-id',required=True)
    args=ap.parse_args();base=checked('runs/'+args.baseline);selected=checked('runs/'+args.run_id)
    candidate=json.loads((selected/'config.json').read_text())['candidates'][0]
    for root in [base,selected]:
        if json.loads((root/'manifest.json').read_text())['status']!='success':raise ValueError('Only completed runs can be rendered')
    h=Hand(checked('models/revo3/right_official_pd_v2/scene.xml'));view=Views(h,None);view.camera.azimuth=150
    with run('render_side_swing',vars(args)) as (out,mf):
        source_record=json.loads((selected/'manifest.json').read_text())['sources'][0]
        topology_path=checked(source_record['path'])
        if digest(topology_path)!=source_record['sha256']:raise ValueError('Source topology hash mismatch')
        topology=np.load(topology_path,allow_pickle=False);parent=topology['parent']
        from ingest_manus import NAMES
        np.testing.assert_array_equal(topology['names'],NAMES)
        mf.update(layout='2x2: exact input skeleton / legacy / observability fix / retreat',
                  source_display='25 saved mapper-input landmarks; no additional reflection; palm-width normalized into robot view',
                  topology={'path':str(topology_path),'sha256':digest(topology_path),'names':NAMES,'parent':parent.tolist()},
                  timing='input frame at t and actual robot state after its command period; no additional frame shift')
        ffmpeg=checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg');video=out/'side_swing_with_input.mp4'
        with (out/'encoder.log').open('w') as log:
            proc=subprocess.Popen([str(ffmpeg),'-y','-hide_banner','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x1280',
                '-framerate','30','-i','-','-an','-c:v','libx264','-threads','1','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE,stderr=log)
            count=0
            try:
                for case in ['known_angles','finger_agility','hand_mobility']:
                    input_path=selected/case/'input.npz'
                    source=np.load(input_path,allow_pickle=False)
                    baseline_source=np.load(base/case/'input.npz',allow_pickle=False)
                    np.testing.assert_array_equal(source['times'],baseline_source['times'])
                    np.testing.assert_array_equal(source['palm_positions_m'],baseline_source['palm_positions_m'])
                    if source['palm_positions_m'].shape!=(len(source['times']),25,3):raise ValueError('Unexpected source landmark shape')
                    mf.setdefault('inputs',[]).append({'case':case,'path':str(input_path),'sha256':digest(input_path),
                        'frames':len(source['times']),'source_kind':'synthetic_known_angles' if case=='known_angles' else 'recorded_manus',
                        'identical_across_comparisons':True})
                    trajectories=[]
                    for root,kind in [(base,'legacy'),(base,'observable'),(selected,candidate)]:
                        p=root/case/kind/'trajectory.npz';tr=np.load(p,allow_pickle=False)
                        trajectories.append(tr)
                        mf.setdefault('trajectories',[]).append({'path':str(p),'sha256':digest(p)})
                    for tr in trajectories[1:]:np.testing.assert_array_equal(tr['times'],trajectories[0]['times'])
                    np.testing.assert_array_equal(source['times'],trajectories[0]['times'])
                    for i,t in enumerate(trajectories[0]['times']):
                        canvas=np.full((1280,1280,3),[18,24,35],dtype=np.uint8)
                        panels=[human_points(view,source['palm_positions_m'][i],parent)]+[view.robot(tr['q_actual'][i]) for tr in trajectories]
                        labels=['INPUT SKELETON','LEGACY','OBSERVABILITY FIX',candidate.upper()]
                        for panel,(label,rgb) in enumerate(zip(labels,panels)):
                            x=(panel%2)*640;y=(panel//2)*640
                            canvas[y+80:y+560,x:x+640]=rgb
                            caption(canvas,label,x+24,y+18,scale=3)
                            caption(canvas,case.replace('_',' ').upper(),x+24,y+52)
                            caption(canvas,f'SOURCE {t:.2f} S / '+('25 INPUT POINTS' if panel==0 else 'SAVED DYNAMICS'),x+24,y+582)
                            caption(canvas,'SYNTHETIC ANGLE FIXTURE' if case=='known_angles' else 'RECORDED / NO ANGLE TRUTH',x+24,y+610)
                        proc.stdin.write(canvas.tobytes());count+=1
                        if (case=='known_angles' and i in [15,255,405]) or (case!='known_angles' and i==45):png(out/f'{case}_{i}.png',canvas)
                        if count%150==0:print(f'render {count}/722',flush=True)
                proc.stdin.close()
                if proc.wait()!=0:raise RuntimeError('Encoder failed')
            finally:
                if proc.poll() is None:proc.kill();proc.wait()
                view.close()
        with (out/'decode_check.log').open('w') as log:
            subprocess.run([str(ffmpeg),'-v','error','-i',str(video),'-f','null','-'],check=True,stderr=log)
        write_json(out/'video.json',{'frames':count,'fps':30,'sha256':digest(video),'full_decode_passed':True,
            'width':1280,'height':1280,'input_landmarks':25,'exact_source_frame_alignment_checked':True,
            'case_boundaries_are_cuts':True,'baseline':args.baseline,'comparison':args.run_id})


if __name__=='__main__':main()
