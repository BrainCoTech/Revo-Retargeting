"""Synchronized MANUS / old / pad-vector replay from saved dynamic states."""
import argparse
import json
import os
import subprocess
from pathlib import Path
from lab_common import configure, checked, run, digest, write_json
configure()
os.environ['MUJOCO_GL']='egl'
os.environ['EGL_PLATFORM']='surfaceless'
import mujoco
import numpy as np
from pinch_geometry import PadHand, source_target
from run_pinch import source_data
from render_vector import png

# Dependency-free, legible uppercase captions; pixels do not modify the render.
FONT={
'A':'01110 10001 10001 11111 10001 10001 10001','B':'11110 10001 10001 11110 10001 10001 11110',
'C':'01111 10000 10000 10000 10000 10000 01111','D':'11110 10001 10001 10001 10001 10001 11110',
'E':'11111 10000 10000 11110 10000 10000 11111','F':'11111 10000 10000 11110 10000 10000 10000',
'G':'01111 10000 10000 10111 10001 10001 01111','H':'10001 10001 10001 11111 10001 10001 10001',
'I':'11111 00100 00100 00100 00100 00100 11111','J':'00111 00010 00010 00010 10010 10010 01100',
'K':'10001 10010 10100 11000 10100 10010 10001','L':'10000 10000 10000 10000 10000 10000 11111',
'M':'10001 11011 10101 10101 10001 10001 10001','N':'10001 11001 11001 10101 10011 10011 10001',
'O':'01110 10001 10001 10001 10001 10001 01110','P':'11110 10001 10001 11110 10000 10000 10000',
'Q':'01110 10001 10001 10001 10101 10010 01101','R':'11110 10001 10001 11110 10100 10010 10001',
'S':'01111 10000 10000 01110 00001 00001 11110','T':'11111 00100 00100 00100 00100 00100 00100',
'U':'10001 10001 10001 10001 10001 10001 01110','V':'10001 10001 10001 10001 10001 01010 00100',
'W':'10001 10001 10001 10101 10101 10101 01010','X':'10001 10001 01010 00100 01010 10001 10001',
'Y':'10001 10001 01010 00100 00100 00100 00100','Z':'11111 00001 00010 00100 01000 10000 11111',
'0':'01110 10001 10011 10101 11001 10001 01110','1':'00100 01100 00100 00100 00100 00100 01110',
'2':'01110 10001 00001 00010 00100 01000 11111','3':'11110 00001 00001 01110 00001 00001 11110',
'4':'00010 00110 01010 10010 11111 00010 00010','5':'11111 10000 10000 11110 00001 00001 11110',
'6':'01110 10000 10000 11110 10001 10001 01110','7':'11111 00001 00010 00100 01000 01000 01000',
'8':'01110 10001 10001 01110 10001 10001 01110','9':'01110 10001 10001 01111 00001 00001 01110',
'.':'00000 00000 00000 00000 00000 00110 00110',':':'00000 00100 00100 00000 00100 00100 00000',
'-':'00000 00000 00000 11111 00000 00000 00000','/':'00001 00001 00010 00100 01000 10000 10000',
'%':'11001 11010 00100 00100 01000 10110 00110','=':'00000 00000 11111 00000 11111 00000 00000',
'+':'00000 00100 00100 11111 00100 00100 00000'}


def caption(rgb,text,x,y,color=(226,232,241),scale=2):
    for char in text.upper():
        for row,bits in enumerate(FONT.get(char,'00000 '*7).split()):
            for col,bit in enumerate(bits):
                if bit=='1':rgb[y+row*scale:y+(row+1)*scale,x+col*scale:x+(col+1)*scale]=color
        x+=6*scale


def connector(scene,p1,p2,color,width=.0015):
    g=scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.array([width]*3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
    mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,width,p1,p2)
    scene.ngeom+=1


def sphere(scene,point,color,radius=.002):
    g=scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([radius]*3),point,np.eye(3).ravel(),np.array(color,dtype=np.float32))
    scene.ngeom+=1


class Views:
    def __init__(self,h,source):
        self.h=h;self.model=h.model;self.data=mujoco.MjData(h.model);self.source=source
        self.renderer=mujoco.Renderer(self.model,height=480,width=640)
        self.opts=mujoco.MjvOption();self.opts.geomgroup[3]=0;self.opts.sitegroup[:]=0
        self.model.vis.headlight.diffuse[:]=.8;self.model.vis.headlight.ambient[:]=.3
        self.camera=mujoco.MjvCamera();self.camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:]=[.035,.008,.11];self.camera.distance=.36
        self.camera.azimuth=240;self.camera.elevation=15

    def robot(self,q,overlay=False):
        self.data.qpos[:]=q;mujoco.mj_forward(self.model,self.data)
        self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.opts)
        if overlay:
            for i,b in enumerate(self.h.pad_body_ids):
                rot=self.data.xmat[b].reshape(3,3);pos=self.data.xpos[b]+rot@self.h.pad_positions[i]
                color=[1,.68,.15,1] if i==0 else [.1,.8,1,1]
                sphere(self.renderer.scene,pos,color,.0008)
                connector(self.renderer.scene,pos,pos+.01*(rot@self.h.pad_normals[i]),color,.00035)
        return self.renderer.render().copy()

    def human(self,points):
        width=np.linalg.norm(points[6]-points[21])
        p=points*(self.h.robot_width/width)@self.h.basis.T+self.h.wrist
        self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.opts)
        self.renderer.scene.ngeom=0
        for i,parent in enumerate(self.source['parent']):
            start=5*self.h.target_number
            color=[1,.68,.15,1] if 1<=i<=4 else [.1,.8,1,1] if start<=i<=start+4 else [.58,.65,.75,1]
            if parent>=0:connector(self.renderer.scene,p[parent],p[i],color)
            sphere(self.renderer.scene,p[i],color)
        return self.renderer.render().copy()

    def close(self):self.renderer.close()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run-id',required=True);ap.add_argument('--preview',action='store_true');args=ap.parse_args()
    source=checked(Path('runs')/args.run_id)
    cfg=json.loads((source/'config.json').read_text())
    model_path=checked(f"models/revo3/{cfg['model_id']}/scene.xml")
    h=PadHand(model_path,cfg);data,points,path=source_data(cfg)
    trajectories={k:np.load(source/k/'trajectory.npz',allow_pickle=False) for k in ['old_vector','pad_vector']}
    views=Views(h,data);raw='source_times' in trajectories['pad_vector'].files
    with run('render_pinch',vars(args)) as (out,mf):
        mf.update(source_run=args.run_id,model_sha256=digest(model_path),source_sha256=digest(path),
                  rendering='q_actual from dynamics, original visual meshes; human skeleton only',overlay='orange/cyan markers and rays are pad centers/normals, visualization only')
        frame=len(trajectories['pad_vector']['q_actual'])-1
        pose_frame=int(np.argmin(abs(data['timestamps']-cfg['source_pose_s'])))
        if args.preview:
            for azimuth in [0,60,150,240]:
                views.camera.azimuth=azimuth
                canvas=np.zeros((560,1920,3),dtype=np.uint8);canvas[:]=[18,24,35]
                panels=[views.human(points[pose_frame])]+[views.robot(trajectories[k]['q_actual'][frame],True) for k in trajectories]
                input_label='SYNTHETIC IK' if cfg.get('source_kind')=='synthetic_ik' else 'MANUS'
                for i,(name,panel) in enumerate(zip([f"{input_label} {cfg['source_pose_s']:.2f} S",'OLD VECTOR',h.target_finger.upper()+' PAD VECTOR'],panels)):
                    canvas[60:540,i*640:(i+1)*640]=panel;caption(canvas,name,i*640+24,20,scale=3)
                png(out/f'preview_az{azimuth}.png',canvas)
            views.camera.lookat[:]=h.pad_fk(trajectories['pad_vector']['q_actual'][frame])[0].mean(axis=0);views.camera.distance=.14
            canvas=np.zeros((1120,1280,3),dtype=np.uint8);canvas[:]=[18,24,35]
            for row,azimuth in enumerate([240,60]):
                views.camera.azimuth=azimuth
                for col,key in enumerate(trajectories):
                    canvas[row*560+60:row*560+540,col*640:(col+1)*640]=views.robot(trajectories[key]['q_actual'][frame],False)
                    caption(canvas,key.replace('_',' ')+f' VIEW {row+1}',col*640+24,row*560+20,scale=3)
            png(out/'pad_closeup.png',canvas)
        else:
            ffmpeg=checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg')
            mf['encoder_sha256']=digest(ffmpeg)
            output=out/('raw_clip.mp4' if raw else 'held_pose.mp4')
            log=(out/'encoder.log').open('w')
            proc=subprocess.Popen([str(ffmpeg),'-y','-hide_banner','-f','rawvideo','-pixel_format','rgb24','-video_size','1920x640','-framerate','30','-i','-','-an','-c:v','libx264','-threads','1','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE,stderr=log)
            try:
                for frame,t in enumerate(trajectories['pad_vector']['times']):
                    source_t=float(trajectories['pad_vector']['source_times'][frame]) if raw else cfg['source_pose_s']
                    pidx=int(np.argmin(abs(data['timestamps']-source_t)))
                    canvas=np.zeros((640,1920,3),dtype=np.uint8);canvas[:]=[18,24,35]
                    panels=[views.human(points[pidx])]+[views.robot(trajectories[k]['q_actual'][frame]) for k in trajectories]
                    source_label='SYNTHETIC IK INPUT' if cfg.get('source_kind')=='synthetic_ik' else 'MANUS SOURCE'
                    for i,(name,panel) in enumerate(zip([source_label,'OLD VECTOR',h.target_finger.upper()+' PAD VECTOR'],panels)):
                        canvas[80:560,i*640:(i+1)*640]=panel;caption(canvas,name,i*640+24,18,scale=3)
                    caption(canvas,f'SOURCE {source_t:.2f} S',24,52)
                    label=('SYNTHETIC 1X PLAYBACK' if cfg.get('source_kind')=='synthetic_ik' else 'RAW 1X PLAYBACK') if raw else 'CONDITIONED CLOSE' if cfg.get('explicit_contact_command',False) else f"POSE {cfg['source_pose_s']:.2f} S FROZEN"
                    caption(canvas,label,24,584)
                    caption(canvas,f'ROLLOUT {t:.2f} S',24,614)
                    for i,k in enumerate(trajectories,1):
                        valid=bool(trajectories[k]['frame_valid_contact'][frame]);color=(93,226,153) if valid else (248,159,105)
                        caption(canvas,'VALID PAD CONTACT' if valid else 'NO VALID PAD CONTACT',i*640+24,584,color)
                        caption(canvas,'SAVED DYNAMIC JOINT STATE',i*640+24,614)
                    if not raw:caption(canvas,'HOLD TEST' if t>2.5 else 'APPROACH / SETTLE',664,52)
                    proc.stdin.write(canvas.tobytes())
                    if (not raw and abs(t-3.5)<.018) or (raw and abs(source_t-cfg['source_pose_s'])<.018):png(out/'comparison.png',canvas)
                proc.stdin.close()
                if proc.wait()!=0:raise RuntimeError('Video encoding failed')
            finally:
                if proc.poll() is None:proc.kill();proc.wait()
                log.close()
            subprocess.run([str(ffmpeg),'-hide_banner','-i',str(output),'-f','null','-'],check=True,stdout=subprocess.DEVNULL,stderr=(out/'decode_check.log').open('w'))
            write_json(out/'video.json',{'frames':len(trajectories['pad_vector']['times']),'fps':30,'source':args.run_id,'kind':'raw_clip' if raw else 'held_input','sha256':digest(output)})
        print(str(out),flush=True)
    views.close()

if __name__=='__main__':main()
