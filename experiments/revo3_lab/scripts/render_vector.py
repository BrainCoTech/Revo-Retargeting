"""Render saved dynamic rollout states with existing EGL; no new simulation or hardware test."""
import argparse
import json
import os
from pathlib import Path
import struct
import zlib
from lab_common import checked, configure, run, write_json
configure()
os.environ['MUJOCO_GL']='egl'
os.environ['EGL_PLATFORM']='surfaceless'
import mujoco
import numpy as np


def png(path, rgb):
    height,width,_=rgb.shape
    def chunk(kind,payload):
        return struct.pack('!I',len(payload))+kind+payload+struct.pack('!I',zlib.crc32(kind+payload)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in rgb)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);p.add_argument('--candidate',required=True)
    p.add_argument('--sequence',default='hand_mobility');args=p.parse_args()
    source=checked(Path('runs')/args.run_id)
    manifest=json.loads((source/'manifest.json').read_text())
    if manifest['status']!='success':
        raise RuntimeError('Only completed rollouts can be rendered')
    archive=checked(source/args.candidate/args.sequence/'trajectories.npz')
    trajectory=np.load(archive,allow_pickle=False)
    with run('render_vector',vars(args)) as (out,record):
        model=mujoco.MjModel.from_xml_path(manifest['model']['path'])
        data=mujoco.MjData(model)
        camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:]=[.03,0,.11];camera.distance=.44;camera.azimuth=150;camera.elevation=10
        model.vis.headlight.diffuse[:]=[.8,.8,.8]
        model.vis.headlight.ambient[:]=[.3,.3,.3]
        opts=mujoco.MjvOption();opts.geomgroup[3]=0;opts.sitegroup[:]=0
        # Two views for fixed source times; q_actual came from mj_step, rendering only resets poses.
        frames=[int(np.argmin(abs(trajectory['timestamps']-t))) for t in [2,8,15,25]]
        with mujoco.Renderer(model,height=480,width=640) as renderer:
            for frame in frames:
                data.qpos[:]=trajectory['q_actual'][frame]
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=camera,scene_option=opts)
                png(out/f'frame_{frame:04d}.png',renderer.render())
        record['source_rollout']=str(archive)
        record['frames']=[{'index':i,'source_time_s':float(trajectory['timestamps'][i])} for i in frames]
        print(out)


if __name__=='__main__':
    main()
