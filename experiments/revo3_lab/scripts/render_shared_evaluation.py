"""Render aligned exact inputs and two public-solver dynamic rollouts."""
import argparse
import json
import os
import re
import subprocess
from lab_common import checked, configure, digest, run, write_json
configure()
os.environ['MUJOCO_GL'] = 'egl'
os.environ['EGL_PLATFORM'] = 'surfaceless'
import numpy as np
from revo3_model import Hand
from render_pinch import Views, caption, png, connector, sphere
from render_side_swing import COLORS


def human_points(view, points, parent, scale):
    """Same fixed palm-width scale and world transform used by the public solver."""
    p = points*scale@view.h.basis.T+view.h.wrist
    view.renderer.update_scene(view.data, camera=view.camera, scene_option=view.opts)
    view.renderer.scene.ngeom = 0
    for i, ancestor in enumerate(parent):
        color = (.9, .93, .97, 1.) if i == 0 else COLORS[0 if i <= 4 else 1+(i-5)//5]
        if ancestor >= 0: connector(view.renderer.scene, p[ancestor], p[i], color, .0012)
        sphere(view.renderer.scene, p[i], color, .0022)
    return view.renderer.render().copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True); ap.add_argument('--candidate', required=True)
    ap.add_argument('--baseline', default='legacy'); ap.add_argument('--case', required=True)
    ap.add_argument('--close-view', action='store_true', help='Fixed common camera focused on thumb/index tips')
    args = ap.parse_args()
    for name in [args.run_id, args.candidate, args.baseline, args.case]:
        if not re.fullmatch('[A-Za-z0-9_]+', name): raise ValueError('Invalid identifier')
    source_run = checked('runs/'+args.run_id)
    source_manifest = json.loads((source_run/'manifest.json').read_text())
    if source_manifest['status'] != 'success': raise ValueError('Render only completed evaluations')
    input_path = source_run/'inputs'/args.case/'input.npz'
    record = json.loads(input_path.with_name('source.json').read_text())
    if digest(input_path) != record['exact_input_sha256']: raise ValueError('Input checksum changed')
    source = np.load(input_path, allow_pickle=False)
    trajectories = []
    paths = []
    for name in [args.baseline, args.candidate]:
        folder = source_run/name/args.case
        metrics = json.loads((folder/'metrics.json').read_text())
        if metrics['exact_input_sha256'] != digest(input_path): raise ValueError('Candidate inputs differ')
        path = folder/'trajectory.npz'; paths.append(path)
        tr = np.load(path, allow_pickle=False)
        np.testing.assert_array_equal(tr['times'], source['times'])
        np.testing.assert_array_equal(tr['input_palm_m'], source['palm_positions_m'])
        np.testing.assert_allclose(tr['actual_times'], tr['times']+1/30, atol=1e-10)
        if len(tr['q_actual']) != len(source['times']): raise ValueError('Frame count mismatch')
        trajectories.append(tr)
    model = checked(source_manifest['model']['path'])
    if digest(model) != source_manifest['model']['sha256']: raise ValueError('Model checksum changed')
    with run('render_shared', vars(args)) as (out, manifest):
        manifest.update(input={'path': str(input_path), 'sha256': digest(input_path)},
            trajectories=[{'path': str(p), 'sha256': digest(p)} for p in paths],
            source_kind=record['source_kind'], layout='input skeleton / baseline actual / candidate actual',
            alignment='source frame t paired with actual state after its command interval; no index shift',
            reflection='input already reflected; no second reflection', warmup_not_shown_s=2.5)
        hand = Hand(model); view = Views(hand, None); view.camera.azimuth = 150
        if args.close_view:
            # Symmetric, fixed camera from both saved actual trajectories; never per-panel tracking.
            view.camera.lookat[:] = np.concatenate([tr['actual_tips_m'][:, :2].reshape(-1, 3)
                                                    for tr in trajectories], axis=0).mean(axis=0)
            view.camera.distance = .22
            view.camera.azimuth = 240
        manifest['camera'] = {'close_view': args.close_view, 'lookat': view.camera.lookat.tolist(),
                              'distance': float(view.camera.distance), 'azimuth': float(view.camera.azimuth),
                              'elevation': float(view.camera.elevation), 'identical_for_all_panels': True}
        ffmpeg = checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg')
        video = out/(args.case+('_close' if args.close_view else '')+'_with_input.mp4')
        with (out/'encoder.log').open('w') as log:
            proc = subprocess.Popen([str(ffmpeg), '-y', '-hide_banner', '-f', 'rawvideo', '-pixel_format', 'rgb24',
                '-video_size', '1920x640', '-framerate', '30', '-i', '-', '-an', '-c:v', 'libx264',
                '-threads', '1', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)],
                stdin=subprocess.PIPE, stderr=log)
            try:
                for i, t in enumerate(source['times']):
                    canvas = np.full((640, 1920, 3), [18, 24, 35], dtype=np.uint8)
                    images = [human_points(view, source['palm_positions_m'][i], source['parent'], metrics['scale'])]
                    images += [view.robot(tr['q_actual'][i]) for tr in trajectories]
                    for panel, (label, rgb) in enumerate(zip(['INPUT SKELETON', args.baseline.upper(), args.candidate.upper()], images)):
                        x = panel*640
                        canvas[80:560, x:x+640] = rgb
                        caption(canvas, label, x+24, 18, scale=3)
                        caption(canvas, args.case.upper(), x+24, 52)
                        caption(canvas, f'SOURCE {t:.2f} S / '+('25 INPUT POINTS' if panel == 0 else 'ACTUAL DYNAMICS'), x+24, 582)
                        caption(canvas, 'SYNTHETIC ANGLE FIXTURE' if record['source_kind'] == 'synthetic_known_angles'
                                else 'RECORDED MANUS / NO ANGLE TRUTH', x+24, 610)
                    proc.stdin.write(canvas.tobytes())
                    if i in {0, len(source['times'])//2, len(source['times'])-1}: png(out/f'frame_{i}.png', canvas)
                    if i % 90 == 0: print('render', i, '/', len(source['times']), flush=True)
                proc.stdin.close()
                if proc.wait() != 0: raise RuntimeError('Encoder failed')
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                view.close()
        with (out/'decode_check.log').open('w') as log:
            subprocess.run([str(ffmpeg), '-v', 'error', '-i', str(video), '-f', 'null', '-'], check=True, stderr=log)
        write_json(out/'video.json', {'path': str(video), 'sha256': digest(video), 'frames': len(source['times']),
            'fps': 30, 'full_decode_passed': True, 'input_in_every_frame': True, 'width': 1920, 'height': 640})


if __name__ == '__main__': main()
