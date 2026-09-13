"""Render the raw MANUS glove skeleton from archived normalized landmarks; rendering only.

No simulation, no retargeting, no hardware test: every frame is drawn from
data/normalized/manus_official_ufbx_v1/<sequence>.npz, which was produced by the
registered ufbx ingest of the official MANUS FBX samples.
"""
import argparse
import json
import os
import subprocess
from pathlib import Path
from lab_common import checked, configure, digest, run, write_json
configure()
os.environ['MUJOCO_GL'] = 'egl'
os.environ['EGL_PLATFORM'] = 'surfaceless'
import mujoco
import numpy as np
from render_pinch import caption
from render_vector import png

# Minimal scene: only lighting matters, every geom is drawn per frame.
SCENE = """<mujoco><visual><headlight diffuse=".8 .8 .8" ambient=".3 .3 .3"/></visual>
<worldbody><light pos="0 0 1" dir="0 0 -1"/></worldbody></mujoco>"""

FINGERS = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
COLOR = {'Hand': (222, 228, 238), 'Thumb': (255, 173, 38), 'Index': (26, 204, 255),
         'Middle': (90, 217, 120), 'Ring': (166, 128, 255), 'Pinky': (243, 140, 190)}
BONE_WIDTH = {'Hand': .0026, 'CMC': .0024, 'MCP': .0022, 'PIP': .0020, 'DIP': .0018, 'TIP': .0018}
JOINT_RADIUS = {'Hand': .0075, 'CMC': .0040, 'MCP': .0042, 'PIP': .0034, 'DIP': .0029, 'TIP': .0032}
DIM = (104, 114, 130)
# Three fixed orthographic-style projections of the palm frame; +z distal, +y radial.
VIEWS = [('PALMAR FACE -X', 180, 0), ('RADIAL SIDE +Y', 90, 0), ('FINGERTIP AXIS +Z', 0, 88)]
PANEL_W, PANEL_H, CANVAS_W, CANVAS_H = 640, 480, 1920, 620
TOP = 80


def finger_of(name):
    for f in FINGERS:
        if name.startswith(f):
            return f
    return 'Hand'


def joint_of(name):
    return name.split('_')[-1] if name != 'Hand' else 'Hand'


def rgba(color):
    """MuJoCo geometry colour; scene colours above are 0-255 canvas values."""
    return np.array([color[0] / 255, color[1] / 255, color[2] / 255, 1.0], dtype=np.float32)


def connectors(scene, p1, p2, color, width):
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.array([width] * 3), np.zeros(3),
                        np.eye(3).ravel(), rgba(color))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, p1, p2)
    scene.ngeom += 1


def sphere(scene, point, color, radius):
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius] * 3), point,
                        np.eye(3).ravel(), rgba(color))
    scene.ngeom += 1


def rect(rgb, x, y, w, h, color):
    rgb[y:y + h, x:x + w] = color


class Skeleton:
    """Draws one MANUS frame into a reused pose-only MuJoCo scene."""

    def __init__(self, names, parent, mask_available):
        self.names = [str(n) for n in names]
        self.parent = parent
        self.mask_available = mask_available
        self.tint = [COLOR[finger_of(n)] for n in self.names]
        self.radius = [JOINT_RADIUS[joint_of(n)] for n in self.names]
        self.width = [BONE_WIDTH[joint_of(self.names[i]) if self.parent[i] >= 0 else 'Hand']
                      for i in range(len(self.names))]
        # Palm rim between neighbouring MCPs: a drawing aid, NOT a source bone.
        self.rim = []
        mcp = [self.names.index(f + '_MCP') for f in FINGERS[1:]]
        for a, b in zip(mcp, mcp[1:]):
            if self.parent[b] != a:
                self.rim.append((a, b))

    def draw(self, renderer, data, camera, points, mask, rim=True):
        renderer.update_scene(data, camera=camera)
        scene = renderer.scene
        for i, parent in enumerate(self.parent):
            if parent < 0:
                continue
            color, width = self.tint[i], self.width[i]
            if self.mask_available and not mask[i]:
                color, width = DIM, width * .6
            connectors(scene, points[parent], points[i], color, width)
        if rim:
            for a, b in self.rim:
                if self.mask_available and not (mask[a] and mask[b]):
                    continue
                connectors(scene, points[a], points[b], DIM, .0012)
        for i in range(len(self.names)):
            color = DIM if (self.mask_available and not mask[i]) else self.tint[i]
            sphere(scene, points[i], color, self.radius[i])
        return renderer.render().copy()


def legend(canvas, x, y, skeleton):
    items = [f for f in FINGERS]
    for f in items:
        rect(canvas, x, y + 2, 12, 12, COLOR[f])
        caption(canvas, f, x + 18, y, COLOR[f])
        x += 26 + 6 * 2 * len(f)
    caption(canvas, 'GREY - PALM RIM / NOT OBSERVABLE', x + 12, y, DIM)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sequence', action='append', choices=['finger_agility', 'hand_mobility'])
    ap.add_argument('--start', type=float, default=None, help='seconds into the sequence')
    ap.add_argument('--end', type=float, default=None)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--frame', choices=['palm', 'world'], default='palm')
    ap.add_argument('--preview', action='store_true', help='one still per view, no video')
    ap.add_argument('--no-rim', action='store_true')
    args = ap.parse_args()
    sequences = args.sequence or ['finger_agility', 'hand_mobility']
    splits = json.loads(checked('data/splits/manus_official_v1.json').read_text())

    with run('render_manus_skeleton', vars(args)) as (out, manifest):
        manifest['rendering'] = ('archived normalized MANUS landmarks as spheres and capsules; '
                                 'camera fixed over the rendered window; no robot, no simulation')
        manifest['landmark_source'] = 'data/normalized/manus_official_ufbx_v1'
        manifest['caveats'] = {
            'palm_rim': 'links between neighbouring MCPs are a drawing aid, not source bones',
            'frame': 'palm = wrist-fixed canonical frame from ingest; world = raw FBX world coordinates',
            'mask': 'greyed landmarks follow the archived observable_mask, not a new measurement',
            'scope': 'no retargeting, no contact, no hardware claim',
        }
        ffmpeg = checked('toolchains/ffmpeg-imageio-0.6.0/ffmpeg')
        manifest['encoder'] = str(ffmpeg)
        manifest['encoder_sha256'] = digest(ffmpeg)
        model = mujoco.MjModel.from_xml_string(SCENE)
        data = mujoco.MjData(model)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.orthographic = 0
        videos, stills = {}, {}
        with mujoco.Renderer(model, height=PANEL_H, width=PANEL_W) as renderer:
            for sequence in sequences:
                archive = checked(f'data/normalized/manus_official_ufbx_v1/{sequence}.npz')
                sha = digest(archive)
                expected = splits['sequence_hashes'].get(sequence)
                if expected and sha != expected:
                    raise ValueError(f'{sequence}: normalized archive hash mismatch')
                d = np.load(archive, allow_pickle=False)
                times = d['timestamps']
                points_all = d['world_positions_m'] if args.frame == 'world' else d['palm_positions_m']
                mask_all = d['observable_mask']
                skeleton = Skeleton(d['names'], [int(x) for x in d['parent']], bool(mask_all.any()))
                start = 0.0 if args.start is None else args.start
                end = float(times[-1]) if args.end is None else args.end
                picked = np.nonzero((times >= start) & (times <= end))[0]
                if len(picked) < 2:
                    raise ValueError(f'{sequence}: window contains {len(picked)} frames')
                frames = picked[::max(1, int(round(1 / np.median(np.diff(times)) / args.fps)))]
                still_at = [frames[int(len(frames) * q)] for q in (.02, .35, .65, .98)]
                if args.preview:
                    frames = np.array(sorted(set(still_at)))
                    still_at = list(frames)
                body = points_all[frames]
                # Framing is computed wrist-relative so both coordinate choices frame identically.
                relative = body - body[:, :1, :]
                radius = float(np.linalg.norm(relative.reshape(-1, 3), axis=1).max())
                views = []
                for (label, azimuth, elevation), (a, b) in zip(VIEWS, [(1, 2), (0, 2), (0, 1)]):
                    plane = relative[:, :, [a, b]].reshape(-1, 2)
                    middle = (plane.min(axis=0) + plane.max(axis=0)) / 2
                    lookat = np.zeros(3)
                    lookat[a], lookat[b] = middle
                    span = float(np.linalg.norm(plane - middle, axis=1).max())
                    views.append({'label': label, 'azimuth': azimuth, 'elevation': elevation,
                                  'in_plane_axes': [a, b], 'lookat_m': lookat.tolist(),
                                  'span_m': span, 'distance_m': 2.9 * span})
                manifest.setdefault('sequences', {})[sequence] = {
                    'archive_sha256': sha, 'frames_in_source': int(len(times)),
                    'window_s': [start, end], 'frames_rendered': int(len(frames)),
                    'frame_stride': int(frames[1] - frames[0]), 'source_fps': round(1 / float(np.median(np.diff(times))), 2),
                    'output_fps': args.fps, 'frame': args.frame,
                    'wrist_relative_bounds_m': {'low': relative.reshape(-1, 3).min(axis=0).tolist(),
                                                'high': relative.reshape(-1, 3).max(axis=0).tolist()},
                    'unobservable_landmark_fraction': round(float(1 - mask_all[frames].mean()), 6),
                    'views': views, 'wrist_relative_radius_m': radius,
                    'palm_rim_links': None if args.no_rim else [[skeleton.names[a], skeleton.names[b]] for a, b in skeleton.rim],
                }
                video = out / f'manus_skeleton_{sequence}.mp4'
                log = (out / f'encoder_{sequence}.log').open('w')
                proc = None if args.preview else subprocess.Popen(
                    [str(ffmpeg), '-y', '-hide_banner', '-f', 'rawvideo', '-pixel_format', 'rgb24',
                     '-video_size', f'{CANVAS_W}x{CANVAS_H}', '-framerate', str(args.fps), '-i', '-', '-an',
                     '-c:v', 'libx264', '-threads', '1', '-crf', '18', '-pix_fmt', 'yuv420p',
                     '-movflags', '+faststart', str(video)], stdin=subprocess.PIPE, stderr=log)
                try:
                    for index, source_frame in enumerate(frames):
                        canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
                        canvas[:] = [18, 24, 35]
                        caption(canvas, 'MANUS SKELETON - ' + sequence.replace('_', ' '), 24, 14, scale=3)
                        caption(canvas, ('PALM FRAME - WRIST ORIGIN' if args.frame == 'palm'
                                         else 'WORLD FRAME - RAW FBX COORDINATES'), 1120, 14)
                        for panel, view in enumerate(views):
                            camera.azimuth, camera.elevation = view['azimuth'], view['elevation']
                            camera.distance = view['distance_m']
                            camera.lookat[:] = body[index, 0] + np.array(view['lookat_m'])
                            image = skeleton.draw(renderer, data, camera, body[index], mask_all[source_frame],
                                                  rim=not args.no_rim)
                            y0, x0 = TOP, panel * PANEL_W
                            canvas[y0:y0 + PANEL_H, x0:x0 + PANEL_W] = image
                            caption(canvas, view['label'], x0 + 24, 48, scale=3)
                        caption(canvas, f'T = {times[source_frame]:.2f} S / {times[-1]:.2f} S', 24, 566)
                        caption(canvas, f'FRAME {source_frame} / {len(times) - 1}   RENDER {index + 1} / {len(frames)}'
                                        f'   {args.fps} FPS', 560, 566)
                        caption(canvas, 'SOURCE - ARCHIVED NORMALIZED MANUS LANDMARKS - NO SIMULATION', 1180, 566)
                        legend(canvas, 24, 590, skeleton)
                        if source_frame in still_at:
                            png(out / f'still_{sequence}_{times[source_frame]:06.2f}s.png', canvas)
                            stills.setdefault(sequence, []).append(f'still_{sequence}_{times[source_frame]:06.2f}s.png')
                        if proc is not None:
                            proc.stdin.write(canvas.tobytes())
                        if index % 200 == 0 or index + 1 == len(frames):
                            print(f'[{sequence}] frame {index + 1}/{len(frames)} source_t={times[source_frame]:.2f}s', flush=True)
                    if proc is None:
                        continue
                    proc.stdin.close()
                    if proc.wait() != 0:
                        raise RuntimeError(f'{sequence}: video encoding failed')
                finally:
                    if proc is not None and proc.poll() is None:
                        proc.kill()
                        proc.wait()
                    log.close()
                check = subprocess.run([str(ffmpeg), '-hide_banner', '-i', str(video), '-f', 'null', '-'],
                                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                (out / f'decode_check_{sequence}.log').write_bytes(check.stderr)
                videos[sequence] = {'file': video.name, 'frames': int(len(frames)), 'fps': args.fps,
                                    'sha256': digest(video), 'bytes': video.stat().st_size}
                print(f'[{sequence}] encoded {video.name} {videos[sequence]["bytes"]} bytes over {len(frames)} frames', flush=True)
        if videos:
            write_json(out / 'video.json', videos)
        manifest['videos'] = videos
        manifest['stills'] = stills
        print(str(out), flush=True)


if __name__ == '__main__':
    main()
