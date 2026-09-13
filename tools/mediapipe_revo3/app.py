"""Local camera/video capture, raw landmark replay, preview and manifested logs."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import sys
import time
import uuid

from assets import ROOT, REPO, sha


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group()
    source.add_argument('--camera', type=int, default=0)
    source.add_argument('--video', type=Path)
    source.add_argument('--replay', type=Path, help='Replay a previous frames.jsonl')
    p.add_argument('--hand', choices=['Right', 'Left'], default='Right')
    p.add_argument('--input-mirrored', action='store_true', help='Unmirror mirrored source BEFORE inference')
    p.add_argument('--no-display', action='store_true')
    p.add_argument('--record-video', action='store_true', help='Save decoded source frames to raw.avi')
    p.add_argument('--max-frames', type=int, default=0)
    p.add_argument('--session', default='unlabelled', help='Subject/session/action tag; stored as metadata')
    p.add_argument('--noise-mm', type=float, default=0, help='Replay-only added Gaussian world-coordinate noise')
    p.add_argument('--dropout', type=float, default=0, help='Replay-only probability of dropping an observation')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--check', action='store_true', help='Check assets, environment and empty-frame inference, no camera')
    args = p.parse_args()
    if args.noise_mm < 0 or not 0 <= args.dropout <= 1 or args.max_frames < 0:
        p.error('Invalid noise/dropout/max-frames value')
    if (args.noise_mm or args.dropout) and not args.replay:
        p.error('Perturbations require --replay; camera recordings stay original')
    if args.replay and args.record_video:
        p.error('Raw landmark replay has no source video to record')
    return args


def json_write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def draw(frame, observation, output, cv2, np):
    from mapper import CHAINS
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
    colors = [(90, 190, 255), (80, 220, 90), (255, 170, 80), (230, 100, 200), (110, 230, 230)]
    if frame is not None:
        view = frame.copy()
        h, w = view.shape[:2]
        for hand in observation.get('detections', []):
            p = np.asarray(hand['image_landmarks'])[:, :2] * [w, h]
            for f, chain in enumerate(CHAINS):
                for a, b in zip(chain, chain[1:]):
                    cv2.line(view, tuple(p[a].astype(int)), tuple(p[b].astype(int)), colors[f], 2)
            cv2.putText(view, hand['side'], tuple(p[0].astype(int)), 0, .6, (255, 255, 255), 2)
        scale = min(640 / w, 480 / h)
        small = cv2.resize(view, (int(w * scale), int(h * scale)))
        canvas[50:50 + small.shape[0], :small.shape[1]] = small
    else:
        cv2.putText(canvas, 'LANDMARK REPLAY (no camera image)', (15, 70), 0, .65, (255, 255, 255), 1)
    local = output['palm_landmarks_m']
    robot = np.asarray(output['robot_landmarks_m'])
    for axes, x0, label in [([1, 2], 660, 'front'), ([0, 2], 975, 'side')]:
        cv2.putText(canvas, f'Revo3 {label}: q_command FK', (x0, 30), 0, .46, (255, 255, 255), 1)
        for f in range(5):
            p = np.vstack([np.zeros(3), robot[f]])[:, axes] * [-1250, -1250] + [x0 + 155, 355]
            for a, b in zip(p, p[1:]):
                cv2.line(canvas, tuple(a.astype(int)), tuple(b.astype(int)), colors[f], 3)
            for v in p:
                cv2.circle(canvas, tuple(v.astype(int)), 4, colors[f], -1)
        if local is not None:
            for f, chain in enumerate(CHAINS):
                p = np.asarray(local)[chain][:, axes] * [-1200, -1200] + [x0 + 155, 665]
                for a, b in zip(p, p[1:]):
                    cv2.line(canvas, tuple(a.astype(int)), tuple(b.astype(int)), colors[f], 2)
        cv2.putText(canvas, 'MediaPipe input (palm frame)', (x0, 430), 0, .46, (190, 190, 190), 1)
    lines = ['Source input (unmirrored for inference)', output['status'],
             f"processing: {output['processing_ms']:.1f} ms | t={observation['timestamp_s']:.3f}s",
             'q / Esc: exit | no hardware output',
             'Kinematic preview; no contact/dynamics validation']
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (15, 550 + i * 30), 0, .58, (230, 230, 230), 1)
    cv2.imshow('MediaPipe -> Revo3', canvas)
    return (cv2.waitKey(1) & 0xff) not in [27, ord('q')]


def main():
    args = arguments()
    if Path(sys.prefix).resolve() != (REPO / '.venv').resolve():
        raise SystemExit('Use bash tools/mediapipe_revo3/run.sh (repository .venv only)')
    import cv2
    import mediapipe as mp
    import numpy as np
    from mapper import Mapper
    folder = ROOT / 'assets'
    for name in ['hand_landmarker.task', 'kinematics.xml']:
        if not (folder / name).exists():
            raise SystemExit('Missing assets. Run bash tools/mediapipe_revo3/setup.sh')
    mapper = Mapper(folder / 'kinematics.xml')
    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(folder / 'hand_landmarker.task'),
                                         delegate=mp.tasks.BaseOptions.Delegate.CPU),
        running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=2,
        min_hand_detection_confidence=.6, min_hand_presence_confidence=.6,
        min_tracking_confidence=.6)
    if args.check:
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as detector:
            result = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,
                                                       data=np.zeros((480, 640, 3), np.uint8)), 0)
        assert not result.hand_landmarks
        print(f'OK: Python {platform.python_version()}, {sys.executable}, Revo3 21 axes, MediaPipe blank-frame inference')
        return

    out = ROOT / 'runs' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    perturbed = bool(args.noise_mm or args.dropout)
    manifest = {'schema_version': 1, 'status': 'running', 'started_at': datetime.now(timezone.utc).isoformat(),
        'args': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        'provenance': ('perturbed_mediapipe_replay' if perturbed else 'mediapipe_replay') if args.replay
                      else ('recorded_video_mediapipe' if args.video else 'live_camera_mediapipe'),
        'input_topology': 'MediaPipe 21 landmarks, original order; no MANUS CMC synthesis',
        'world_units': 'estimated metres; not measured ground truth', 'robot_side': 'Right',
        'output': 'q_command_rad, official FK only; no q_actual or contact simulation',
        'environment': sys.prefix, 'python': platform.python_version(),
        'packages': {d.metadata['Name']: d.version for d in importlib.metadata.distributions()},
        'joint_names': mapper.names, 'joint_limits_rad': np.stack([mapper.lo, mapper.hi], axis=1).tolist(),
        'asset_sha256': {p.name: sha(p) for p in folder.iterdir() if p.is_file()},
        'code_sha256': {p.name: sha(p) for p in Path(__file__).parent.glob('*.py')},
        'source_sha256': sha(args.replay or args.video) if (args.replay or args.video) else None,
        'timestamp_policy': 'camera monotonic time after read; video presentation time with fps fallback; source timestamps retained on replay',
        'raw_video_policy': 'decoded frames in source order; per-frame timestamp_s in frames.jsonl is authoritative'}
    snapshot = out / 'code'
    snapshot.mkdir()
    for path in Path(__file__).parent.iterdir():
        if path.is_file():
            shutil.copyfile(path, snapshot / path.name)
    manifest['code_snapshot'] = 'code/'
    json_write(out / 'manifest.json', manifest)
    print(f'Run: {out}', flush=True)
    cap = detector = video_writer = replay_file = None
    log = None
    statuses, latencies, frame_count = Counter(), [], 0
    rng = np.random.default_rng(args.seed)
    last_stamp, last_ms = -1., -1
    start = time.monotonic()
    try:
        log = (out / 'frames.jsonl').open('x')
        if args.replay:
            replay_file = args.replay.open()
        else:
            cap = cv2.VideoCapture(str(args.video) if args.video else args.camera)
            if not cap.isOpened():
                raise RuntimeError('Cannot open source. On macOS allow camera access for Terminal/Codex; try --camera 1.')
            fps = cap.get(cv2.CAP_PROP_FPS)
            fps = float(fps) if np.isfinite(fps) and fps > 0 else 30.
            detector = mp.tasks.vision.HandLandmarker.create_from_options(options)
        while not args.max_frames or frame_count < args.max_frames:
            frame = None
            if replay_file:
                line = replay_file.readline()
                if not line:
                    break
                source = json.loads(line)
                observation = {k: source[k] for k in ['timestamp_s', 'detections']}
                observation['source_frame_index'] = source['frame_index']
            else:
                ok, frame = cap.read()
                if not ok:
                    if args.video:
                        break
                    raise RuntimeError('Camera frame read failed')
                stamp = (cap.get(cv2.CAP_PROP_POS_MSEC) / 1000 if args.video else time.monotonic() - start)
                if not np.isfinite(stamp) or stamp <= last_stamp:
                    stamp = frame_count / fps if args.video else time.monotonic() - start
                if stamp <= last_stamp:
                    raise RuntimeError('Non-monotonic source timestamps')
                observation = {'timestamp_s': stamp, 'detections': []}
            stamp = float(observation['timestamp_s'])
            if not np.isfinite(stamp) or stamp <= last_stamp:
                raise ValueError('Replay/source timestamps must be finite and strictly increasing')
            begin = time.perf_counter()
            if frame is not None:
                if args.record_video:
                    if video_writer is None:
                        video_writer = cv2.VideoWriter(str(out / 'raw.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                                       fps, (frame.shape[1], frame.shape[0]))
                        if not video_writer.isOpened():
                            raise RuntimeError('Cannot create raw.avi')
                    video_writer.write(frame)
                if args.input_mirrored:
                    frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_ms = max(last_ms + 1, round(stamp * 1000))
                result = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), mp_ms)
                last_ms = mp_ms
                observation['inference_timestamp_ms'] = mp_ms
                for image_lm, world_lm, handedness in zip(result.hand_landmarks, result.hand_world_landmarks, result.handedness):
                    category = handedness[0]
                    observation['detections'].append({'side': category.category_name,
                        'handedness_score': float(category.score),
                        'image_landmarks': [[p.x, p.y, p.z] for p in image_lm],
                        'world_landmarks_m': [[p.x, p.y, p.z] for p in world_lm]})
            candidates = [d for d in observation['detections'] if d['side'] == args.hand and d['handedness_score'] >= .7]
            # Ambiguous duplicate handedness is rejected, never arbitrarily switched.
            selected = candidates[0] if len(candidates) == 1 else None
            world = np.asarray(selected['world_landmarks_m']) if selected is not None else None
            drop = False
            if world is not None and perturbed:
                drop = rng.random() < args.dropout
                world = None if drop else world + rng.normal(0, args.noise_mm / 1000, world.shape)
            output = mapper.update(world, stamp, args.hand)
            output['processing_ms'] = (time.perf_counter() - begin) * 1000
            row = dict(observation, frame_index=frame_count, selected_side=args.hand,
                       mapper_world_landmarks_m=None if world is None else world.tolist(),
                       injected_dropout=drop, output=output)
            log.write(json.dumps(row, allow_nan=False) + '\n')
            log.flush()
            statuses[output['status']] += 1
            latencies.append(output['processing_ms'])
            frame_count += 1
            last_stamp = stamp
            if not args.no_display:
                if not draw(frame, observation, output, cv2, np):
                    break
                if replay_file or args.video:
                    # Playback pacing does not affect source dt or mapped values.
                    time.sleep(max(0, min(.05, stamp - (time.monotonic() - start))))
        if frame_count == 0:
            raise RuntimeError('Source contained no decodable frames')
        manifest['status'] = 'complete'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
    except Exception as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        raise
    finally:
        for resource in [cap, video_writer]:
            if resource is not None:
                resource.release()
        for resource in [detector, replay_file, log]:
            if resource is not None:
                resource.close()
        if not args.no_display:
            cv2.destroyAllWindows()
        manifest['frames'] = frame_count
        manifest['finished_at'] = datetime.now(timezone.utc).isoformat()
        json_write(out / 'manifest.json', manifest)
        json_write(out / 'summary.json', {'frames': frame_count, 'statuses': dict(statuses),
            'tracking_fraction': statuses['tracking'] / max(1, frame_count),
            'processing_ms_percentiles_50_95_99': np.percentile(latencies, [50, 95, 99]).tolist() if latencies else [],
            'latency_scope': 'inference + mapping + optional video write; excludes capture, display and JSON I/O'})
        print(f"{manifest['status']}: {frame_count} frames; {out}")


if __name__ == '__main__':
    main()
