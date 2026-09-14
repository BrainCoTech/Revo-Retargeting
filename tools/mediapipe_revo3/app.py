"""Local camera/video capture, raw landmark replay, preview and manifested logs."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import select
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
    p.add_argument('--solver', choices=['shared', 'baseline'], default='shared')
    p.add_argument('--scale', type=float, default=1., help='Length multiplier: robot bones in bone_scaled, wrist offsets in wrist_scaled')
    p.add_argument('--palm-x-sign', type=int, choices=[-1, 1], default=1)
    p.add_argument('--target-mode', choices=['wrist_scaled', 'bone_scaled'], help='Override endpoint profile target construction')
    p.add_argument('--solver-config', type=Path, help='Endpoint profile JSON; solver section and max_gap_s are used')
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
    p.add_argument('--guide', action='store_true', help='Timed camera recording with action labels and timeline.json')
    p.add_argument('--guide-plan', action='store_true', help='Print the guided protocol without opening the camera')
    p.add_argument('--guide-voice', action='store_true', help='Speak Chinese prompts using macOS say')
    args = p.parse_args()
    if not math.isfinite(args.scale) or args.scale <= 0:
        p.error('--scale must be positive and finite')
    if args.solver == 'baseline' and (args.target_mode or args.solver_config or args.scale != 1 or args.palm_x_sign != 1):
        p.error('Scale, palm reflection and solver config overrides require --solver shared')
    if args.noise_mm < 0 or not 0 <= args.dropout <= 1 or args.max_frames < 0:
        p.error('Invalid noise/dropout/max-frames value')
    if (args.noise_mm or args.dropout) and not args.replay:
        p.error('Perturbations require --replay; camera recordings stay original')
    if args.replay and args.record_video:
        p.error('Raw landmark replay has no source video to record')
    if args.guide and (args.video or args.replay or args.check):
        p.error('--guide requires live camera input')
    if args.guide_voice and not (args.guide or args.guide_plan):
        p.error('--guide-voice requires --guide')
    if args.guide:
        args.record_video = True
        if args.no_display and not sys.stdin.isatty() and not args.guide_plan:
            p.error('Guided recording needs a preview window or interactive terminal for Enter')
    return args


def json_write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def draw(frame, observation, output, cv2, np, visual):
    from mapper import CHAINS
    guide = observation.get('guide')
    canvas = np.zeros((880 if guide else 720, 1280, 3), dtype=np.uint8)
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
    actual = 'q_actual_rad' in output
    views = visual.render(output['q_actual_rad'] if actual else output['q_command_rad'])
    for index, (axes, screen_scale, x0, label) in enumerate([
            ([1, 2], [1200, -1200], 660, 'palm'),
            ([0, 2], [-1200, -1200], 975, 'thumb side')]):
        cv2.putText(canvas, f'Revo3 {label}: {"actual simulation" if actual else "command pose"}', (x0, 30), 0, .46, (255, 255, 255), 1)
        x = min(x0, 970)
        canvas[45:405, x:x + 310] = views[index]
        if local is not None:
            for f, chain in enumerate(CHAINS):
                p = np.asarray(local, dtype=float)[chain][:, axes] * screen_scale + [x0 + 155, 665]
                for a, b in zip(p, p[1:]):
                    if not np.isfinite([a, b]).all():
                        continue
                    cv2.line(canvas, tuple(a.astype(int)), tuple(b.astype(int)), colors[f], 2)
        cv2.putText(canvas, 'MediaPipe input (palm frame)', (x0, 430), 0, .46, (190, 190, 190), 1)
    lines = ['Source input (unmirrored for inference)', output['status'],
             f"processing: {output['processing_ms']:.1f} ms | t={observation['timestamp_s']:.3f}s",
             'q / Esc: exit | no hardware output',
             'MuJoCo dynamics; contact not validated' if actual else 'Kinematic preview; no contact/dynamics validation']
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (15, 550 + i * 30), 0, .58, (230, 230, 230), 1)
    if guide:
        color = (90, 230, 90) if guide['phase'] == 'record' else (80, 200, 255)
        if guide['phase'] == 'done':
            prompt_lines = ['GUIDED RECORDING COMPLETE']
        elif guide['phase'] == 'review':
            prompt_lines = ['LAST ACTION COMPLETE | ENTER: finish and save | R: retry last action']
        else:
            timing = ('Press ENTER when ready' if guide['phase'] == 'prepare' and guide['remaining_s'] is None
                      else f"{math.ceil(guide['remaining_s'])} seconds left")
            prompt_lines = [f"{guide['phase'].upper()} {guide['action_index']}/{guide['action_count']}"
                            f"  {guide['action_id']}  |  {timing}",
                            *guide['display_lines'],
                            ('ENTER: next action | R: retry previous action | q / Esc: save and exit'
                             if guide['phase'] == 'prepare' else
                             f"Attempt {guide.get('attempt', 1)} | R: restart current action | q / Esc: save and exit")]
        for i, line in enumerate(prompt_lines):
            cv2.putText(canvas, line, (15, 745 + 32 * i), 0, .65, color, 1)
    cv2.imshow('MediaPipe -> Revo3', canvas)
    return cv2.waitKey(1) & 0xff


def terminal_command():
    """Drain terminal lines without blocking capture or queuing future starts."""
    command = None
    if sys.stdin.isatty():
        while select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            value = line.strip().lower()
            if value == 'r':
                command = 'restart'
            elif value == '' and command != 'restart':
                command = 'start'
    return command


def main():
    args = arguments()
    if args.guide_plan:
        from guide import print_plan
        print_plan()
        return
    if Path(sys.prefix).resolve() != (REPO / '.venv').resolve():
        raise SystemExit('Use bash tools/mediapipe_revo3/run.sh (repository .venv only)')
    import cv2
    import numpy as np
    folder = ROOT / 'assets'
    if args.solver == 'shared':
        from local_model import prepare_local_model
        from shared_mapper import SharedMapper
        model_path = prepare_local_model()
        mapper = SharedMapper(model_path, scale=args.scale, palm_x_sign=args.palm_x_sign,
                              solver_config=args.solver_config, target_mode=args.target_mode)
    else:
        from mapper import Mapper
        model_path = folder / 'kinematics.xml'
        if not model_path.exists():
            raise SystemExit('Missing assets. Run bash tools/mediapipe_revo3/setup.sh')
        mapper = Mapper(model_path)
    from guide import Guide
    guide = Guide(voice=args.guide_voice) if args.guide else None
    if not args.replay or args.check:
        import mediapipe as mp
        if not (folder / 'hand_landmarker.task').exists():
            raise SystemExit('Missing detector. Run bash tools/mediapipe_revo3/setup.sh')
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
        'output': ('q_command_rad and q_actual_rad; official PD MuJoCo dynamics; contact not validated'
                   if args.solver == 'shared' else 'q_command_rad, official FK only; no q_actual or contact simulation'),
        'model': {'path': str(model_path), 'sha256': sha(model_path)},
        'time_convention': ('q_actual sampled before applying current q_command; origin first source timestamp; no final extrapolation'
                            if args.solver == 'shared' else 'kinematic command pose'),
        'environment': sys.prefix, 'python': platform.python_version(),
        'packages': {d.metadata['Name']: d.version for d in importlib.metadata.distributions()},
        'joint_names': mapper.names, 'joint_limits_rad': np.stack([mapper.lo, mapper.hi], axis=1).tolist(),
        'asset_sha256': {p.name: sha(p) for p in folder.iterdir() if p.is_file()},
        'code_sha256': {p.name: sha(p) for p in Path(__file__).parent.glob('*.py')},
        'source_sha256': sha(args.replay or args.video) if (args.replay or args.video) else None,
        'timestamp_policy': 'camera monotonic time after read; video presentation time with fps fallback; source timestamps retained on replay',
        'raw_video_policy': 'decoded frames in source order; per-frame timestamp_s in frames.jsonl is authoritative'}
    snapshot = out / 'code'
    if guide:
        manifest['guided_recording'] = {'protocol': 'hand_baseline_manual_retry_v6', 'timeline': 'timeline.json',
            'action_duration_s': guide.duration_s, 'start_policy': 'enter_each_action',
            'labels': 'prompted actions, not verified execution/contact'}
    snapshot.mkdir()
    for path in Path(__file__).parent.iterdir():
        if path.is_file():
            shutil.copyfile(path, snapshot / path.name)
    if args.solver == 'shared':
        from shared_mapper import CORE
        core_snapshot = snapshot / 'shared_core'
        core_snapshot.mkdir()
        for path in CORE.glob('*.py'):
            shutil.copyfile(path, core_snapshot / path.name)
            manifest['code_sha256']['shared_core/' + path.name] = sha(path)
        receipt = model_path.parent / 'model.json'
        shutil.copyfile(receipt, out / 'model.json')
        shutil.copyfile(model_path, out / 'model.xml')
        shutil.copyfile(mapper.profile_path, out / 'solver_profile.json')
        manifest['model']['receipt'] = 'model.json'
        manifest['model']['receipt_sha256'] = sha(receipt)
        manifest['model']['xml_snapshot'] = 'model.xml'
        manifest['solver_profile'] = {'path': str(mapper.profile_path), 'sha256': sha(mapper.profile_path),
                                      'snapshot': 'solver_profile.json'}
        manifest['solver_config'] = mapper.config
        manifest['input_transform'] = {'scale': args.scale, 'palm_x_sign': args.palm_x_sign, 'hand_side': args.hand, 'target_mode': mapper.target_mode}
        manifest['max_gap_s'] = mapper.max_gap_s
    manifest['code_snapshot'] = 'code/'
    json_write(out / 'manifest.json', manifest)
    print(f'Run: {out}', flush=True)
    cap = detector = video_writer = replay_file = None
    log = None
    visual = None
    statuses, latencies, frame_count = Counter(), [], 0
    rng = np.random.default_rng(args.seed)
    last_stamp, last_ms = -1., -1
    playback_origin = None
    start = time.monotonic()
    try:
        if not args.no_display:
            from assets import prepare_visual
            from visual import RobotVisual
            visual = RobotVisual(model_path if args.solver == 'shared' else prepare_visual(), mapper.names)
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
                if 'guide' in source:
                    observation['guide'] = source['guide']
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
            if guide:
                observation['guide'] = guide.label(stamp)
                guide.prompt(observation['guide'])
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
            if guide:
                guide.record(observation['guide'], frame_count, stamp, output['status'])
            statuses[output['status']] += 1
            latencies.append(output['processing_ms'])
            frame_count += 1
            last_stamp = stamp
            if guide and observation['guide']['phase'] == 'done':
                break
            key = None
            if not args.no_display:
                if replay_file or args.video:
                    if playback_origin is None:
                        playback_origin = (stamp, time.monotonic())
                    # Display follows source-relative time; inference startup is excluded.
                    delay = (stamp-playback_origin[0])-(time.monotonic()-playback_origin[1])
                    if delay > 0:
                        time.sleep(delay)
                key = draw(frame, observation, output, cv2, np, visual)
                if key in [27, ord('q')]:
                    break
            if guide:
                command = terminal_command()
                if key in [ord('r'), ord('R')] or command == 'restart':
                    guide.request_restart()
                elif key in [10, 13] or command == 'start':
                    guide.request_start()
        if frame_count == 0:
            raise RuntimeError('Source contained no decodable frames')
        manifest['status'] = ('partial' if guide and not guide.timeline(last_stamp, '')['completed']
                              else 'complete')
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
    except Exception as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        raise
    finally:
        if guide:
            guide.stop_speech()
            json_write(out / 'timeline.json', guide.timeline(last_stamp, manifest['status']))
        for resource in [cap, video_writer]:
            if resource is not None:
                resource.release()
        for resource in [detector, replay_file, log]:
            if resource is not None:
                resource.close()
        if visual is not None:
            visual.close()
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
