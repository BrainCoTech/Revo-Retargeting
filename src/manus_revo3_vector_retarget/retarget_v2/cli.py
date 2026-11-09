"""Independent MANUS retargeting, live MuJoCo and offline evaluation."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from .paths import source_key
import time

import numpy as np

from .input import Calibration, FINGERS, read_capture
from .model import RobotModel
from .motion import MotionGuard
from .solver import Retargeter, SolverConfig


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=json_default, allow_nan=False)+"\n")


def summary(values):
    values = np.asarray(values)
    return dict(mean=float(np.mean(values)), p50=float(np.median(values)), p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def evaluate(args):
    hand_side = getattr(args, "hand_side", "right")
    meta, rows = read_capture(args.capture)
    calibration_rows = [r for r in rows if r["phase"] == "calibration"][:120]
    calibration = Calibration.from_frames([r["msg"] for r in calibration_rows], args.normal_sign,
                                          hand_side=hand_side)
    config = SolverConfig(**json.loads(args.config.read_text())) if args.config else SolverConfig()
    if args.max_frames:
        rows = rows[:args.max_frames]
    rows = rows[::args.stride]
    args.output.mkdir(parents=True, exist_ok=False)
    save_json(args.output/"run.json", dict(complete=False, capture=str(args.capture.resolve())))
    with RobotModel(model_version=args.model, hand_side=hand_side) as robot:
        solver = Retargeter(robot, calibration, config)
        guard = MotionGuard(robot.lower, robot.upper, config.velocity_limit, config.acceleration_limit,
                            initial_q=np.clip(np.zeros(21), robot.lower, robot.upper))
        times, raw_q, cmd_q, cmd_dq, chains, phases, metrics, statistics = [], [], [], [], [], [], [], []
        stream_times, stream_q, stream_dq = [0.], [guard.state.q], [guard.state.dq]
        previous_t = float(rows[0]["t"])
        start = time.perf_counter()
        for i, row in enumerate(rows):
            t = float(row["t"])
            if i and t-previous_t > .25:
                raise ValueError("Input gap >250ms: capture cannot be evaluated as continuous tracking")
            q, status = solver.solve(row["msg"])
            # Integrate the command reference on short time steps. This is the
            # exact same guard to be used by the simulated servo consumer.
            dt = t-previous_t if i else 1/60
            substeps = max(1, int(np.ceil(dt/.005)))
            for j in range(substeps):
                out, velocity = guard.step(q, dt/substeps)
                stream_times.append(t-dt+(j+1)*dt/substeps)
                stream_q.append(out)
                stream_dq.append(velocity)
            measured = solver.metrics(out, solver.last)
            ik_metrics = solver.metrics(q, solver.last)
            measured["ik_direction_deg"] = ik_metrics["direction_deg"]
            measured["ik_pair_error_mm"] = ik_metrics["pair_error_mm"]
            measured["ik_robot_tip_distance_mm"] = ik_metrics["robot_tip_distance_mm"]
            rpoints = robot.points(out)[solver.chain_ids]
            target_chain = np.concatenate([rpoints[:, :1], rpoints[:, :1]+np.cumsum(
                solver.last["directions_world"]*robot.finger_lengths[:, :, None], axis=1)], axis=1)
            times.append(t)
            raw_q.append(q)
            cmd_q.append(out)
            cmd_dq.append(velocity)
            chains.append(target_chain)
            phases.append(row["phase"])
            metrics.append(measured)
            statistics.append(status)
            previous_t = t
            if (i+1) % 600 == 0:
                print(f"{i+1}/{len(rows)} frames; elapsed={time.perf_counter()-start:.1f}s", flush=True)
        metric_arrays = {name: np.array([m[name] for m in metrics]) for name in metrics[0]}
        q_array, dq_array = np.array(cmd_q), np.array(cmd_dq)
        assert np.isfinite(q_array).all()
        assert np.all(q_array >= robot.lower-1e-9) and np.all(q_array <= robot.upper+1e-9)
        assert np.max(np.abs(dq_array)) <= config.velocity_limit+1e-8
        np.savez_compressed(args.output/"trajectory.npz", t=times, q_ik=raw_q, q_cmd=cmd_q, dq_cmd=cmd_dq,
                            target_chain=chains, phase=phases, joint_names=robot.joint_names, **metric_arrays)
        # Source-frame command traces are adequate for playback. Dense guard
        # samples are retained separately for acceleration and limit auditing.
        stream_times = np.array(stream_times[1:])
        stream_q, stream_dq = np.array(stream_q[1:]), np.array(stream_dq[1:])
        np.savez_compressed(args.output/"command_trace.npz", t=stream_times, q=stream_q, dq=stream_dq,
                            joint_names=robot.joint_names)
        accel = np.diff(stream_dq, axis=0)/np.diff(stream_times)[:, None]
        per_phase = {}
        phase_array = np.array(phases)
        for phase in dict.fromkeys(phases):
            selection = phase_array == phase
            per_phase[phase] = dict(frames=int(selection.sum()),
                                   direction_deg=summary(metric_arrays["direction_deg"][selection]),
                                   ik_direction_deg=summary(metric_arrays["ik_direction_deg"][selection]),
                                   pair_error_mm=summary(metric_arrays["pair_error_mm"][selection]))
        pinch = {}
        for j, finger in enumerate(FINGERS[1:]):
            selected = (metric_arrays["human_tip_distance_mm"][:, j] < config.pinch_on_m*1000) & (phase_array == "pinch")
            pinch[finger] = dict(frames=int(selected.sum()))
            if selected.any():
                pinch[finger].update(human_gap_mm=summary(metric_arrays["human_tip_distance_mm"][selected, j]),
                                     robot_gap_mm=summary(metric_arrays["robot_tip_distance_mm"][selected, j]),
                                     ik_robot_gap_mm=summary(metric_arrays["ik_robot_tip_distance_mm"][selected, j]),
                                     error_mm=summary(metric_arrays["pair_error_mm"][selected, j]))
        report = dict(real_capture=True, model_version=robot.model_version, hand_side=hand_side,
                      model_metadata=robot.model_metadata,
                      frames=len(rows), duration_s=times[-1]-times[0],
                      processing_s=time.perf_counter()-start,
                      solve_ms=summary([s["solve_ms"] for s in statistics]),
                      accepted_frames=sum(bool(s["accepted"]) for s in statistics),
                      convergence_frames=sum(s["converged"] for s in statistics),
                      direction_deg=summary(metric_arrays["direction_deg"]),
                      ik_direction_deg=summary(metric_arrays["ik_direction_deg"]),
                      pair_error_mm=summary(metric_arrays["pair_error_mm"]),
                      per_phase=per_phase, pinch=pinch,
                      max_command_velocity_rad_s=float(np.abs(stream_dq).max()),
                      max_command_acceleration_rad_s2=float(np.abs(accel).max()),
                      within_model_limits=True,
                      coordinate_note="21 named URDF joint coordinates, NOT verified real motor coordinates",
                      collision_note="Soft centreline separation only; no mesh collision or contact-force validation",
                      quality_note="Metrics measure agreement with constructed targets, not independent motion ground truth")
        if report["max_command_acceleration_rad_s2"] > config.acceleration_limit+1e-6:
            raise ValueError("Command acceleration audit failed")
        save_json(args.output/"report.json", report)
        save_json(args.output/"geometry.json", robot.geometry_report)
        hashes = {source_key(p, robot): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in sorted(Path(__file__).parent.glob("*.py"))}
        for path in robot.asset_paths:
            hashes[source_key(path, robot)] = hashlib.sha256(path.read_bytes()).hexdigest()
        import mujoco
        save_json(args.output/"run.json", dict(complete=True, capture=str(args.capture.resolve()),
                                             capture_sha256=meta["raw_sha256"], config=asdict(config),
                                             model_version=robot.model_version,
                                             hand_side=hand_side,
                                             model_metadata=robot.model_metadata,
                                             calibration=asdict(calibration), pair_scale=solver.scale,
                                             source_hashes=hashes, joint_names=robot.joint_names,
                                             mujoco_version=mujoco.__version__,
                                             stride=args.stride,
                                             command_trace_sha256=hashlib.sha256((args.output/"command_trace.npz").read_bytes()).hexdigest(),
                                             trajectory_sha256=hashlib.sha256((args.output/"trajectory.npz").read_bytes()).hexdigest()))
        print(json.dumps({k: report[k] for k in ("frames", "solve_ms", "ik_direction_deg", "pinch",
                                                 "max_command_velocity_rad_s", "max_command_acceleration_rad_s2")},
                         indent=2, default=json_default))
        print(f"Saved: {args.output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("evaluate")
    p.add_argument("--hand-side", choices=("left", "right"), default="right")
    p.add_argument("capture", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--config", type=Path)
    p.add_argument("--normal-sign", type=float, choices=(-1., 1.), default=-1.)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--max-frames", type=int)
    p.add_argument("--model", choices=("may3",), default="may3",
                   help="Current workspace revo3_description URDF")
    p = commands.add_parser("view")
    p.add_argument("run", type=Path)
    p.add_argument("--speed", type=float, default=1.)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--kinematic", action="store_true")
    p.add_argument("--render", type=Path)
    p.add_argument("--time", type=float, default=55.)
    p = commands.add_parser("simulate")
    p.add_argument("run", type=Path)
    p = commands.add_parser("live", help="Subscribe to the selected MANUS hand and drive MuJoCo only")
    p.add_argument("--hand-side", choices=("left", "right"), default="right")
    p.add_argument("--model", choices=("may3",), default="may3")
    p.add_argument("--domain", type=int, default=None, help="Override ROS_DOMAIN_ID; otherwise inherit the ROS environment")
    p.add_argument("--glove-id", type=int)
    p.add_argument("--calibration-frames", type=int, default=120)
    p.add_argument("--timeout", type=float, default=.25)
    p.add_argument("--normal-sign", type=float, choices=(-1., 1.), default=-1.)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--auto-start", action="store_true", help="Start after initial calibration; never auto-resume a stop")
    p = commands.add_parser("hardware", help="Windowless service-controlled Revo3 ROS output (real or driver mock)")
    p.add_argument("--hand-side", choices=("left", "right"), default="right")
    p.add_argument("--model", choices=("may3",), default="may3")
    p.add_argument("--domain", type=int, default=None, help="Override ROS_DOMAIN_ID; otherwise inherit the ROS environment")
    p.add_argument("--glove-id", type=int)
    p.add_argument("--namespace", help="Hardware namespace; defaults to /revo3_<hand-side>")
    p.add_argument("--node-name", help="Override ROS node/service prefix for bringup compatibility")
    p.add_argument("--profile", type=Path)
    p.add_argument("--interpolate", action="store_true",
                   help="Enable 200 Hz MIT interpolation and startup ramp; default publishes each solved glove frame directly")
    p.add_argument("--motion-scale", type=float,
                   help="Optional velocity*s, acceleration*s^2; cannot combine with independent limit overrides")
    p.add_argument("--max-velocity-rad-s", type=float,
                   help="Finite positive MIT velocity cap; also limits position slew with --interpolate; defaults to profile")
    p.add_argument("--max-acceleration-rad-s2", type=float,
                   help="Finite positive acceleration cap with --interpolate only; defaults to profile")
    p.add_argument("--calibration-frames", type=int, default=120)
    p.add_argument("--normal-sign", type=float, choices=(-1., 1.), default=-1.)
    p.add_argument("--cache-dir", type=Path, help="Bone-length cache directory; default $XDG_CACHE_HOME/manus_revo3_vector_retarget")
    p.add_argument("--thumb-offset-deg", type=float,
                   help="Override the profile thumb_direction_offset angle (deg, -90..90); larger lowers thumb MCP")
    args = parser.parse_args()
    if getattr(args, "stride", 1) < 1 or (getattr(args, "max_frames", None) is not None and args.max_frames < 2):
        parser.error("stride >=1, max-frames >=2")
    if args.command == "hardware":
        args.namespace = args.namespace or f"/revo3_{args.hand_side}"
        if ((args.domain is not None and not 0 <= args.domain <= 232) or args.calibration_frames < 30
                or (args.motion_scale is not None and
                    (not np.isfinite(args.motion_scale) or not 0 < args.motion_scale <= 1))):
            parser.error("hardware requires domain 0..232, calibration-frames >=30, optional 0<motion-scale<=1")
        overrides = (args.max_velocity_rad_s, args.max_acceleration_rad_s2)
        if args.motion_scale is not None and any(value is not None for value in overrides):
            parser.error("Use motion-scale or independent velocity/acceleration caps, not both")
        if args.thumb_offset_deg is not None and not (np.isfinite(args.thumb_offset_deg)
                                                      and -90. <= args.thumb_offset_deg <= 90.):
            parser.error("thumb-offset-deg must be within [-90, 90]")
        for value, name in zip(overrides, ("velocity", "acceleration")):
            if value is not None and (not np.isfinite(value) or value <= 0):
                parser.error(f"{name} cap must be finite and positive")
        from .hardware import hardware
        hardware(args)
    elif args.command == "live":
        if ((args.domain is not None and not 0 <= args.domain <= 232) or args.calibration_frames < 30
                or not np.isfinite(args.timeout) or args.timeout <= 0):
            parser.error("live requires domain 0..232, calibration-frames >=30, timeout >0")
        from .live import live
        live(args)
    elif args.command == "evaluate":
        evaluate(args)
    elif args.command == "view":
        from .viewer import view
        view(args)
    else:
        from .viewer import simulate
        simulate(args)


if __name__ == "__main__":
    main()
