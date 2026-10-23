"""Audited servo simulation and replay of command or simulated joint states.

Dynamic playback is a replay of MuJoCo servo output computed at the original
capture time scale. Playback speed never changes the simulated physics. Neither
these states nor tracking statistics are measurements from the physical hand.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import threading
import time
import zlib

import mujoco
import numpy as np

from .input import FINGERS
from .model import RobotModel
from .paths import source_key, source_path


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _summary(values):
    values = np.asarray(values)
    return dict(mean=float(np.mean(values)), p50=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def _model_sources(run, robot):
    metadata = json.loads((Path(run) / "run.json").read_text())
    source_hashes = metadata.get("source_hashes", {})
    model_path = source_key(Path(__file__).with_name("model.py"), robot)
    urdf_path = source_key(robot.urdf_path, robot)
    required = {model_path, urdf_path, *(source_key(path, robot) for path in robot.asset_paths)}
    if not required.issubset(source_hashes):
        raise ValueError("Missing portable model hashes; regenerate this run with evaluate")
    checked = {}
    for relative, expected in source_hashes.items():
        if relative in (model_path, urdf_path) or relative.startswith("description/"):
            path = source_path(relative, robot)
            if not path.is_file() or _sha(path) != expected:
                raise ValueError(f"Model or asset changed since retargeting: {relative}")
            checked[relative] = expected
    return checked


def _run_model_version(run):
    metadata = json.loads((Path(run) / "run.json").read_text())
    # Old/missing model metadata cannot identify the current workspace model.
    version = metadata.get("model_version")
    if version != "may3":
        raise ValueError(f"Unknown recorded model version: {version}")
    return version


def _run_hand_side(run):
    metadata = json.loads((Path(run) / "run.json").read_text())
    # Runs recorded before left-hand support always used the right-hand model.
    hand_side = metadata.get("hand_side", "right")
    if hand_side not in ("left", "right"):
        raise ValueError(f"Unknown recorded hand side: {hand_side}")
    return hand_side


def _load_run(run, robot):
    run = Path(run)
    metadata = json.loads((run / "run.json").read_text())
    if not metadata.get("complete"):
        raise ValueError("Retargeting run is incomplete")
    if _run_model_version(run) != robot.model_version:
        raise ValueError("Recorded model version does not match the replay model")
    if _run_hand_side(run) != robot.hand_side:
        raise ValueError("Recorded hand side does not match the replay model")
    _model_sources(run, robot)
    if _sha(run / "trajectory.npz") != metadata["trajectory_sha256"]:
        raise ValueError("Trajectory checksum mismatch")
    with np.load(run / "trajectory.npz", allow_pickle=False) as archive:
        trajectory = {key: archive[key].copy() for key in archive.files}
    times, q = trajectory["t"], trajectory["q_cmd"]
    if times.ndim != 1 or len(times) < 2 or q.shape != (len(times), len(robot.joint_names)):
        raise ValueError("Invalid trajectory shape")
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Trajectory timestamps must increase")
    if not np.isfinite(q).all() or trajectory["target_chain"].shape != (len(times), 5, 4, 3):
        raise ValueError("Invalid joint states or target chains")
    if not np.isfinite(trajectory["target_chain"]).all():
        raise ValueError("Non-finite target chain")
    if trajectory["phase"].shape != times.shape:
        raise ValueError("Phase labels do not match trajectory frames")
    if tuple(trajectory["joint_names"]) != tuple(robot.joint_names):
        raise ValueError("Trajectory joint order does not match the model")
    if np.any(q < robot.lower - 1e-8) or np.any(q > robot.upper + 1e-8):
        raise ValueError("Command trajectory exceeds model joint limits")
    return metadata, trajectory


def _command_trace(run, trajectory, robot):
    path = Path(run) / "command_trace.npz"
    if path.exists():
        metadata = json.loads((Path(run) / "run.json").read_text())
        if metadata.get("command_trace_sha256") and _sha(path) != metadata["command_trace_sha256"]:
            raise ValueError("Dense command trace checksum mismatch")
        with np.load(path, allow_pickle=False) as archive:
            times, q = archive["t"].copy(), archive["q"].copy()
            if tuple(archive["joint_names"]) != tuple(robot.joint_names):
                raise ValueError("Dense command trace uses a different joint order")
        source = "command_trace.npz"
    else:
        times, q, source = trajectory["t"], trajectory["q_cmd"], "trajectory.npz"
        path = Path(run) / source
    if (times.ndim != 1 or len(times) < 2 or q.shape != (len(times), 21)
            or not np.isfinite(times).all() or not np.isfinite(q).all()
            or np.any(np.diff(times) <= 0)):
        raise ValueError("Invalid simulation command trace")
    if np.any(q < robot.lower - 1e-8) or np.any(q > robot.upper + 1e-8):
        raise ValueError("Dense command trace exceeds model limits")
    if times[-1] < trajectory["t"][-1] - 1e-8:
        raise ValueError("Command trace ends before source trajectory")
    return times, q, source, _sha(path)


def _simulate(run, trajectory, robot):
    from .simulation import ServoSimulation

    times, reference, source, source_hash = _command_trace(run, trajectory, robot)
    started = time.perf_counter()
    simulation = ServoSimulation(robot, timestep=.002)
    if tuple(simulation.joint_names) != tuple(robot.joint_names):
        raise ValueError("Simulator joint order differs from trajectory")
    simulation.reset(reference[0])
    joint_ids = np.array([mujoco.mj_name2id(simulation.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                          for name in robot.joint_names])
    dof_indices = simulation.model.jnt_dofadr[joint_ids]
    measured, velocity, torque = np.empty_like(reference), np.empty_like(reference), np.empty_like(reference)
    measured[0] = simulation.data.qpos[simulation.q_indices]
    velocity[0] = simulation.data.qvel[dof_indices]
    torque[0] = simulation.data.qfrc_actuator[dof_indices]
    for i in range(1, len(times)):
        # Causal zero-order hold: a reference arriving at t_i is not available
        # during [t_(i-1), t_i). Compare feedback at t_i to the new reference.
        measured[i] = simulation.step(reference[i - 1], float(times[i] - times[i - 1]))
        velocity[i] = simulation.data.qvel[dof_indices]
        torque[i] = simulation.data.qfrc_actuator[dof_indices]
        if not (np.isfinite(measured[i]).all() and np.isfinite(velocity[i]).all() and np.isfinite(torque[i]).all()):
            raise ValueError(f"Non-finite servo state at sample {i}, t={times[i]:.6f}")
    source_q = np.column_stack([np.interp(trajectory["t"], times, measured[:, j]) for j in range(21)])
    error_deg = np.rad2deg(np.abs(measured - reference))
    warnings = np.asarray(simulation.data.warning.number).copy()
    physics_report = simulation.report
    run = Path(run)
    np.savez_compressed(run / "simulation_trace.npz", t=times, q_reference=reference,
                        q_sim=measured, dq_sim=velocity, actuator_torque=torque,
                        source_t=trajectory["t"], source_q_sim=source_q,
                        joint_names=robot.joint_names)
    report = dict(
        complete=True, backend="MuJoCo position-servo dynamics", hardware_measurement=False,
        model_version=robot.model_version, hand_side=robot.hand_side,
        input_trace=source, input_trace_sha256=source_hash,
        trajectory_sha256=_sha(run / "trajectory.npz"),
        simulation_source_sha256=_sha(Path(__file__).with_name("simulation.py")),
        model_source_hashes=_model_sources(run, robot), mujoco_version=mujoco.__version__,
        viewer_source_sha256=_sha(__file__), trace_sha256=_sha(run / "simulation_trace.npz"),
        command_samples=len(times), duration_s=float(times[-1] - times[0]),
        input_start_t=float(times[0]), input_end_t=float(times[-1]),
        processing_s=time.perf_counter() - started, timestep_max_s=.002,
        finite=True, within_model_limits=bool(physics_report["max_measured_limit_violation_rad"] <= 1e-6),
        within_soft_limit_tolerance=not physics_report["measured_limit_violation"],
        max_limit_violation_rad=physics_report["max_measured_limit_violation_rad"],
        tracking_error_deg=_summary(error_deg),
        source_frame_tracking_error_deg=_summary(np.rad2deg(abs(source_q - trajectory["q_cmd"]))),
        per_joint_tracking_error_deg={name: _summary(error_deg[:, j]) for j, name in enumerate(robot.joint_names)},
        physics_tracking_max_error_deg=float(np.rad2deg(physics_report["max_tracking_error_rad"])),
        max_simulated_velocity_rad_s=physics_report["max_measured_velocity_rad_s"],
        max_simulated_actuator_torque_nm=physics_report["max_actuator_force_nm"],
        mujoco_warning_counts=warnings.tolist(),
        physics_step_audit=physics_report,
        contacts_enabled=False,
        limitation="No contact dynamics, verified hardware motor mapping, measured motor model, or real-hand feedback. Joint limits refer to the URDF model.",
        max_command_interval_s=float(np.max(np.diff(times))),
        command_interpolation="Causal zero-order hold: reference[i-1] is applied during [t[i-1], t[i]); feedback at t[i] is compared with reference[i]. Physics steps <=2 ms.",
    )
    (run / "simulation_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return source_q, report


def _dynamic_states(run, trajectory, robot):
    run = Path(run)
    report_path, trace_path = run / "simulation_report.json", run / "simulation_trace.npz"
    if report_path.exists() and trace_path.exists():
        report = json.loads(report_path.read_text())
        _, _, _, input_hash = _command_trace(run, trajectory, robot)
        model_sources = _model_sources(run, robot)
        if (report.get("complete") and report.get("trajectory_sha256") == _sha(run / "trajectory.npz")
                and report.get("input_trace_sha256") == input_hash
                and report.get("simulation_source_sha256") == _sha(Path(__file__).with_name("simulation.py"))
                and report.get("model_source_hashes") == model_sources
                and report.get("mujoco_version") == mujoco.__version__
                and report.get("viewer_source_sha256") == _sha(__file__)
                and report.get("trace_sha256") == _sha(trace_path)):
            with np.load(trace_path, allow_pickle=False) as data:
                states = data["source_q_sim"].copy()
            if states.shape == trajectory["q_cmd"].shape and np.isfinite(states).all():
                return states, report
    return _simulate(run, trajectory, robot)


def simulate(args):
    with RobotModel(model_version=_run_model_version(args.run), hand_side=_run_hand_side(args.run)) as robot:
        _, trajectory = _load_run(args.run, robot)
        _, report = _simulate(args.run, trajectory, robot)
    error = report["tracking_error_deg"]
    print(f"MuJoCo servo simulation: {report['command_samples']} commands / {report['duration_s']:.2f}s; "
          f"joint error p95={error['p95']:.3f} deg, max={error['max']:.3f} deg; "
          f"finite={report['finite']}, within exact URDF limits={report['within_model_limits']}, "
          f"within 1mrad soft-limit tolerance={report['within_soft_limit_tolerance']}")
    print(f"Simulation only, contact disabled. Saved: {Path(args.run) / 'simulation_report.json'}")


def _style(robot):
    robot.model.geom_rgba[:, :3] = [.65, .69, .75]
    robot.model.geom_rgba[:, 3] = .40
    robot.model.vis.global_.offwidth = 1280
    robot.model.vis.global_.offheight = 900
    robot.model.vis.rgba.haze[:] = [.06, .075, .11, 1.]
    robot.model.vis.headlight.ambient[:] = [.45, .45, .45]
    robot.model.vis.headlight.diffuse[:] = [.75, .75, .75]


def _camera():
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.03, .025, .107]
    camera.distance, camera.azimuth, camera.elevation = .39, 28., -10.
    return camera


def _options(labels=False):
    option = mujoco.MjvOption()
    option.sitegroup[:] = 0
    option.label = mujoco.mjtLabel.mjLABEL_SELECTION if labels else mujoco.mjtLabel.mjLABEL_NONE
    return option


def _geometry(scene, start, end, color, radius):
    if scene.ngeom >= scene.maxgeom:
        raise ValueError("MuJoCo scene geometry buffer exhausted")
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                      np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, start, end)
    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
    scene.ngeom += 1


def _skeletons(scene, robot_chain, target_chain):
    for chain, color, radius in ((robot_chain, [.10, .60, 1., 1.], .0014),
                                 (target_chain, [1., .49, .13, 1.], .0011)):
        for finger in chain:
            for start, end in zip(finger[:-1], finger[1:]):
                _geometry(scene, start, end, color, radius)


def _frame(robot, trajectory, states, timestamp):
    t = trajectory["t"]
    timestamp = float(np.clip(timestamp, t[0], t[-1]))
    i = min(int(np.searchsorted(t, timestamp, side="right") - 1), len(t) - 1)
    j = min(i + 1, len(t) - 1)
    alpha = (timestamp - t[i]) / (t[j] - t[i]) if j != i else 0.
    q = states[i] * (1 - alpha) + states[j] * alpha
    robot_chain = robot.points(q).reshape(5, 5, 3)[:, :4].copy()
    target = trajectory["target_chain"][i] * (1 - alpha) + trajectory["target_chain"][j] * alpha
    return timestamp, str(trajectory["phase"][i]), robot_chain, target


def _text(mode, timestamp, phase, paused=False):
    return (f"{mode} | t={timestamp:6.2f}s | phase={phase}" + (" | PAUSED" if paused else ""),
            "BLUE robot skeleton   ORANGE morphology-normalized target\n"
            "Simulation only; no contact or physical-hand feedback")


def _write_png(path, pixels):
    """Encode the rendered RGB framebuffer without optional image packages."""
    height, width, channels = pixels.shape
    if channels != 3 or pixels.dtype != np.uint8:
        raise ValueError("PNG output requires RGB uint8 pixels")

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)

    scanlines = b"".join(b"\0" + row.tobytes() for row in pixels)
    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" +
                          chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
                          chunk(b"IDAT", zlib.compress(scanlines, 6)) + chunk(b"IEND", b""))


def _render(path, robot, trajectory, states, timestamp, mode):

    timestamp, phase, chain, target = _frame(robot, trajectory, states, timestamp)
    with mujoco.Renderer(robot.model, height=900, width=1280) as renderer:
        renderer.update_scene(robot.data, camera=_camera(), scene_option=_options())
        _skeletons(renderer.scene, chain, target)
        pixels = renderer.render().copy()
        if pixels.shape != (900, 1280, 3) or np.std(pixels[100:800]) < 2:
            raise RuntimeError("Offscreen rendering returned a blank image")
        title, detail = _text(mode, timestamp, phase)
        viewport = mujoco.MjrRect(0, 0, 1280, 900)
        # Renderer owns this context; overlay into its already-rendered buffer
        # before reading pixels again, so text does not depend on Pillow/fonts.
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           viewport, title, "", renderer._mjr_context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                           viewport, detail, "", renderer._mjr_context)
        mujoco.mjr_readPixels(pixels, None, viewport, renderer._mjr_context)
        pixels = np.flipud(pixels).copy()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_png(path, pixels)
    print(f"Saved {mode} frame at {timestamp:.2f}s: {path}")


def view(args):
    if not np.isfinite(args.speed) or args.speed <= 0:
        raise ValueError("Playback speed must be a finite positive number")
    if not np.isfinite(args.time):
        raise ValueError("Render time must be finite")
    with RobotModel(model_version=_run_model_version(args.run), hand_side=_run_hand_side(args.run)) as robot:
        _, trajectory = _load_run(args.run, robot)
        if args.kinematic:
            states, mode = trajectory["q_cmd"], "KINEMATIC COMMAND REFERENCE"
        else:
            states, report = _dynamic_states(args.run, trajectory, robot)
            mode = "DYNAMIC SIMULATED SERVO REPLAY"
            print(f"Dynamic simulation tracking p95: {report['tracking_error_deg']['p95']:.3f} deg. No contact dynamics.")
        _style(robot)
        if args.render:
            _render(args.render, robot, trajectory, states, args.time, mode)
            return
        import mujoco.viewer

        controls = dict(paused=False, replay=False, labels=True, seek=0.)
        control_lock = threading.Lock()

        def key_callback(key):
            with control_lock:
                if key == 32:
                    controls["paused"] = not controls["paused"]
                elif key in (82, 114):
                    controls["replay"] = True
                elif key in (76, 108):
                    controls["labels"] = not controls["labels"]
                elif key == 263:
                    controls["seek"] -= 5.
                elif key == 262:
                    controls["seek"] += 5.

        cursor, last_wall = float(trajectory["t"][0]), time.monotonic()
        print("Space pause/resume; R replay; L labels; Left/Right seek 5s. Close window to exit.")
        from .viewer_lifecycle import passive_viewer
        with passive_viewer(robot.model, robot.data, key_callback=key_callback) as handle:
            camera = _camera()
            with handle.lock():
                handle.cam.lookat[:] = camera.lookat
                handle.cam.distance, handle.cam.azimuth, handle.cam.elevation = camera.distance, camera.azimuth, camera.elevation
                handle.opt.sitegroup[:] = 0
            last_wall = time.monotonic()
            while handle.is_running():
                now = time.monotonic()
                with control_lock:
                    if controls["replay"]:
                        cursor, controls["replay"] = float(trajectory["t"][0]), False
                        controls["paused"] = False
                    cursor += controls["seek"]
                    controls["seek"] = 0.
                    if not controls["paused"]:
                        cursor += (now - last_wall) * args.speed
                    if cursor > trajectory["t"][-1]:
                        if args.loop:
                            cursor = float(trajectory["t"][0])
                        else:
                            cursor, controls["paused"] = float(trajectory["t"][-1]), True
                    cursor = max(cursor, float(trajectory["t"][0]))
                    paused, labels = controls["paused"], controls["labels"]
                last_wall = now
                with handle.lock():
                    timestamp, phase, chain, target = _frame(robot, trajectory, states, cursor)
                    handle.user_scn.ngeom = 0
                    _skeletons(handle.user_scn, chain, target)
                title, detail = _text(mode, timestamp, phase, paused)
                if labels:
                    handle.set_texts([(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, ""),
                                      (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, detail + "\nSpace: pause | R: replay | L: labels | arrows: seek", "")])
                else:
                    handle.clear_texts()
                handle.sync()
                time.sleep(.008)
