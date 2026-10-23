"""MANUS subscription -> independent optimizer -> live MuJoCo servo viewer."""
from contextlib import ExitStack, nullcontext
from queue import Empty, SimpleQueue
import sys
import time

from .live_control import LiveController
from .model import RobotModel
from .ros_input import ManusSubscriber


def live(args):
    hand_side = getattr(args, "hand_side", "right")
    controls = SimpleQueue()
    domain_label = args.domain if args.domain is not None else "ROS environment"
    print(f"MANUS {hand_side.upper()} -> MUJOCO LIVE | domain={domain_label}", flush=True)
    print(f"Keep your {hand_side} hand comfortable and steady for calibration. Space: start/pause/resume; R: recalibrate; Q: exit.", flush=True)
    print("Simulation only, contacts disabled.", flush=True)

    def key_callback(key):
        if key in (32, 82, 114, 81, 113):
            controls.put(key)

    with ExitStack() as stack:
        robot = stack.enter_context(RobotModel(model_version=getattr(args, "model", "may3"),
                                               hand_side=hand_side))
        print(f"Model: {robot.model_version} | {robot.urdf_path}", flush=True)
        controller = LiveController(robot, args.calibration_frames, args.timeout, args.normal_sign)
        stack.callback(controller.close)
        source = stack.enter_context(ManusSubscriber(None, args.glove_id, args.domain,
                                                     hand_side=hand_side))
        print("MANUS inputs: " + ", ".join(source.topics), flush=True)
        handle = None
        if not args.headless:
            import mujoco
            import mujoco.viewer
            from .viewer import _camera, _skeletons, _style
            _style(controller.simulation)
            from .viewer_lifecycle import passive_viewer
            handle = stack.enter_context(passive_viewer(
                controller.simulation.model, controller.simulation.data,
                key_callback=key_callback, show_left_ui=False, show_right_ui=False))
            with handle.lock():
                camera = _camera()
                handle.cam.lookat[:] = camera.lookat
                handle.cam.distance, handle.cam.azimuth, handle.cam.elevation = camera.distance, camera.azimuth, camera.elevation
                handle.opt.sitegroup[:] = 0
        started = last_tick = time.monotonic()
        last_render = 0.
        previous_state = None
        sequence = -1
        auto_start_used = not args.auto_start
        quitting = False
        try:
            while not quitting and (handle is None or handle.is_running()):
                cycle_start = time.monotonic()
                # Advance only commands already available at the preceding tick.
                with handle.lock() if handle else nullcontext():
                    controller.tick(cycle_start-last_tick, cycle_start)
                last_tick = cycle_start
                sample = source.latest(after_sequence=sequence)
                if sample is not None:
                    sequence = sample.sequence
                    controller.ingest(sample.message, sample.received_monotonic,
                                      sample.sequence, time.monotonic())
                now = time.monotonic()
                if not auto_start_used and controller.solver is not None:
                    # At most once per process; never re-arm a recovered stream.
                    auto_start_used = True
                    controller.resume(now)
                while True:
                    try:
                        key = controls.get_nowait()
                    except Empty:
                        break
                    if key == 32:
                        if controller.status(now)["state"] == "tracking":
                            controller.pause()
                        elif not controller.resume(now):
                            print("Start refused: wait for calibration, fresh valid input and completed braking.", flush=True)
                    elif key in (82, 114):
                        controller.request_calibration()
                        auto_start_used = True
                        print(f"Recalibration requested. Keep the {hand_side} hand comfortable and steady; Space starts after READY.", flush=True)
                    elif key in (81, 113):
                        quitting = True
                current = controller.status(now)
                receiver = source.status()
                if receiver.get("thread_error"):
                    controller.stop("subscriber_error")
                    raise RuntimeError(receiver["thread_error"])
                input_error = current["last_error"] or receiver.get("last_error")
                transition = (current["state"], current["stop_reason"], input_error)
                if transition != previous_state:
                    event = dict(t=now-started, state=current["state"], reason=current["stop_reason"],
                                 error=input_error, valid_frames=current["valid_frames"])
                    print(f"{event['t']:6.1f}s {event['state'].upper()} | {event['reason'] or 'following'}"
                          + (f" | {event['error']}" if event["error"] else ""), flush=True)
                    previous_state = transition
                if handle and now-last_render >= 1/30:
                    age = "--" if current["input_age_s"] is None else f"{current['input_age_s']*1000:.0f} ms"
                    solve = current["solve"]
                    timing = "--" if solve is None else f"{solve['solve_ms']:.1f} ms"
                    title = (f"LIVE MANUS -> REVO3 {hand_side.upper()} ({robot.model_version}) | {current['state'].upper()}\n"
                             f"domain {domain_label}  {', '.join(source.topics)}  glove {current['glove_id']}\n"
                             f"calibration {current['calibration_count']}/{args.calibration_frames}"
                             f"  input age {age}  solve {timing}")
                    detail = ("Space: start / pause / resume   R: recalibrate   Q: exit\n"
                              "BLUE simulated hand   ORANGE target | SIMULATION ONLY, CONTACTS OFF")
                    if current["state"] in ("waiting", "calibrating"):
                        detail = f"Keep {hand_side} hand comfortable and steady; waiting for valid SDK skeleton.\n"+detail
                    elif current["state"] in ("ready", "hold"):
                        detail = f"{current['stop_reason'] or ''} | Press Space with fresh input to start.\n"+detail
                    if current["last_error"] or receiver.get("last_error"):
                        detail = str(current["last_error"] or receiver["last_error"])+"\n"+detail
                    with handle.lock():
                        handle.user_scn.ngeom = 0
                        if controller.target_chain is not None:
                            chain = robot.points(controller.simulation.measured_q).reshape(5, 5, 3)[:, :4]
                            _skeletons(handle.user_scn, chain, controller.target_chain)
                    handle.set_texts([(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, ""),
                                      (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, detail, "")])
                    handle.sync()
                    last_render = now
                time.sleep(max(0., .005-(time.monotonic()-cycle_start)))
        except KeyboardInterrupt:
            pass
        finally:
            # Final simulated braking is bounded and never resets qpos.
            active_error = sys.exc_info()[0] is not None
            controller.pause()
            try:
                for _ in range(60):
                    last_tick += .005
                    with handle.lock() if handle and handle.is_running() else nullcontext():
                        controller.tick(.005, last_tick)
                    if controller.guard.state.mode == "hold":
                        break
            except Exception as exc:
                print(f"Final simulated braking failed: {type(exc).__name__}: {exc}",
                      file=sys.stderr, flush=True)
                if not active_error:
                    raise
    print("Live simulation stopped.", flush=True)
