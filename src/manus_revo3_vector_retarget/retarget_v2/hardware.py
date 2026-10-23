"""Windowless ROS retargeting; services own enable, pause and recalibration."""
from contextlib import ExitStack
import signal
import time

from .hardware_profile import HardwareProfile
from .calibration_session import CalibrationSession
from .model import RobotModel
from .ros_hardware import HardwareTransport
from .ros_input import ManusSubscriber
from .solver import Retargeter


def hardware(args):
    # Ctrl+C can reach a child via its process group and again via ROS launch.
    # Let the first interrupt unwind resources; do not interrupt executor joins.
    previous_handler = signal.getsignal(signal.SIGINT)

    def interrupt_once(_signum, _frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt_once)
    try:
        _run_hardware(args)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def _run_hardware(args):
    model_version = getattr(args, "model", "may3")
    hand_side = getattr(args, "hand_side", "right")
    namespace = args.namespace or f"/revo3_{hand_side}"
    interpolate = getattr(args, "interpolate", False)
    node_name = getattr(args, "node_name", None) or f"manus_revo3_vector_retarget_{hand_side}"
    domain_label = args.domain if args.domain is not None else "ROS environment"
    print(f"INDEPENDENT RETARGET -> REVO3 {hand_side.upper()} | domain={domain_label} | model={model_version}", flush=True)
    print("Output starts disabled. Wear the glove normally; a comfortable hand shape is enough for bone-length calibration.", flush=True)
    print(f"Enable/pause: /{node_name}/enable_retarget (std_srvs/srv/SetBool)", flush=True)
    print(f"Recalibrate: /{node_name}/recalibrate (std_srvs/srv/Trigger)", flush=True)
    print("Pause stops MIT publication; it does not power off the motors.", flush=True)

    with ExitStack() as stack:
        robot = stack.enter_context(RobotModel(model_version=model_version, hand_side=hand_side))
        profile = HardwareProfile(robot, path=args.profile, motion_scale=args.motion_scale,
                                  max_velocity_rad_s=getattr(args, "max_velocity_rad_s", None),
                                  max_acceleration_rad_s2=getattr(args, "max_acceleration_rad_s2", None))
        transport = stack.enter_context(HardwareTransport(
            profile, domain_id=args.domain, namespace=namespace, interpolate=interpolate,
            node_name=node_name))
        source = stack.enter_context(ManusSubscriber(glove_id=args.glove_id, domain_id=args.domain,
                                                     hand_side=hand_side))
        if interpolate:
            print(f"Interpolated MIT output: {profile.command.publish_hz:g} Hz; "
                  f"velocity={profile.command.max_velocity_rad_s:g} rad/s, "
                  f"acceleration={profile.command.max_acceleration_rad_s2:g} rad/s^2; "
                  f"startup={profile.command.startup_duration_s:g} s", flush=True)
        else:
            print("Direct MIT output: one command per solved glove frame, using latest JointState; "
                  "no position interpolation or startup ramp.", flush=True)
        session = CalibrationSession(calibration_frames=args.calibration_frames,
                                     hand_side=hand_side, normal_sign=args.normal_sign,
                                     cache_dir=args.cache_dir)
        started = time.monotonic()
        last_progress = 0.
        # Valid input wakes the solver immediately. Timeout wakes only service
        # maintenance/status/watchdogs when input is absent; it does not pace IK.
        maintenance_interval = min(.05, profile.command.max_timer_gap_s / 2,
                                   profile.command.target_timeout_s / 2)
        sequence = -1
        generation = 0
        solver = None
        solve_status = None
        target_model_q = target_hardware_q = None
        input_error = None
        last_valid = None
        last_transition = None
        valid_frames = 0
        try:
            while True:
                requested = transport.take_recalibration_request()
                sample = None
                if requested is None:
                    sample = source.wait_for_sample(after_sequence=sequence,
                                                    timeout=maintenance_interval)
                    # A service may invalidate calibration while this thread
                    # waits. Discard that sample before touching the old solver.
                    requested = transport.take_recalibration_request()
                processing_started = time.monotonic()
                if requested is not None:
                    generation = requested
                    session.reset(force=True)
                    solver = solve_status = None
                    target_model_q = target_hardware_q = None
                    input_error = last_valid = None
                    # Ignore the sample that was already cached at the request.
                    latest = source.latest()
                    if latest is not None:
                        sequence = latest.sequence
                    sample = None
                    print("Recalibration started; output paused. Enable again after READY.", flush=True)
                now = time.monotonic()
                if sample is not None:
                    sequence = sample.sequence
                    try:
                        if now-sample.received_monotonic > profile.command.target_timeout_s:
                            raise ValueError("Stale Manus frame")
                        if last_valid is not None and sample.received_monotonic-last_valid > profile.command.target_timeout_s:
                            transport.pause("manus_gap")
                            if solver is None:
                                session.clear_pending()
                        if solver is None:
                            cal = session.advance(sample.message)
                            if cal is not None:
                                solver = Retargeter(robot, cal, joint_limits=(
                                    profile.model_lower, profile.model_upper))
                        if solver is not None:
                            solve_started = time.monotonic()
                            q, solve_status = solver.solve(sample.message)
                            solve_finished = time.monotonic()
                            solve_status.update(
                                input_to_solve_ms=(solve_started-sample.received_monotonic)*1000,
                                total_solve_ms=(solve_finished-solve_started)*1000)
                            if not solve_status["accepted"]:
                                raise ValueError("Retarget optimizer rejected this frame")
                            # Exclude intentional idle waiting from the stall
                            # check, but pause before handing over a slow result.
                            if solve_finished-processing_started > profile.command.max_timer_gap_s:
                                transport.pause("solver_loop_stall")
                            target_model_q = q.tolist()
                            target_hardware_q = profile.model_to_hardware(q).tolist()
                            if not transport.set_target(q, sample.received_monotonic, generation):
                                rejected = transport.snapshot()
                                raise ValueError(rejected.get("last_error") or "Hardware controller rejected target")
                            solve_status['input_to_target_ms'] = (time.monotonic()-sample.received_monotonic)*1000
                        last_valid = sample.received_monotonic
                        valid_frames += 1
                        input_error = None
                    except (ValueError, KeyError, TypeError, FloatingPointError) as exc:
                        input_error = str(exc)
                        transport.pause("invalid_manus_or_target")
                        if solver is None:
                            session.clear_pending()
                now = time.monotonic()
                if last_valid is not None and now-last_valid > profile.command.target_timeout_s:
                    transport.pause("stale_manus")
                    if solver is None:
                        session.clear_pending()
                current = transport.snapshot()
                receiver = source.status()
                if receiver.get("thread_error") or current.get("thread_error"):
                    transport.pause("thread_error")
                    raise RuntimeError(receiver.get("thread_error") or current["thread_error"])
                calibration_status = session.status()
                phase = calibration_status['state'].upper()
                if last_valid is None or now-last_valid > profile.command.target_timeout_s:
                    phase = 'WAITING_INPUT'
                    input_error = receiver.get('last_error') or 'Waiting for fresh glove input'
                front = dict(phase=phase, calibrated=solver is not None,
                             calibration_count=calibration_status['count'],
                             calibration_frames=calibration_status['required_frames'],
                             glove_id=calibration_status['glove_id'], hand_side=hand_side,
                             cache_used=calibration_status['cache_used'],
                             cache_path=calibration_status['cache_path'],
                             cache_error=calibration_status['cache_error'],
                             valid_frames=valid_frames,
                             manus_age_s=None if last_valid is None else now-last_valid,
                             input_error=input_error, solve=solve_status,
                             reason=input_error or ('Checking saved bone lengths' if phase == 'CHECKING_CACHE'
                                    else 'Collecting bone lengths' if phase == 'CALIBRATING' else phase),
                             target_model_q=target_model_q, target_hardware_q=target_hardware_q)
                transport.update_retarget_status(front, generation)
                current = transport.snapshot()
                transition = (current['phase'], current.get('reason'), current.get('last_error'))
                changed = transition != last_transition
                last_transition = transition
                if changed or (not front['calibrated'] and now-last_progress >= 1.):
                    print(f"{now-started:6.1f}s {current['phase']} | "
                          f"calibration {front['calibration_count']}/{front['calibration_frames']} | "
                          f"{current['reason']}"
                          + (f" | cache: {front['cache_error']}" if front['cache_error'] else ""), flush=True)
                    last_progress = now
                if time.monotonic()-processing_started > profile.command.max_timer_gap_s:
                    transport.pause("solver_loop_stall")
        except KeyboardInterrupt:
            pass
        finally:
            transport.pause("exit")
