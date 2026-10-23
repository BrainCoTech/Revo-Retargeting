#!/usr/bin/env python3
"""Exercise the actual live CLI over isolated ROS DDS with synthetic Manus data.

Run with ``python src/manus_revo3_vector_retarget/tests/ros_live_smoke.py``. The script reloads into
the same Python/ROS environment as run.sh, publishes only on domain 197 by
default, and refuses live domains 25 and 188. No hardware driver is started.
The test observes control state in memory and prints its report; it creates no
results directory or persistent trace files.
This is intentionally a standalone integration test, not a pytest collection.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]


def ros_environment():
    """Load Humble and the Manus wire-interface package, without old algorithms."""
    workspace = Path(os.environ.get("MANUS_ROS_WORKSPACE", ROOT.parents[1]))
    setups = [Path("/opt/ros/humble/setup.bash"),
              workspace / "install/manus_ros2_msgs/share/manus_ros2_msgs/package.bash"]
    if not setups[1].is_file():
        setups[1] = workspace / "install/share/manus_ros2_msgs/package.bash"
    for path in setups:
        if not path.is_file():
            raise FileNotFoundError(f"ROS setup missing: {path}; check MANUS_ROS_WORKSPACE")
    environment = dict(os.environ, PYTHONPATH=str(ROOT), PYTHONNOUSERSITE=os.environ.get("PYTHONNOUSERSITE", "0"),
                       PYTHONDONTWRITEBYTECODE="1", ROS_LOCALHOST_ONLY="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    command = 'set -e; source "$1"; unset COLCON_CURRENT_PREFIX; source "$2"; env -0'
    result = subprocess.run(["bash", "--noprofile", "--norc", "-c", command, "ros-smoke",
                             *map(str, setups)], env=environment, check=True, capture_output=True)
    environment = dict(item.split("=", 1) for item in result.stdout.decode().split("\0") if "=" in item)
    environment["PYTHONPATH"] = str(ROOT) + ":" + environment.get("PYTHONPATH", "")
    return environment


def bootstrap():
    if os.environ.get("_REVO3_SMOKE_ROS_READY") == str(Path(__file__).resolve()):
        return
    environment = ros_environment()
    environment["_REVO3_SMOKE_ROS_READY"] = str(Path(__file__).resolve())
    python = sys.executable
    os.execve(str(python), [str(python), str(Path(__file__).resolve()), *sys.argv[1:]], environment)


def load_fixture():
    # The fixture lives with SDK contract tests, and imports pytest decorators.
    # Optional custom interpreters may not include pytest. Append Ubuntu's
    # packages only when needed, preserving the interpreter's numerical runtime.
    if importlib.util.find_spec("pytest") is None:
        sys.path.append("/usr/lib/python3/dist-packages")
    spec = importlib.util.spec_from_file_location("independent_sdk_test_fixture", ROOT / "tests/test_input.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.sdk_hand


def verify(observation, published):
    import numpy as np
    state = observation["state"]
    events, trace = observation["events"], observation["trace"]
    assert state["complete"], state
    assert state["model_version"] == "may3", state.get("model_version")
    assert state["model_metadata"]["description_package"] == "revo3_description"
    assert state["hardware_commands"] is False
    assert state["contacts_enabled"] is False
    tracking = [event for event in events if event["state"] == "tracking"]
    held = [event for event in events if event["state"] == "hold" and event["reason"] == "stale_input"]
    assert tracking and held, f"Expected tracking then a stale-input hold: {events}"
    first_hold = held[0]
    assert tracking[0]["t"] < first_hold["t"]
    assert not any(event["state"] == "tracking" and event["t"] > first_hold["t"] for event in events), \
        "Consumer resumed tracking without explicit operator input"
    controller, receiver = state["controller"], state["receiver"]
    assert controller["valid_frames"] > first_hold["valid_frames"] + 40, \
        "Recovery did not process enough new valid frames"
    assert receiver["selected_glove_id"] == -19701234
    assert receiver["rejected"] == 0, receiver
    assert receiver["robot_command_publishers"] == 0
    assert state["physics"]["mujoco_warning_count"] == 0, state["physics"]
    for row in trace:
        for field in ("q", "dq", "q_sim"):
            assert len(row[field]) == 21 and np.isfinite(row[field]).all(), (field, row)
    settled = [row for row in trace if row["t"] >= first_hold["t"] + .1]
    assert len(settled) >= 50, "Insufficient trace after the watchdog hold"
    assert all(row["state"] == "hold" for row in settled), "State left hold after stream recovery"
    q = np.array([row["q"] for row in settled])
    dq = np.array([row["dq"] for row in settled])
    np.testing.assert_allclose(q, np.broadcast_to(q[0], q.shape), atol=1e-12, rtol=0.)
    np.testing.assert_allclose(dq, 0., atol=1e-12, rtol=0.)
    moving = np.array([row["q"] for row in trace if row["state"] == "tracking"])
    assert np.max(np.ptp(moving, axis=0)) > .05, "Synthetic shape changes did not move the simulated command"
    return dict(passed=True, synthetic=True, hardware_commands=False, model_version=state["model_version"],
                ros_domain_id=state["ros_domain_id"], topic=state["topic"],
                glove_id=receiver["selected_glove_id"], published_frames=published,
                received_frames=receiver["received"], valid_frames=controller["valid_frames"],
                valid_frames_at_stale_hold=first_hold["valid_frames"],
                tracking_started_s=tracking[0]["t"], stale_hold_s=first_hold["t"],
                recovery_auto_resumed=False, held_trace_samples=len(settled),
                held_command_variation_rad=float(np.ptp(q, axis=0).max()),
                mujoco_warning_count=state["physics"]["mujoco_warning_count"],
                events=events)


def consumer(args):
    """Test-only instrumentation around the actual CLI, using memory only."""
    from retarget_v2 import cli, live

    observation = dict(state={}, events=[], trace=[])
    started = time.monotonic()
    previous_transition = None

    class ObservedController(live.LiveController):
        def status(self, now):
            nonlocal previous_transition
            current = super().status(now)
            elapsed = now - started
            transition = (current["state"], current["stop_reason"], current["last_error"])
            if transition != previous_transition:
                observation["events"].append(dict(
                    t=elapsed, state=current["state"], reason=current["stop_reason"],
                    error=current["last_error"], valid_frames=current["valid_frames"],
                ))
                previous_transition = transition
            observation["trace"].append(dict(
                t=elapsed, state=current["state"], q=current["command_q"],
                dq=current["command_dq"], q_sim=current["measured_q"],
            ))
            return current

        def close(self):
            observation["state"].update(
                controller=self.status(time.monotonic()), physics=self.simulation.report,
                model_version=self.robot.model_version, model_metadata=self.robot.model_metadata,
            )
            super().close()

    class ObservedSubscriber(live.ManusSubscriber):
        def close(self):
            observation["state"]["receiver"] = self.status()
            super().close()

    live.LiveController = ObservedController
    live.ManusSubscriber = ObservedSubscriber
    sys.argv = ["retarget", "live", "--headless", "--auto-start",
                "--domain", str(args.domain), "--glove-id", "-19701234",
                "--calibration-frames", "30"]
    cli.main()
    observation["state"].update(complete=True, hardware_commands=False, contacts_enabled=False,
                                 ros_domain_id=args.domain, topic=args.topic)
    print("SMOKE_OBSERVATION=" + json.dumps(observation, allow_nan=False), flush=True)


def run(args):
    import numpy as np
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.set_message import set_message_fields
    from manus_ros2_msgs.msg import ManusGlove

    sdk_hand = load_fixture()
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    command = [sys.executable, str(Path(__file__).resolve()), "--consumer",
               "--domain", str(args.domain), "--topic", args.topic]
    context = Context()
    context.init(args=[], domain_id=args.domain)
    node = Node("independent_manus_synthetic_smoke", context=context, enable_rosout=False,
                start_parameter_services=False)
    publisher = node.create_publisher(ManusGlove, args.topic, qos_profile_sensor_data)
    process = None
    published = 0
    report = dict(passed=False, synthetic=True, ros_domain_id=args.domain, topic=args.topic)
    try:
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
            ready_deadline = time.monotonic() + 4.
            while publisher.get_subscription_count() < 1:
                if process.poll() is not None:
                    log.seek(0)
                    raise RuntimeError("Live consumer exited before discovery:\n" + log.read())
                if time.monotonic() >= ready_deadline:
                    raise RuntimeError("DDS subscriber discovery timed out")
                time.sleep(.02)
            print(f"Synthetic DDS subscriber discovered on domain {args.domain}; running 2+2+1+2 second stimulus", flush=True)
            started = next_frame = time.monotonic()
            while time.monotonic() - started < 7.:
                elapsed = time.monotonic() - started
                if process.poll() is not None:
                    log.seek(0)
                    raise RuntimeError("Live consumer exited during stimulus:\n" + log.read())
                # 0-2 s: stable natural-open calibration; 2-4 s: flexion changes.
                # 4-5 s: deliberate silence; 5-7 s: different moving hand shape.
                if not 4. <= elapsed < 5.:
                    bend = 0. if elapsed < 2. else (35. + 25. * np.sin(2. * np.pi * elapsed / 1.8))
                    if elapsed >= 5.:
                        bend = 60. + 15. * np.sin(2. * np.pi * elapsed / .8)
                    fields = sdk_hand(pip_bend_degrees=float(bend))
                    fields["glove_id"] = -19701234
                    message = ManusGlove()
                    set_message_fields(message, fields)
                    publisher.publish(message)
                    published += 1
                next_frame += 1. / 60.
                time.sleep(max(0., next_frame - time.monotonic()))
            # The consumer runs continuously; the test owns its observation
            # window and requests the same graceful shutdown as Ctrl+C.
            while time.monotonic() - started < 10.:
                if process.poll() is not None:
                    log.seek(0)
                    raise RuntimeError("Live consumer exited before shutdown:\n" + log.read())
                time.sleep(.02)
            process.send_signal(signal.SIGINT)
            returncode = process.wait(timeout=8.)
            if returncode != 0:
                log.seek(0)
                raise RuntimeError(f"Live consumer exited {returncode}:\n" + log.read())
            log.seek(0)
            output = log.read()
            observation = next((json.loads(line.split("=", 1)[1]) for line in output.splitlines()
                                if line.startswith("SMOKE_OBSERVATION=")), None)
            assert observation is not None, output
        report = verify(observation, published)
        print(json.dumps(report, indent=2), flush=True)
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        context.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=197)
    parser.add_argument("--topic", choices=("/manus_glove_0", "/manus_glove_1"), default="/manus_glove_0")
    parser.add_argument("--consumer", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or args.domain in (25, 188):
        parser.error("Use an isolated ROS domain in 0..232; live domains 25 and 188 are forbidden")
    bootstrap()
    consumer(args) if args.consumer else run(args)


if __name__ == "__main__":
    main()
