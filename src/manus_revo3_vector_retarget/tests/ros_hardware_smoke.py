#!/usr/bin/env python3
"""Isolated DDS test of hardware output, using simulated ROS feedback only.

No device driver, serial port, or physical hand is used. Live domains 25, 177 and 188 are forbidden;
the default is 197 with ROS_LOCALHOST_ONLY=1. Tests the actual windowless hardware CLI services and cache
with synthetic glove input and JointState-only feedback. Run: python src/manus_revo3_vector_retarget/tests/ros_hardware_smoke.py
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]


def bootstrap():
    if os.environ.get("_REVO3_HW_SMOKE_READY") == str(Path(__file__).resolve()):
        return
    workspace = Path(os.environ.get("MANUS_ROS_WORKSPACE", ROOT.parents[1]))
    setups = [Path("/opt/ros/humble/setup.bash")]
    for package in ("manus_ros2_msgs", "revo3_mit_controller_msgs"):
        path = workspace / f"install/{package}/share/{package}/package.bash"
        if not path.is_file():
            path = workspace / f"install/share/{package}/package.bash"
        setups.append(path)
    for path in setups:
        if not path.is_file():
            raise FileNotFoundError(path)
    environment = dict(os.environ, PYTHONPATH=str(ROOT), PYTHONNOUSERSITE=os.environ.get("PYTHONNOUSERSITE", "0"),
                       PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1",
                       ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR="/tmp/revo3_hw_smoke/ros")
    Path(environment["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    command = 'set -e; for setup in "$@"; do unset COLCON_CURRENT_PREFIX; source "$setup"; done; env -0'
    result = subprocess.run(["bash", "--noprofile", "--norc", "-c", command, "hw-smoke", *map(str, setups)],
                            check=True, capture_output=True, env=environment)
    environment = dict(item.split("=", 1) for item in result.stdout.decode().split("\0") if "=" in item)
    environment["_REVO3_HW_SMOKE_READY"] = str(Path(__file__).resolve())
    python = sys.executable
    os.execve(python, [python, str(Path(__file__).resolve()), *sys.argv[1:]], environment)


class MockController:
    """JointState-only feedback, synthetic glove input, and a real DDS sink."""

    def __init__(self, profile, domain, node_name=None):
        import numpy as np
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from manus_ros2_msgs.msg import ManusGlove
        from revo3_mit_controller_msgs.msg import Revo3MITCommand
        from rosidl_runtime_py.set_message import set_message_fields
        from ros_live_smoke import load_fixture

        self.profile, self.side = profile, profile.hand_side
        self.context = Context()
        self.context.init(args=[], domain_id=domain)
        self.node = Node("joint_forward_mit_controller", namespace=f"/revo3_{self.side}",
                         context=self.context, enable_rosout=False, start_parameter_services=False)
        self.lock = threading.RLock()
        self.commands, self.statuses, self.errors = [], [], []
        self.latest_status = None
        self.feedback_enabled, self.glove_enabled = True, False
        self.wrong_glove_enabled = False
        self.glove_frames = 0
        self.glove_id = -19701234 if self.side == "right" else -19701235
        self.topic = f"/revo3_{self.side}/joint_forward_mit_controller/commands"
        # Deliberately use both publisher numbers across the two test runs.
        # Runtime selection must follow the message's side, not topic order.
        self.glove_topic = "/manus_glove_1" if self.side == "right" else "/manus_glove_0"
        other_glove_topic = "/manus_glove_0" if self.side == "right" else "/manus_glove_1"
        self.service_prefix = "/" + (node_name or f"manus_revo3_vector_retarget_{self.side}")
        self.JointState, self.ManusGlove = JointState, ManusGlove
        self.SetBool, self.Trigger = SetBool, Trigger
        self.fixture, self.set_fields = load_fixture(), set_message_fields
        self.q = np.clip(np.zeros(21), profile.lower, profile.upper)
        self.feedback_publisher = self.node.create_publisher(
            JointState, f"/revo3_{self.side}/revo3_joint_state/joint_states", qos_profile_sensor_data)
        self.glove_publisher = self.node.create_publisher(ManusGlove, self.glove_topic, qos_profile_sensor_data)
        self.wrong_glove_publisher = self.node.create_publisher(ManusGlove, other_glove_topic, qos_profile_sensor_data)
        self.node.create_subscription(Revo3MITCommand, self.topic, self.receive, 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, self.service_prefix + "/status", self.receive_status, status_qos)
        self.enable_client = self.node.create_client(SetBool, self.service_prefix + "/enable_retarget")
        self.recalibrate_client = self.node.create_client(Trigger, self.service_prefix + "/recalibrate")
        self.node.create_timer(.02, self.feedback)
        self.node.create_timer(1. / 60., self.glove)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.spin, name="synthetic-hardware-dds", daemon=True)
        self.thread.start()

    def spin(self):
        try:
            while not self.stop.is_set():
                self.executor.spin_once(timeout_sec=.05)
        except BaseException as exc:
            if not self.stop.is_set():
                self.errors.append(repr(exc))

    def feedback(self):
        if not self.feedback_enabled:
            return
        message = self.JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(self.profile.names)
        message.position, message.velocity = self.q.tolist(), [0.] * 21
        self.feedback_publisher.publish(message)

    def glove(self):
        if not self.glove_enabled and not self.wrong_glove_enabled:
            return
        import math
        bend = 30. + 20. * math.sin(self.glove_frames / 60. * 2. * math.pi / 2.)
        fields = self.fixture(pip_bend_degrees=bend)
        fields["glove_id"] = self.glove_id
        if self.side == "left":
            fields["side"] = "Left"
            for node in fields["raw_nodes"]:
                node["pose"]["position"]["x"] *= -1.
        if self.glove_enabled:
            message = self.ManusGlove()
            self.set_fields(message, fields)
            self.glove_publisher.publish(message)
        if self.wrong_glove_enabled:
            wrong_fields = deepcopy(fields)
            wrong_fields["side"] = "Left" if self.side == "right" else "Right"
            message = self.ManusGlove()
            self.set_fields(message, wrong_fields)
            self.wrong_glove_publisher.publish(message)
        self.glove_frames += 1

    def receive(self, message):
        with self.lock:
            self.commands.append(dict(received_at=time.monotonic(),
                stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                names=list(message.joint_names), q=list(message.position), dq=list(message.velocity),
                kp=list(message.kp), kd=list(message.kd), effort=list(message.effort)))

    def receive_status(self, message):
        with self.lock:
            self.latest_status = dict(received_at=time.monotonic(), status=json.loads(message.data))
            self.statuses.append(self.latest_status)

    def status(self, after=0.):
        with self.lock:
            if self.latest_status is None or self.latest_status["received_at"] < after:
                return None
            return deepcopy(self.latest_status["status"])

    def command_rows(self, after=0.):
        with self.lock:
            return [dict(row) for row in self.commands if row["received_at"] > after]

    def call(self, process, enabled=None):
        client = self.recalibrate_client if enabled is None else self.enable_client
        wait_until(process, lambda: client.service_is_ready(), "service discovery", timeout=15.)
        request = self.Trigger.Request() if enabled is None else self.SetBool.Request(data=enabled)
        future = client.call_async(request)
        wait_until(process, future.done, "service response", timeout=8.)
        return future.result()

    def close(self):
        self.stop.set()
        self.executor.wake()
        self.thread.join(timeout=3.)
        self.executor.shutdown(timeout_sec=3.)
        self.node.destroy_node()
        self.context.shutdown()


class Consumer:
    """An actual CLI subprocess, always stopped and reaped by the test."""

    def __init__(self, args, mock, name, extra_args=()):
        # The test owns reports/stdout capture. The runtime gets an empty cwd
        # and may persist only bone lengths in the separate explicit cache.
        self.run_dir = args.output / name
        self.run_dir.mkdir()
        self.log_path = args.output / f"{name}.log"
        self.log = self.log_path.open("w")
        command = [sys.executable, "-m", "retarget_v2.cli", "hardware", "--model", args.model,
                   "--hand-side", args.hand_side, "--domain", str(args.domain),
                   "--glove-id", str(mock.glove_id),
                   "--cache-dir", str(args.output / "cache"), *extra_args]
        if args.interpolate:
            command.append("--interpolate")
        if args.node_name:
            command.extend(["--node-name", args.node_name])
        environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1",
                           RETARGET_RESULTS_DIR=str(self.run_dir / "results"),
                           PYTHONPATH=str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""))
        try:
            self.process = subprocess.Popen(command, env=environment, cwd=self.run_dir,
                                            stdout=self.log, stderr=subprocess.STDOUT)
        except BaseException:
            self.log.close()
            raise

    def check(self):
        if self.process.poll() is not None:
            raise RuntimeError(f"Consumer exited {self.process.returncode}; see {self.log_path}")

    def assert_no_runtime_files(self):
        created = sorted(str(path.relative_to(self.run_dir)) for path in self.run_dir.rglob("*"))
        assert not created, f"Runtime created files in its empty working directory: {created}"

    def close(self, verify=False):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
        self.log.close()
        if verify:
            assert self.process.returncode == 0, f"Consumer shutdown failed; see {self.log_path}"
            self.assert_no_runtime_files()


def wait_until(process, predicate, description, timeout=15.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None:
            process.check()
        value = predicate()
        if value:
            return value
        time.sleep(.02)
    raise AssertionError(f"Timed out waiting for {description}")


def wait_status(process, mock, predicate, after=0., timeout=15.):
    def inspect():
        assert not mock.errors, mock.errors
        state = mock.status(after)
        return state if state is not None and predicate(state) else None
    try:
        return wait_until(process, inspect, "status condition", timeout)
    except AssertionError as exc:
        raise AssertionError(f"{exc}; last status: {mock.status()}") from exc


def observe(process, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        process.check()
        time.sleep(.02)


def assert_stopped(mock, since):
    assert not mock.command_rows(since + .05), "Commands continued beyond the 50 ms DDS drain allowance"


def verify_commands(rows, profile, interpolate):
    import numpy as np
    assert len(rows) >= (300 if interpolate else 150), f"Too few MIT messages: {len(rows)}"
    for row in rows:
        assert tuple(row["names"]) == profile.names
        for field in ("q", "dq", "kp", "kd", "effort"):
            assert len(row[field]) == 21 and np.isfinite(row[field]).all(), (field, row)
        assert np.all(np.asarray(row["q"]) >= profile.lower - 1e-9)
        assert np.all(np.asarray(row["q"]) <= profile.upper + 1e-9)
        np.testing.assert_allclose(row["kp"], profile.kp)
        np.testing.assert_allclose(row["kd"], profile.kd)
        assert not np.any(row["effort"])
    maximum_velocity = float(np.max(np.abs([row["dq"] for row in rows])))
    rate = (len(rows) - 1) / (rows[-1]["stamp"] - rows[0]["stamp"])
    lower_rate, upper_rate = (170., 230.) if interpolate else (57., 63.)
    assert lower_rate <= rate <= upper_rate, f"Unexpected MIT publication rate: {rate:.1f} Hz"
    assert maximum_velocity > .01, "MIT v_des remained zero for a moving target"
    return dict(messages=len(rows), rate_hz=rate, max_vdes_rad_s=maximum_velocity)


def verify_direct_targets(rows, status_rows):
    """Compare whole DDS commands with solver targets exposed by real status."""
    import numpy as np
    maximum_error = 0.
    for status_row in status_rows:
        target = status_row["status"]["retarget"]["target_hardware_q"]
        # Status and command use distinct DDS topics. Permit their callbacks
        # to arrive out of order while still comparing this exact target.
        candidates = [row["q"] for row in rows
                      if abs(row["received_at"] - status_row["received_at"]) < .1]
        assert candidates, "No command near enabled status"
        error = float(np.min(np.max(np.abs(np.asarray(candidates) - target), axis=1)))
        assert error <= 1e-9, f"Direct output differed from solved target by {error} rad"
        maximum_error = max(maximum_error, error)
    return dict(matched_targets=len(status_rows), maximum_error_rad=maximum_error)


def run(args):
    from retarget_v2.model import RobotModel
    from retarget_v2.hardware_profile import HardwareProfile

    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, synthetic=True, ros_domain_id=args.domain,
                  hand_side=args.hand_side, interpolate=args.interpolate, physical_driver_started=False,
                  feedback_interfaces=["sensor_msgs/msg/JointState"])
    mock = consumer = None
    try:
        with RobotModel(model_version=args.model, hand_side=args.hand_side) as robot:
            profile = HardwareProfile(robot)
            mock = MockController(profile, args.domain, args.node_name)
            consumer = Consumer(args, mock, "first_run")
            rejected = mock.call(consumer, enabled=True)
            assert not rejected.success, "Enable succeeded before glove input/calibration"
            report["early_enable_rejected"] = rejected.message
            assert not mock.command_rows()
            wrong_started = time.monotonic()
            mock.wrong_glove_enabled = True
            observe(consumer, .8)
            wrong_only = wait_status(consumer, mock, lambda state: not state["calibrated"], after=wrong_started)
            assert wrong_only["calibration_count"] == 0, "Opposite hand populated the calibration window"
            assert not mock.command_rows()
            mock.wrong_glove_enabled = False
            mock.glove_enabled = True
            ready = wait_status(consumer, mock, lambda state: state["phase"] == "READY")
            assert ready["calibration_count"] == ready["calibration_frames"] == 120, ready
            assert ready["cache_used"] is False
            assert ready["feedback_source"] == "joint_state"
            assert not mock.command_rows(), "Calibration completion published commands without enable"
            report["first_calibration_frames"] = ready["calibration_count"]
            report["glove_selection"] = dict(topic=mock.glove_topic, selected_by="side", wrong_side_rejected=True)
            print("First 120-frame calibration ready; output remains disabled", flush=True)

            # Only an initial JointState is needed: stopping that stream must
            # not recreate the removed hardware sampling watchdog.
            mock.feedback_enabled = False
            enabled = mock.call(consumer, enabled=True)
            assert enabled.success, enabled.message
            started = time.monotonic()
            observe(consumer, 3.2)
            running = wait_status(consumer, mock, lambda state: state["enabled"], after=started)
            assert running["feedback_age_s"] > .25, running
            assert running["interpolate"] is args.interpolate, running
            assert running["output_mode"] == ("interpolated" if args.interpolate else "direct"), running
            command_rows = mock.command_rows(started)
            report["mit_output"] = verify_commands(command_rows, profile, args.interpolate)
            # Verify the actual CLI consumes 60 Hz input without the former
            # 60 Hz gate + polling overhead reducing throughput to 54-55 Hz.
            with mock.lock:
                samples = [row for row in mock.statuses
                           if row["received_at"] > started and row["status"]["enabled"]]
            first, last = samples[0], samples[-1]
            interval = last["received_at"] - first["received_at"]
            assert interval >= 2., "Need multiple status periods to measure solve throughput"
            frames = (last["status"]["retarget"]["valid_frames"]
                      - first["status"]["retarget"]["valid_frames"])
            solve_rate = frames / interval
            assert 57. <= solve_rate <= 63., f"60 Hz input solved at {solve_rate:.1f} Hz"
            timings = [row["status"]["retarget"]["solve"] for row in samples]
            for timing in timings:
                assert 0. <= timing["input_to_solve_ms"] <= timing["input_to_target_ms"]
                assert 0. <= timing["total_solve_ms"] <= timing["input_to_target_ms"]
            report["solver_schedule"] = dict(input_hz=60., solve_hz=solve_rate,
                input_to_solve_ms=[t["input_to_solve_ms"] for t in timings],
                total_solve_ms=[t["total_solve_ms"] for t in timings],
                input_to_target_ms=[t["input_to_target_ms"] for t in timings])
            if not args.interpolate:
                report["direct_targets"] = verify_direct_targets(command_rows, samples)
            report["feedback_sampling_not_required_after_initial_pose"] = True

            # A direct publisher must not repeat the last target on a timer.
            # Then exercise expiry and prove fresh input cannot auto-enable.
            mock.glove_enabled = False
            gap_at = time.monotonic()
            observe(consumer, .18)
            if not args.interpolate:
                assert_stopped(mock, gap_at)
                report["direct_stops_without_repeated_target"] = True
            timed_out = wait_status(consumer, mock, lambda state: not state["enabled"], after=gap_at)
            assert timed_out["calibrated"]
            observe(consumer, .15)
            assert_stopped(mock, gap_at + .3)
            resumed_at = time.monotonic()
            mock.glove_enabled = True
            resumed = wait_status(consumer, mock, lambda state: state["phase"] == "READY", after=resumed_at)
            assert not resumed["enabled"]
            observe(consumer, .25)
            assert_stopped(mock, resumed_at)
            assert mock.call(consumer, enabled=True).success
            observe(consumer, .3)
            assert mock.command_rows(resumed_at), "Explicit enable after input gap produced no command"
            report["input_gap"] = dict(paused=True, auto_resumed=False, explicit_enable_resumed=True)

            paused = mock.call(consumer, enabled=False)
            assert paused.success, paused.message
            paused_at = time.monotonic()
            status = wait_status(consumer, mock, lambda state: not state["enabled"], after=paused_at)
            assert status["calibrated"] and status["calibration_count"] == 120
            observe(consumer, .3)
            assert_stopped(mock, paused_at)
            report["pause_retains_calibration"] = True

            # Withhold input while requesting recalibration. Trigger must reply
            # before any samples can be collected and invalidate the old target.
            assert mock.call(consumer, enabled=True).success
            observe(consumer, .3)
            mock.glove_enabled = False
            requested_at = time.monotonic()
            accepted = mock.call(consumer)
            response_at = time.monotonic()
            assert accepted.success, accepted.message
            assert response_at - requested_at < 1.5, "Recalibration service waited for sample collection"
            assert not mock.call(consumer, enabled=True).success
            unready = wait_status(consumer, mock, lambda state: not state["calibrated"], after=response_at)
            assert unready["calibration_count"] == 0, unready
            assert_stopped(mock, response_at)
            mock.glove_enabled = True
            ready = wait_status(consumer, mock, lambda state: state["phase"] == "READY", after=response_at)
            assert ready["calibration_count"] == ready["calibration_frames"] == 120
            assert ready["cache_used"] is False
            observe(consumer, .3)
            assert_stopped(mock, response_at)
            report["recalibration"] = dict(service_response_s=response_at - requested_at,
                                            frames=ready["calibration_count"], auto_resumed=False)
            enabled_at = time.monotonic()
            assert mock.call(consumer, enabled=True).success
            observe(consumer, .4)
            assert mock.command_rows(enabled_at), "Explicit enable after recalibration produced no command"
            assert mock.call(consumer, enabled=False).success
            print("Services verified: pause retains calibration; recalibration requires explicit enable", flush=True)

            consumer.close(verify=True)
            consumer = None
            wait_until(None, lambda: not mock.node.get_publishers_info_by_topic(mock.topic), "old publisher removal")
            mock.glove_enabled = False
            mock.feedback_enabled = True
            restarted_at = time.monotonic()
            before_restart = len(mock.command_rows())
            consumer = Consumer(args, mock, "cached_run")
            assert not mock.call(consumer, enabled=True).success
            mock.glove_enabled = True
            cached = wait_status(consumer, mock, lambda state: state["phase"] == "READY", after=restarted_at)
            assert cached["cache_used"] is True, cached
            assert cached["calibration_count"] == cached["calibration_frames"] == 30, cached
            assert len(mock.command_rows()) == before_restart, "Restart automatically enabled output"
            assert mock.call(consumer, enabled=True).success
            observe(consumer, .4)
            assert len(mock.command_rows()) > before_restart
            assert mock.call(consumer, enabled=False).success
            consumer.close(verify=True)
            consumer = None
            report["cache_restart"] = dict(verification_frames=30, cache_used=True, auto_enabled=False)

            # Also exercise a real runtime startup failure, not argparse: a
            # missing hardware profile must leave no partial session files.
            consumer = Consumer(args, mock, "failed_run", extra_args=(
                "--profile", str(args.output / "missing_profile.yaml")))
            wait_until(None, lambda: consumer.process.poll() is not None, "expected startup failure")
            assert consumer.process.returncode != 0, "Missing hardware profile unexpectedly succeeded"
            consumer.assert_no_runtime_files()
            consumer.close()
            consumer = None
            cache_files = list((args.output / "cache").iterdir())
            assert len(cache_files) == 1 and cache_files[0].suffix == ".json", cache_files
            report["runtime_files"] = dict(normal_exit=[], failed_exit=[], cache_files=1,
                                            test_reports_owned_by_tester=True)
            report["passed"] = True
            print(json.dumps(report, indent=2), flush=True)
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if consumer is not None:
            consumer.close()
        if mock is not None:
            report["mock_errors"] = list(mock.errors)
            (args.output / "commands.jsonl").write_text("".join(json.dumps(row) + "\n" for row in mock.command_rows()))
            with mock.lock:
                (args.output / "status.jsonl").write_text("".join(json.dumps(row) + "\n" for row in mock.statuses))
            mock.close()
        (args.output / "smoke_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("may3",), default="may3")
    parser.add_argument("--hand-side", choices=("right", "left"), default="right")
    parser.add_argument("--node-name", help="Exercise bringup-compatible node and service names")
    parser.add_argument("--domain", type=int, default=197)
    parser.add_argument("--interpolate", action="store_true",
                        help="Test the optional 200 Hz interpolator; default tests direct per-frame publication")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "results" / datetime.now().strftime("ros_hardware_smoke_%Y%m%d_%H%M%S_%f"))
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or args.domain in (25, 177, 188):
        parser.error("Use isolated domain 0..232; live domains 25, 177 and 188 are forbidden")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    bootstrap()
    run(args)


if __name__ == "__main__":
    main()
