#!/usr/bin/env python3
"""Probe Revo3 ROS joint mapping with one-joint-at-a-time sine commands."""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState

from revo3_mit_controller_msgs.msg import Revo3MITCommand


JOINT_SUFFIXES = (
    "little_MPR_joint",
    "little_MCP_joint",
    "little_PIP_joint",
    "little_DIP_joint",
    "ring_MPR_joint",
    "ring_MCP_joint",
    "ring_PIP_joint",
    "ring_DIP_joint",
    "middle_MPR_joint",
    "middle_MCP_joint",
    "middle_PIP_joint",
    "middle_DIP_joint",
    "index_MPR_joint",
    "index_MCP_joint",
    "index_PIP_joint",
    "index_DIP_joint",
    "thumb_MCP_joint",
    "thumb_PIP_joint",
    "thumb_DIP_joint",
    "thumb_CMP_joint",
    "thumb_CMR_joint",
)


@dataclass
class ProbeResult:
    joint: str
    response_p2p_deg: float
    max_other_p2p_deg: float
    max_other_joint: str
    velocity_p2p_deg_s: float
    max_other_velocity_p2p_deg_s: float
    max_other_velocity_joint: str
    current_p2p: float
    max_other_current_p2p: float
    max_other_current_joint: str
    status: str


def _topic(namespace: str, suffix: str) -> str:
    namespace = namespace.strip().strip("/")
    suffix = suffix.strip().strip("/")
    return f"/{namespace}/{suffix}" if namespace else f"/{suffix}"


class Revo3JointMappingProbe(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("revo3_joint_mapping_probe")
        self.args = args
        self.side = args.side.strip().lower()
        self.joint_names = [f"{self.side}_{suffix}" for suffix in JOINT_SUFFIXES]
        namespace = args.namespace or f"revo3_{self.side}"
        self.command_topic = args.command_topic or _topic(namespace, "joint_forward_mit_controller/commands")
        self.state_topic = args.state_topic or _topic(namespace, "revo3_joint_state/joint_states")
        self.dynamic_state_topic = (
            args.dynamic_state_topic or _topic(namespace, "revo3_joint_state/dynamic_joint_states")
        )

        self.publisher = self.create_publisher(Revo3MITCommand, self.command_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.dynamic_subscription = self.create_subscription(
            DynamicJointState,
            self.dynamic_state_topic,
            self._on_dynamic_joint_state,
            qos_profile_sensor_data,
        )
        self.latest_state: JointState | None = None
        self.latest_dynamic_state: DynamicJointState | None = None
        self.state_msg_count = 0
        self.dynamic_state_msg_count = 0
        self.seen_joint_names: list[str] = []
        self.stop_requested = False
        self.record_enabled = bool(args.record or args.output_dir)
        self.record_rows: list[dict[str, object]] = []
        self.record_prefix = self._make_record_prefix()

        self.get_logger().info(f"command_topic={self.command_topic}")
        self.get_logger().info(f"state_topic={self.state_topic}")
        self.get_logger().info(f"dynamic_state_topic={self.dynamic_state_topic}")
        if self.record_enabled:
            self.get_logger().info(f"record_prefix={self.record_prefix}")

    def _make_record_prefix(self) -> Path | None:
        if not self.record_enabled:
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(self.args.output_dir or "/tmp/revo3_joint_mapping").expanduser()
        prefix = self.args.output_prefix or f"revo3_{self.side}_joint_mapping_{stamp}"
        return output_dir / prefix

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_state = msg
        self.state_msg_count += 1
        self.seen_joint_names = list(msg.name)

    def _on_dynamic_joint_state(self, msg: DynamicJointState) -> None:
        self.latest_dynamic_state = msg
        self.dynamic_state_msg_count += 1

    def _state_positions(self) -> dict[str, float] | None:
        msg = self.latest_state
        if msg is None:
            return None
        count = min(len(msg.name), len(msg.position))
        return {msg.name[i]: float(msg.position[i]) for i in range(count)}

    def _state_values(self, field: str) -> dict[str, float] | None:
        msg = self.latest_state
        if msg is None:
            return None
        values = getattr(msg, field)
        count = min(len(msg.name), len(values))
        return {msg.name[i]: float(values[i]) for i in range(count)}

    def _dynamic_values(self, interface_name: str) -> dict[str, float] | None:
        msg = self.latest_dynamic_state
        if msg is None:
            return None
        values: dict[str, float] = {}
        count = min(len(msg.joint_names), len(msg.interface_values))
        for i in range(count):
            interface_value = msg.interface_values[i]
            try:
                value_index = interface_value.interface_names.index(interface_name)
            except ValueError:
                continue
            if value_index < len(interface_value.values):
                values[msg.joint_names[i]] = float(interface_value.values[value_index])
        return values

    def _record_sample(
        self,
        probe_index: int,
        probe_joint: str,
        probe_time_s: float,
        command_positions: list[float],
        command_velocities: list[float],
        positions: dict[str, float] | None,
        velocities: dict[str, float] | None,
        currents: dict[str, float] | None,
    ) -> None:
        if not self.record_enabled:
            return
        row: dict[str, object] = {
            "wall_time_s": f"{time.time():.9f}",
            "probe_index": probe_index,
            "probe_joint": probe_joint,
            "probe_time_s": f"{probe_time_s:.9f}",
        }
        positions = positions or {}
        velocities = velocities or {}
        currents = currents or {}
        for idx, name in enumerate(self.joint_names):
            row[f"cmd_pos_deg:{name}"] = f"{math.degrees(command_positions[idx]):.9f}"
            row[f"cmd_vel_deg_s:{name}"] = f"{math.degrees(command_velocities[idx]):.9f}"
            row[f"state_pos_deg:{name}"] = (
                f"{math.degrees(positions[name]):.9f}" if name in positions else ""
            )
            row[f"state_vel_deg_s:{name}"] = (
                f"{math.degrees(velocities[name]):.9f}" if name in velocities else ""
            )
            row[f"state_current:{name}"] = f"{currents[name]:.9f}" if name in currents else ""
        self.record_rows.append(row)

    def _record_fieldnames(self) -> list[str]:
        fields = ["wall_time_s", "probe_index", "probe_joint", "probe_time_s"]
        for name in self.joint_names:
            fields.extend(
                [
                    f"cmd_pos_deg:{name}",
                    f"cmd_vel_deg_s:{name}",
                    f"state_pos_deg:{name}",
                    f"state_vel_deg_s:{name}",
                    f"state_current:{name}",
                ]
            )
        return fields

    def write_record_files(self, results: list[ProbeResult]) -> None:
        if not self.record_enabled or self.record_prefix is None:
            return
        self.record_prefix.parent.mkdir(parents=True, exist_ok=True)

        samples_path = self.record_prefix.with_suffix(".samples.csv")
        with samples_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self._record_fieldnames())
            writer.writeheader()
            writer.writerows(self.record_rows)

        summary_path = self.record_prefix.with_suffix(".summary.csv")
        with summary_path.open("w", newline="", encoding="utf-8") as f:
            fieldnames = list(asdict(results[0]).keys()) if results else list(ProbeResult.__annotations__)
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for result in results:
                writer.writerow(asdict(result))

        meta_path = self.record_prefix.with_suffix(".meta.json")
        metadata = {
            "side": self.side,
            "joint_names": self.joint_names,
            "command_topic": self.command_topic,
            "state_topic": self.state_topic,
            "dynamic_state_topic": self.dynamic_state_topic,
            "samples_csv": str(samples_path),
            "summary_csv": str(summary_path),
            "units": {
                "cmd_pos_deg": "degree",
                "cmd_vel_deg_s": "degree/second",
                "state_pos_deg": "degree",
                "state_vel_deg_s": "degree/second",
                "state_current": "A",
            },
            "args": vars(self.args),
        }
        with meta_path.open("w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, sort_keys=True)

        self.get_logger().info(f"wrote samples: {samples_path}")
        self.get_logger().info(f"wrote summary: {summary_path}")
        self.get_logger().info(f"wrote metadata: {meta_path}")

    def wait_for_state(self, timeout_s: float) -> dict[str, float]:
        deadline = time.monotonic() + timeout_s
        missing = list(self.joint_names)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            positions = self._state_positions()
            if positions is None:
                continue
            missing = [name for name in self.joint_names if name not in positions]
            if not missing:
                return positions
        topics = self.get_topic_names_and_types()
        topic_lines = [
            f"{name}: {', '.join(types)}"
            for name, types in topics
            if "joint_state" in name or "joint_states" in name or "revo3" in name
        ]
        seen = self.seen_joint_names[:]
        state_publishers = self.count_publishers(self.state_topic)
        command_subscribers = self.count_subscribers(self.command_topic)
        raise RuntimeError(
            "Timed out waiting for joint state.\n"
            f"  state_topic: {self.state_topic}\n"
            f"  state_publishers: {state_publishers}\n"
            f"  messages_received: {self.state_msg_count}\n"
            f"  dynamic_state_topic: {self.dynamic_state_topic}\n"
            f"  dynamic_messages_received: {self.dynamic_state_msg_count}\n"
            f"  command_topic: {self.command_topic}\n"
            f"  command_subscribers: {command_subscribers}\n"
            f"  missing_joints: {missing}\n"
            f"  seen_joint_names: {seen if seen else 'none'}\n"
            f"  visible_revo3_topics: {topic_lines if topic_lines else 'none'}"
        )

    def publish_command(self, positions: list[float], velocities: list[float] | None = None) -> None:
        msg = Revo3MITCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)
        msg.position = list(positions)
        msg.velocity = velocities if velocities is not None else [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)
        msg.kp = [float(self.args.kp)] * len(positions)
        msg.kd = [float(self.args.kd)] * len(positions)
        self.publisher.publish(msg)

    def hold(self, target: list[float], duration_s: float) -> None:
        end_time = time.monotonic() + duration_s
        period = 1.0 / self.args.command_hz
        while rclpy.ok() and not self.stop_requested and time.monotonic() < end_time:
            self.publish_command(target)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def probe_joint(self, joint_index: int, baseline: list[float]) -> ProbeResult:
        joint = self.joint_names[joint_index]
        amp = math.radians(float(self.args.amplitude_deg))
        freq = float(self.args.frequency_hz)
        duration = float(self.args.duration_s)
        period = 1.0 / self.args.command_hz
        position_samples: dict[str, list[float]] = {name: [] for name in self.joint_names}
        velocity_samples: dict[str, list[float]] = {name: [] for name in self.joint_names}
        current_samples: dict[str, list[float]] = {name: [] for name in self.joint_names}

        start = time.monotonic()
        while rclpy.ok() and not self.stop_requested:
            now = time.monotonic()
            t = now - start
            if t >= duration:
                break
            phase = 2.0 * math.pi * freq * t
            command = list(baseline)
            command[joint_index] = baseline[joint_index] + amp * math.sin(phase)
            velocity = [0.0] * len(command)
            velocity[joint_index] = amp * 2.0 * math.pi * freq * math.cos(phase)
            self.publish_command(command, velocity)
            rclpy.spin_once(self, timeout_sec=0.0)
            positions = self._state_positions()
            if positions is not None:
                for name in self.joint_names:
                    if name in positions:
                        position_samples[name].append(positions[name])
            velocities = self._state_values("velocity")
            if velocities is not None:
                for name in self.joint_names:
                    if name in velocities:
                        velocity_samples[name].append(velocities[name])
            currents = self._dynamic_values("current")
            if currents is not None:
                for name in self.joint_names:
                    if name in currents:
                        current_samples[name].append(currents[name])
            self._record_sample(
                joint_index,
                joint,
                t,
                command,
                velocity,
                positions,
                velocities,
                currents,
            )
            time.sleep(period)

        position_p2p = {
            name: (max(values) - min(values)) if len(values) >= 2 else 0.0
            for name, values in position_samples.items()
        }
        velocity_p2p = {
            name: (max(values) - min(values)) if len(values) >= 2 else 0.0
            for name, values in velocity_samples.items()
        }
        current_p2p = {
            name: (max(values) - min(values)) if len(values) >= 2 else 0.0
            for name, values in current_samples.items()
        }
        response = math.degrees(position_p2p[joint])
        others = {name: value for name, value in position_p2p.items() if name != joint}
        max_other_joint, max_other_rad = max(others.items(), key=lambda item: item[1])
        max_other = math.degrees(max_other_rad)

        velocity_response = math.degrees(velocity_p2p[joint])
        velocity_others = {name: value for name, value in velocity_p2p.items() if name != joint}
        max_other_velocity_joint, max_other_velocity_rad_s = max(
            velocity_others.items(), key=lambda item: item[1]
        )
        max_other_velocity = math.degrees(max_other_velocity_rad_s)

        current_response = current_p2p[joint]
        current_others = {name: value for name, value in current_p2p.items() if name != joint}
        max_other_current_joint, max_other_current = max(
            current_others.items(), key=lambda item: item[1]
        )

        min_response = max(1e-6, float(self.args.min_response_deg))
        cross_limit = max(float(self.args.cross_talk_deg), response * float(self.args.cross_talk_ratio))
        if response < min_response:
            status = "LOW_RESPONSE"
        elif max_other > cross_limit:
            status = "CROSSTALK"
        else:
            status = "OK"
        return ProbeResult(
            joint,
            response,
            max_other,
            max_other_joint,
            velocity_response,
            max_other_velocity,
            max_other_velocity_joint,
            current_response,
            max_other_current,
            max_other_current_joint,
            status,
        )

    def run_probe(self) -> list[ProbeResult]:
        current = self.wait_for_state(self.args.wait_state_s)
        baseline = [current[name] for name in self.joint_names]
        self.get_logger().info("Holding current position before probe...")
        self.hold(baseline, self.args.pre_hold_s)

        results: list[ProbeResult] = []
        for idx, joint in enumerate(self.joint_names):
            if self.stop_requested:
                break
            self.get_logger().info(f"Probing {idx:02d} {joint}")
            result = self.probe_joint(idx, baseline)
            results.append(result)
            print(
                f"{idx:02d} {result.joint:30s} "
                f"response_p2p={result.response_p2p_deg:7.3f} deg "
                f"max_other={result.max_other_p2p_deg:7.3f} deg "
                f"other_joint={result.max_other_joint:30s} "
                f"velocity_p2p={result.velocity_p2p_deg_s:8.3f} deg/s "
                f"current_p2p={result.current_p2p:8.3f} "
                f"{result.status}",
                flush=True,
            )
            self.hold(baseline, self.args.between_hold_s)

        self.get_logger().info("Returning to baseline...")
        self.hold(baseline, self.args.post_hold_s)
        return results


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--namespace", default="", help="Default: revo3_<side>")
    parser.add_argument("--command-topic", default="")
    parser.add_argument("--state-topic", default="")
    parser.add_argument("--dynamic-state-topic", default="")
    parser.add_argument("--amplitude-deg", type=float, default=8.0)
    parser.add_argument("--frequency-hz", type=float, default=0.35)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--command-hz", type=float, default=50.0)
    parser.add_argument("--kp", type=float, default=2.0)
    parser.add_argument("--kd", type=float, default=0.25)
    parser.add_argument("--wait-state-s", type=float, default=5.0)
    parser.add_argument("--pre-hold-s", type=float, default=1.0)
    parser.add_argument("--between-hold-s", type=float, default=0.8)
    parser.add_argument("--post-hold-s", type=float, default=1.5)
    parser.add_argument("--min-response-deg", type=float, default=1.0)
    parser.add_argument("--cross-talk-deg", type=float, default=1.0)
    parser.add_argument("--cross-talk-ratio", type=float, default=0.35)
    parser.add_argument("--record", action="store_true", help="Save command/state samples to CSV.")
    parser.add_argument("--output-dir", default="", help="Directory for recorded CSV/JSON files.")
    parser.add_argument("--output-prefix", default="", help="Output filename prefix without extension.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = Revo3JointMappingProbe(args)

    def _signal_handler(signum, frame):
        del signum, frame
        node.stop_requested = True

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        results = node.run_probe()
        node.write_record_files(results)
        failed = [r for r in results if r.status != "OK"]
        print("\nSummary:")
        print(f"  total={len(results)} ok={len(results) - len(failed)} failed={len(failed)}")
        for result in failed:
            print(
                f"  {result.status}: {result.joint} "
                f"response_p2p={result.response_p2p_deg:.3f}deg "
                f"max_other={result.max_other_p2p_deg:.3f}deg "
                f"other_joint={result.max_other_joint}"
            )
        return 1 if failed else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
