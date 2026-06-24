#!/usr/bin/env python3
"""Publish a simple quintic Revo3 MIT command trajectory for joint response tests."""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from revo3_mit_controller_msgs.msg import Revo3MITCommand

from .joint_state_aligner import controller_joint_names


VALID_HAND_MODES = {"left", "right", "both"}
DEFAULT_COMMAND_SUFFIX = "joint_forward_mit_controller/commands"


def _as_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _command_topic(side: str, suffix: str, use_revo3_namespace: bool) -> str:
    suffix = suffix.strip().strip("/") or DEFAULT_COMMAND_SUFFIX
    if use_revo3_namespace:
        return f"/revo3_{side}/{suffix}"
    return f"/{suffix}"


def _quintic_smooth(alpha: float) -> tuple[float, float]:
    alpha = min(1.0, max(0.0, float(alpha)))
    position_scale = 10.0 * alpha**3 - 15.0 * alpha**4 + 6.0 * alpha**5
    velocity_scale = 30.0 * alpha**2 - 60.0 * alpha**3 + 30.0 * alpha**4
    return position_scale, velocity_scale


class QuinticJointTest(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("revo3_quintic_joint_test")
        self.args = args
        self.sides = ["left", "right"] if args.hand_mode == "both" else [args.hand_mode]
        self._command_publishers = {
            side: self.create_publisher(
                Revo3MITCommand,
                getattr(args, f"{side}_command_topic")
                or _command_topic(side, args.command_topic_suffix, args.use_revo3_namespace),
                10,
            )
            for side in self.sides
        }
        for side, publisher in self._command_publishers.items():
            self.get_logger().info(f"Publishing {side} quintic MIT commands -> {publisher.topic_name}")

    def _publish(self, side: str, position: list[float], velocity: list[float]) -> None:
        msg = Revo3MITCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = controller_joint_names(f"{side}_")
        msg.position = position
        msg.velocity = velocity
        msg.kp = [float(self.args.kp)] * len(msg.joint_names)
        msg.kd = [float(self.args.kd)] * len(msg.joint_names)
        self._command_publishers[side].publish(msg)

    def _publish_all(self, position_value: float, velocity_value: float) -> None:
        for side in self.sides:
            count = len(controller_joint_names(f"{side}_"))
            self._publish(side, [position_value] * count, [velocity_value] * count)

    def run(self) -> None:
        target_rad = math.radians(float(self.args.target_deg))
        rate_hz = float(self.args.rate_hz)
        period_s = 1.0 / rate_hz
        move_duration_s = float(self.args.move_duration_s)
        hold_s = float(self.args.hold_s)
        transitions = [(0.0, target_rad), (target_rad, 0.0)] * int(self.args.cycles)

        self.get_logger().info(
            f"Quintic test: hand_mode={self.args.hand_mode} target={self.args.target_deg:.1f}deg "
            f"move={move_duration_s:.2f}s hold={hold_s:.2f}s cycles={self.args.cycles}"
        )

        self._hold(0.0, hold_s, rate_hz)
        for start, end in transitions:
            self._move(start, end, move_duration_s, period_s)
            self._hold(end, hold_s, rate_hz)
        self._hold(0.0, max(hold_s, 0.5), rate_hz)
        self.get_logger().info("Quintic test complete.")

    def _move(self, start: float, end: float, duration_s: float, period_s: float) -> None:
        delta = end - start
        start_time = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            alpha = elapsed / duration_s if duration_s > 0.0 else 1.0
            position_scale, velocity_scale = _quintic_smooth(alpha)
            position = start + delta * position_scale
            velocity = delta * velocity_scale / duration_s if duration_s > 0.0 else 0.0
            self._publish_all(position, velocity)
            rclpy.spin_once(self, timeout_sec=0.0)
            if alpha >= 1.0:
                break
            time.sleep(period_s)

    def _hold(self, position: float, duration_s: float, rate_hz: float) -> None:
        end_time = time.monotonic() + max(0.0, float(duration_s))
        period_s = 1.0 / float(rate_hz)
        while rclpy.ok() and time.monotonic() < end_time:
            self._publish_all(position, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_s)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move Revo3 joints 0 -> target -> 0 with quintic interpolation.")
    parser.add_argument("--hand-mode", choices=sorted(VALID_HAND_MODES), default="both")
    parser.add_argument("--target-deg", type=float, default=40.0)
    parser.add_argument("--cycles", type=int, default=2, help="Number of 0->target->0 cycles.")
    parser.add_argument("--move-duration-s", type=float, default=2.0)
    parser.add_argument("--hold-s", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--use-revo3-namespace", type=_as_bool, default=True)
    parser.add_argument("--command-topic-suffix", default=DEFAULT_COMMAND_SUFFIX)
    parser.add_argument("--left-command-topic", default="")
    parser.add_argument("--right-command-topic", default="")
    args = parser.parse_args(remove_ros_args(argv)[1:])
    if args.cycles < 1:
        parser.error("--cycles must be >= 1")
    for name in ("move_duration_s", "hold_s", "rate_hz"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and > 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv if argv is None else argv)
    rclpy.init(args=None)
    node = QuinticJointTest(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
