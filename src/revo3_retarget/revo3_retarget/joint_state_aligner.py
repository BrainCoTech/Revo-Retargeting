#!/usr/bin/env python3
"""Republish Revo3 JointState messages ordered like MIT command joint_names."""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from revo3_mit_controller_msgs.msg import Revo3MITCommand
from sensor_msgs.msg import JointState


VALID_HAND_MODES = {"left", "right", "both"}
DEFAULT_COMMAND_SUFFIX = "joint_forward_mit_controller/commands"
DEFAULT_OUTPUT_SUFFIX = "joint_states_aligned"


def controller_joint_names(prefix: str) -> list[str]:
    return [
        f"{prefix}little_MPR_joint", f"{prefix}little_MCP_joint",
        f"{prefix}little_PIP_joint", f"{prefix}little_DIP_joint",
        f"{prefix}ring_MPR_joint", f"{prefix}ring_MCP_joint",
        f"{prefix}ring_PIP_joint", f"{prefix}ring_DIP_joint",
        f"{prefix}middle_MPR_joint", f"{prefix}middle_MCP_joint",
        f"{prefix}middle_PIP_joint", f"{prefix}middle_DIP_joint",
        f"{prefix}index_MPR_joint", f"{prefix}index_MCP_joint",
        f"{prefix}index_PIP_joint", f"{prefix}index_DIP_joint",
        f"{prefix}thumb_MCP_joint", f"{prefix}thumb_PIP_joint",
        f"{prefix}thumb_DIP_joint", f"{prefix}thumb_CMP_joint",
        f"{prefix}thumb_CMR_joint",
    ]


def _as_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _command_topic(side: str, suffix: str, use_revo3_namespace: bool) -> str:
    suffix = suffix.strip().strip("/") or DEFAULT_COMMAND_SUFFIX
    if use_revo3_namespace:
        return f"/revo3_{side}/{suffix}"
    return f"/{suffix}"


def _state_topic(side: str, use_revo3_namespace: bool) -> str:
    if use_revo3_namespace:
        return f"/revo3_{side}/revo3_joint_state/joint_states"
    return "/revo3_joint_state/joint_states"


def _aligned_state_topic(side: str, output_suffix: str, use_revo3_namespace: bool) -> str:
    suffix = output_suffix.strip().strip("/") or DEFAULT_OUTPUT_SUFFIX
    if use_revo3_namespace:
        return f"/revo3_{side}/revo3_joint_state/{suffix}"
    return f"/revo3_joint_state/{suffix}"


def _values_by_name(names: list[str], values) -> dict[str, float]:
    if len(values) != len(names):
        return {}
    return {
        str(name): float(value)
        for name, value in zip(names, values)
        if math.isfinite(float(value))
    }


def _ordered_values(order: list[str], source: dict[str, float]) -> list[float]:
    return [source[name] for name in order if name in source]


@dataclass
class SideAlignState:
    side: str
    command_order: list[str] = field(default_factory=list)
    warned_missing: set[str] = field(default_factory=set)


class JointStateAligner(Node):
    def __init__(self, sides: list[str], args: argparse.Namespace):
        super().__init__("revo3_joint_state_aligner")
        self.sides = sides
        self.states: dict[str, SideAlignState] = {
            side: SideAlignState(side=side, command_order=controller_joint_names(f"{side}_"))
            for side in sides
        }

        for side in sides:
            cmd_topic = getattr(args, f"{side}_command_topic")
            state_topic = getattr(args, f"{side}_state_topic")
            output_topic = getattr(args, f"{side}_output_topic")
            if not cmd_topic:
                cmd_topic = _command_topic(side, args.command_topic_suffix, args.use_revo3_namespace)
            if not state_topic:
                state_topic = _state_topic(side, args.use_revo3_namespace)
            if not output_topic:
                output_topic = _aligned_state_topic(side, args.output_suffix, args.use_revo3_namespace)

            publisher = self.create_publisher(JointState, output_topic, 10)
            self.create_subscription(
                Revo3MITCommand,
                cmd_topic,
                lambda msg, side=side: self._on_command(side, msg),
                10,
            )
            self.create_subscription(
                JointState,
                state_topic,
                lambda msg, side=side, publisher=publisher: self._on_state(side, msg, publisher),
                10,
            )
            self.get_logger().info(
                f"Aligning {side} state={state_topic} using command={cmd_topic} -> {output_topic}"
            )

    def _on_command(self, side: str, msg: Revo3MITCommand) -> None:
        if msg.joint_names:
            self.states[side].command_order = [str(name) for name in msg.joint_names]

    def _on_state(self, side: str, msg: JointState, publisher) -> None:
        state = self.states[side]
        names = [str(name) for name in msg.name]
        position_by_name = _values_by_name(names, msg.position)
        velocity_by_name = _values_by_name(names, msg.velocity)
        effort_by_name = _values_by_name(names, msg.effort)

        ordered_names = [name for name in state.command_order if name in position_by_name]
        missing = sorted(set(state.command_order) - set(ordered_names))
        new_missing = [name for name in missing if name not in state.warned_missing]
        if new_missing:
            state.warned_missing.update(new_missing)
            self.get_logger().warn(
                f"{side} JointState missing {len(new_missing)} command joints; "
                f"first missing: {', '.join(new_missing[:4])}"
            )

        out = JointState()
        out.header = msg.header
        out.name = ordered_names
        out.position = _ordered_values(ordered_names, position_by_name)
        if velocity_by_name:
            out.velocity = _ordered_values(ordered_names, velocity_by_name)
        if effort_by_name:
            out.effort = _ordered_values(ordered_names, effort_by_name)
        publisher.publish(out)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Republish Revo3 JointState messages in command joint_names order."
    )
    parser.add_argument("--hand-mode", choices=sorted(VALID_HAND_MODES), default="both")
    parser.add_argument("--use-revo3-namespace", type=_as_bool, default=True)
    parser.add_argument("--command-topic-suffix", default=DEFAULT_COMMAND_SUFFIX)
    parser.add_argument("--output-suffix", default=DEFAULT_OUTPUT_SUFFIX)
    parser.add_argument("--left-command-topic", default="")
    parser.add_argument("--right-command-topic", default="")
    parser.add_argument("--left-state-topic", default="")
    parser.add_argument("--right-state-topic", default="")
    parser.add_argument("--left-output-topic", default="")
    parser.add_argument("--right-output-topic", default="")
    return parser.parse_args(remove_ros_args(argv)[1:])


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv if argv is None else argv)
    sides = ["left", "right"] if args.hand_mode == "both" else [args.hand_mode]
    rclpy.init(args=None)
    node = JointStateAligner(sides, args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
