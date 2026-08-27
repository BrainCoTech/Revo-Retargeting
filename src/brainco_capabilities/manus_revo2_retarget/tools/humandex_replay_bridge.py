#!/usr/bin/env python3
"""Bridge HumanDex replay pose+joint data into the ManusGlove contract.

This is intentionally runnable from the source tree for hardware/replay tests.
It fills ManusGlove.raw_nodes from /humandex_eef_pose and fills
ManusGlove.ergonomics from HumanDex JointState, because the Revo2 retargeter
uses ergonomics stretch values for the four non-thumb fingers.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Dict, Iterable, List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from manus_ros2_msgs.msg import ManusErgonomics, ManusGlove, ManusRawNode
from rclpy.node import Node
from sensor_msgs.msg import JointState


FINGERTIP_COUNT = 5
# HumanDex per-hand PoseArray order: index, middle, ring, little, thumb.
# Manus compatibility order: thumb, index, middle, ring, pinky.
OUTPUT_SOURCE_INDICES = (4, 0, 1, 2, 3)
MANUS_NODE_IDS = (4, 9, 14, 19, 24)
CHAIN_TYPES = ("thumb", "index", "middle", "ring", "pinky")
HUMANDEX_FINGERS = ("index", "middle", "ring", "little")
MANUS_FINGERS = ("Index", "Middle", "Ring", "Pinky")
STRETCH_JOINTS = ("MCP", "PIP", "DIP")


def compatibility_pose(source: Pose) -> Pose:
    """Apply the inverse legacy MANUS axis map expected by retarget_node."""
    result = Pose()
    result.position.x = -source.position.y
    result.position.y = -source.position.x
    result.position.z = source.position.z
    result.orientation.w = 1.0
    return result


def raw_node(node_id: int, parent_node_id: int, joint_type: str, chain_type: str, pose: Pose):
    node = ManusRawNode()
    node.node_id = int(node_id)
    node.parent_node_id = int(parent_node_id)
    node.joint_type = joint_type
    node.chain_type = chain_type
    node.pose = pose
    return node


def ergonomics_item(name: str, value: float):
    item = ManusErgonomics()
    item.type = name
    item.value = float(value)
    return item


class HumanDexReplayBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("humandex_replay_bridge")
        self.args = args
        self.side = args.side
        self.glove_id = 0 if self.side == "left" else 1
        self.zero_positions: Optional[Dict[str, float]] = None
        self.latest_positions: Optional[Dict[str, float]] = None
        self.pose_count = 0
        self.published_count = 0
        self.last_log_time = 0.0

        self.publisher = self.create_publisher(ManusGlove, args.output_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseArray,
            args.pose_topic,
            self.pose_callback,
            10,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            args.joint_topic,
            self.joint_callback,
            10,
        )
        self.get_logger().info(
            "HumanDex replay bridge: "
            f"side={self.side} pose={args.pose_topic} joint={args.joint_topic} "
            f"output={args.output_topic} stretch_source={args.stretch_source} "
            f"sign={args.stretch_sign:.2f} scale={args.stretch_scale:.2f}"
        )

    def joint_callback(self, msg: JointState):
        if len(msg.position) < len(msg.name):
            self.get_logger().warning(
                "Dropping JointState: "
                f"position count {len(msg.position)} < name count {len(msg.name)}"
            )
            return
        positions = {name: float(msg.position[i]) for i, name in enumerate(msg.name)}
        if self.zero_positions is None:
            self.zero_positions = dict(positions)
            self.get_logger().info("Captured first HumanDex joint frame as ergonomics zero.")
        self.latest_positions = positions

    def select_hand_poses(self, poses: List[Pose]) -> List[Pose]:
        expected = 10 if self.args.input_hand_mode == "both" else FINGERTIP_COUNT
        if len(poses) != expected:
            raise ValueError(
                f"expected {expected} poses for input_hand_mode={self.args.input_hand_mode}, "
                f"got {len(poses)}"
            )
        begin = 5 if self.args.input_hand_mode == "both" and self.side == "right" else 0
        return list(poses[begin : begin + FINGERTIP_COUNT])

    def flex_deg_for_finger(self, humandex_finger: str) -> float:
        if self.latest_positions is None or self.zero_positions is None:
            return 0.0

        source = self.args.stretch_source
        if source == "weighted":
            deltas = []
            weights = (0.50, 0.35, 0.15)
            for joint in STRETCH_JOINTS:
                name = f"{self.side}_{humandex_finger}_{joint}_joint"
                if name not in self.latest_positions or name not in self.zero_positions:
                    continue
                deltas.append(
                    weights[len(deltas)]
                    * (self.latest_positions[name] - self.zero_positions[name])
                )
            delta_rad = sum(deltas)
        else:
            name = f"{self.side}_{humandex_finger}_{source}_joint"
            if name not in self.latest_positions or name not in self.zero_positions:
                return 0.0
            delta_rad = self.latest_positions[name] - self.zero_positions[name]

        flex_deg = self.args.stretch_sign * delta_rad * 180.0 / math.pi * self.args.stretch_scale
        return max(0.0, min(self.args.max_stretch_deg, flex_deg))

    def build_ergonomics(self) -> List[ManusErgonomics]:
        result: List[ManusErgonomics] = []
        for humandex_finger, manus_finger in zip(HUMANDEX_FINGERS, MANUS_FINGERS):
            flex_deg = self.flex_deg_for_finger(humandex_finger)
            for joint in STRETCH_JOINTS:
                result.append(ergonomics_item(f"{manus_finger}{joint}Stretch", flex_deg))
        return result

    def pose_callback(self, msg: PoseArray):
        self.pose_count += 1
        if self.latest_positions is None:
            return
        if self.args.expected_frame_id and msg.header.frame_id != self.args.expected_frame_id:
            self.get_logger().warning(
                "Dropping PoseArray with "
                f"frame_id={msg.header.frame_id!r}, expected {self.args.expected_frame_id!r}"
            )
            return
        try:
            selected = self.select_hand_poses(list(msg.poses))
        except ValueError as exc:
            self.get_logger().warning(f"Dropping PoseArray: {exc}")
            return

        palm_pose = Pose()
        palm_pose.orientation.w = 1.0

        out = ManusGlove()
        out.glove_id = self.glove_id
        out.side = self.side
        out.raw_nodes.append(raw_node(0, -1, "palm", "palm", palm_pose))
        out.raw_sensor.append(palm_pose)

        for out_index, source_index in enumerate(OUTPUT_SOURCE_INDICES):
            pose = compatibility_pose(selected[source_index])
            out.raw_nodes.append(
                raw_node(MANUS_NODE_IDS[out_index], 0, "tip", CHAIN_TYPES[out_index], pose)
            )
            out.raw_sensor.append(pose)

        out.ergonomics = self.build_ergonomics()
        out.raw_node_count = len(out.raw_nodes)
        out.ergonomics_count = len(out.ergonomics)
        out.raw_sensor_count = len(out.raw_sensor)
        out.raw_sensor_orientation = palm_pose.orientation
        self.publisher.publish(out)
        self.published_count += 1

        now = time.monotonic()
        if self.published_count == 1 or now - self.last_log_time > 2.0:
            self.last_log_time = now
            preview = {
                finger: round(self.flex_deg_for_finger(finger), 2)
                for finger in HUMANDEX_FINGERS
            }
            self.get_logger().info(
                f"Published {self.published_count} ManusGlove frames; "
                f"four-finger flex deg={preview}"
            )


def parse_args(argv: Optional[Iterable[str]] = None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--input-hand-mode", choices=("left", "right", "both"), default="left")
    parser.add_argument("--pose-topic", default="/humandex_replay/eef_pose")
    parser.add_argument("--joint-topic", default="/humandex_replay/left_joint_states")
    parser.add_argument("--output-topic", default="/manus_glove_0")
    parser.add_argument("--expected-frame-id", default="hand_base_link_local")
    parser.add_argument("--stretch-source", choices=("MPR", "MCP", "PIP", "DIP", "weighted"), default="MPR")
    parser.add_argument("--stretch-sign", type=float, default=-1.0)
    parser.add_argument("--stretch-scale", type=float, default=4.0)
    parser.add_argument("--max-stretch-deg", type=float, default=85.0)
    return parser.parse_args(argv)


def main(argv: Optional[Iterable[str]] = None):
    args = parse_args(argv)
    rclpy.init()
    node = HumanDexReplayBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
