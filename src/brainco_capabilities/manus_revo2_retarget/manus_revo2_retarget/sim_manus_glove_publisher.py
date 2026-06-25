"""Synthetic Manus glove publisher for retargeting simulation tests."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from manus_ros2_msgs.msg import ManusGlove, ManusRawNode
from rclpy.node import Node


VALID_HAND_MODES = {"left", "right", "both"}
TIP_NODE_IDS = {
    "thumb": 4,
    "index": 9,
    "middle": 14,
    "ring": 19,
    "pinky": 24,
}
THUMB_PIP_NODE_ID = 2
THUMB_DIP_NODE_ID = 3


def _raw_pose_from_retarget_xyz(xyz):
    """Invert retarget_node's raw Manus -> retarget coordinate transform."""
    pose = Pose()
    pose.position.x = -float(xyz[1])
    pose.position.y = -float(xyz[0])
    pose.position.z = float(xyz[2])
    pose.orientation.w = 1.0
    return pose


def _raw_node(node_id, parent_node_id, xyz, joint_type="", chain_type=""):
    node = ManusRawNode()
    node.node_id = int(node_id)
    node.parent_node_id = int(parent_node_id)
    node.joint_type = joint_type
    node.chain_type = chain_type
    node.pose = _raw_pose_from_retarget_xyz(xyz)
    return node


def _hand_points(side, t):
    phase = 0.8 if side == "right" else 0.0
    mirror = -1.0 if side == "left" else 1.0
    grip = 0.5 + 0.5 * math.sin(0.85 * t + phase)
    thumb_flex = 0.5 + 0.5 * math.sin(0.95 * t + phase + 0.5)
    thumb_splay = math.sin(0.65 * t + phase)

    thumb_tip = [
        mirror * (0.014 + 0.026 * thumb_splay - 0.010 * thumb_flex),
        0.062 - 0.034 * thumb_flex,
        0.038 + 0.016 * thumb_splay,
    ]
    thumb_pip = [
        mirror * (0.009 + 0.014 * thumb_splay - 0.004 * thumb_flex),
        0.045 - 0.020 * thumb_flex,
        0.026 + 0.009 * thumb_splay,
    ]
    thumb_dip = [
        0.55 * thumb_tip[0] + 0.45 * thumb_pip[0],
        0.55 * thumb_tip[1] + 0.45 * thumb_pip[1],
        0.55 * thumb_tip[2] + 0.45 * thumb_pip[2],
    ]

    finger_x = [-0.030, -0.010, 0.012, 0.032]
    finger_y_offsets = [0.006, 0.000, -0.002, -0.008]
    finger_z_offsets = [0.000, 0.004, 0.002, -0.004]
    fingers = {}
    for index, finger in enumerate(("index", "middle", "ring", "pinky")):
        finger_grip = min(1.0, grip * (0.86 + 0.05 * index))
        fingers[finger] = [
            mirror * finger_x[index],
            0.092 + finger_y_offsets[index] - 0.036 * finger_grip,
            0.106 + finger_z_offsets[index] - 0.024 * finger_grip,
        ]

    return {
        "thumb_pip": thumb_pip,
        "thumb_dip": thumb_dip,
        "thumb": thumb_tip,
        **fingers,
    }


class SimManusGlovePublisher(Node):
    """Publishes deterministic ManusGlove messages without Manus hardware."""

    def __init__(self, hand_mode="both", rate_hz=60.0):
        super().__init__("sim_manus_glove_publisher")
        self.hand_mode = hand_mode
        self.rate_hz = max(1.0, float(rate_hz))
        self.start_time = time.monotonic()

        sides = []
        if hand_mode in ("left", "both"):
            sides.append("left")
        if hand_mode in ("right", "both"):
            sides.append("right")

        self.publishers_by_side = {}
        for topic_index, side in enumerate(sides):
            topic_name = f"/manus_glove_{topic_index}"
            self.publishers_by_side[side] = self.create_publisher(
                ManusGlove,
                topic_name,
                10,
            )
            self.get_logger().info(f"Publishing simulated {side} glove on {topic_name}")

        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish_once)

    def _publish_once(self):
        t = time.monotonic() - self.start_time
        for glove_id, (side, publisher) in enumerate(self.publishers_by_side.items()):
            points = _hand_points(side, t)
            msg = ManusGlove()
            msg.glove_id = glove_id
            msg.side = side
            msg.raw_nodes = [
                _raw_node(THUMB_PIP_NODE_ID, 1, points["thumb_pip"], "pip", "thumb"),
                _raw_node(THUMB_DIP_NODE_ID, 2, points["thumb_dip"], "dip", "thumb"),
                _raw_node(TIP_NODE_IDS["thumb"], 3, points["thumb"], "tip", "thumb"),
                _raw_node(TIP_NODE_IDS["index"], 8, points["index"], "tip", "index"),
                _raw_node(TIP_NODE_IDS["middle"], 13, points["middle"], "tip", "middle"),
                _raw_node(TIP_NODE_IDS["ring"], 18, points["ring"], "tip", "ring"),
                _raw_node(TIP_NODE_IDS["pinky"], 23, points["pinky"], "tip", "pinky"),
            ]
            msg.raw_node_count = len(msg.raw_nodes)
            msg.ergonomics_count = 0
            msg.raw_sensor_orientation.w = 1.0
            msg.raw_sensor_count = 0
            publisher.publish(msg)


def parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Publish synthetic ManusGlove data for Revo2 retarget simulation."
    )
    parser.add_argument(
        "--hand-mode",
        "--hand_mode",
        dest="hand_mode",
        default="both",
        choices=sorted(VALID_HAND_MODES),
        help="Which simulated hand data to publish.",
    )
    parser.add_argument(
        "--rate",
        dest="rate_hz",
        default=60.0,
        type=float,
        help="Publish rate in Hz.",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    cli_args, ros_args = parse_cli_args(argv)

    rclpy.init(args=ros_args)
    node = SimManusGlovePublisher(
        hand_mode=cli_args.hand_mode,
        rate_hz=cli_args.rate_hz,
    )
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
