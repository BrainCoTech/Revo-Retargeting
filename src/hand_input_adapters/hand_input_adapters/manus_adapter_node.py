"""Normalize native MANUS glove messages into HandKinematics."""

from __future__ import annotations

import math

from hand_teleop_msgs.msg import HandKinematics
from manus_ros2_msgs.msg import ManusGlove
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .common import (
    append_joint,
    append_landmark,
    ergonomics_joint_name,
    finite_point,
    selected_sides,
    side_value,
    validate_message,
)


NODE_LANDMARKS = {
    2: "thumb_pip",
    3: "thumb_dip",
    4: "thumb_tip",
    9: "index_tip",
    14: "middle_tip",
    19: "ring_tip",
    24: "little_tip",
}


class ManusHandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("manus_hand_adapter")
        self.declare_parameter("hand_mode", "right")
        self.declare_parameter("left_input_topic", "/manus_glove_0")
        self.declare_parameter("right_input_topic", "/manus_glove_1")
        self.declare_parameter("left_output_topic", "/hand_kinematics/left")
        self.declare_parameter("right_output_topic", "/hand_kinematics/right")
        self.declare_parameter("left_frame_id", "hand_retarget_left")
        self.declare_parameter("right_frame_id", "hand_retarget_right")

        self.sides = selected_sides(self.get_parameter("hand_mode").value)
        self.frames = {
            side: str(self.get_parameter(f"{side}_frame_id").value)
            for side in self.sides
        }
        self.output_publishers = {
            side: self.create_publisher(
                HandKinematics,
                str(self.get_parameter(f"{side}_output_topic").value),
                20,
            )
            for side in self.sides
        }
        self.input_subscriptions = [
            self.create_subscription(
                ManusGlove,
                str(self.get_parameter(f"{side}_input_topic").value),
                lambda message, expected=side: self._callback(message, expected),
                20,
            )
            for side in self.sides
        ]
        self.published = {side: 0 for side in self.sides}

    def _callback(self, message: ManusGlove, expected_side: str) -> None:
        side = str(message.side).strip().lower() or expected_side
        if side != expected_side:
            self.get_logger().warning(
                f"Dropping MANUS frame on {expected_side} topic with side={side!r}.",
                throttle_duration_sec=2.0,
            )
            return

        output = HandKinematics()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.frames[side]
        output.side = side_value(side)
        output.source = "manus"

        canonical_joints = {}
        for item in message.ergonomics:
            canonical = ergonomics_joint_name(item.type)
            value = math.radians(float(item.value))
            if canonical is not None and math.isfinite(value):
                canonical_joints[canonical] = value
        for name, value in canonical_joints.items():
            append_joint(output, name, value)
        for finger in ("index", "middle", "ring", "little"):
            available = False
            flexion = 0.0
            for joint, weight in (("mcp", 0.50), ("pip", 0.35), ("dip", 0.15)):
                value = canonical_joints.get(f"{finger}_{joint}")
                if value is not None:
                    available = True
                    flexion += weight * max(0.0, value)
            if available:
                append_joint(output, f"{finger}_flexion", flexion)

        for node in message.raw_nodes:
            landmark = NODE_LANDMARKS.get(int(node.node_id))
            if landmark is None:
                continue
            raw = node.pose.position
            append_landmark(output, landmark, finite_point(-raw.y, -raw.x, raw.z))

        reason = validate_message(output)
        if reason is not None or "thumb_tip" not in output.landmark_names:
            self.get_logger().warning(
                f"Dropping invalid MANUS {side} frame: {reason or 'missing thumb_tip'}",
                throttle_duration_sec=2.0,
            )
            return
        self.output_publishers[side].publish(output)
        self.published[side] += 1
        if self.published[side] == 1:
            self.get_logger().info(f"Published first canonical MANUS {side} frame.")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ManusHandAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
