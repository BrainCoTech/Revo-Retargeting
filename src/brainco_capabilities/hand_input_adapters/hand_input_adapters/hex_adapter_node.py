"""Normalize time-bounded Hex angle/position JSON pairs into HandKinematics."""

from __future__ import annotations

import json
import math
import time
from typing import Any

from hand_teleop_msgs.msg import HandKinematics
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .common import (
    append_joint,
    append_landmark,
    finite_point,
    selected_sides,
    side_value,
    validate_message,
)


FINGERS = ("thumb", "index", "middle", "ring", "little")
ANGLE_SPECS = (
    ("pitch", "mcp"),
    ("side", "spread"),
    ("two_pitch", "pip"),
    ("end_pitch", "dip"),
)
LANDMARK_SPECS = (
    ("thumb_pip", "hc_Thumb2_{suffix}"),
    ("thumb_dip", "hc_Thumb3_{suffix}"),
    ("thumb_tip", "hc_Thumb4_{suffix}"),
    ("index_tip", "hc_Index4_{suffix}"),
    ("middle_tip", "hc_Middle4_{suffix}"),
    ("ring_tip", "hc_Ring4_{suffix}"),
    ("little_tip", "hc_Pinky4_{suffix}"),
)


class HexHandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("hex_hand_adapter")
        self.declare_parameter("hand_mode", "right")
        self.declare_parameter("angles_topic", "/hex_glove/raw_angles")
        self.declare_parameter("positions_topic", "/hex_glove/raw_positions")
        self.declare_parameter("left_output_topic", "/hand_kinematics/left")
        self.declare_parameter("right_output_topic", "/hand_kinematics/right")
        self.declare_parameter("left_frame_id", "hand_retarget_left")
        self.declare_parameter("right_frame_id", "hand_retarget_right")
        self.declare_parameter("max_pair_skew_sec", 0.03)
        self.declare_parameter("position_scale", 0.01)
        self.declare_parameter("angle_scale", 1.0)
        self.declare_parameter("zero_angles_on_first_frame", False)
        self.declare_parameter("left_flexion_sign", 1.0)
        self.declare_parameter("right_flexion_sign", 1.0)
        self.declare_parameter("left_spread_sign", 1.0)
        self.declare_parameter("right_spread_sign", 1.0)

        self.sides = selected_sides(self.get_parameter("hand_mode").value)
        self.max_pair_skew = max(0.0, float(self.get_parameter("max_pair_skew_sec").value))
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.angle_scale = float(self.get_parameter("angle_scale").value)
        self.zero_on_first = bool(self.get_parameter("zero_angles_on_first_frame").value)
        self.flexion_sign = {
            side: float(self.get_parameter(f"{side}_flexion_sign").value)
            for side in self.sides
        }
        self.spread_sign = {
            side: float(self.get_parameter(f"{side}_spread_sign").value)
            for side in self.sides
        }
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
            for side in ("left", "right")
        }
        self.latest_angles: tuple[dict[str, Any], float] | None = None
        self.angle_zero: dict[str, dict[str, list[float]]] = {}
        self.published = {side: 0 for side in self.sides}

        self.create_subscription(
            String,
            str(self.get_parameter("angles_topic").value),
            self._angles_callback,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("positions_topic").value),
            self._positions_callback,
            20,
        )

    def _parse(self, message: String, label: str) -> dict[str, Any] | None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f"Dropping malformed Hex {label} JSON: {error}",
                throttle_duration_sec=2.0,
            )
            return None
        return value if isinstance(value, dict) else None

    def _angles_callback(self, message: String) -> None:
        payload = self._parse(message, "angles")
        if payload is None:
            return
        now = time.monotonic()
        self.latest_angles = (payload, now)
        if self.zero_on_first:
            self._capture_zero(payload)

    def _positions_callback(self, message: String) -> None:
        payload = self._parse(message, "positions")
        if payload is None or self.latest_angles is None:
            return
        angles, angle_time = self.latest_angles
        now = time.monotonic()
        skew = abs(now - angle_time)
        if skew > self.max_pair_skew:
            self.get_logger().warning(
                f"Dropping Hex position frame: angle/position skew {skew:.4f}s exceeds "
                f"{self.max_pair_skew:.4f}s.",
                throttle_duration_sec=2.0,
            )
            return
        stamp = self.get_clock().now().to_msg()
        for side, hand_key, suffix in (
            ("left", "leftHand", "L"),
            ("right", "rightHand", "R"),
        ):
            if side not in self.sides:
                continue
            positions_hand = payload.get(hand_key)
            angles_hand = angles.get(hand_key)
            if isinstance(positions_hand, dict):
                self._publish_side(side, suffix, positions_hand, angles_hand, stamp)

    def _capture_zero(self, payload: dict[str, Any]) -> None:
        for side, hand_key in (("left", "leftHand"), ("right", "rightHand")):
            if side not in self.sides:
                continue
            if side in self.angle_zero:
                continue
            hand = payload.get(hand_key)
            if not isinstance(hand, dict):
                continue
            copied = {
                key: [float(value) for value in values]
                for key in ("pitch", "side", "two_pitch", "end_pitch")
                if isinstance((values := hand.get(key)), list)
            }
            if copied:
                self.angle_zero[side] = copied
                self.get_logger().info(f"Captured Hex {side} angle zero frame.")

    def _angle(self, hand: Any, side: str, key: str, index: int) -> float | None:
        if not isinstance(hand, dict):
            return None
        values = hand.get(key)
        if not isinstance(values, list) or index >= len(values):
            return None
        try:
            value = float(values[index])
        except (TypeError, ValueError):
            return None
        zero_values = self.angle_zero.get(side, {}).get(key)
        if zero_values is not None and index < len(zero_values):
            value -= zero_values[index]
        sign = self.spread_sign[side] if key == "side" else self.flexion_sign[side]
        return math.radians(value * self.angle_scale * sign)

    def _publish_side(self, side, suffix, positions, angles, stamp) -> None:
        output = HandKinematics()
        output.header.stamp = stamp
        output.header.frame_id = self.frames[side]
        output.side = side_value(side)
        output.source = "hex"

        canonical_joints = {}
        for finger_index, finger in enumerate(FINGERS):
            for key, joint in ANGLE_SPECS:
                value = self._angle(angles, side, key, finger_index)
                if value is not None:
                    name = f"{finger}_{joint}"
                    canonical_joints[name] = value
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

        for landmark, template in LANDMARK_SPECS:
            raw = positions.get(template.format(suffix=suffix))
            if not isinstance(raw, dict):
                continue
            try:
                point = finite_point(
                    float(raw.get("x", 0.0)) * self.position_scale,
                    float(raw.get("y", 0.0)) * self.position_scale,
                    float(raw.get("z", 0.0)) * self.position_scale,
                )
            except (TypeError, ValueError):
                point = None
            append_landmark(output, landmark, point)

        reason = validate_message(output)
        if reason is not None or "thumb_tip" not in output.landmark_names:
            self.get_logger().warning(
                f"Dropping invalid Hex {side} frame: {reason or 'missing thumb_tip'}",
                throttle_duration_sec=2.0,
            )
            return
        self.output_publishers[side].publish(output)
        self.published[side] += 1
        if self.published[side] == 1:
            self.get_logger().info(f"Published first canonical Hex {side} frame.")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = HexHandAdapter()
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
