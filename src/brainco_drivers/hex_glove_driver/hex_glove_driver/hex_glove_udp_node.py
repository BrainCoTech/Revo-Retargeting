#!/usr/bin/env python3
"""ROS2 UDP driver for Hexacercle glove broadcast data."""

from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import Pose
from manus_ros2_msgs.msg import ManusErgonomics, ManusGlove, ManusRawNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


HEX_TO_MANUS_NODE_SPECS = [
    (0, -1, "PalmOriginLocal", "palm", "palm"),
    (1, 0, "hc_Thumb1_{suffix}", "mcp", "thumb"),
    (2, 1, "hc_Thumb2_{suffix}", "pip", "thumb"),
    (3, 2, "hc_Thumb3_{suffix}", "dip", "thumb"),
    (4, 3, "hc_Thumb4_{suffix}", "tip", "thumb"),
    (5, 0, "PalmOriginLocal", "base", "index"),
    (6, 5, "hc_Index1_{suffix}", "mcp", "index"),
    (7, 6, "hc_Index2_{suffix}", "pip", "index"),
    (8, 7, "hc_Index3_{suffix}", "dip", "index"),
    (9, 8, "hc_Index4_{suffix}", "tip", "index"),
    (10, 0, "PalmOriginLocal", "base", "middle"),
    (11, 10, "hc_Middle1_{suffix}", "mcp", "middle"),
    (12, 11, "hc_Middle2_{suffix}", "pip", "middle"),
    (13, 12, "hc_Middle3_{suffix}", "dip", "middle"),
    (14, 13, "hc_Middle4_{suffix}", "tip", "middle"),
    (15, 0, "PalmOriginLocal", "base", "ring"),
    (16, 15, "hc_Ring1_{suffix}", "mcp", "ring"),
    (17, 16, "hc_Ring2_{suffix}", "pip", "ring"),
    (18, 17, "hc_Ring3_{suffix}", "dip", "ring"),
    (19, 18, "hc_Ring4_{suffix}", "tip", "ring"),
    (20, 0, "PalmOriginLocal", "base", "pinky"),
    (21, 20, "hc_Pinky1_{suffix}", "mcp", "pinky"),
    (22, 21, "hc_Pinky2_{suffix}", "pip", "pinky"),
    (23, 22, "hc_Pinky3_{suffix}", "dip", "pinky"),
    (24, 23, "hc_Pinky4_{suffix}", "tip", "pinky"),
]

FINGER_NAMES = ("Thumb", "Index", "Middle", "Ring", "Pinky")
ANGLE_KEYS = ("pitch", "side", "two_pitch", "end_pitch")
THUMB_POSITION_CALIBRATION_NODE_IDS = {2, 3, 4}


@dataclass
class UdpJsonStream:
    name: str
    server_host: str
    server_port: int
    publisher: Any

    def __post_init__(self) -> None:
        self.server_address = (self.server_host, self.server_port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.setblocking(False)
        self.last_connect_sent = 0.0
        self.frame_count = 0
        self.json_error_count = 0

    @property
    def local_address(self) -> tuple[str, int]:
        return self.socket.getsockname()

    def close(self) -> None:
        self.socket.close()


class HexGloveUdpNode(Node):
    """Receive Hex UDP JSON frames and publish ROS2 raw/adapted topics."""

    def __init__(self) -> None:
        super().__init__("hex_glove_udp_node")

        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("angles_port", 9011)
        self.declare_parameter("positions_port", 9013)
        self.declare_parameter("connect_message", "CONNECT")
        self.declare_parameter("raw_angles_topic", "/hex_glove/raw_angles")
        self.declare_parameter("raw_positions_topic", "/hex_glove/raw_positions")
        self.declare_parameter("publish_adapter_glove", True)
        self.declare_parameter("publish_manus_glove", True)
        self.declare_parameter("left_glove_topic", "/hex_glove_0")
        self.declare_parameter("right_glove_topic", "/hex_glove_1")
        self.declare_parameter("left_manus_topic", "/hex_glove_0")
        self.declare_parameter("right_manus_topic", "/hex_glove_1")
        self.declare_parameter("position_scale", 0.01)
        self.declare_parameter("revo2_coordinate_transform", True)
        self.declare_parameter("angle_scale", 1.0)
        self.declare_parameter("zero_angles_on_first_frame", False)
        self.declare_parameter("left_stretch_sign", 1.0)
        self.declare_parameter("right_stretch_sign", 1.0)
        self.declare_parameter("left_spread_sign", 1.0)
        self.declare_parameter("right_spread_sign", 1.0)
        self.declare_parameter("thumb_position_calibration_enabled", False)
        self.declare_parameter(
            "left_thumb_position_matrix",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter(
            "right_thumb_position_matrix",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("left_thumb_position_scale_xyz", [1.0, 1.0, 1.0])
        self.declare_parameter("right_thumb_position_scale_xyz", [1.0, 1.0, 1.0])
        self.declare_parameter("left_thumb_position_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("right_thumb_position_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("connect_period_sec", 1.0)
        self.declare_parameter("topic", "")

        self.server_host = self.get_string_parameter("server_host")
        angles_port = self.get_int_parameter("angles_port")
        positions_port = self.get_int_parameter("positions_port")
        self.connect_message = self.get_string_parameter("connect_message").encode(
            "utf-8"
        )
        raw_angles_topic = self.get_string_parameter("raw_angles_topic")
        raw_positions_topic = self.get_string_parameter("raw_positions_topic")
        legacy_raw_topic = self.get_string_parameter("topic")
        if legacy_raw_topic:
            raw_positions_topic = legacy_raw_topic

        publish_adapter_glove = self.get_bool_parameter("publish_adapter_glove")
        publish_manus_glove = self.get_bool_parameter("publish_manus_glove")
        self.publish_adapter_glove = publish_adapter_glove and publish_manus_glove

        left_glove_topic = self.get_string_parameter("left_glove_topic")
        right_glove_topic = self.get_string_parameter("right_glove_topic")
        legacy_left_topic = self.get_string_parameter("left_manus_topic")
        legacy_right_topic = self.get_string_parameter("right_manus_topic")
        if legacy_left_topic != "/hex_glove_0" and left_glove_topic == "/hex_glove_0":
            left_glove_topic = legacy_left_topic
        if (
            legacy_right_topic != "/hex_glove_1"
            and right_glove_topic == "/hex_glove_1"
        ):
            right_glove_topic = legacy_right_topic

        self.position_scale = self.get_float_parameter("position_scale")
        self.revo2_coordinate_transform = self.get_bool_parameter(
            "revo2_coordinate_transform"
        )
        self.angle_scale = self.get_float_parameter("angle_scale")
        self.zero_angles_on_first_frame = self.get_bool_parameter(
            "zero_angles_on_first_frame"
        )
        self.left_stretch_sign = self.get_float_parameter("left_stretch_sign")
        self.right_stretch_sign = self.get_float_parameter("right_stretch_sign")
        self.left_spread_sign = self.get_float_parameter("left_spread_sign")
        self.right_spread_sign = self.get_float_parameter("right_spread_sign")
        self.thumb_position_calibration_enabled = self.get_bool_parameter(
            "thumb_position_calibration_enabled"
        )
        self.left_thumb_position_matrix = self.get_float_array_parameter(
            "left_thumb_position_matrix", 9
        )
        self.right_thumb_position_matrix = self.get_float_array_parameter(
            "right_thumb_position_matrix", 9
        )
        self.left_thumb_position_scale_xyz = self.get_float_array_parameter(
            "left_thumb_position_scale_xyz", 3
        )
        self.right_thumb_position_scale_xyz = self.get_float_array_parameter(
            "right_thumb_position_scale_xyz", 3
        )
        self.left_thumb_position_offset_xyz = self.get_float_array_parameter(
            "left_thumb_position_offset_xyz", 3
        )
        self.right_thumb_position_offset_xyz = self.get_float_array_parameter(
            "right_thumb_position_offset_xyz", 3
        )
        self.connect_period_sec = self.get_float_parameter("connect_period_sec")

        self.raw_angles_publisher = self.create_publisher(String, raw_angles_topic, 10)
        self.raw_positions_publisher = self.create_publisher(
            String, raw_positions_topic, 10
        )
        self.left_glove_publisher = None
        self.right_glove_publisher = None
        if self.publish_adapter_glove:
            self.left_glove_publisher = self.create_publisher(
                ManusGlove,
                left_glove_topic,
                10,
            )
            self.right_glove_publisher = self.create_publisher(
                ManusGlove,
                right_glove_topic,
                10,
            )

        self.angles_stream = UdpJsonStream(
            "angles", self.server_host, angles_port, self.raw_angles_publisher
        )
        self.positions_stream = UdpJsonStream(
            "positions", self.server_host, positions_port, self.raw_positions_publisher
        )
        self.latest_angles_payload: dict[str, Any] | None = None
        self.latest_positions_payload: dict[str, Any] | None = None
        self.angle_zero_by_side: dict[str, dict[str, list[float]] | None] = {
            "left": None,
            "right": None,
        }
        self.adapter_frame_count = 0

        angles_host, angles_local_port = self.angles_stream.local_address
        positions_host, positions_local_port = self.positions_stream.local_address
        self.get_logger().info(
            "Hex glove UDP node started: "
            f"angles local={angles_host}:{angles_local_port} -> "
            f"{self.server_host}:{angles_port}, "
            f"positions local={positions_host}:{positions_local_port} -> "
            f"{self.server_host}:{positions_port}"
        )
        self.get_logger().info(
            f"Raw topics: angles={raw_angles_topic}, positions={raw_positions_topic}"
        )
        if self.publish_adapter_glove:
            self.get_logger().info(
                "Publishing ManusGlove adapter topics: "
                f"left={left_glove_topic}, right={right_glove_topic}, "
                f"position_scale={self.position_scale}, "
                f"revo2_coordinate_transform={self.revo2_coordinate_transform}, "
                f"angle_scale={self.angle_scale}, "
                f"zero_angles_on_first_frame={self.zero_angles_on_first_frame}, "
                f"stretch_signs=({self.left_stretch_sign}, {self.right_stretch_sign}), "
                f"spread_signs=({self.left_spread_sign}, {self.right_spread_sign}), "
                "thumb_position_calibration_enabled="
                f"{self.thumb_position_calibration_enabled}"
            )
            if self.thumb_position_calibration_enabled:
                self.get_logger().info(
                    "Thumb position calibration: "
                    f"left_matrix={self.left_thumb_position_matrix}, "
                    f"left_scale={self.left_thumb_position_scale_xyz}, "
                    f"left_offset_m={self.left_thumb_position_offset_xyz}, "
                    f"right_matrix={self.right_thumb_position_matrix}, "
                    f"right_scale={self.right_thumb_position_scale_xyz}, "
                    f"right_offset_m={self.right_thumb_position_offset_xyz}"
                )
        self.get_logger().info(
            "Make sure HexacercleGloveComputer.exe is running and data broadcast is "
            "enabled."
        )

        self.timer = self.create_timer(0.001, self.poll_sockets)

    def get_string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def get_bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def get_int_parameter(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def get_float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def get_float_array_parameter(self, name: str, length: int) -> tuple[float, ...]:
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)) or len(value) != length:
            raise ValueError(f"Parameter {name} must be a list of {length} numbers.")
        return tuple(float(item) for item in value)

    def poll_sockets(self) -> None:
        self.send_connect_if_needed(self.angles_stream)
        self.send_connect_if_needed(self.positions_stream)
        self.poll_stream(self.angles_stream)
        self.poll_stream(self.positions_stream)

    def send_connect_if_needed(self, stream: UdpJsonStream) -> None:
        now = time.monotonic()
        if now - stream.last_connect_sent < self.connect_period_sec:
            return
        try:
            stream.socket.sendto(self.connect_message, stream.server_address)
        except OSError as exc:
            self.get_logger().warning(
                f"Failed to send Hex {stream.name} CONNECT packet: {exc}"
            )
            return
        stream.last_connect_sent = now

    def poll_stream(self, stream: UdpJsonStream) -> None:
        while rclpy.ok():
            try:
                data, _addr = stream.socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"Hex {stream.name} UDP socket error: {exc}")
                return

            text = data.decode("utf-8", errors="replace")
            payload = self.parse_json_payload(stream, text)
            if payload is None or not self.has_hand_payload(payload):
                continue

            self.publish_raw_text(stream, text)
            stream.frame_count += 1
            if stream.name == "angles":
                self.capture_angle_zero_if_needed(payload)
                self.latest_angles_payload = payload
                self.log_frame_count(stream, "angle")
            else:
                self.latest_positions_payload = payload
                self.log_frame_count(stream, "position")
                if self.publish_adapter_glove:
                    self.publish_adapter_gloves()

    def parse_json_payload(
        self, stream: UdpJsonStream, text: str
    ) -> dict[str, Any] | None:
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as exc:
            stream.json_error_count += 1
            if stream.json_error_count <= 3:
                self.get_logger().warning(
                    f"Failed to parse Hex {stream.name} JSON: {exc}"
                )
            return None
        if not isinstance(payload, dict):
            return None
        return payload

    @staticmethod
    def has_hand_payload(payload: dict[str, Any]) -> bool:
        return isinstance(payload.get("leftHand"), dict) or isinstance(
            payload.get("rightHand"), dict
        )

    def publish_raw_text(self, stream: UdpJsonStream, text: str) -> None:
        msg = String()
        msg.data = text
        try:
            stream.publisher.publish(msg)
        except Exception:
            if not rclpy.ok():
                return
            raise

    def log_frame_count(self, stream: UdpJsonStream, label: str) -> None:
        if stream.frame_count == 1:
            self.get_logger().info(f"Received first Hex glove {label} frame.")
        elif stream.frame_count % 500 == 0:
            self.get_logger().info(
                f"Published {stream.frame_count} Hex glove {label} frames."
            )

    def publish_adapter_gloves(self) -> None:
        if self.latest_positions_payload is None:
            return
        self.publish_one_adapter_glove(
            positions_hand=self.latest_positions_payload.get("leftHand"),
            angles_hand=self.get_latest_angles_hand("leftHand"),
            side="left",
            suffix="L",
            glove_id=0,
            publisher=self.left_glove_publisher,
        )
        self.publish_one_adapter_glove(
            positions_hand=self.latest_positions_payload.get("rightHand"),
            angles_hand=self.get_latest_angles_hand("rightHand"),
            side="right",
            suffix="R",
            glove_id=1,
            publisher=self.right_glove_publisher,
        )
        self.adapter_frame_count += 1
        if self.adapter_frame_count == 1:
            self.get_logger().info("Published first Hex ManusGlove adapter frame.")
        elif self.adapter_frame_count % 500 == 0:
            self.get_logger().info(
                f"Published {self.adapter_frame_count} Hex ManusGlove adapter frames."
            )

    def get_latest_angles_hand(self, hand_key: str) -> dict[str, Any] | None:
        if not isinstance(self.latest_angles_payload, dict):
            return None
        hand_data = self.latest_angles_payload.get(hand_key)
        if not isinstance(hand_data, dict):
            return None
        return hand_data

    def capture_angle_zero_if_needed(self, payload: dict[str, Any]) -> None:
        if not self.zero_angles_on_first_frame:
            return
        for hand_key, side in (("leftHand", "left"), ("rightHand", "right")):
            if self.angle_zero_by_side[side] is not None:
                continue
            hand_angles = payload.get(hand_key)
            if not isinstance(hand_angles, dict):
                continue
            zero = self.copy_angle_arrays(hand_angles)
            if not zero:
                continue
            self.angle_zero_by_side[side] = zero
            self.get_logger().info(
                f"Captured Hex {side} open-hand angle zero from first angle frame."
            )

    @staticmethod
    def copy_angle_arrays(hand_angles: dict[str, Any]) -> dict[str, list[float]]:
        copied = {}
        for key in ANGLE_KEYS:
            values = hand_angles.get(key)
            if not isinstance(values, list):
                continue
            copied_values = []
            for value in values:
                try:
                    copied_values.append(float(value))
                except (TypeError, ValueError):
                    copied_values.append(0.0)
            copied[key] = copied_values
        return copied

    def publish_one_adapter_glove(
        self,
        *,
        positions_hand: Any,
        angles_hand: dict[str, Any] | None,
        side: str,
        suffix: str,
        glove_id: int,
        publisher: Any,
    ) -> None:
        if publisher is None or not isinstance(positions_hand, dict):
            return

        msg = ManusGlove()
        msg.glove_id = glove_id
        msg.side = side
        msg.raw_sensor_orientation.w = 1.0

        for (
            node_id,
            parent_id,
            key_template,
            joint_type,
            chain_type,
        ) in HEX_TO_MANUS_NODE_SPECS:
            key = key_template.format(suffix=suffix)
            point = positions_hand.get(key)
            if not isinstance(point, dict):
                continue
            msg.raw_nodes.append(
                self.make_raw_node(
                    node_id,
                    parent_id,
                    joint_type,
                    chain_type,
                    point,
                    side=side,
                )
            )

        msg.raw_node_count = len(msg.raw_nodes)
        msg.ergonomics = self.make_ergonomics(angles_hand, side=side)
        msg.ergonomics_count = len(msg.ergonomics)
        msg.raw_sensor_count = 0
        if not msg.raw_nodes:
            return
        try:
            publisher.publish(msg)
        except Exception:
            if not rclpy.ok():
                return
            raise

    def make_ergonomics(
        self, hand_angles: dict[str, Any] | None, *, side: str
    ) -> list[ManusErgonomics]:
        if not isinstance(hand_angles, dict):
            return []

        stretch_sign = (
            self.left_stretch_sign if side == "left" else self.right_stretch_sign
        )
        spread_sign = self.left_spread_sign if side == "left" else self.right_spread_sign
        ergonomics = []
        for index, finger_name in enumerate(FINGER_NAMES):
            self.append_angle_ergonomics(
                ergonomics,
                f"{finger_name}MCPStretch",
                self.relative_angle_at(hand_angles, "pitch", index, side),
                stretch_sign,
            )
            spread_name = (
                "ThumbMCPSpread" if finger_name == "Thumb" else f"{finger_name}Spread"
            )
            self.append_angle_ergonomics(
                ergonomics,
                spread_name,
                self.relative_angle_at(hand_angles, "side", index, side),
                spread_sign,
            )
            self.append_angle_ergonomics(
                ergonomics,
                f"{finger_name}PIPStretch",
                self.relative_angle_at(hand_angles, "two_pitch", index, side),
                stretch_sign,
            )
            self.append_angle_ergonomics(
                ergonomics,
                f"{finger_name}DIPStretch",
                self.relative_angle_at(hand_angles, "end_pitch", index, side),
                stretch_sign,
            )

        return ergonomics

    def append_angle_ergonomics(
        self,
        ergonomics: list[ManusErgonomics],
        ergo_type: str,
        value: float | None,
        sign: float,
    ) -> None:
        if value is None:
            return
        ergo = ManusErgonomics()
        ergo.type = ergo_type
        ergo.value = float(value * self.angle_scale * sign)
        ergonomics.append(ergo)

    @staticmethod
    def angle_at(hand_angles: dict[str, Any], key: str, index: int) -> float | None:
        values = hand_angles.get(key)
        if not isinstance(values, list) or index >= len(values):
            return None
        try:
            return float(values[index])
        except (TypeError, ValueError):
            return None

    def relative_angle_at(
        self, hand_angles: dict[str, Any], key: str, index: int, side: str
    ) -> float | None:
        angle = self.angle_at(hand_angles, key, index)
        if angle is None:
            return None
        zero_angles = self.angle_zero_by_side.get(side)
        if not zero_angles:
            return angle
        zero = self.angle_at(zero_angles, key, index)
        if zero is None:
            return angle
        return angle - zero

    def make_raw_node(
        self,
        node_id: int,
        parent_node_id: int,
        joint_type: str,
        chain_type: str,
        point: dict[str, Any],
        *,
        side: str,
    ) -> ManusRawNode:
        node = ManusRawNode()
        node.node_id = int(node_id)
        node.parent_node_id = int(parent_node_id)
        node.joint_type = joint_type
        node.chain_type = chain_type
        node.pose = self.pose_from_hex_point(point)
        if (
            self.thumb_position_calibration_enabled
            and chain_type == "thumb"
            and node_id in THUMB_POSITION_CALIBRATION_NODE_IDS
        ):
            self.apply_thumb_position_calibration(node.pose, side)
        return node

    def pose_from_hex_point(self, point: dict[str, Any]) -> Pose:
        x = float(point.get("x", 0.0)) * self.position_scale
        y = float(point.get("y", 0.0)) * self.position_scale
        z = float(point.get("z", 0.0)) * self.position_scale

        pose = Pose()
        if self.revo2_coordinate_transform:
            pose.position.x = -y
            pose.position.y = -x
            pose.position.z = z
        else:
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    def apply_thumb_position_calibration(self, pose: Pose, side: str) -> None:
        if side == "left":
            matrix = self.left_thumb_position_matrix
            scale = self.left_thumb_position_scale_xyz
            offset = self.left_thumb_position_offset_xyz
        else:
            matrix = self.right_thumb_position_matrix
            scale = self.right_thumb_position_scale_xyz
            offset = self.right_thumb_position_offset_xyz
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        mx = matrix[0] * x + matrix[1] * y + matrix[2] * z
        my = matrix[3] * x + matrix[4] * y + matrix[5] * z
        mz = matrix[6] * x + matrix[7] * y + matrix[8] * z
        pose.position.x = mx * scale[0] + offset[0]
        pose.position.y = my * scale[1] + offset[1]
        pose.position.z = mz * scale[2] + offset[2]

    def destroy_node(self) -> bool:
        self.angles_stream.close()
        self.positions_stream.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = HexGloveUdpNode()
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
