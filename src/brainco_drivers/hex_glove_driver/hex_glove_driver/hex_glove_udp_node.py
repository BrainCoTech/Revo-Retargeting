#!/usr/bin/env python3
"""Receive Hexacercle UDP streams and publish validated raw JSON frames."""

from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class UdpJsonStream:
    """Runtime state for one Hex UDP endpoint."""

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
    """Transport-only Hex driver; device semantics belong in an adapter."""

    def __init__(self) -> None:
        super().__init__("hex_glove_udp_node")

        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("angles_port", 9011)
        self.declare_parameter("positions_port", 9013)
        self.declare_parameter("connect_message", "CONNECT")
        self.declare_parameter("connect_period_sec", 1.0)
        self.declare_parameter("raw_angles_topic", "/hex_glove/raw_angles")
        self.declare_parameter("raw_positions_topic", "/hex_glove/raw_positions")

        server_host = str(self.get_parameter("server_host").value)
        angles_port = int(self.get_parameter("angles_port").value)
        positions_port = int(self.get_parameter("positions_port").value)
        self.connect_message = str(
            self.get_parameter("connect_message").value
        ).encode("utf-8")
        self.connect_period_sec = max(
            0.05, float(self.get_parameter("connect_period_sec").value)
        )
        angles_topic = str(self.get_parameter("raw_angles_topic").value)
        positions_topic = str(self.get_parameter("raw_positions_topic").value)

        self.angles_stream = UdpJsonStream(
            "angles",
            server_host,
            angles_port,
            self.create_publisher(String, angles_topic, 20),
        )
        self.positions_stream = UdpJsonStream(
            "positions",
            server_host,
            positions_port,
            self.create_publisher(String, positions_topic, 20),
        )

        for stream in (self.angles_stream, self.positions_stream):
            local_host, local_port = stream.local_address
            self.get_logger().info(
                f"Hex {stream.name}: {local_host}:{local_port} -> "
                f"{server_host}:{stream.server_port}"
            )
        self.get_logger().info(
            f"Hex raw topics: angles={angles_topic}, positions={positions_topic}"
        )
        self.timer = self.create_timer(0.001, self._poll)

    def _poll(self) -> None:
        for stream in (self.angles_stream, self.positions_stream):
            self._send_connect_if_needed(stream)
            self._poll_stream(stream)

    def _send_connect_if_needed(self, stream: UdpJsonStream) -> None:
        now = time.monotonic()
        if now - stream.last_connect_sent < self.connect_period_sec:
            return
        try:
            stream.socket.sendto(self.connect_message, stream.server_address)
        except OSError as error:
            self.get_logger().warning(
                f"Failed to send Hex {stream.name} CONNECT packet: {error}",
                throttle_duration_sec=2.0,
            )
            return
        stream.last_connect_sent = now

    def _poll_stream(self, stream: UdpJsonStream) -> None:
        while rclpy.ok():
            try:
                data, _address = stream.socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error(f"Hex {stream.name} UDP socket error: {error}")
                return

            text = data.decode("utf-8", errors="replace")
            if not self._is_hand_payload(stream, text):
                continue
            message = String()
            message.data = text
            stream.publisher.publish(message)
            stream.frame_count += 1
            if stream.frame_count == 1 or stream.frame_count % 500 == 0:
                self.get_logger().info(
                    f"Published {stream.frame_count} Hex {stream.name} raw frame(s)."
                )

    def _is_hand_payload(self, stream: UdpJsonStream, text: str) -> bool:
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as error:
            stream.json_error_count += 1
            if stream.json_error_count <= 3:
                self.get_logger().warning(
                    f"Dropping malformed Hex {stream.name} JSON: {error}"
                )
            return False
        return isinstance(payload, dict) and (
            isinstance(payload.get("leftHand"), dict)
            or isinstance(payload.get("rightHand"), dict)
        )

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
