"""Single-device SDK stream with bounded reconnect backoff."""
import importlib
import math
from pathlib import Path
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from revohuman_msgs.msg import RawFrame

from .conversion import convert_frame


def load_sdk(path):
    if path:
        directory = Path(path).expanduser().resolve()
        if not (directory / "revohuman_glove.py").is_file():
            raise ValueError(f"SDK module not found in {directory}")
        sys.path.insert(0, str(directory))
    return importlib.import_module("revohuman_glove")


class RevoHumanDriver(Node):
    def __init__(self):
        super().__init__("revohuman_driver")
        for name, default in (("sdk_path", ""), ("port", ""), ("side", "left"),
                              ("target_rate_hz", 100.0), ("timeout_sec", 1.0),
                              ("reconnect_delay_sec", 2.0)):
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.port = value("port")
        self.side = value("side")
        if not self.port or self.side not in ("left", "right"):
            raise ValueError("Explicit port and side:=left|right are required")
        self.sdk = load_sdk(value("sdk_path"))
        self.rate = float(value("target_rate_hz"))
        self.timeout = float(value("timeout_sec"))
        self.delay = float(value("reconnect_delay_sec"))
        if not (math.isfinite(self.rate) and
                self.sdk.STREAM_MIN_RATE_HZ <= self.rate <= self.sdk.STREAM_MAX_RATE_HZ):
            raise ValueError("target_rate_hz is outside SDK limits")
        if any(not math.isfinite(v) or v <= 0 for v in (self.timeout, self.delay)):
            raise ValueError("timeout_sec and reconnect_delay_sec must be positive")
        self.publisher = self.create_publisher(
            RawFrame, f"/revohuman/{self.side}/raw", 10)
        self.stopped = threading.Event()
        self.context.on_shutdown(self.stopped.set)

    def run(self):
        # One thread owns the serial connection; SDK contexts STOP and close it
        # on normal shutdown, receive errors and disconnects alike.
        while rclpy.ok(context=self.context) and not self.stopped.is_set():
            try:
                with self.sdk.DeviceManager.connect(
                        port=self.port, timeout=self.timeout) as glove:
                    info = glove.device_info.read()
                    self.get_logger().info(
                        f"Connected {self.side} on {self.port}: {info.product_type.name}")
                    with glove.sensor_data.stream(self.rate) as stream:
                        while rclpy.ok(context=self.context) and not self.stopped.is_set():
                            frame = stream.recv()
                            if self.stopped.is_set() or not rclpy.ok(context=self.context):
                                break
                            message = convert_frame(
                                frame, RawFrame(), self.get_clock().now().to_msg(), self.side)
                            self.publisher.publish(message)
            except (KeyboardInterrupt, ROSInterruptException):
                break
            except (self.sdk.RevoHumanError, OSError) as exc:
                if self.stopped.is_set():
                    break
                self.get_logger().error(f"Sensor connection failed: {exc}; reconnecting")
                self.stopped.wait(self.delay)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RevoHumanDriver()
        node.run()
    except (KeyboardInterrupt, ROSInterruptException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
