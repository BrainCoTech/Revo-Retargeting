#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node
from revo3_mit_controller_msgs.msg import Revo3MITCommand

from manus_revo3_retarget.anyteleop_retarget import AnyTeleopSolver, JOINT_NAMES, _glove_points
from manus_revo3_retarget.mit_linear_interpolator import LinearMitCommandInterpolator, duration_from_hz


class AnyTeleopNode(Node):
    def __init__(self):
        super().__init__("manus_revo3_retarget_anyteleop")
        self.hand_mode = str(self._param("hand_mode", "both")).strip().lower()
        self.use_namespace = bool(self._param("use_revo3_namespace", True))
        self.command_suffix = str(
            self._param("command_topic_suffix", "joint_forward_mit_controller/commands")
        )
        self.target_suffix = str(
            self._param(
                "retarget_target_topic_suffix",
                "joint_forward_mit_controller/retarget_targets",
            )
        )
        self.publish_hz = float(self._param("mit_command_publish_hz", 200.0))
        period_s = duration_from_hz(self.publish_hz, "mit_command_publish_hz")
        self.filter_alpha = float(self._param("anyteleop_filter_alpha", 0.4))
        default_config = (
            Path(get_package_share_directory("manus_revo3_retarget"))
            / "config"
            / "anyteleop_retarget.yaml"
        )
        config = Path(str(self._param("anyteleop_config", str(default_config))))
        sides = ("left", "right") if self.hand_mode == "both" else (self.hand_mode,)
        if any(side not in ("left", "right") for side in sides):
            raise ValueError("hand_mode must be left, right, or both")
        description = Path(get_package_share_directory("revo3_description"))
        self.solvers = {side: AnyTeleopSolver(side, config, description, self.filter_alpha) for side in sides}
        self.names = {side: [name.format(side=side) for name in JOINT_NAMES] for side in sides}
        self.interpolators = {
            side: LinearMitCommandInterpolator(period_s) for side in sides
        }
        self.target_publishers = {
            side: self.create_publisher(
                Revo3MITCommand, self._topic(side, self.target_suffix), 10
            )
            for side in sides
        }
        self.command_publishers = {
            side: self.create_publisher(
                Revo3MITCommand, self._topic(side, self.command_suffix), 10
            )
            for side in sides
        }
        self.create_subscription(ManusGlove, "/manus_glove_0", self._on_glove, 10)
        self.create_subscription(ManusGlove, "/manus_glove_1", self._on_glove, 10)
        self.create_timer(period_s, self._publish_latest)
        self.get_logger().info(
            f"AnyTeleop retarget ready hand_mode={self.hand_mode} "
            f"command_hz={self.publish_hz:.1f}"
        )

    def _param(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _topic(self, side: str, suffix: str) -> str:
        suffix = suffix.strip("/")
        return f"/revo3_{side}/{suffix}" if self.use_namespace else f"/{suffix}"

    def _calibrated(self, side: str, positions: np.ndarray) -> np.ndarray:
        output = positions.copy()
        for index, name in enumerate(self.names[side]):
            short = name.removeprefix(f"{side}_")
            scale = float(self._param(f"physical_{side}_{short}_scale", 1.0))
            offset = math.radians(
                float(self._param(f"physical_{side}_{short}_offset_deg", 0.0))
            )
            output[index] = output[index] * scale + offset
        return output

    def _gains(self, side: str) -> tuple[list[float], list[float]]:
        gains = []
        for field, default in (("kp", 0.4), ("kd", 0.05)):
            fallback = float(self._param(f"mit_default_{field}", default))
            values = [float(self._param(f"mit_{name}_{field}", -1.0))
                      for name in self.names[side]]
            gains.append([v if math.isfinite(v) and v >= 0.0 else fallback for v in values])
        return tuple(gains)

    def _message(self, sample) -> Revo3MITCommand:
        message = Revo3MITCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(sample.joint_names)
        message.position = list(sample.position)
        message.velocity = (list(sample.velocity) if self._param("mit_velocity_feedforward_enabled", True)
                            else [0.0] * len(sample.position))
        message.effort = [0.0] * len(sample.position)
        message.kp = list(sample.kp)
        message.kd = list(sample.kd)
        return message

    def _on_glove(self, message: ManusGlove) -> None:
        side = str(message.side).strip().lower()
        if side not in self.solvers:
            return
        try:
            points = _glove_points(message, side)
            position = self._calibrated(side, self.solvers[side].retarget(points))
            now_s = time.monotonic()
            kp, kd = self._gains(side)
            names = self.names[side]
            self.interpolators[side].update_target(names, position, kp, kd, now_s)
            target = Revo3MITCommand()
            target.header.stamp = self.get_clock().now().to_msg()
            target.joint_names = list(names)
            target.position = position.tolist()
            target.velocity = [0.0] * len(names)
            target.effort = [0.0] * len(names)
            target.kp = kp
            target.kd = kd
            self.target_publishers[side].publish(target)
        except (RuntimeError, ValueError) as error:
            self.get_logger().warning(f"dropped {side} MANUS frame: {error}", throttle_duration_sec=2.0)

    def _publish_latest(self) -> None:
        now_s = time.monotonic()
        for side, interpolator in self.interpolators.items():
            sample = interpolator.sample(now_s)
            if sample is not None:
                self.command_publishers[side].publish(self._message(sample))


def main() -> None:
    rclpy.init()
    node = AnyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
