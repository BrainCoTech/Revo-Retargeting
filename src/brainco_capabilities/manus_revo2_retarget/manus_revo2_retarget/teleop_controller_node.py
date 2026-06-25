#!/usr/bin/env python3
"""Split Revo2 teleoperation controller.

Consumes retargeted target JointState messages and Revo2 feedback JointState
messages, then publishes rad/s velocity commands for revo2_driver.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from manus_revo2_retarget.revo2_joints import (
    REVO2_JOINT_SUFFIXES,
    REVO2_JOINT_UPPER_LIMITS_RAD,
    command_order_label,
)


VALID_HAND_MODES = {"left", "right", "both"}


def _parse_joint_indices(value, name: str) -> tuple[int, ...]:
    if value is None:
        return (2, 3, 4, 5)
    if isinstance(value, str):
        parts = [item.strip() for item in value.split(",") if item.strip()]
    else:
        parts = list(value)
    if not parts:
        raise ValueError(f"{name} must contain at least one joint index.")
    indices = []
    for item in parts:
        index = int(item)
        if index < 0 or index > 5:
            raise ValueError(f"{name} joint indices must be in [0, 5].")
        if index not in indices:
            indices.append(index)
    return tuple(indices)


def _parse_six_float_vector(value, name: str, default: float) -> np.ndarray:
    if value is None:
        return np.full(6, default, dtype=float)
    if isinstance(value, str):
        parts = [item.strip() for item in value.split(",") if item.strip()]
    else:
        parts = list(value)
    if len(parts) != 6:
        raise ValueError(f"{name} must contain exactly 6 values.")
    return np.asarray([float(item) for item in parts], dtype=float)


@dataclass
class SideState:
    target_position: np.ndarray
    filtered_target: np.ndarray
    actual_position: np.ndarray
    last_error: np.ndarray
    filtered_derivative: np.ndarray
    target_velocity: np.ndarray
    command_velocity: np.ndarray
    target_ready: bool = False
    feedback_ready: bool = False
    filter_initialized: bool = False
    target_time: float | None = None
    feedback_time: float | None = None
    last_pd_time: float | None = None


class Revo2TeleopController(Node):
    """PD velocity controller for split Revo2 teleoperation."""

    def __init__(self) -> None:
        super().__init__("revo2_teleop_controller")

        self._declare_parameters()
        self.hand_mode = str(self.get_parameter("hand_mode").value).lower()
        if self.hand_mode not in VALID_HAND_MODES:
            raise ValueError(f"Invalid hand_mode: {self.hand_mode}")
        self.enable_left = self.hand_mode in ("left", "both")
        self.enable_right = self.hand_mode in ("right", "both")

        self.control_hz = float(self.get_parameter("control_hz").value)
        if self.control_hz <= 0.0:
            raise ValueError("control_hz must be positive.")
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        self.feedback_timeout = float(
            self.get_parameter("pd_velocity.velocity_feedback_timeout").value
        )

        self.target_filter_alpha = float(
            np.clip(float(self.get_parameter("target_filter.alpha").value), 0.0, 1.0)
        )
        fast_alpha_value = self.get_parameter("target_filter.fast_alpha").value
        if fast_alpha_value is None:
            self.target_filter_fast_alpha = self.target_filter_alpha
        else:
            self.target_filter_fast_alpha = float(np.clip(float(fast_alpha_value), 0.0, 1.0))
        self.target_filter_fast_alpha = max(
            self.target_filter_alpha, self.target_filter_fast_alpha
        )
        self.target_filter_fast_threshold = float(
            max(0.0, float(self.get_parameter("target_filter.fast_threshold").value))
        )

        self.velocity_kp = float(self.get_parameter("pd_velocity.velocity_kp").value)
        self.velocity_kd = float(self.get_parameter("pd_velocity.velocity_kd").value)
        self.pd_derivative_alpha = float(
            np.clip(float(self.get_parameter("pd_velocity.derivative_alpha").value), 0.0, 1.0)
        )
        self.velocity_deadband = float(
            max(0.0, float(self.get_parameter("pd_velocity.velocity_deadband").value))
        )
        thumb_deadband = self.get_parameter("pd_velocity.thumb_velocity_deadband").value
        self.thumb_velocity_deadband = (
            self.velocity_deadband
            if thumb_deadband is None
            else float(max(0.0, float(thumb_deadband)))
        )
        ring_deadband = self.get_parameter("pd_velocity.ring_velocity_deadband").value
        self.ring_velocity_deadband = (
            self.velocity_deadband
            if ring_deadband is None
            else float(max(0.0, float(ring_deadband)))
        )
        self.thumb_velocity_min = float(
            max(0.0, float(self.get_parameter("pd_velocity.thumb_velocity_min").value))
        )
        self.ring_velocity_min = float(
            max(0.0, float(self.get_parameter("pd_velocity.ring_velocity_min").value))
        )
        self.thumb_velocity_kp_scale = float(
            max(0.0, float(self.get_parameter("pd_velocity.thumb_velocity_kp_scale").value))
        )
        self.ring_velocity_kp_scale = float(
            max(0.0, float(self.get_parameter("pd_velocity.ring_velocity_kp_scale").value))
        )
        self.thumb_velocity_brake_zone = float(
            max(0.0, float(self.get_parameter("pd_velocity.thumb_velocity_brake_zone").value))
        )
        self.thumb_velocity_brake_max = float(
            max(0.0, float(self.get_parameter("pd_velocity.thumb_velocity_brake_max").value))
        )
        self.four_finger_extension_velocity_scale = float(
            max(0.0, float(self.get_parameter("pd_velocity.four_finger_extension_velocity_scale").value))
        )
        self.four_finger_extension_joints = _parse_joint_indices(
            self.get_parameter("pd_velocity.four_finger_extension_joints").value,
            "pd_velocity.four_finger_extension_joints",
        )
        self.velocity_max = float(
            max(0.0, float(self.get_parameter("pd_velocity.velocity_max").value))
        )
        self.velocity_slew_rate = float(
            max(0.0, float(self.get_parameter("pd_velocity.velocity_slew_rate").value))
        )
        self.velocity_zero_epsilon = min(0.002, max(1e-6, self.velocity_max * 1e-3))

        feedback_scale = float(self.get_parameter("feedback.position_scale").value)
        self.feedback_position_scales = _parse_six_float_vector(
            self.get_parameter("feedback.position_scales").value,
            "feedback.position_scales",
            feedback_scale,
        )
        self.feedback_position_offsets = _parse_six_float_vector(
            self.get_parameter("feedback.position_offsets").value,
            "feedback.position_offsets",
            0.0,
        )

        self.left_state = self._make_side_state()
        self.right_state = self._make_side_state()
        self.left_publisher = None
        self.right_publisher = None
        self._last_warn_ts = 0.0

        self._setup_side("left", self.left_state, self.enable_left)
        self._setup_side("right", self.right_state, self.enable_right)

        self.get_logger().info(
            "Revo2 teleop controller ready: "
            f"hand_mode={self.hand_mode}, order=[{command_order_label()}], "
            "target/feedback=rad, command=rad/s"
        )
        self.create_timer(1.0 / self.control_hz, self._control_callback)

    def _declare_parameters(self) -> None:
        self.declare_parameter("hand_mode", "right")
        self.declare_parameter("control_hz", 100.0)
        self.declare_parameter("target_timeout", 0.3)
        self.declare_parameter("left_target_joint_state_topic", "/revo2_left/revo2_pid_controller/target_joint_states")
        self.declare_parameter("right_target_joint_state_topic", "/revo2_right/revo2_pid_controller/target_joint_states")
        self.declare_parameter("revo2_driver.left_velocity_command_topic", "/revo2_left/joint_forward_vel_controller/commands")
        self.declare_parameter("revo2_driver.right_velocity_command_topic", "/revo2_right/joint_forward_vel_controller/commands")
        self.declare_parameter("revo2_driver.left_joint_state_topic", "/revo2_left/revo2_joint_state/joint_states")
        self.declare_parameter("revo2_driver.right_joint_state_topic", "/revo2_right/revo2_joint_state/joint_states")
        self.declare_parameter("target_filter.alpha", 0.45)
        self.declare_parameter("target_filter.fast_alpha", 0.9)
        self.declare_parameter("target_filter.fast_threshold", 0.095993109)
        self.declare_parameter("pd_velocity.velocity_kp", 2.4)
        self.declare_parameter("pd_velocity.velocity_kd", 0.0)
        self.declare_parameter("pd_velocity.derivative_alpha", 0.25)
        self.declare_parameter("pd_velocity.velocity_deadband", 0.013962634)
        self.declare_parameter("pd_velocity.velocity_max", 2.094395102)
        self.declare_parameter("pd_velocity.velocity_slew_rate", 1.0)
        self.declare_parameter("pd_velocity.velocity_feedback_timeout", 0.3)
        self.declare_parameter("pd_velocity.thumb_velocity_deadband", 0.020943951)
        self.declare_parameter("pd_velocity.thumb_velocity_min", 0.0)
        self.declare_parameter("pd_velocity.thumb_velocity_kp_scale", 2.4)
        self.declare_parameter("pd_velocity.thumb_velocity_brake_zone", 0.209439510)
        self.declare_parameter("pd_velocity.thumb_velocity_brake_max", 0.628318531)
        self.declare_parameter("pd_velocity.ring_velocity_deadband", 0.020943951)
        self.declare_parameter("pd_velocity.ring_velocity_min", 0.0)
        self.declare_parameter("pd_velocity.ring_velocity_kp_scale", 1.5)
        self.declare_parameter("pd_velocity.four_finger_extension_velocity_scale", 1.5)
        self.declare_parameter("pd_velocity.four_finger_extension_joints", "2,3,4,5")
        self.declare_parameter("feedback.position_scale", 1.0)
        self.declare_parameter("feedback.position_scales", "1,1,1,1,1,1")
        self.declare_parameter("feedback.position_offsets", "0,0,0,0,0,0")

    @staticmethod
    def _make_side_state() -> SideState:
        zeros = np.zeros(6, dtype=float)
        return SideState(
            target_position=zeros.copy(),
            filtered_target=zeros.copy(),
            actual_position=zeros.copy(),
            last_error=zeros.copy(),
            filtered_derivative=zeros.copy(),
            target_velocity=zeros.copy(),
            command_velocity=zeros.copy(),
        )

    def _setup_side(self, side: str, state: SideState, enabled: bool) -> None:
        if not enabled:
            return
        target_topic = str(self.get_parameter(f"{side}_target_joint_state_topic").value)
        feedback_topic = str(self.get_parameter(f"revo2_driver.{side}_joint_state_topic").value)
        command_topic = str(self.get_parameter(f"revo2_driver.{side}_velocity_command_topic").value)

        self.create_subscription(
            JointState,
            target_topic,
            lambda msg, side=side: self._on_target(msg, side),
            10,
        )
        self.create_subscription(
            JointState,
            feedback_topic,
            lambda msg, side=side: self._on_feedback(msg, side),
            10,
        )
        publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        if side == "left":
            self.left_publisher = publisher
        else:
            self.right_publisher = publisher
        self.get_logger().info(
            f"{side} controller topics: target={target_topic}, "
            f"feedback={feedback_topic}, command={command_topic}"
        )

    def _side_state(self, side: str) -> SideState:
        if side == "left":
            return self.left_state
        if side == "right":
            return self.right_state
        raise ValueError(f"Invalid side: {side}")

    def _publisher(self, side: str):
        return self.left_publisher if side == "left" else self.right_publisher

    def _joint_state_positions(self, msg: JointState, side: str, *, feedback: bool) -> np.ndarray:
        if len(msg.name) == 0 and len(msg.position) >= 6:
            positions = np.asarray(msg.position[:6], dtype=float)
        else:
            name_to_index = {name: i for i, name in enumerate(msg.name)}
            prefix = "left" if side == "left" else "right"
            positions = np.zeros(6, dtype=float)
            for i, suffix in enumerate(REVO2_JOINT_SUFFIXES):
                candidates = (f"{prefix}_{suffix}", suffix)
                index = next((name_to_index[name] for name in candidates if name in name_to_index), None)
                if index is None or index >= len(msg.position):
                    raise ValueError(
                        f"{side} JointState missing joint {prefix}_{suffix}; names={list(msg.name)}"
                    )
                positions[i] = float(msg.position[index])
        positions = np.clip(positions, 0.0, np.asarray(REVO2_JOINT_UPPER_LIMITS_RAD, dtype=float))
        if feedback:
            positions = positions * self.feedback_position_scales + self.feedback_position_offsets
        return positions

    def _on_target(self, msg: JointState, side: str) -> None:
        try:
            target = self._joint_state_positions(msg, side, feedback=False)
        except Exception as exc:
            self._warn_throttled(f"Failed to parse {side} target JointState: {exc}")
            return
        state = self._side_state(side)
        state.target_position = target
        state.target_time = time.time()
        if not state.target_ready:
            self.get_logger().info(f"Received first {side} target JointState.")
        state.target_ready = True

    def _on_feedback(self, msg: JointState, side: str) -> None:
        try:
            actual = self._joint_state_positions(msg, side, feedback=True)
        except Exception as exc:
            self._warn_throttled(f"Failed to parse {side} Revo2 feedback JointState: {exc}")
            return
        state = self._side_state(side)
        state.actual_position = actual
        state.feedback_time = time.time()
        if not state.feedback_ready:
            self.get_logger().info(f"Received first {side} Revo2 feedback JointState.")
        state.feedback_ready = True

    def _warn_throttled(self, message: str) -> None:
        now = time.time()
        if now - self._last_warn_ts > 1.0:
            self._last_warn_ts = now
            self.get_logger().warning(message)

    def _filtered_target(self, state: SideState) -> np.ndarray:
        target = state.target_position
        if not state.filter_initialized:
            state.filtered_target = target.copy()
            state.filter_initialized = True
            return state.filtered_target.copy()
        if self.target_filter_fast_threshold <= 0.0:
            alpha = self.target_filter_alpha
        else:
            delta = np.abs(target - state.filtered_target)
            alpha = np.where(
                delta >= self.target_filter_fast_threshold,
                self.target_filter_fast_alpha,
                self.target_filter_alpha,
            )
        state.filtered_target = (1.0 - alpha) * state.filtered_target + alpha * target
        return state.filtered_target.copy()

    def _update_target_velocity(self, state: SideState, now: float) -> None:
        target = self._filtered_target(state)
        error = target - state.actual_position
        dt = 0.1 if state.last_pd_time is None else max(now - state.last_pd_time, 1e-3)
        raw_derivative = (error - state.last_error) / dt
        state.filtered_derivative = (
            (1.0 - self.pd_derivative_alpha) * state.filtered_derivative
            + self.pd_derivative_alpha * raw_derivative
        )
        target_velocity = self.velocity_kp * error + self.velocity_kd * state.filtered_derivative
        target_velocity[:2] *= self.thumb_velocity_kp_scale
        target_velocity[4] *= self.ring_velocity_kp_scale

        deadbands = np.full(6, self.velocity_deadband, dtype=float)
        deadbands[:2] = self.thumb_velocity_deadband
        deadbands[4] = self.ring_velocity_deadband
        target_velocity[np.abs(error) <= deadbands] = 0.0

        for joint_index in self.four_finger_extension_joints:
            if target_velocity[joint_index] < 0.0:
                target_velocity[joint_index] *= self.four_finger_extension_velocity_scale
        target_velocity = np.clip(target_velocity, -self.velocity_max, self.velocity_max)

        if self.thumb_velocity_min > 0.0:
            thumb_min = min(self.thumb_velocity_min, self.velocity_max)
            thumb_active = (
                (np.abs(error[:2]) > self.thumb_velocity_deadband)
                & (np.abs(target_velocity[:2]) < thumb_min)
            )
            thumb_velocity = target_velocity[:2]
            thumb_velocity[thumb_active] = np.sign(error[:2][thumb_active]) * thumb_min
            target_velocity[:2] = thumb_velocity

        if (
            self.ring_velocity_min > 0.0
            and abs(error[4]) > self.ring_velocity_deadband
            and abs(target_velocity[4]) < self.ring_velocity_min
        ):
            target_velocity[4] = np.sign(error[4]) * min(self.ring_velocity_min, self.velocity_max)

        if self.thumb_velocity_brake_zone > 0.0 and self.thumb_velocity_brake_max > 0.0:
            thumb_error_abs = np.abs(error[:2])
            thumb_brake_active = thumb_error_abs < self.thumb_velocity_brake_zone
            if np.any(thumb_brake_active):
                brake_cap = (
                    min(self.thumb_velocity_brake_max, self.velocity_max)
                    * thumb_error_abs[thumb_brake_active]
                    / self.thumb_velocity_brake_zone
                )
                thumb_velocity = target_velocity[:2]
                thumb_velocity[thumb_brake_active] = np.clip(
                    thumb_velocity[thumb_brake_active],
                    -brake_cap,
                    brake_cap,
                )
                target_velocity[:2] = thumb_velocity

        state.last_error = error
        state.last_pd_time = now
        state.target_velocity = target_velocity

    def _publish_side(self, side: str, state: SideState, now: float) -> None:
        publisher = self._publisher(side)
        if publisher is None:
            return
        if not state.target_ready:
            self._warn_throttled(f"Waiting for {side} target JointState.")
            return
        if not state.feedback_ready:
            self._warn_throttled(f"Waiting for {side} Revo2 feedback JointState.")
            return

        target_stale = state.target_time is None or now - state.target_time > self.target_timeout
        feedback_stale = (
            state.feedback_time is None or now - state.feedback_time > self.feedback_timeout
        )
        if target_stale or feedback_stale:
            state.target_velocity = np.zeros(6, dtype=float)
            reason = "target" if target_stale else "feedback"
            self._warn_throttled(f"{side} {reason} timeout, velocity target set to zero.")
        else:
            self._update_target_velocity(state, now)

        zero_target = np.abs(state.target_velocity) < self.velocity_zero_epsilon
        reversing = (state.target_velocity * state.command_velocity) < 0.0
        state.command_velocity[zero_target | reversing] = 0.0
        state.command_velocity = state.command_velocity + np.clip(
            state.target_velocity - state.command_velocity,
            -self.velocity_slew_rate,
            self.velocity_slew_rate,
        )
        state.command_velocity[np.abs(state.command_velocity) < self.velocity_zero_epsilon] = 0.0

        msg = Float64MultiArray()
        msg.data = [float(v) for v in state.command_velocity]
        publisher.publish(msg)

    def _control_callback(self) -> None:
        now = time.time()
        if self.enable_left:
            self._publish_side("left", self.left_state, now)
        if self.enable_right:
            self._publish_side("right", self.right_state, now)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = Revo2TeleopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
