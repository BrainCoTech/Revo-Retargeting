#!/usr/bin/env python3
import argparse
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_SUFFIXES = (
    "thumb_proximal_joint",
    "thumb_metacarpal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)
JOINT_SHORT_NAMES = (
    "thumb_prox",
    "thumb_meta",
    "index_prox",
    "middle_prox",
    "ring_prox",
    "pinky_prox",
)
JOINT_DISPLAY_NAMES = (
    "thumb_prox",
    "thumb_meta",
    "index",
    "middle",
    "ring",
    "pinky",
)


def joint_names_for_side(side):
    return tuple(f"{side}_{suffix}" for suffix in JOINT_SUFFIXES)


def short_joint_names_for_side(side):
    return tuple(f"{side}_{name}" for name in JOINT_SHORT_NAMES)


def parse_joint_selection(value):
    if not value:
        return list(range(len(JOINT_SHORT_NAMES)))

    aliases = {name: idx for idx, name in enumerate(JOINT_SHORT_NAMES)}
    aliases.update({name: idx for idx, name in enumerate(JOINT_DISPLAY_NAMES)})
    aliases.update(
        {
            "thumb": 0,
            "thumb_proximal": 0,
            "thumb_meta": 1,
            "thumb_metacarpal": 1,
            "index_prox": 2,
            "middle_prox": 3,
            "ring_prox": 4,
            "pinky_prox": 5,
        }
    )

    selected = []
    for token in value.split(","):
        key = token.strip()
        if not key:
            continue
        idx = int(key) if key.isdigit() else aliases.get(key)
        if idx is None:
            raise ValueError(f"unknown joint selector: {key}")
        if idx < 0 or idx >= len(JOINT_SHORT_NAMES):
            raise ValueError(f"joint index out of range: {idx}")
        if idx not in selected:
            selected.append(idx)
    return selected


def vector_from_joint_state(msg, candidate_names, field="position"):
    values_by_name = getattr(msg, field)
    if len(values_by_name) < len(msg.name):
        return None

    index_by_name = {name: i for i, name in enumerate(msg.name)}
    values = []
    for names in candidate_names:
        found = None
        for name in names:
            idx = index_by_name.get(name)
            if idx is not None and idx < len(values_by_name):
                found = float(values_by_name[idx])
                break
        if found is None:
            return None
        values.append(found)
    return np.asarray(values, dtype=float)


def candidate_joint_names_for_side(side):
    full_names = joint_names_for_side(side)
    short_names = short_joint_names_for_side(side)
    legacy_short_names = tuple(
        f"{side}_{name}" for name in JOINT_DISPLAY_NAMES
    )
    return tuple(
        (
            short_name,
            legacy_short_name,
            full_name,
            full_name.removeprefix(f"{side}_"),
        )
        for short_name, legacy_short_name, full_name in zip(
            short_names,
            legacy_short_names,
            full_names,
        )
    )


class SeriesBuffer:
    def __init__(self, max_samples):
        self.t = deque(maxlen=max_samples)
        self.y = deque(maxlen=max_samples)

    def append(self, timestamp, values):
        self.t.append(float(timestamp))
        self.y.append(np.asarray(values, dtype=float))

    def arrays(self):
        if not self.t:
            return np.asarray([]), np.empty((0, len(JOINT_SHORT_NAMES)))
        return np.asarray(self.t), np.vstack(self.y)


class Revo2RetargetPlotter(Node):
    def __init__(self, args):
        super().__init__("revo2_retarget_plot")
        self.side = args.side
        self.start_time = time.monotonic()
        self.window_sec = args.window
        self.selected = parse_joint_selection(args.joints)

        if args.target_topic:
            target_topics = [args.target_topic]
        else:
            target_topics = [
                f"/revo2_{self.side}/revo2_pid_controller/target_joint_states",
                f"/revo2_{self.side}/retarget/joint_states",
                f"/set_{self.side}_hand_joints",
            ]
        actual_topic = args.actual_topic or f"/revo2_{self.side}/revo2_joint_state/joint_states"
        command_topic = args.command_topic

        self.candidate_names = candidate_joint_names_for_side(self.side)

        self.target = SeriesBuffer(args.max_samples)
        self.actual = SeriesBuffer(args.max_samples)
        self.error = SeriesBuffer(args.max_samples)
        self.actual_velocity = SeriesBuffer(args.max_samples)
        self.command = SeriesBuffer(args.max_samples)
        self.latest_target = None
        self.latest_actual = None
        self.target_count = 0
        self.actual_count = 0
        self.actual_velocity_count = 0
        self.command_count = 0
        self.last_target_topic = None
        self.last_bad_target_names = None
        self.last_bad_actual_names = None

        for topic in target_topics:
            self.create_subscription(
                JointState,
                topic,
                lambda msg, topic=topic: self.target_cb(msg, topic),
                10,
            )
        self.create_subscription(JointState, actual_topic, self.actual_cb, 10)
        if command_topic:
            self.create_subscription(Float64MultiArray, command_topic, self.command_cb, 10)
        self.create_timer(1.0, self.status_cb)

        self.get_logger().info(f"Target position topic(s): {', '.join(target_topics)}")
        self.get_logger().info(f"Actual position topic: {actual_topic}")
        if command_topic:
            self.get_logger().info(f"Command velocity topic: {command_topic}")
        self.get_logger().info("Velocity plot uses JointState.velocity; command is overlaid when available.")
        self.get_logger().info("Joint order: " + ", ".join(JOINT_SHORT_NAMES))

    def now_sec(self):
        return time.monotonic() - self.start_time

    def target_cb(self, msg, topic):
        values = vector_from_joint_state(msg, self.candidate_names, field="position")
        if values is None:
            self.last_bad_target_names = list(msg.name)
            self.get_logger().warning(
                f"Target JointState on {topic} does not contain expected Revo2 joints: {list(msg.name)}",
                throttle_duration_sec=2.0,
            )
            return
        t = self.now_sec()
        self.target_count += 1
        self.last_target_topic = topic
        self.latest_target = values
        self.target.append(t, values)
        self.append_error(t)

    def actual_cb(self, msg):
        values = vector_from_joint_state(msg, self.candidate_names, field="position")
        if values is None:
            self.last_bad_actual_names = list(msg.name)
            self.get_logger().warning(
                f"Actual JointState does not contain expected Revo2 joints: {list(msg.name)}",
                throttle_duration_sec=2.0,
            )
            return
        t = self.now_sec()
        self.actual_count += 1
        self.latest_actual = values
        self.actual.append(t, values)
        self.append_error(t)
        velocity = vector_from_joint_state(msg, self.candidate_names, field="velocity")
        if velocity is not None:
            self.actual_velocity_count += 1
            self.actual_velocity.append(t, velocity)

    def command_cb(self, msg):
        if len(msg.data) < len(JOINT_SHORT_NAMES):
            self.get_logger().warning(
                "Velocity command has fewer than 6 values",
                throttle_duration_sec=2.0,
            )
            return
        self.command_count += 1
        self.command.append(self.now_sec(), np.asarray(msg.data[:6], dtype=float))

    def append_error(self, timestamp):
        if self.latest_target is None or self.latest_actual is None:
            return
        self.error.append(timestamp, self.latest_target - self.latest_actual)

    def status_cb(self):
        overlap = 0.0
        target_t, _ = self.target.arrays()
        actual_t, _ = self.actual.arrays()
        if target_t.size and actual_t.size:
            overlap_start = max(float(target_t[0]), float(actual_t[0]))
            overlap_end = min(float(target_t[-1]), float(actual_t[-1]))
            overlap = max(0.0, overlap_end - overlap_start)
        self.get_logger().info(
            f"received target={self.target_count} actual={self.actual_count} "
            f"actual_vel={self.actual_velocity_count} cmd={self.command_count} "
            f"active_target={self.last_target_topic or 'none'} "
            f"overlap={overlap:.2f}s"
        )


class MatplotlibView:
    def __init__(self, node, args):
        try:
            import matplotlib as mpl
            mpl.rcParams["figure.raise_window"] = bool(args.raise_window)
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "matplotlib is required. Try: pip install matplotlib"
            ) from exc

        self.node = node
        self.args = args
        self.plt = plt
        plt.ion()
        self.fig, self.axes = plt.subplots(3, 1, sharex=True, figsize=(13, 9))
        self.fig.canvas.manager.set_window_title("Revo2 retarget target vs actual")

        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        self.pos_lines = {}
        self.err_lines = {}
        self.vel_lines = {}
        self.cmd_lines = {}

        for idx in self.node.selected:
            name = JOINT_DISPLAY_NAMES[idx]
            color = colors[idx % len(colors)]
            actual_line, = self.axes[0].plot(
                [],
                [],
                color=color,
                linestyle="--",
                linewidth=1.2,
                alpha=0.55,
                label=f"{name} actual",
                zorder=2,
            )
            target_line, = self.axes[0].plot(
                [],
                [],
                color=color,
                linestyle="-",
                linewidth=2.2,
                alpha=0.95,
                label=f"{name} target",
                zorder=3,
            )
            err_line, = self.axes[1].plot([], [], color=color, linewidth=1.8, label=name)
            vel_line, = self.axes[2].plot(
                [],
                [],
                color=color,
                linestyle="--",
                linewidth=1.2,
                alpha=0.65,
                label=f"{name} actual vel",
            )
            cmd_line, = self.axes[2].plot(
                [],
                [],
                color=color,
                linewidth=1.6,
                label=f"{name} cmd",
            )
            self.pos_lines[idx] = (target_line, actual_line)
            self.err_lines[idx] = err_line
            self.vel_lines[idx] = vel_line
            self.cmd_lines[idx] = cmd_line

        self.axes[0].set_ylabel("position [rad]")
        self.axes[1].set_ylabel("target - actual [rad]")
        self.axes[2].set_ylabel("velocity [rad/s]")
        self.axes[2].set_xlabel("time [s]")
        for ax in self.axes:
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize="small")
        self.fig.tight_layout(rect=(0.0, 0.0, 0.82, 1.0))

    def update(self):
        target_t, target_y = self.node.target.arrays()
        actual_t, actual_y = self.node.actual.arrays()
        actual_velocity_t, actual_velocity_y = self.node.actual_velocity.arrays()
        command_t, command_y = self.node.command.arrays()

        current_t = self.node.now_sec()
        left_t = max(0.0, current_t - self.node.window_sec)

        for idx in self.node.selected:
            target_line, actual_line = self.pos_lines[idx]
            if target_t.size:
                mask = target_t >= left_t
                target_line.set_data(target_t[mask], target_y[mask, idx])
            if actual_t.size:
                mask = actual_t >= left_t
                actual_line.set_data(actual_t[mask], actual_y[mask, idx])
            if target_t.size and actual_t.size:
                start_t = max(left_t, float(target_t[0]), float(actual_t[0]))
                end_t = min(current_t, float(target_t[-1]), float(actual_t[-1]))
                if end_t > start_t:
                    samples = max(2, min(600, int((end_t - start_t) * 100)))
                    error_t = np.linspace(start_t, end_t, samples)
                    target_interp = np.interp(error_t, target_t, target_y[:, idx])
                    actual_interp = np.interp(error_t, actual_t, actual_y[:, idx])
                    self.err_lines[idx].set_data(error_t, target_interp - actual_interp)
            if command_t.size:
                mask = command_t >= left_t
                self.cmd_lines[idx].set_data(command_t[mask], command_y[mask, idx])
            if actual_velocity_t.size:
                mask = actual_velocity_t >= left_t
                self.vel_lines[idx].set_data(
                    actual_velocity_t[mask],
                    actual_velocity_y[mask, idx],
                )

        for ax in self.axes:
            ax.set_xlim(left_t, max(left_t + 1.0, current_t))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def is_open(self):
        return self.plt.fignum_exists(self.fig.number)


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Plot Revo2 retarget target, actual, error, and command velocity."
    )
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--window", type=float, default=10.0)
    parser.add_argument("--joints", default="", help="Comma-separated names or indices. Default: all.")
    parser.add_argument("--target-topic", default="")
    parser.add_argument("--actual-topic", default="")
    parser.add_argument(
        "--command-topic",
        default="",
        help=(
            "Optional Float64MultiArray velocity command topic. "
            "Leave empty for ros2_control PID mode."
        ),
    )
    parser.add_argument("--max-samples", type=int, default=4000)
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=10.0,
        help="Matplotlib refresh rate. Lower this if the plot window steals focus.",
    )
    parser.add_argument(
        "--raise-window",
        action="store_true",
        help="Allow Matplotlib to request raising the plot window.",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    rclpy.init(args=None)
    node = Revo2RetargetPlotter(args)
    view = MatplotlibView(node, args)
    refresh_period = 1.0 / max(0.5, float(args.refresh_hz))
    next_update = time.monotonic()

    try:
        while rclpy.ok() and view.is_open():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_update:
                view.update()
                next_update = now + refresh_period
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
