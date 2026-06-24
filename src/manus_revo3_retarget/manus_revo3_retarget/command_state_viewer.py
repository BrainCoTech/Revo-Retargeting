#!/usr/bin/env python3
"""Tk time-series viewer for Revo3 command and feedback topics."""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from revo3_mit_controller_msgs.msg import Revo3MITCommand
from sensor_msgs.msg import JointState


VALID_HAND_MODES = {"left", "right", "both"}
COMMAND_COLOR = "#1f77b4"
STATE_COLOR = "#2ca02c"
ERROR_BG = "#fff0f0"
NORMAL_BG = "#ffffff"
STALE_FG = "#777777"


def controller_joint_names(prefix: str) -> list[str]:
    return [
        f"{prefix}little_MPR_joint", f"{prefix}little_MCP_joint",
        f"{prefix}little_PIP_joint", f"{prefix}little_DIP_joint",
        f"{prefix}ring_MPR_joint", f"{prefix}ring_MCP_joint",
        f"{prefix}ring_PIP_joint", f"{prefix}ring_DIP_joint",
        f"{prefix}middle_MPR_joint", f"{prefix}middle_MCP_joint",
        f"{prefix}middle_PIP_joint", f"{prefix}middle_DIP_joint",
        f"{prefix}index_MPR_joint", f"{prefix}index_MCP_joint",
        f"{prefix}index_PIP_joint", f"{prefix}index_DIP_joint",
        f"{prefix}thumb_MCP_joint", f"{prefix}thumb_PIP_joint",
        f"{prefix}thumb_DIP_joint", f"{prefix}thumb_CMP_joint",
        f"{prefix}thumb_CMR_joint",
    ]


def command_topic(side: str, suffix: str, use_revo3_namespace: bool) -> str:
    suffix = suffix.strip().strip("/") or "joint_forward_mit_controller/commands"
    if use_revo3_namespace:
        return f"/revo3_{side}/{suffix}"
    return f"/{suffix}"


def state_topic(side: str, use_revo3_namespace: bool) -> str:
    if use_revo3_namespace:
        return f"/revo3_{side}/revo3_joint_state/joint_states"
    return "/revo3_joint_state/joint_states"


def _copy_numeric(values) -> list[float]:
    return [float(v) for v in values]


def _as_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _fmt(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:+.3f}"


def _age(age: float | None) -> str:
    if age is None:
        return "never"
    return f"{age:.2f}s ago"


@dataclass
class SideData:
    side: str
    joint_names: list[str]
    history_sec: float
    lock: threading.RLock = field(default_factory=threading.RLock)
    command_position: dict[str, float] = field(default_factory=dict)
    command_velocity: dict[str, float] = field(default_factory=dict)
    command_effort: dict[str, float] = field(default_factory=dict)
    command_kp: dict[str, float] = field(default_factory=dict)
    command_kd: dict[str, float] = field(default_factory=dict)
    state_position: dict[str, float] = field(default_factory=dict)
    state_velocity: dict[str, float] = field(default_factory=dict)
    state_effort: dict[str, float] = field(default_factory=dict)
    command_history: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    state_history: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    last_command_time: float | None = None
    last_state_time: float | None = None
    command_count: int = 0
    state_count: int = 0

    def __post_init__(self) -> None:
        self.command_history = {name: [] for name in self.joint_names}
        self.state_history = {name: [] for name in self.joint_names}

    def update_command(self, msg: Revo3MITCommand) -> None:
        names = list(msg.joint_names) if msg.joint_names else self.joint_names
        now = time.monotonic()
        with self.lock:
            positions = _copy_numeric(msg.position)
            self._update(self.command_position, names, positions)
            self._append(self.command_history, names, positions, now)
            self._update(self.command_velocity, names, _copy_numeric(msg.velocity))
            self._update(self.command_effort, names, _copy_numeric(msg.effort))
            self._update(self.command_kp, names, _copy_numeric(msg.kp))
            self._update(self.command_kd, names, _copy_numeric(msg.kd))
            self.last_command_time = now
            self.command_count += 1
            self._trim(now)

    def update_state(self, msg: JointState) -> None:
        names = list(msg.name)
        now = time.monotonic()
        with self.lock:
            positions = _copy_numeric(msg.position)
            self._update(self.state_position, names, positions)
            self._append(self.state_history, names, positions, now)
            self._update(self.state_velocity, names, _copy_numeric(msg.velocity))
            self._update(self.state_effort, names, _copy_numeric(msg.effort))
            self.last_state_time = now
            self.state_count += 1
            self._trim(now)

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "joint_names": list(self.joint_names),
                "command_position": dict(self.command_position),
                "command_velocity": dict(self.command_velocity),
                "command_effort": dict(self.command_effort),
                "command_kp": dict(self.command_kp),
                "command_kd": dict(self.command_kd),
                "state_position": dict(self.state_position),
                "state_velocity": dict(self.state_velocity),
                "state_effort": dict(self.state_effort),
                "command_history": {k: list(v) for k, v in self.command_history.items()},
                "state_history": {k: list(v) for k, v in self.state_history.items()},
                "last_command_time": self.last_command_time,
                "last_state_time": self.last_state_time,
                "command_count": self.command_count,
                "state_count": self.state_count,
            }

    @staticmethod
    def _update(target: dict[str, float], names: list[str], values: list[float]) -> None:
        if not values:
            return
        for name, value in zip(names, values):
            target[str(name)] = float(value)

    @staticmethod
    def _append(
        target: dict[str, list[tuple[float, float]]],
        names: list[str],
        values: list[float],
        now: float,
    ) -> None:
        if not values:
            return
        for name, value in zip(names, values):
            target.setdefault(str(name), []).append((now, float(value)))

    def _trim(self, now: float) -> None:
        cutoff = now - max(1.0, self.history_sec)
        for history in (self.command_history, self.state_history):
            for name, samples in history.items():
                while samples and samples[0][0] < cutoff:
                    samples.pop(0)


class CommandStateNode(Node):
    def __init__(self, sides: list[str], args: argparse.Namespace):
        super().__init__("revo3_command_state_viewer")
        self.sides = sides
        self.data: dict[str, SideData] = {
            side: SideData(
                side=side,
                joint_names=controller_joint_names(f"{side}_"),
                history_sec=args.history_sec,
            )
            for side in sides
        }

        for side in sides:
            cmd_topic = getattr(args, f"{side}_command_topic")
            st_topic = getattr(args, f"{side}_state_topic")
            if not cmd_topic:
                cmd_topic = command_topic(side, args.command_topic_suffix, args.use_revo3_namespace)
            if not st_topic:
                st_topic = state_topic(side, args.use_revo3_namespace)

            self.create_subscription(
                Revo3MITCommand,
                cmd_topic,
                lambda msg, side=side: self.data[side].update_command(msg),
                10,
            )
            self.create_subscription(
                JointState,
                st_topic,
                lambda msg, side=side: self.data[side].update_state(msg),
                10,
            )
            self.get_logger().info(
                f"Watching {side} command={cmd_topic} state={st_topic}"
            )


class JointPlot:
    def __init__(self, parent: tk.Widget, joint_name: str, plot_height: int):
        self.joint_name = joint_name
        self.frame = ttk.Frame(parent, padding=(4, 3))
        self.canvas = tk.Canvas(
            self.frame,
            height=plot_height,
            background=NORMAL_BG,
            highlightthickness=1,
            highlightbackground="#d6d6d6",
        )
        self.canvas.pack(fill="x", expand=True)

    def draw(
        self,
        now: float,
        history_sec: float,
        command_samples: list[tuple[float, float]],
        state_samples: list[tuple[float, float]],
        command_position: float | None,
        state_position: float | None,
        warn_error_rad: float,
        stale: bool,
    ) -> None:
        width = max(480, int(self.canvas.winfo_width()))
        height = max(70, int(self.canvas.winfo_height()))
        self.canvas.delete("all")

        error = None if command_position is None or state_position is None else state_position - command_position
        warn = error is not None and math.isfinite(error) and abs(error) >= warn_error_rad
        bg = ERROR_BG if warn and not stale else NORMAL_BG
        self.canvas.configure(background=bg)

        left = 48
        right = 10
        top = 12
        bottom = 20
        plot_w = width - left - right
        plot_h = height - top - bottom
        x_min = now - history_sec
        x_max = now

        all_values = [
            value
            for _, value in command_samples + state_samples
            if math.isfinite(value)
        ]
        if command_position is not None and math.isfinite(command_position):
            all_values.append(command_position)
        if state_position is not None and math.isfinite(state_position):
            all_values.append(state_position)
        if all_values:
            y_min = min(all_values)
            y_max = max(all_values)
            if y_max - y_min < 0.02:
                center = 0.5 * (y_min + y_max)
                y_min = center - 0.01
                y_max = center + 0.01
            pad = max(0.02, 0.08 * (y_max - y_min))
            y_min -= pad
            y_max += pad
        else:
            y_min, y_max = -1.0, 1.0

        self._grid(left, top, plot_w, plot_h, width, height, y_min, y_max, stale)
        self._polyline(command_samples, x_min, x_max, y_min, y_max, left, top, plot_w, plot_h, COMMAND_COLOR)
        self._polyline(state_samples, x_min, x_max, y_min, y_max, left, top, plot_w, plot_h, STATE_COLOR)

        fg = STALE_FG if stale else "#202020"
        short_name = self.joint_name.split("_", 1)[1] if "_" in self.joint_name else self.joint_name
        self.canvas.create_text(6, 6, anchor="nw", text=short_name, fill=fg)
        self.canvas.create_text(
            width - 12,
            6,
            anchor="ne",
            text=f"cmd {_fmt(command_position)}   state {_fmt(state_position)}   err {_fmt(error)}",
            fill=fg,
        )

    def _grid(
        self,
        left: int,
        top: int,
        plot_w: int,
        plot_h: int,
        width: int,
        height: int,
        y_min: float,
        y_max: float,
        stale: bool,
    ) -> None:
        fg = "#dcdcdc"
        axis = "#a8a8a8"
        label = STALE_FG if stale else "#606060"
        for i in range(5):
            x = left + int(plot_w * i / 4)
            self.canvas.create_line(x, top, x, top + plot_h, fill=fg)
        for i in range(3):
            y = top + int(plot_h * i / 2)
            self.canvas.create_line(left, y, width - 10, y, fill=fg)
        self.canvas.create_rectangle(left, top, width - 10, height - 20, outline=axis)
        self.canvas.create_text(left - 4, top, anchor="ne", text=f"{y_max:+.2f}", fill=label)
        self.canvas.create_text(left - 4, top + plot_h, anchor="ne", text=f"{y_min:+.2f}", fill=label)
        self.canvas.create_text(left, height - 4, anchor="sw", text="old", fill=label)
        self.canvas.create_text(width - 10, height - 4, anchor="se", text="now", fill=label)

    def _polyline(
        self,
        samples: list[tuple[float, float]],
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        left: int,
        top: int,
        plot_w: int,
        plot_h: int,
        color: str,
    ) -> None:
        points: list[float] = []
        for stamp, value in samples:
            if stamp < x_min or stamp > x_max or not math.isfinite(value):
                continue
            x = left + plot_w * (stamp - x_min) / max(0.001, x_max - x_min)
            y = top + plot_h * (1.0 - (value - y_min) / max(0.001, y_max - y_min))
            points.extend([x, y])
        if len(points) >= 4:
            self.canvas.create_line(*points, fill=color, width=2, smooth=False)
        elif len(points) == 2:
            x, y = points
            self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)


class SidePlotPanel:
    def __init__(
        self,
        parent: tk.Widget,
        side: str,
        joint_names: list[str],
        history_sec: float,
        stale_sec: float,
        warn_error_rad: float,
        plot_height: int,
    ):
        self.side = side
        self.history_sec = history_sec
        self.stale_sec = stale_sec
        self.warn_error_rad = warn_error_rad
        self.frame = ttk.Frame(parent, padding=(8, 6))

        self.status_var = tk.StringVar(value=f"{side}: waiting for topics")
        ttk.Label(self.frame, textvariable=self.status_var).pack(anchor="w")
        legend = ttk.Frame(self.frame)
        legend.pack(anchor="w", pady=(2, 4))
        ttk.Label(legend, text="command position", foreground=COMMAND_COLOR).pack(side="left")
        ttk.Label(legend, text="   state position", foreground=STATE_COLOR).pack(side="left")

        outer = ttk.Frame(self.frame)
        outer.pack(fill="both", expand=True)
        self.scroll_canvas = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=self.scroll_canvas.yview)
        self.scroll_canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.scroll_canvas.pack(side="left", fill="both", expand=True)

        self.inner = ttk.Frame(self.scroll_canvas)
        self.window_id = self.scroll_canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.inner.bind("<Configure>", self._on_inner_configure)
        self.scroll_canvas.bind("<Configure>", self._on_canvas_configure)
        self.scroll_canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        self.plots: dict[str, JointPlot] = {}
        for joint_name in joint_names:
            plot = JointPlot(self.inner, joint_name, plot_height=plot_height)
            plot.frame.pack(fill="x", expand=True)
            self.plots[joint_name] = plot

    def update(self, snapshot: dict[str, object]) -> None:
        now = time.monotonic()
        last_command = snapshot["last_command_time"]
        last_state = snapshot["last_state_time"]
        command_age = None if last_command is None else now - float(last_command)
        state_age = None if last_state is None else now - float(last_state)
        stale = (
            command_age is None or state_age is None
            or command_age > self.stale_sec or state_age > self.stale_sec
        )
        self.status_var.set(
            f"{self.side}: cmd {_age(command_age)} ({snapshot['command_count']} msg), "
            f"state {_age(state_age)} ({snapshot['state_count']} msg), "
            f"window {self.history_sec:.1f}s"
        )

        cmd_pos = snapshot["command_position"]
        state_pos = snapshot["state_position"]
        cmd_history = snapshot["command_history"]
        state_history = snapshot["state_history"]

        for joint in snapshot["joint_names"]:
            self.plots[joint].draw(
                now=now,
                history_sec=self.history_sec,
                command_samples=cmd_history.get(joint, []),
                state_samples=state_history.get(joint, []),
                command_position=cmd_pos.get(joint),
                state_position=state_pos.get(joint),
                warn_error_rad=self.warn_error_rad,
                stale=stale,
            )

    def _on_inner_configure(self, _event) -> None:
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def _on_canvas_configure(self, event) -> None:
        self.scroll_canvas.itemconfigure(self.window_id, width=event.width)

    def _on_mousewheel(self, event) -> None:
        if self.frame.winfo_ismapped():
            self.scroll_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")


class CommandStateApp:
    def __init__(self, node: CommandStateNode, args: argparse.Namespace):
        self.node = node
        self.args = args
        self.root = tk.Tk()
        self.root.title("Revo3 command/state time-series viewer")
        self.root.geometry("1200x820")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        title = ttk.Label(
            self.root,
            text="Revo3 command/state time-series viewer",
            font=("TkDefaultFont", 13, "bold"),
            padding=(8, 8, 8, 2),
        )
        title.pack(anchor="w")

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True)
        self.panels: dict[str, SidePlotPanel] = {}
        for side in self.node.sides:
            panel = SidePlotPanel(
                notebook,
                side=side,
                joint_names=self.node.data[side].joint_names,
                history_sec=args.history_sec,
                stale_sec=args.stale_sec,
                warn_error_rad=args.warn_error_rad,
                plot_height=args.plot_height,
            )
            notebook.add(panel.frame, text=side)
            self.panels[side] = panel

        footer = ttk.Label(
            self.root,
            text="Blue: command position. Green: state position. Red background means |state-cmd| exceeds the warning threshold.",
            padding=(8, 2, 8, 8),
        )
        footer.pack(anchor="w")

    def run(self) -> None:
        self._refresh()
        self.root.mainloop()

    def close(self) -> None:
        self.root.quit()

    def _refresh(self) -> None:
        for side, panel in self.panels.items():
            panel.update(self.node.data[side].snapshot())
        self.root.after(max(50, int(self.args.update_ms)), self._refresh)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand-mode", default="both", choices=sorted(VALID_HAND_MODES))
    parser.add_argument("--use-revo3-namespace", type=_as_bool, default=True)
    parser.add_argument("--command-topic-suffix", default="joint_forward_mit_controller/commands")
    parser.add_argument("--left-command-topic", default="")
    parser.add_argument("--right-command-topic", default="")
    parser.add_argument("--left-state-topic", default="")
    parser.add_argument("--right-state-topic", default="")
    parser.add_argument("--update-ms", type=int, default=100)
    parser.add_argument("--history-sec", type=float, default=10.0)
    parser.add_argument("--stale-sec", type=float, default=1.0)
    parser.add_argument("--warn-error-rad", type=float, default=0.10)
    parser.add_argument("--plot-height", type=int, default=92)
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args(remove_ros_args(sys.argv)[1:])
    sides = ["left", "right"] if args.hand_mode == "both" else [args.hand_mode]

    rclpy.init(args=sys.argv)
    node = CommandStateNode(sides, args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        app = CommandStateApp(node, args)
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
