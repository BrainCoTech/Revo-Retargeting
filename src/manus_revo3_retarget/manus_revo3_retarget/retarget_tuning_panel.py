#!/usr/bin/env python3
from __future__ import annotations

import argparse
import getpass
import json
import os
import re
import shutil
import signal
import sys
import tempfile
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import rclpy
import yaml
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool


OPERATOR_ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")
CALIB_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_-]*$")  # 标定名，允许为空
DEFAULT_CALIBRATION_DIR = Path.home() / ".brainco" / "retarget_calibration"
DEFAULT_ROBOT_CONFIG = Path.home() / ".brainco" / "robot_config" / "robot_config.json"

# 标定库子目录名与 latest 指针文件名。标定库结构：
#   <retarget_calibration_dir>/library/<side>/<operator>_<name>_<ts>.yaml
#   <retarget_calibration_dir>/library/<side>/latest.json
# latest.json 只记录**相对该 side 目录的文件名**（便于整目录搬迁），不存绝对路径。
LIBRARY_SUBDIR = "library"
LATEST_POINTER_NAME = "latest.json"
LATEST_POINTER_SCHEMA = 1


@dataclass(frozen=True)
class ParamSpec:
    key: str
    label: str
    minimum: float
    maximum: float
    step: float
    kind: str = "float"
    digits: int = 3

    def name(self, side: str) -> str:
        return self.key.format(side=side)


SPREAD_SPECS = []

for finger in ("index", "middle", "ring", "little"):
    label_finger = {"index": "Index", "middle": "Middle", "ring": "Ring", "little": "Little"}[finger]
    SPREAD_SPECS.extend(
        (
            ParamSpec(
                f"physical_{{side}}_{finger}_MPR_joint_scale",
                f"{label_finger} MPR scale",
                -2.5,
                2.5,
                0.05,
            ),
            ParamSpec(
                f"physical_{{side}}_{finger}_MPR_joint_offset_deg",
                f"{label_finger} MPR offset deg",
                -40.0,
                40.0,
                0.5,
                digits=2,
            ),
        )
    )

FOUR_FINGER_SPECS = []

for finger in ("index", "middle", "ring", "little"):
    for joint in ("MCP", "PIP", "DIP"):
        label_finger = {"index": "Index", "middle": "Middle", "ring": "Ring", "little": "Little"}[finger]
        FOUR_FINGER_SPECS.append(
            ParamSpec(
                f"physical_{{side}}_{finger}_{joint}_joint_scale",
                f"{label_finger} {joint} scale",
                0.0,
                2.5,
                0.05,
            )
        )
        FOUR_FINGER_SPECS.append(
            ParamSpec(
                f"physical_{{side}}_{finger}_{joint}_joint_offset_deg",
                f"{label_finger} {joint} offset deg",
                -30.0,
                30.0,
                0.5,
                digits=2,
            )
        )

THUMB_SPECS = [
    ParamSpec("thumb_ik_posture_weight", "IK posture weight", 0.0, 2.0, 0.05),
    ParamSpec("thumb_ik_smooth_weight", "IK smooth weight", 0.0, 2.0, 0.05),
    ParamSpec("thumb_ik_max_iterations", "IK max iterations", 0.0, 60.0, 1.0, kind="int", digits=0),
    ParamSpec("thumb_ik_max_step_deg", "IK max step deg", 0.0, 20.0, 0.5, digits=2),
    ParamSpec("thumb_ik_max_frame_delta_deg", "IK max frame delta deg", 0.0, 30.0, 0.5, digits=2),
    ParamSpec("thumb_ik_damping", "IK damping", 0.0, 0.2, 0.005, digits=4),
    ParamSpec("thumb_ik_step_size", "IK step size", 0.0, 1.0, 0.01, digits=3),
    ParamSpec("thumb_ik_tolerance", "IK tolerance", 0.0, 0.01, 0.0001, digits=5),
    ParamSpec("legacy_{side}_physical_thumb_pip_ik_scale", "PIP IK task weight", 0.0, 3.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_dip_ik_scale", "DIP IK task weight", 0.0, 3.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_reach_scale", "Thumb reach geometry gain", 0.2, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_ik_position_scale", "IK position geometry gain", 0.2, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_ema_prev", "Thumb EMA previous weight", 0.0, 1.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_ema_cur", "Thumb EMA current weight", 0.0, 1.0, 0.05),
]

for joint in ("CMP", "CMR", "MCP", "PIP", "DIP"):
    THUMB_SPECS.extend(
        (
            ParamSpec(f"physical_{{side}}_thumb_{joint}_joint_scale", f"{joint} scale", 0.0, 2.5, 0.05),
            ParamSpec(
                f"physical_{{side}}_thumb_{joint}_joint_offset_deg",
                f"{joint} offset deg",
                -30.0,
                30.0,
                0.5,
                digits=2,
            ),
        )
    )

BOOL_PARAMS = [
    ("legacy_{side}_physical_middle_spread_dynamic", "Middle spread dynamic"),
]

STAGE_TO_MODE = {
    "spread": "spread_only",
    "four_finger": "four_finger_only",
    "thumb": "full",
}

STAGE_DESCRIPTIONS = {
    "spread": "只输出四个 MPR；四指弯曲关节和大拇指强制为 0°。",
    "four_finger": "只输出四指 MCP/PIP/DIP；MPR 和大拇指强制为 0°。",
    "thumb": "全部关节输出；保留已完成的 MPR 和四指参数，用于对指调节。",
}


class ParameterClient(Node):
    def __init__(self):
        super().__init__("retarget_tuning_panel")
        self.target_node = ""
        self._get_client = None
        self._set_client = None
        self._enable_client = None

    @staticmethod
    def _normalize_target(target_node: str) -> str:
        target = target_node.strip().rstrip("/")
        if not target:
            raise ValueError("target node cannot be empty")
        return target if target.startswith("/") else f"/{target}"

    @staticmethod
    def _fallback_target(target: str) -> str | None:
        # 单侧启动（hand_mode:=left/right）时节点被 remap 成不带侧后缀的
        # /manus_revo3_retarget；only both 模式才是 /manus_revo3_retarget_{side}。
        # 面板默认按 both 命名去连，单侧启动就连不上，回退到无后缀节点名。
        for suffix in ("_left", "_right"):
            if target.endswith(suffix):
                stripped = target[: -len(suffix)]
                # 剥离后须仍是合法节点名（非空、非纯 "/"），否则不回退。
                return stripped if stripped.strip("/") else None
        return None

    def _build_clients(self, target: str) -> None:
        for client in (self._get_client, self._set_client, self._enable_client):
            if client is not None:
                self.destroy_client(client)
        self.target_node = target
        self._get_client = self.create_client(GetParameters, f"{self.target_node}/get_parameters")
        self._set_client = self.create_client(SetParameters, f"{self.target_node}/set_parameters")
        self._enable_client = self.create_client(SetBool, f"{self.target_node}/enable_retarget")

    def set_target(self, target_node: str) -> None:
        target = self._normalize_target(target_node)
        if target == self.target_node and self._get_client is not None:
            return
        self._build_clients(target)

    def _clients_ready(self) -> bool:
        return (
            self._get_client is not None
            and self._set_client is not None
            and self._enable_client is not None
            and self._get_client.service_is_ready()
            and self._set_client.service_is_ready()
            and self._enable_client.service_is_ready()
        )

    def wait_ready(self, timeout_s: float = 2.0) -> None:
        if self._get_client is None or self._set_client is None or self._enable_client is None:
            raise RuntimeError("target node is not configured")
        fallback = self._fallback_target(self.target_node)
        fallback_tried = False
        deadline = time.monotonic() + timeout_s
        while rclpy.ok():
            if self._clients_ready():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                # 带侧后缀连不上（多为单侧启动），改连无后缀节点重试一次。
                if fallback is not None and not fallback_tried:
                    fallback_tried = True
                    self._build_clients(fallback)
                    deadline = time.monotonic() + timeout_s
                    continue
                raise TimeoutError(f"Timed out waiting for {self.target_node} parameter services")
            self._get_client.wait_for_service(timeout_sec=min(0.2, remaining))
            self._set_client.wait_for_service(timeout_sec=min(0.2, max(deadline - time.monotonic(), 0.0)))
            self._enable_client.wait_for_service(timeout_sec=min(0.2, max(deadline - time.monotonic(), 0.0)))

    def _call(self, future, timeout_s: float = 2.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise TimeoutError(f"Timed out calling {self.target_node}")
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(str(exc)) from exc
        return future.result()

    def get_values(self, names: list[str]) -> dict[str, object]:
        if self._get_client is None:
            raise RuntimeError("target node is not configured")
        request = GetParameters.Request()
        request.names = names
        response = self._call(self._get_client.call_async(request))
        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
        }

    def set_values(self, values: dict[str, object]) -> None:
        if self._set_client is None:
            raise RuntimeError("target node is not configured")
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]
        response = self._call(self._set_client.call_async(request), timeout_s=4.0)
        failures = [result.reason or "unknown reason" for result in response.results if not result.successful]
        if failures:
            raise RuntimeError("; ".join(failures))

    def set_enabled(self, enabled: bool) -> None:
        if self._enable_client is None:
            raise RuntimeError("target node is not configured")
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self._call(self._enable_client.call_async(request), timeout_s=3.0)
        if not response.success:
            raise RuntimeError(response.message or "enable_retarget failed")

    def set_enabled_for(self, target_node: str, enabled: bool, timeout_s: float = 1.0) -> None:
        target = self._normalize_target(target_node)
        client = self.create_client(SetBool, f"{target}/enable_retarget")
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise TimeoutError(f"Timed out waiting for {target}/enable_retarget")
            request = SetBool.Request()
            request.data = bool(enabled)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            if not future.done():
                raise TimeoutError(f"Timed out calling {target}/enable_retarget")
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(response.message if response is not None else "enable_retarget failed")
        finally:
            self.destroy_client(client)


class SliderRow:
    def __init__(self, parent: ttk.Frame, spec: ParamSpec, side_getter, apply_callback, row: int):
        self.spec = spec
        self.side_getter = side_getter
        self.apply_callback = apply_callback
        self.var = tk.DoubleVar(value=0.0)
        self.entry_var = tk.StringVar(value="0")

        self.label = ttk.Label(parent, text=spec.label)
        self.label.grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)
        self.scale = ttk.Scale(
            parent,
            from_=spec.minimum,
            to=spec.maximum,
            variable=self.var,
            command=self._on_slide,
        )
        self.scale.grid(row=row, column=1, sticky="ew", pady=2)
        self.entry = ttk.Entry(parent, textvariable=self.entry_var, width=8)
        self.entry.grid(row=row, column=2, sticky="ew", padx=(6, 0), pady=2)
        self.entry.bind("<Return>", lambda _event: self._entry_to_var(apply_now=True))
        self.scale.bind("<ButtonRelease-1>", lambda _event: self.apply_callback())
        # +/- 按钮：按 spec.step 精确增减一步，比拖滑块准。点一下即应用到节点。
        self.btn_minus = ttk.Button(parent, text="−", width=2,
                                    command=lambda: self._nudge(-1))
        self.btn_minus.grid(row=row, column=3, padx=(4, 0), pady=2)
        self.btn_plus = ttk.Button(parent, text="+", width=2,
                                   command=lambda: self._nudge(1))
        self.btn_plus.grid(row=row, column=4, padx=(2, 0), pady=2)

    @property
    def name(self) -> str:
        return self.spec.name(self.side_getter())

    def _format(self, value: float) -> str:
        if self.spec.kind == "int":
            return str(int(round(value)))
        return f"{float(value):.{self.spec.digits}f}"

    def _on_slide(self, _value: str) -> None:
        value = self._quantize(self.var.get())
        self.entry_var.set(self._format(value))

    def _quantize(self, value: float) -> float:
        step = max(float(self.spec.step), 1e-9)
        value = round((float(value) - self.spec.minimum) / step) * step + self.spec.minimum
        value = min(self.spec.maximum, max(self.spec.minimum, value))
        return int(round(value)) if self.spec.kind == "int" else value

    def _entry_to_var(self, apply_now: bool = False) -> None:
        try:
            value = self._quantize(float(self.entry_var.get()))
        except ValueError:
            self.entry_var.set(self._format(self.var.get()))
            return
        self.var.set(value)
        self.entry_var.set(self._format(value))
        if apply_now:
            self.apply_callback()

    def _nudge(self, direction: int) -> None:
        """按 step 增减一步（direction=+1/-1），量化、更新显示并应用到节点。"""
        step = max(float(self.spec.step), 1e-9)
        value = self._quantize(self.var.get() + direction * step)
        self.var.set(value)
        self.entry_var.set(self._format(value))
        self.apply_callback()

    def get_value(self) -> object:
        self._entry_to_var()
        value = self._quantize(self.var.get())
        return int(round(value)) if self.spec.kind == "int" else float(value)

    def set_value(self, value: object) -> None:
        numeric = self._quantize(float(value))
        self.var.set(numeric)
        self.entry_var.set(self._format(numeric))

    def set_enabled(self, enabled: bool) -> None:
        state = "!disabled" if enabled else "disabled"
        self.scale.state((state,))
        self.entry.state((state,))
        self.btn_minus.state((state,))
        self.btn_plus.state((state,))


class RetargetTuningPanel:
    def __init__(
        self,
        root: tk.Tk,
        client: ParameterClient,
        node_by_side: dict[str, str],
        default_side: str,
        operator: str,
        calibration_dir: Path,
    ):
        self.root = root
        self.client = client
        self.node_by_side = dict(node_by_side)
        self.rows: list[SliderRow] = []
        self.section_rows: dict[str, list[SliderRow]] = {}
        self.bool_vars: dict[str, tk.BooleanVar] = {}
        self.bool_widgets: dict[str, ttk.Checkbutton] = {}
        self.initial_values: dict[str, dict[str, object]] = {}
        self.output_enabled = False

        root.title("Revo3 分阶段重定向标定")
        root.geometry("1380x900")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.side_var = tk.StringVar(value=default_side)
        self.hand_text_var = tk.StringVar()
        self.node_text_var = tk.StringVar()
        self.stage_var = tk.StringVar(value="spread")
        self.stage_description_var = tk.StringVar()
        self.status_var = tk.StringVar(value="")
        self.operator_var = tk.StringVar(value=operator)
        self.calibration_dir_var = tk.StringVar(value=str(calibration_dir))
        self._calib_name_var = tk.StringVar(value="")
        self._history_var = tk.StringVar(value="")

        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="当前标定手:").pack(side="left")
        ttk.Label(toolbar, textvariable=self.hand_text_var, font=("TkDefaultFont", 12, "bold")).pack(
            side="left", padx=(6, 12)
        )
        self.switch_button = ttk.Button(toolbar, command=self.switch_hand)
        self.switch_button.pack(side="left", padx=3)
        ttk.Label(toolbar, textvariable=self.node_text_var).pack(side="left", padx=(10, 12))
        ttk.Button(toolbar, text="刷新", command=self.refresh).pack(side="left", padx=3)
        ttk.Button(toolbar, text="应用当前阶段", command=self.apply_current_stage).pack(side="left", padx=3)
        ttk.Button(toolbar, text="恢复连接时参数", command=self.reset_initial).pack(side="left", padx=3)
        ttk.Button(toolbar, text="导出 Override YAML", command=self.export_yaml).pack(side="left", padx=3)
        self.enable_button = ttk.Button(toolbar, command=self.toggle_output)
        self.enable_button.pack(side="left", padx=(12, 3))
        ttk.Button(toolbar, text="停止全部输出", command=self.stop_all_output).pack(side="left", padx=3)
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        export_frame = ttk.Frame(root, padding=(8, 0, 8, 8))
        export_frame.pack(fill="x")
        ttk.Label(export_frame, text="遥操员 ID:").pack(side="left")
        ttk.Entry(export_frame, textvariable=self.operator_var, width=18).pack(side="left", padx=(6, 14))
        ttk.Label(export_frame, text="标定目录:").pack(side="left")
        ttk.Entry(export_frame, textvariable=self.calibration_dir_var, width=72).pack(
            side="left", fill="x", expand=True, padx=(6, 6)
        )
        ttk.Button(export_frame, text="选择目录", command=self.choose_calibration_dir).pack(side="left")

        # 标定库：命名保存 + 历史选择（本侧）。导出会存到 library/<side>/ 并更新 latest 指针。
        library_frame = ttk.Frame(root, padding=(8, 0, 8, 8))
        library_frame.pack(fill="x")
        ttk.Label(library_frame, text="标定名(可选):").pack(side="left")
        ttk.Entry(library_frame, textvariable=self._calib_name_var, width=18).pack(side="left", padx=(6, 14))
        ttk.Label(library_frame, text="历史标定:").pack(side="left")
        self._history_combo = ttk.Combobox(
            library_frame, textvariable=self._history_var, width=48, state="readonly"
        )
        self._history_combo.pack(side="left", fill="x", expand=True, padx=(6, 6))
        ttk.Button(library_frame, text="设为最新", command=self._set_history_latest).pack(side="left", padx=3)
        ttk.Button(library_frame, text="刷新列表", command=self._refresh_history).pack(side="left")

        stage_frame = ttk.LabelFrame(root, text="标定阶段", padding=8)
        stage_frame.pack(fill="x", padx=8, pady=(0, 8))
        for text, value in (
            ("1. Spread / MPR", "spread"),
            ("2. 四指弯曲", "four_finger"),
            ("3. 大拇指 / 对指", "thumb"),
        ):
            ttk.Radiobutton(
                stage_frame,
                text=text,
                value=value,
                variable=self.stage_var,
                command=self._on_stage_changed,
            ).pack(side="left", padx=(0, 18))
        ttk.Label(stage_frame, textvariable=self.stage_description_var).pack(side="left", padx=(12, 0))

        bool_frame = ttk.Frame(root, padding=(8, 0, 8, 6))
        bool_frame.pack(fill="x")
        for key, label in BOOL_PARAMS:
            var = tk.BooleanVar(value=False)
            widget = ttk.Checkbutton(
                bool_frame,
                text=label,
                variable=var,
                command=lambda: self.apply_stage("spread"),
            )
            widget.pack(side="left", padx=(0, 14))
            self.bool_vars[key] = var
            self.bool_widgets[key] = widget

        panes = ttk.PanedWindow(root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._add_section(panes, "spread", "Spread / MPR", SPREAD_SPECS)
        self._add_section(panes, "four_finger", "四指弯曲", FOUR_FINGER_SPECS)
        self._add_section(panes, "thumb", "大拇指 / Pinch", THUMB_SPECS)

        self._update_hand_ui()
        self._update_stage_widgets()
        for node in self.node_by_side.values():
            self._disable_node_best_effort(node)
        self._refresh_history()
        self.refresh()

    def _add_section(
        self,
        panes: ttk.PanedWindow,
        stage: str,
        title: str,
        specs: list[ParamSpec],
    ) -> None:
        outer = ttk.LabelFrame(panes, text=title, padding=6)
        panes.add(outer, weight=1)
        canvas = tk.Canvas(outer, highlightthickness=0)
        scroll = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        body = ttk.Frame(canvas)
        body.columnconfigure(1, weight=1)
        body.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=body, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        stage_rows = []
        for row_index, spec in enumerate(specs):
            row = SliderRow(body, spec, self.side_var.get, lambda s=stage: self.apply_stage(s), row_index)
            self.rows.append(row)
            stage_rows.append(row)
        self.section_rows[stage] = stage_rows

    @property
    def current_side(self) -> str:
        return self.side_var.get()

    @property
    def current_node(self) -> str:
        return self.node_by_side[self.current_side]

    def _all_names(self) -> list[str]:
        side = self.current_side
        names = [row.spec.name(side) for row in self.rows]
        names.extend(key.format(side=side) for key in self.bool_vars)
        return names

    def _current_values(self) -> dict[str, object]:
        values = {row.name: row.get_value() for row in self.rows}
        side = self.current_side
        for key, var in self.bool_vars.items():
            values[key.format(side=side)] = bool(var.get())
        return values

    def _stage_values(self, stage: str) -> dict[str, object]:
        values = {row.name: row.get_value() for row in self.section_rows[stage]}
        if stage == "spread":
            for key, var in self.bool_vars.items():
                values[key.format(side=self.current_side)] = bool(var.get())
        return values

    def _load_current(self) -> None:
        self.client.set_target(self.current_node)
        self.client.wait_ready()
        # wait_ready 可能已回退到无后缀节点（单侧启动）；把实际连上的节点名
        # 同步回 node_by_side，后续 switch_hand / 失能等操作才不会再连错名。
        actual = self.client.target_node
        if actual != self.current_node:
            self.node_by_side[self.current_side] = actual
            self.node_text_var.set(actual)
        values = self.client.get_values(self._all_names())
        for row in self.rows:
            value = values.get(row.name)
            if value is not None:
                row.set_value(value)
        for key, var in self.bool_vars.items():
            var.set(bool(values.get(key.format(side=self.current_side), False)))
        self.initial_values.setdefault(self.current_side, dict(values))
        self.client.set_values({"retarget_calibration_mode": STAGE_TO_MODE[self.stage_var.get()]})

    def refresh(self) -> None:
        try:
            self._load_current()
            self._status(f"已连接 {self.client.target_node}")
        except Exception as exc:
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("连接失败")

    def apply_stage(self, stage: str) -> None:
        if stage != self.stage_var.get():
            return
        try:
            self.client.set_values(self._stage_values(stage))
            self._status(f"已应用 {stage}")
        except Exception as exc:
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("应用失败")

    def apply_current_stage(self) -> None:
        self.apply_stage(self.stage_var.get())

    def _on_stage_changed(self) -> None:
        self._update_stage_widgets()
        was_enabled = self.output_enabled
        try:
            if was_enabled:
                self.client.set_enabled(False)
                self.output_enabled = False
            self.client.set_values({"retarget_calibration_mode": STAGE_TO_MODE[self.stage_var.get()]})
            if was_enabled:
                self.client.set_enabled(True)
                self.output_enabled = True
            self._update_enable_button()
            self._status(f"已切换到 {self.stage_var.get()} 阶段")
        except Exception as exc:
            self._update_enable_button()
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("阶段切换失败，输出保持关闭")

    def _update_stage_widgets(self) -> None:
        active = self.stage_var.get()
        self.stage_description_var.set(STAGE_DESCRIPTIONS[active])
        for stage, rows in self.section_rows.items():
            for row in rows:
                row.set_enabled(stage == active)
        bool_state = "!disabled" if active == "spread" else "disabled"
        for widget in self.bool_widgets.values():
            widget.state((bool_state,))

    def toggle_output(self) -> None:
        try:
            if self.output_enabled:
                self.client.set_enabled(False)
                self.output_enabled = False
                self._status(f"{self.current_side} 输出已关闭")
            else:
                other = "left" if self.current_side == "right" else "right"
                self._disable_node_best_effort(self.node_by_side[other])
                self.client.set_values({"retarget_calibration_mode": STAGE_TO_MODE[self.stage_var.get()]})
                self.client.set_enabled(True)
                self.output_enabled = True
                self._status(f"仅启用 {self.current_side} 输出")
            self._update_enable_button()
        except Exception as exc:
            self.output_enabled = False
            self._update_enable_button()
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("使能操作失败")

    def stop_all_output(self) -> None:
        for node in self.node_by_side.values():
            self._disable_node_best_effort(node)
        self.output_enabled = False
        self._update_enable_button()
        self._status("左右手输出均已关闭")

    def switch_hand(self) -> None:
        old_side = self.current_side
        new_side = "left" if old_side == "right" else "right"
        was_enabled = self.output_enabled
        try:
            if was_enabled:
                self.client.set_enabled(False)
            self.output_enabled = False
            self._disable_node_best_effort(self.node_by_side[new_side])
            self.side_var.set(new_side)
            self._update_hand_ui()
            self._load_current()
            self._refresh_history()
            if was_enabled:
                self.client.set_enabled(True)
                self.output_enabled = True
            self._update_enable_button()
            self._status(f"已切换到 {new_side}")
        except Exception as exc:
            self.side_var.set(old_side)
            self._update_hand_ui()
            try:
                self.client.set_target(self.node_by_side[old_side])
                self.client.wait_ready()
                if was_enabled:
                    self.client.set_enabled(True)
                    self.output_enabled = True
            except Exception:
                self.output_enabled = False
            self._update_enable_button()
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("切手失败")

    def reset_initial(self) -> None:
        values = self.initial_values.get(self.current_side)
        if not values:
            return
        was_enabled = self.output_enabled
        try:
            if was_enabled:
                self.client.set_enabled(False)
                self.output_enabled = False
            self.client.set_values(values)
            self._load_current()
            if was_enabled:
                self.client.set_enabled(True)
                self.output_enabled = True
            self._update_enable_button()
            self._status("已恢复连接时参数")
        except Exception as exc:
            self._update_enable_button()
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("恢复失败")

    def export_yaml(self) -> None:
        try:
            values = self._current_values()
            self.client.set_values(values)
            values = {
                key: value
                for key, value in self.client.get_values(self._all_names()).items()
                if value is not None
            }
        except Exception as exc:
            messagebox.showerror("Revo3 标定", str(exc))
            self._status("导出前读取参数失败")
            return
        operator = self.operator_var.get().strip()
        if not OPERATOR_ID_PATTERN.fullmatch(operator):
            messagebox.showerror("Revo3 标定", "遥操员 ID 只允许字母、数字、下划线和横线。")
            self._status("遥操员 ID 无效")
            return
        calibration_dir = Path(self.calibration_dir_var.get().strip()).expanduser()
        if not calibration_dir.is_absolute():
            messagebox.showerror("Revo3 标定", "标定目录必须是绝对路径（也支持 ~/...）。")
            self._status("标定目录无效")
            return
        try:
            calibration_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("Revo3 标定", f"无法创建标定目录：{exc}")
            self._status("创建标定目录失败")
            return

        calib_name = self._calib_name_var.get().strip()
        if not CALIB_NAME_PATTERN.fullmatch(calib_name):
            messagebox.showerror("Revo3 标定", "标定名只允许字母、数字、下划线和横线（可留空）。")
            self._status("标定名无效")
            return

        side = self.current_side
        data = {"manus_revo3_retarget": {"ros__parameters": values}}
        payload = yaml.safe_dump(data, sort_keys=True, allow_unicode=True)

        # 落盘到标定库：library/<side>/<operator>_<name>_<ts>.yaml，并更新该侧 latest 指针。
        lib_dir = library_side_dir(calibration_dir, side)
        try:
            lib_dir.mkdir(parents=True, exist_ok=True)
            lib_filename = build_library_filename(operator, calib_name, side)
            lib_target = lib_dir / lib_filename
            _atomic_write_with_backup(lib_target, payload)
            write_latest_pointer(calibration_dir, side, lib_filename)
        except OSError as exc:
            messagebox.showerror("Revo3 标定", f"导出到标定库失败：{exc}")
            self._status("导出失败")
            return

        # 同步覆盖顶层兼容文件 <operator>_revo3_<side>.yaml：让 robot_config 里
        # retarget_calibration_<side> 留空时，旧 start_robot 固定名逻辑也拿到这次最新标定。
        compat_name = f"{operator}_revo3_{side}.yaml"
        compat_status = ""
        try:
            _atomic_write_with_backup(calibration_dir / compat_name, payload)
        except OSError as exc:
            compat_status = f"（顶层兼容文件 {compat_name} 写入失败：{exc}）"

        self.calibration_dir_var.set(str(calibration_dir))
        self._refresh_history()
        self._status(
            f"已导出并设为该侧最新：{lib_target}；已同步 {compat_name}{compat_status}"
        )

    def choose_calibration_dir(self) -> None:
        initial_dir = Path(self.calibration_dir_var.get().strip()).expanduser()
        if not initial_dir.is_dir():
            initial_dir = DEFAULT_CALIBRATION_DIR
        path = filedialog.askdirectory(title="选择 Revo3 操作员标定目录", initialdir=str(initial_dir))
        if path:
            self.calibration_dir_var.set(path)
            self._refresh_history()

    def _refresh_history(self) -> None:
        """刷新当前侧历史标定下拉；latest 指向的项加 “(最新)” 标注并预选。"""
        combo = getattr(self, "_history_combo", None)
        if combo is None:
            return
        calibration_dir = Path(self.calibration_dir_var.get().strip()).expanduser()
        side = self.current_side
        latest = read_latest_pointer(calibration_dir, side)
        names = [p.name for p in list_library(calibration_dir, side)]
        display = [f"{n}  (最新)" if n == latest else n for n in names]
        combo["values"] = display
        preselect = ""
        if latest and latest in names:
            preselect = f"{latest}  (最新)"
        elif display:
            preselect = display[0]
        self._history_var.set(preselect)

    def _set_history_latest(self) -> None:
        """把下拉选中的历史标定设为该侧 latest（改写 latest 指针）。"""
        selected = self._history_var.get().strip()
        if not selected:
            self._status("未选择历史标定")
            return
        filename = selected.split("  (最新)")[0]
        calibration_dir = Path(self.calibration_dir_var.get().strip()).expanduser()
        side = self.current_side
        if not (library_side_dir(calibration_dir, side) / filename).is_file():
            messagebox.showerror("Revo3 标定", f"标定文件不存在：{filename}")
            self._status("设为最新失败")
            return
        try:
            write_latest_pointer(calibration_dir, side, filename)
            # 同步顶层兼容文件，让 start_robot 固定名逻辑也切到这一份。
            operator = self.operator_var.get().strip()
            if OPERATOR_ID_PATTERN.fullmatch(operator):
                src = library_side_dir(calibration_dir, side) / filename
                shutil.copy2(src, calibration_dir / f"{operator}_revo3_{side}.yaml")
        except OSError as exc:
            messagebox.showerror("Revo3 标定", f"设为最新失败：{exc}")
            self._status("设为最新失败")
            return
        self._refresh_history()
        self._status(f"已将 {filename} 设为 {side} 侧最新标定")

    def _disable_node_best_effort(self, node: str) -> None:
        try:
            self.client.set_enabled_for(node, False, timeout_s=0.5)
        except Exception:
            pass

    def _update_hand_ui(self) -> None:
        side = self.current_side
        self.hand_text_var.set("右手 RIGHT" if side == "right" else "左手 LEFT")
        self.node_text_var.set(self.node_by_side[side])
        self.switch_button.configure(text="一键切换到左手" if side == "right" else "一键切换到右手")
        self._update_enable_button()

    def _update_enable_button(self) -> None:
        text = "关闭当前手输出" if self.output_enabled else "启用当前手输出"
        self.enable_button.configure(text=text)

    def release_nodes(self) -> list[str]:
        errors = []
        for side, node in self.node_by_side.items():
            if side not in self.initial_values:
                self._disable_node_best_effort(node)
                continue
            try:
                self.client.set_enabled_for(node, False, timeout_s=2.0)
            except Exception as exc:
                errors.append(f"{node}: {exc}")
        return errors

    def _on_close(self) -> bool:
        errors = self.release_nodes()
        if errors:
            messagebox.showerror("Revo3 标定", "未能确认释放命令，请检查节点后重试关闭：\n" + "\n".join(errors))
            return False
        self.root.destroy()
        return True

    def _status(self, text: str) -> None:
        self.status_var.set(text)


def _default_node(side: str) -> str:
    return f"/manus_revo3_retarget_{side}"


def _load_robot_export_defaults(robot_config: Path) -> tuple[str, Path]:
    try:
        with robot_config.expanduser().open(encoding="utf-8") as stream:
            config = json.load(stream)
    except (OSError, ValueError):
        return "", DEFAULT_CALIBRATION_DIR
    if not isinstance(config, dict):
        return "", DEFAULT_CALIBRATION_DIR
    operator = config.get("teleop_operator", "")
    calibration_dir = config.get("retarget_calibration_dir", "")
    if not isinstance(operator, str):
        operator = ""
    if not isinstance(calibration_dir, str) or not calibration_dir.strip():
        directory = DEFAULT_CALIBRATION_DIR
    else:
        directory = Path(calibration_dir).expanduser()
    return operator.strip(), directory


def _atomic_write_with_backup(target: Path, payload: str) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
            temporary_path = Path(stream.name)
        if target.exists():
            shutil.copy2(target, Path(f"{target}.bak"))
        os.replace(temporary_path, target)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def library_side_dir(calibration_dir: Path, side: str) -> Path:
    """某侧标定库目录：<calibration_dir>/library/<side>。"""
    return Path(calibration_dir).expanduser() / LIBRARY_SUBDIR / side


def build_library_filename(operator: str, name: str, side: str) -> str:
    """拼标定库文件名：<operator>_<name>_<ts>.yaml；name 为空则 <operator>_<ts>.yaml。

    operator/name 都已按各自 pattern 校验过（只含 [A-Za-z0-9_-]），故无需再转义。
    使用本地时间与纳秒后缀，避免同一秒连续导出覆盖历史。
    """
    timestamp_ns = time.time_ns()
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(timestamp_ns // 1_000_000_000))
    ts += f"_{timestamp_ns % 1_000_000_000:09d}"
    stem = f"{operator}_{name}_{ts}" if name else f"{operator}_{ts}"
    return f"{stem}.yaml"


def write_latest_pointer(calibration_dir: Path, side: str, filename: str) -> Path:
    """原子写某侧 latest 指针，指向 filename（相对该 side 目录）。返回指针路径。"""
    if not filename or Path(filename).name != filename or Path(filename).suffix != ".yaml":
        raise ValueError("latest must reference a YAML filename within the side library")
    side_dir = library_side_dir(calibration_dir, side)
    pointer = side_dir / LATEST_POINTER_NAME
    payload = json.dumps(
        {
            "schema": LATEST_POINTER_SCHEMA,
            "filename": filename,
            "updated_at_unix": time.time(),
        },
        ensure_ascii=False,
        indent=2,
    )
    _atomic_write_with_backup(pointer, payload)
    return pointer


def read_latest_pointer(calibration_dir: Path, side: str) -> str | None:
    """读某侧 latest 指针里的 filename；文件缺失/损坏/指向不存在的文件时返回 None。"""
    side_dir = library_side_dir(calibration_dir, side)
    pointer = side_dir / LATEST_POINTER_NAME
    try:
        with pointer.open(encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, ValueError):
        return None
    filename = data.get("filename") if isinstance(data, dict) else None
    if (
        not isinstance(filename, str)
        or not filename
        or Path(filename).name != filename
        or Path(filename).suffix != ".yaml"
    ):
        return None
    if not (side_dir / filename).is_file():
        return None
    return filename


def list_library(calibration_dir: Path, side: str) -> list[Path]:
    """列某侧标定库里的 *.yaml（按修改时间倒序，最新在前），排除 latest.json/.bak。"""
    side_dir = library_side_dir(calibration_dir, side)
    if not side_dir.is_dir():
        return []
    files = [
        path
        for path in side_dir.glob("*.yaml")
        if path.is_file() and not path.name.endswith(".bak")
    ]
    return sorted(files, key=lambda p: p.stat().st_mtime, reverse=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Staged calibration front-end for Revo3 retargeting.")
    parser.add_argument("--node", default="", help="Override the initial side's target node.")
    parser.add_argument("--left-node", default=_default_node("left"))
    parser.add_argument("--right-node", default=_default_node("right"))
    parser.add_argument("--side", default="right", choices=("left", "right"))
    parser.add_argument(
        "--robot-config",
        default=os.environ.get("ROBOT_CONFIG", str(DEFAULT_ROBOT_CONFIG)),
        help="Read teleop_operator and retarget_calibration_dir defaults from this robot config.",
    )
    parser.add_argument("--operator", default=None, help="Override the teleop operator ID used for export names.")
    parser.add_argument("--calibration-dir", default=None, help="Override the export directory.")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    node_by_side = {"left": args.left_node, "right": args.right_node}
    if args.node:
        node_by_side[args.side] = args.node
    config_operator, config_calibration_dir = _load_robot_export_defaults(Path(args.robot_config))
    operator = args.operator if args.operator is not None else config_operator
    if not operator:
        operator = getpass.getuser()
    calibration_dir = (
        Path(args.calibration_dir).expanduser()
        if args.calibration_dir is not None
        else config_calibration_dir
    )

    # Keep ROS alive until the release service has completed. rclpy's default
    # signal handler would invalidate the context before our finally block.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    client = ParameterClient()
    ui = None
    close_requested = False

    def request_close(_signum, _frame):
        nonlocal close_requested
        close_requested = True

    previous_handlers = {sig: signal.signal(sig, request_close)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        root = tk.Tk()
        ui = RetargetTuningPanel(root, client, node_by_side, args.side, operator, calibration_dir)

        def poll_close():
            nonlocal close_requested
            if close_requested:
                close_requested = False
                if ui._on_close():
                    return
            root.after(100, poll_close)

        root.after(100, poll_close)
        root.mainloop()
    finally:
        if ui is not None and rclpy.ok():
            for error in ui.release_nodes():
                client.get_logger().error("Release failed: " + error)
        client.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
