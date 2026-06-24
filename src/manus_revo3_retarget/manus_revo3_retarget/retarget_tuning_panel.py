#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
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
from rclpy.utilities import remove_ros_args


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


SPREAD_SPECS = [
    ParamSpec("legacy_{side}_physical_index_spread_offset_deg", "Index offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("legacy_{side}_physical_middle_spread_offset_deg", "Middle offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("legacy_{side}_physical_ring_spread_offset_deg", "Ring offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("legacy_{side}_physical_pinky_spread_offset_deg", "Little offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("legacy_{side}_physical_index_spread_scale", "Index scale", -2.0, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_middle_spread_scale", "Middle scale", -2.0, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_ring_spread_scale", "Ring scale", -2.0, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_pinky_spread_scale", "Little scale", -2.0, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_ring_spread_forward_scale", "Ring forward scale", -2.0, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_ring_spread_backward_scale", "Ring backward scale", -2.0, 2.0, 0.05),
    ParamSpec("physical_{side}_index_MPR_joint_offset_deg", "Index MPR out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_middle_MPR_joint_offset_deg", "Middle MPR out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_ring_MPR_joint_offset_deg", "Ring MPR out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_little_MPR_joint_offset_deg", "Little MPR out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_index_MPR_joint_scale", "Index MPR out scale", -2.0, 2.0, 0.05),
    ParamSpec("physical_{side}_middle_MPR_joint_scale", "Middle MPR out scale", -2.0, 2.0, 0.05),
    ParamSpec("physical_{side}_ring_MPR_joint_scale", "Ring MPR out scale", -2.0, 2.0, 0.05),
    ParamSpec("physical_{side}_little_MPR_joint_scale", "Little MPR out scale", -2.0, 2.0, 0.05),
    ParamSpec("spread_guard_min_adjacent_distance_mm", "Guard min distance mm", 0.0, 40.0, 0.5, digits=2),
    ParamSpec("spread_guard_converging_joint_weight", "Guard converge weight", 0.0, 2.0, 0.05),
    ParamSpec("spread_guard_keep_weight", "Guard keep weight", 0.0, 0.5, 0.01),
    ParamSpec("spread_guard_step_size", "Guard step size", 0.0, 2.0, 0.05),
    ParamSpec("spread_guard_max_iterations", "Guard max iterations", 0.0, 40.0, 1.0, kind="int", digits=0),
    ParamSpec("spread_guard_max_step_deg", "Guard max step deg", 0.0, 20.0, 0.5, digits=2),
    ParamSpec("spread_guard_max_correction_deg", "Guard max correction deg", 0.0, 40.0, 0.5, digits=2),
]

CONTROL_SPECS = [
    ParamSpec("mit_default_kp", "Global MIT kp", 0.0, 20.0, 0.05),
    ParamSpec("mit_default_kd", "Global MIT kd", 0.0, 2.0, 0.005),
]

FOUR_FINGER_SPECS = [
    ParamSpec("legacy_{side}_physical_index_angle_scale", "Index angle scale", 0.0, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_four_finger_mcp_scale", "All MCP scale", 0.0, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_middle_ring_dip_scale", "Middle/Ring DIP scale", 0.0, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_pinky_angle_scale", "Little angle scale", 0.0, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_pinky_dip_pip_scale", "Little DIP/PIP scale", 0.0, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_pinky_mcp_scale", "Little MCP scale", 0.0, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_all_finger_angle_scale", "Middle/Ring angle scale", 0.0, 2.5, 0.05),
]

for finger in ("index", "middle", "ring", "little"):
    for joint in ("MCP", "PIP", "DIP"):
        label_finger = {"index": "Index", "middle": "Middle", "ring": "Ring", "little": "Little"}[finger]
        FOUR_FINGER_SPECS.append(
            ParamSpec(
                f"physical_{{side}}_{finger}_{joint}_joint_offset_deg",
                f"{label_finger} {joint} out offset deg",
                -30.0,
                30.0,
                0.5,
                digits=2,
            )
        )
        FOUR_FINGER_SPECS.append(
            ParamSpec(
                f"physical_{{side}}_{finger}_{joint}_joint_scale",
                f"{label_finger} {joint} out scale",
                0.0,
                2.5,
                0.05,
            )
        )

THUMB_SPECS = [
    ParamSpec("thumb_ik_posture_weight", "IK posture weight", 0.0, 2.0, 0.05),
    ParamSpec("thumb_ik_smooth_weight", "IK smooth weight", 0.0, 2.0, 0.05),
    ParamSpec("thumb_ik_max_iterations", "IK max iterations", 0.0, 60.0, 1.0, kind="int", digits=0),
    ParamSpec("thumb_ik_max_step_deg", "IK max step deg", 0.0, 20.0, 0.5, digits=2),
    ParamSpec("thumb_ik_max_frame_delta_deg", "IK max frame delta deg", 0.0, 30.0, 0.5, digits=2),
    ParamSpec("{side}_thumb_cmp_offset_deg_physical", "CMP retarget offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("{side}_thumb_cmp_scale_physical", "CMP retarget scale", 0.05, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_cmr_offset_deg", "CMR retarget offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("legacy_{side}_physical_thumb_mcp_offset_deg", "MCP retarget offset deg", -40.0, 40.0, 0.5, digits=2),
    ParamSpec("legacy_{side}_physical_thumb_mcp_scale", "MCP retarget scale", 0.05, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_pip_scale", "PIP retarget scale", 0.05, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_dip_scale", "DIP retarget scale", 0.05, 2.5, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_pip_ik_scale", "PIP IK target weight scale", 0.0, 3.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_dip_ik_scale", "DIP IK target weight scale", 0.0, 3.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_reach_scale", "Thumb reach scale", 0.2, 2.0, 0.05),
    ParamSpec("legacy_{side}_physical_thumb_ik_position_scale", "Thumb IK position scale", 0.2, 2.0, 0.05),
    ParamSpec("physical_{side}_thumb_CMP_joint_offset_deg", "CMP out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_thumb_CMR_joint_offset_deg", "CMR out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_thumb_MCP_joint_offset_deg", "MCP out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_thumb_PIP_joint_offset_deg", "PIP out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_thumb_DIP_joint_offset_deg", "DIP out offset deg", -30.0, 30.0, 0.5, digits=2),
    ParamSpec("physical_{side}_thumb_CMP_joint_scale", "CMP out scale", 0.0, 2.5, 0.05),
    ParamSpec("physical_{side}_thumb_CMR_joint_scale", "CMR out scale", 0.0, 2.5, 0.05),
    ParamSpec("physical_{side}_thumb_MCP_joint_scale", "MCP out scale", 0.0, 2.5, 0.05),
    ParamSpec("physical_{side}_thumb_PIP_joint_scale", "PIP out scale", 0.0, 2.5, 0.05),
    ParamSpec("physical_{side}_thumb_DIP_joint_scale", "DIP out scale", 0.0, 2.5, 0.05),
    ParamSpec("pinch_manus_contact_index_mm", "Pinch Manus index mm", 0.0, 60.0, 0.5, digits=2),
    ParamSpec("pinch_manus_contact_middle_mm", "Pinch Manus middle mm", 0.0, 60.0, 0.5, digits=2),
    ParamSpec("pinch_manus_contact_ring_mm", "Pinch Manus ring mm", 0.0, 60.0, 0.5, digits=2),
    ParamSpec("pinch_manus_contact_little_mm", "Pinch Manus little mm", 0.0, 60.0, 0.5, digits=2),
    ParamSpec("pinch_model_contact_index_mm", "Pinch model index mm", 0.0, 20.0, 0.25, digits=2),
    ParamSpec("pinch_model_contact_middle_mm", "Pinch model middle mm", 0.0, 20.0, 0.25, digits=2),
    ParamSpec("pinch_model_contact_ring_mm", "Pinch model ring mm", 0.0, 20.0, 0.25, digits=2),
    ParamSpec("pinch_model_contact_little_mm", "Pinch model little mm", 0.0, 20.0, 0.25, digits=2),
    ParamSpec("pinch_distance_weight", "Pinch distance weight", 0.0, 100.0, 1.0, digits=2),
    ParamSpec("pinch_keep_weight", "Pinch keep weight", 0.0, 1.0, 0.01),
    ParamSpec("pinch_smooth_weight", "Pinch smooth weight", 0.0, 1.0, 0.01),
    ParamSpec("pinch_max_iterations", "Pinch max iterations", 0.0, 100.0, 1.0, kind="int", digits=0),
    ParamSpec("pinch_max_step_deg", "Pinch max step deg", 0.0, 20.0, 0.5, digits=2),
    ParamSpec("pinch_max_correction_deg", "Pinch max correction deg", 0.0, 60.0, 0.5, digits=2),
]

BOOL_PARAMS = [
    ("spread_guard_enabled", "Spread guard enabled"),
    ("legacy_{side}_physical_middle_spread_dynamic", "Middle spread dynamic"),
    ("pinch_enabled", "Pinch enabled"),
    ("pinch_optimize_finger_spread", "Pinch optimize finger spread"),
]


class ParameterClient(Node):
    def __init__(self):
        super().__init__("retarget_tuning_panel")
        self.target_node = ""
        self._get_client = None
        self._set_client = None

    def set_target(self, target_node: str) -> None:
        target = target_node.strip().rstrip("/")
        if not target:
            raise ValueError("target node cannot be empty")
        self.target_node = target if target.startswith("/") else f"/{target}"
        self._get_client = self.create_client(GetParameters, f"{self.target_node}/get_parameters")
        self._set_client = self.create_client(SetParameters, f"{self.target_node}/set_parameters")

    def wait_ready(self, timeout_s: float = 2.0) -> None:
        if self._get_client is None or self._set_client is None:
            raise RuntimeError("target node is not configured")
        deadline = time.monotonic() + timeout_s
        while rclpy.ok():
            if self._get_client.service_is_ready() and self._set_client.service_is_ready():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"Timed out waiting for {self.target_node} parameter services")
            self._get_client.wait_for_service(timeout_sec=min(0.2, remaining))
            self._set_client.wait_for_service(timeout_sec=min(0.2, max(deadline - time.monotonic(), 0.0)))

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


class SliderRow:
    def __init__(self, parent: ttk.Frame, spec: ParamSpec, side_getter, apply_callback, row: int):
        self.spec = spec
        self.side_getter = side_getter
        self.apply_callback = apply_callback
        self.var = tk.DoubleVar(value=0.0)
        self.entry_var = tk.StringVar(value="0")

        ttk.Label(parent, text=spec.label).grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)
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

    def get_value(self) -> object:
        self._entry_to_var()
        value = self._quantize(self.var.get())
        return int(round(value)) if self.spec.kind == "int" else float(value)

    def set_value(self, value: object) -> None:
        numeric = self._quantize(float(value))
        self.var.set(numeric)
        self.entry_var.set(self._format(numeric))


class RetargetTuningPanel:
    def __init__(self, root: tk.Tk, client: ParameterClient, default_node: str, default_side: str):
        self.root = root
        self.client = client
        self.initial_values: dict[str, object] = {}
        self.rows: list[SliderRow] = []
        self.bool_vars: dict[str, tk.BooleanVar] = {}

        root.title("Revo3 retarget tuning")
        root.geometry("1320x860")

        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="Target node").pack(side="left")
        self.node_var = tk.StringVar(value=default_node)
        ttk.Entry(toolbar, textvariable=self.node_var, width=34).pack(side="left", padx=(6, 12))
        ttk.Label(toolbar, text="Side").pack(side="left")
        self.side_var = tk.StringVar(value=default_side)
        side_box = ttk.Combobox(toolbar, textvariable=self.side_var, values=("left", "right"), width=7, state="readonly")
        side_box.pack(side="left", padx=(6, 12))
        side_box.bind("<<ComboboxSelected>>", lambda _event: self.refresh())
        ttk.Button(toolbar, text="Refresh", command=self.refresh).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Apply All", command=self.apply_all).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Reset", command=self.reset_initial).pack(side="left", padx=3)
        ttk.Button(toolbar, text="Export YAML", command=self.export_yaml).pack(side="left", padx=3)
        self.status_var = tk.StringVar(value="")
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        bool_frame = ttk.Frame(root, padding=(8, 0, 8, 6))
        bool_frame.pack(fill="x")
        for key, label in BOOL_PARAMS:
            var = tk.BooleanVar(value=False)
            self.bool_vars[key] = var
            ttk.Checkbutton(bool_frame, text=label, variable=var, command=self.apply_all).pack(side="left", padx=(0, 14))

        control_frame = ttk.LabelFrame(root, text="Control", padding=6)
        control_frame.pack(fill="x", padx=8, pady=(0, 8))
        control_frame.columnconfigure(1, weight=1)
        for row_index, spec in enumerate(CONTROL_SPECS):
            self.rows.append(SliderRow(control_frame, spec, self.side_var.get, self.apply_all, row_index))

        panes = ttk.PanedWindow(root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self._add_section(panes, "Spread / MPR", SPREAD_SPECS)
        self._add_section(panes, "Four Fingers", FOUR_FINGER_SPECS)
        self._add_section(panes, "Thumb / Pinch", THUMB_SPECS)

        self.refresh()

    def _add_section(self, panes: ttk.PanedWindow, title: str, specs: list[ParamSpec]) -> None:
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
        for row_index, spec in enumerate(specs):
            self.rows.append(SliderRow(body, spec, self.side_var.get, self.apply_all, row_index))

    def _all_names(self) -> list[str]:
        side = self.side_var.get()
        names = [row.spec.name(side) for row in self.rows]
        names.extend(key.format(side=side) for key in self.bool_vars)
        return names

    def _current_values(self) -> dict[str, object]:
        values = {row.name: row.get_value() for row in self.rows}
        side = self.side_var.get()
        for key, var in self.bool_vars.items():
            values[key.format(side=side)] = bool(var.get())
        return values

    def refresh(self) -> None:
        try:
            self.client.set_target(self.node_var.get())
            self.client.wait_ready()
            values = self.client.get_values(self._all_names())
            for row in self.rows:
                value = values.get(row.name)
                if value is not None:
                    row.set_value(value)
            side = self.side_var.get()
            for key, var in self.bool_vars.items():
                var.set(bool(values.get(key.format(side=side), False)))
            self.initial_values = dict(values)
            self._status(f"Connected {self.client.target_node}")
        except Exception as exc:
            messagebox.showerror("Retarget tuning", str(exc))
            self._status("Connection failed")

    def apply_all(self) -> None:
        try:
            self.client.set_values(self._current_values())
            self._status("Applied")
        except Exception as exc:
            messagebox.showerror("Retarget tuning", str(exc))
            self._status("Apply failed")

    def reset_initial(self) -> None:
        if not self.initial_values:
            return
        try:
            self.client.set_values(self.initial_values)
            self.refresh()
            self._status("Reset")
        except Exception as exc:
            messagebox.showerror("Retarget tuning", str(exc))
            self._status("Reset failed")

    def export_yaml(self) -> None:
        values = self._current_values()
        default_name = f"retarget_tuning_{self.side_var.get()}.yaml"
        path = filedialog.asksaveasfilename(
            title="Export current retarget tuning",
            initialfile=default_name,
            defaultextension=".yaml",
            filetypes=(("YAML", "*.yaml"), ("All files", "*")),
        )
        if not path:
            return
        data = {"manus_revo3_retarget": {"ros__parameters": values}}
        Path(path).write_text(yaml.safe_dump(data, sort_keys=True, allow_unicode=True), encoding="utf-8")
        self._status(f"Exported {path}")

    def _status(self, text: str) -> None:
        self.status_var.set(text)


def _default_node(side: str) -> str:
    return f"/manus_revo3_retarget_{side}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Tk front-end for online Revo3 retarget tuning.")
    parser.add_argument("--node", default="", help="Target retarget node, e.g. /manus_revo3_retarget_left.")
    parser.add_argument("--side", default="right", choices=("left", "right"))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    client = ParameterClient()
    root = tk.Tk()
    try:
        RetargetTuningPanel(root, client, args.node or _default_node(args.side), args.side)
        root.mainloop()
    finally:
        client.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
