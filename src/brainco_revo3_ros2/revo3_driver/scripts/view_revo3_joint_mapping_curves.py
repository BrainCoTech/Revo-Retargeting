#!/usr/bin/env python3
"""Qt viewer for Revo3 joint mapping sample CSV files."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    except ImportError as exc:
        raise SystemExit(
            "Qt binding not found. Install PySide6 or PyQt5, for example: "
            "sudo apt install python3-pyqt5"
        ) from exc


QUANTITIES = {
    "Position": ("state_pos_deg", "cmd_pos_deg", "deg"),
    "Velocity": ("state_vel_deg_s", "cmd_vel_deg_s", "deg/s"),
    "Current": ("state_current", "", "A"),
}

VIEW_SELECTED = "Selected joint"
VIEW_ALL = "All joints"


def _to_float(value: str) -> float:
    if value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_samples(path: Path) -> tuple[list[dict[str, str]], list[str], list[tuple[int, str]]]:
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    joints = []
    prefix = "state_pos_deg:"
    for field in fieldnames:
        if field.startswith(prefix):
            joints.append(field[len(prefix):])

    probes: list[tuple[int, str]] = []
    seen = set()
    for row in rows:
        try:
            idx = int(row.get("probe_index", "0"))
        except ValueError:
            idx = 0
        joint = row.get("probe_joint", "")
        key = (idx, joint)
        if key not in seen:
            seen.add(key)
            probes.append(key)
    return rows, joints, probes


class PlotWidget(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.rows: list[dict[str, str]] = []
        self.joints: list[str] = []
        self.probe_index = 0
        self.joint = ""
        self.quantity = "Position"
        self.view_mode = VIEW_SELECTED
        self.show_others = True
        self.setMinimumSize(1040, 720)

    def set_data(self, rows: list[dict[str, str]], joints: list[str]) -> None:
        self.rows = rows
        self.joints = joints
        if joints and not self.joint:
            self.joint = joints[0]
        self.update()

    def _probe_rows(self) -> list[dict[str, str]]:
        return [
            row for row in self.rows
            if int(float(row.get("probe_index", "0") or 0)) == self.probe_index
        ]

    def _series_for_joint(
        self,
        rows: list[dict[str, str]],
        joint: str,
        prefix: str,
    ) -> tuple[list[float], list[float]]:
        xs = [_to_float(row.get("probe_time_s", "")) for row in rows]
        ys = [_to_float(row.get(f"{prefix}:{joint}", "")) for row in rows]
        points = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
        if not points:
            return [], []
        return [p[0] for p in points], [p[1] for p in points]

    def _series_for_selected(self) -> list[tuple[str, list[float], list[float], QtGui.QColor, int, bool]]:
        rows = self._probe_rows()
        if not rows or not self.joint:
            return []
        state_prefix, cmd_prefix, _unit = QUANTITIES[self.quantity]
        series: list[tuple[str, list[float], list[float], QtGui.QColor, int, bool]] = []

        if self.show_others:
            for joint in self.joints:
                if joint == self.joint:
                    continue
                xs, ys = self._series_for_joint(rows, joint, state_prefix)
                if xs:
                    series.append((joint, xs, ys, QtGui.QColor(170, 170, 170, 80), 1, False))

        if cmd_prefix:
            xs, ys = self._series_for_joint(rows, self.joint, cmd_prefix)
            if xs:
                series.append((f"cmd {self.joint}", xs, ys, QtGui.QColor("#e67e22"), 2, True))

        xs, ys = self._series_for_joint(rows, self.joint, state_prefix)
        if xs:
            series.append((f"state {self.joint}", xs, ys, QtGui.QColor("#1f77b4"), 3, False))
        return series

    def _series_for_one_joint(
        self,
        rows: list[dict[str, str]],
        joint: str,
    ) -> list[tuple[str, list[float], list[float], QtGui.QColor, int, bool]]:
        state_prefix, cmd_prefix, _unit = QUANTITIES[self.quantity]
        series: list[tuple[str, list[float], list[float], QtGui.QColor, int, bool]] = []
        if cmd_prefix:
            xs, ys = self._series_for_joint(rows, joint, cmd_prefix)
            if xs:
                series.append(("cmd", xs, ys, QtGui.QColor("#e67e22"), 1, True))
        xs, ys = self._series_for_joint(rows, joint, state_prefix)
        if xs:
            color = QtGui.QColor("#d62728") if joint == self._current_probe_joint() else QtGui.QColor("#1f77b4")
            series.append(("state", xs, ys, color, 2, False))
        return series

    def _current_probe_joint(self) -> str:
        rows = self._probe_rows()
        return rows[0].get("probe_joint", "") if rows else ""

    def _draw_series_plot(
        self,
        painter: QtGui.QPainter,
        plot: QtCore.QRect,
        series: list[tuple[str, list[float], list[float], QtGui.QColor, int, bool]],
        title: str,
        unit: str,
        *,
        dense: bool,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor("#333333"), 1))
        painter.drawRect(plot)
        if not series:
            painter.drawText(plot, QtCore.Qt.AlignCenter, "No data")
            return

        all_x = [x for _label, xs, _ys, _color, _width, _dash in series for x in xs]
        all_y = [y for _label, _xs, ys, _color, _width, _dash in series for y in ys]
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        if x_min == x_max:
            x_max = x_min + 1.0
        if y_min == y_max:
            pad = 1.0 if y_min == 0.0 else abs(y_min) * 0.1
            y_min -= pad
            y_max += pad
        else:
            pad = (y_max - y_min) * 0.08
            y_min -= pad
            y_max += pad

        def sx(x: float) -> float:
            return plot.left() + (x - x_min) / (x_max - x_min) * plot.width()

        def sy(y: float) -> float:
            return plot.bottom() - (y - y_min) / (y_max - y_min) * plot.height()

        grid_pen = QtGui.QPen(QtGui.QColor("#dddddd"), 1)
        text_pen = QtGui.QPen(QtGui.QColor("#333333"), 1)
        painter.setFont(QtGui.QFont("Sans", 8 if dense else 9))
        tick_count = 3 if dense else 6
        for i in range(tick_count):
            denom = max(1, tick_count - 1)
            gx = plot.left() + i * plot.width() / denom
            gy = plot.top() + i * plot.height() / denom
            painter.setPen(grid_pen)
            painter.drawLine(int(gx), plot.top(), int(gx), plot.bottom())
            painter.drawLine(plot.left(), int(gy), plot.right(), int(gy))
            painter.setPen(text_pen)
            x_val = x_min + i * (x_max - x_min) / denom
            y_val = y_max - i * (y_max - y_min) / denom
            if not dense or i in (0, tick_count - 1):
                painter.drawText(int(gx) - 20, plot.bottom() + 14, f"{x_val:.1f}")
                painter.drawText(plot.left() - 48, int(gy) + 4, f"{y_val:.1f}")

        for label, xs, ys, color, width, dashed in series:
            del label
            if len(xs) < 2:
                continue
            pen = QtGui.QPen(color, width)
            if dashed:
                pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(pen)
            path = QtGui.QPainterPath()
            path.moveTo(sx(xs[0]), sy(ys[0]))
            for x, y in zip(xs[1:], ys[1:]):
                path.lineTo(sx(x), sy(y))
            painter.drawPath(path)

        painter.setPen(text_pen)
        painter.setFont(QtGui.QFont("Sans", 8 if dense else 10))
        title_text = title if dense else f"{title} ({unit})"
        painter.drawText(plot.left(), plot.top() - 8, title_text)

        if dense:
            return

        painter.drawText(plot.center().x() - 38, self.height() - 14, "time (s)")
        legend_x = plot.right() - 250
        legend_y = plot.top() + 12
        for label, _xs, _ys, color, width, dashed in series[-2:]:
            pen = QtGui.QPen(color, width)
            if dashed:
                pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(legend_x, legend_y, legend_x + 28, legend_y)
            painter.setPen(text_pen)
            painter.drawText(legend_x + 36, legend_y + 4, label)
            legend_y += 20

    def _paint_selected(self, painter: QtGui.QPainter) -> None:
        _state_prefix, _cmd_prefix, unit = QUANTITIES[self.quantity]
        plot = self.rect().adjusted(76, 34, -24, -54)
        self._draw_series_plot(
            painter,
            plot,
            self._series_for_selected(),
            self.quantity,
            unit,
            dense=False,
        )

    def _paint_all_joints(self, painter: QtGui.QPainter) -> None:
        rows = self._probe_rows()
        _state_prefix, _cmd_prefix, unit = QUANTITIES[self.quantity]
        title_rect = QtCore.QRect(18, 10, self.width() - 36, 24)
        painter.setPen(QtGui.QPen(QtGui.QColor("#333333"), 1))
        painter.setFont(QtGui.QFont("Sans", 10))
        painter.drawText(
            title_rect,
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            f"{self.quantity} ({unit}) - red state line is the probed joint",
        )

        if not rows or not self.joints:
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "No data")
            return

        cols = 3
        rows_count = math.ceil(len(self.joints) / cols)
        left = 66
        top = 54
        right = 18
        bottom = 26
        gap_x = 24
        gap_y = 28
        cell_w = (self.width() - left - right - gap_x * (cols - 1)) / cols
        cell_h = (self.height() - top - bottom - gap_y * (rows_count - 1)) / rows_count

        for idx, joint in enumerate(self.joints):
            row = idx // cols
            col = idx % cols
            x = int(left + col * (cell_w + gap_x))
            y = int(top + row * (cell_h + gap_y))
            plot = QtCore.QRect(x, y, int(cell_w), int(cell_h))
            self._draw_series_plot(
                painter,
                plot,
                self._series_for_one_joint(rows, joint),
                joint,
                unit,
                dense=True,
            )

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        if self.view_mode == VIEW_ALL:
            self._paint_all_joints(painter)
        else:
            self._paint_selected(painter)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, path: Path | None) -> None:
        super().__init__()
        self.setWindowTitle("Revo3 Joint Mapping Curves")
        self.rows: list[dict[str, str]] = []
        self.joints: list[str] = []
        self.probes: list[tuple[int, str]] = []

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        controls = QtWidgets.QHBoxLayout()

        self.open_button = QtWidgets.QPushButton("Open")
        self.view_combo = QtWidgets.QComboBox()
        self.view_combo.addItems([VIEW_SELECTED, VIEW_ALL])
        self.probe_combo = QtWidgets.QComboBox()
        self.joint_combo = QtWidgets.QComboBox()
        self.quantity_combo = QtWidgets.QComboBox()
        self.quantity_combo.addItems(list(QUANTITIES.keys()))
        self.show_others = QtWidgets.QCheckBox("Show other joints")
        self.show_others.setChecked(True)

        controls.addWidget(self.open_button)
        controls.addWidget(QtWidgets.QLabel("View"))
        controls.addWidget(self.view_combo)
        controls.addWidget(QtWidgets.QLabel("Probe"))
        controls.addWidget(self.probe_combo, 1)
        controls.addWidget(QtWidgets.QLabel("Joint"))
        controls.addWidget(self.joint_combo, 1)
        controls.addWidget(QtWidgets.QLabel("Value"))
        controls.addWidget(self.quantity_combo)
        controls.addWidget(self.show_others)
        layout.addLayout(controls)

        self.plot = PlotWidget()
        layout.addWidget(self.plot, 1)
        self.setCentralWidget(central)

        self.open_button.clicked.connect(self.open_file)
        self.view_combo.currentTextChanged.connect(self._sync_plot)
        self.probe_combo.currentIndexChanged.connect(self._sync_plot)
        self.joint_combo.currentTextChanged.connect(self._sync_plot)
        self.quantity_combo.currentTextChanged.connect(self._sync_plot)
        self.show_others.toggled.connect(self._sync_plot)

        if path is not None:
            self.load_file(path)

    def open_file(self) -> None:
        filename, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open Revo3 samples CSV",
            "",
            "CSV files (*.csv);;All files (*)",
        )
        if filename:
            self.load_file(Path(filename))

    def load_file(self, path: Path) -> None:
        self.rows, self.joints, self.probes = load_samples(path)
        self.setWindowTitle(f"Revo3 Joint Mapping Curves - {path.name}")

        self.probe_combo.blockSignals(True)
        self.joint_combo.blockSignals(True)
        self.probe_combo.clear()
        self.joint_combo.clear()
        for idx, joint in self.probes:
            self.probe_combo.addItem(f"{idx:02d} {joint}", idx)
        self.joint_combo.addItems(self.joints)
        self.probe_combo.blockSignals(False)
        self.joint_combo.blockSignals(False)
        self.plot.set_data(self.rows, self.joints)
        self._sync_plot()

    def _sync_plot(self) -> None:
        view_mode = self.view_combo.currentText() or VIEW_SELECTED
        self.plot.probe_index = int(self.probe_combo.currentData() or 0)
        self.plot.joint = self.joint_combo.currentText()
        self.plot.quantity = self.quantity_combo.currentText() or "Position"
        self.plot.view_mode = view_mode
        self.plot.show_others = self.show_others.isChecked()
        self.joint_combo.setEnabled(view_mode == VIEW_SELECTED)
        self.show_others.setEnabled(view_mode == VIEW_SELECTED)
        self.plot.update()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("samples_csv", nargs="?", help="Path to *.samples.csv from test script.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    path = Path(args.samples_csv).expanduser() if args.samples_csv else None
    app = QtWidgets.QApplication(sys.argv[:1])
    window = MainWindow(path)
    window.resize(1400, 960)
    window.show()
    return app.exec() if hasattr(app, "exec") else app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
