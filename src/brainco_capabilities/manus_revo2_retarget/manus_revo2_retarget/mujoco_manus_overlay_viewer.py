#!/usr/bin/env python3
"""MuJoCo Revo2 viewer with MANUS raw keypoint overlay."""
from __future__ import annotations

import argparse
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from manus_ros2_msgs.msg import ManusGlove

from .mujoco_joint_state_viewer import MIMIC_SUFFIX, _default_urdf, _joint_qposadr, _joint_range
from .revo2_joints import REVO2_JOINT_SHORT_NAMES, REVO2_JOINT_SUFFIXES


VALID_HAND_MODES = {"left", "right"}
MANUS_OVERLAY_TIMEOUT_S = 2.0
DEFAULT_MANUS_TOPICS = ("/manus_glove_0", "/manus_glove_1")
DEFAULT_MANUS_SCALE = 1.13

MANUS_TIP_NODE_IDS: dict[str, int] = {
    "thumb": 4,
    "index": 9,
    "middle": 14,
    "ring": 19,
    "pinky": 24,
}

REVO2_TIP_BODY_CANDIDATES: dict[str, tuple[str, ...]] = {
    "thumb": ("thumb_tip", "thumb_distal_link"),
    "index": ("index_tip", "index_distal_link"),
    "middle": ("middle_tip", "middle_distal_link"),
    "ring": ("ring_tip", "ring_distal_link"),
    "pinky": ("pinky_tip", "pinky_distal_link"),
}

FINGER_RGBA: dict[str, np.ndarray] = {
    "thumb": np.array([1.0, 0.68, 0.08, 0.96], dtype=float),
    "index": np.array([0.10, 0.75, 1.0, 0.96], dtype=float),
    "middle": np.array([0.18, 0.88, 0.34, 0.96], dtype=float),
    "ring": np.array([0.94, 0.38, 0.86, 0.96], dtype=float),
    "pinky": np.array([1.0, 0.30, 0.24, 0.96], dtype=float),
}


def _side_from_msg(msg: ManusGlove) -> str | None:
    side = str(msg.side).strip().lower()
    if side in ("left", "l"):
        return "left"
    if side in ("right", "r"):
        return "right"
    return None


def _finger_for_node(node_id: int) -> str | None:
    if 1 <= node_id <= 4:
        return "thumb"
    if 5 <= node_id <= 9:
        return "index"
    if 10 <= node_id <= 14:
        return "middle"
    if 15 <= node_id <= 19:
        return "ring"
    if 20 <= node_id <= 24:
        return "pinky"
    return None


def _transform_manus_point(point: np.ndarray, scale: float) -> np.ndarray:
    x, y, z = np.asarray(point, dtype=float)
    return np.array([-y, -x, z], dtype=float) * float(scale)


def _parse_topics(value: str) -> tuple[str, ...]:
    topics = tuple(item.strip() for item in value.split(",") if item.strip())
    if not topics:
        raise argparse.ArgumentTypeError("at least one MANUS topic is required")
    return topics


def _parse_offset(value: str) -> np.ndarray:
    pieces = [item.strip() for item in value.split(",")]
    if len(pieces) != 3:
        raise argparse.ArgumentTypeError("offset must be formatted as x,y,z")
    try:
        return np.array([float(item) for item in pieces], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("offset must contain three numbers") from exc


def _default_manus_scene_offset(side: str) -> np.ndarray:
    return np.array([0.18 if side == "right" else -0.18, 0.0, 0.0], dtype=float)


def _append_sphere(scn, pos: np.ndarray, radius: float, rgba: np.ndarray) -> None:
    if scn.ngeom >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(
        scn.geoms[scn.ngeom],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, radius, radius], dtype=float),
        np.asarray(pos, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        np.asarray(rgba, dtype=float),
    )
    scn.ngeom += 1


def _append_capsule(scn, p1: np.ndarray, p2: np.ndarray, radius: float, rgba: np.ndarray) -> None:
    if scn.ngeom >= scn.maxgeom:
        return
    geom = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3, dtype=float),
        np.zeros(3, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        np.asarray(rgba, dtype=float),
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        radius,
        np.asarray(p1, dtype=float),
        np.asarray(p2, dtype=float),
    )
    scn.ngeom += 1


def _append_label(scn, pos: np.ndarray, text: str, rgba: np.ndarray) -> None:
    if scn.ngeom >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(
        scn.geoms[scn.ngeom],
        mujoco.mjtGeom.mjGEOM_LABEL,
        np.array([0.012, 0.012, 0.012], dtype=float),
        np.asarray(pos, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        np.asarray(rgba, dtype=float),
    )
    scn.geoms[scn.ngeom].label = text
    scn.ngeom += 1


class MujocoManusOverlayViewer(Node):
    """Visualize Revo2 joint state and MANUS raw skeleton in one MuJoCo scene."""

    def __init__(
        self,
        hand_mode: str,
        joint_topic: str | None,
        thumb_debug_topic: str | None,
        manus_topics: tuple[str, ...],
        urdf_path: str | None,
        manus_scale: float,
        manus_offset: np.ndarray,
        align_on_first_frame: bool,
        relative_to_root: bool,
        draw_revo2_tip_links: bool,
    ):
        super().__init__(f"mujoco_{hand_mode}_manus_overlay_viewer")
        self.hand_mode = hand_mode
        self.prefix = f"{hand_mode}_"
        self.manus_scale = float(manus_scale)
        self.manual_manus_offset = (
            _default_manus_scene_offset(hand_mode)
            if manus_offset is None
            else np.asarray(manus_offset, dtype=float)
        )
        self.align_on_first_frame = bool(align_on_first_frame)
        self.relative_to_root = bool(relative_to_root)
        self.draw_revo2_tip_links = bool(draw_revo2_tip_links)

        self.urdf_path = _default_urdf(hand_mode) if urdf_path is None else urdf_path
        self.model = mujoco.MjModel.from_xml_path(str(self.urdf_path))
        self.data = mujoco.MjData(self.model)
        self.joint_map = self._build_joint_map()
        self.mimic_map = self._build_mimic_map()
        self.tip_body_ids = self._build_tip_body_ids()

        self.manus_points: dict[int, np.ndarray] = {}
        self.manus_parents: dict[int, int] = {}
        self.manus_time: float | None = None
        self.thumb_ik_target: np.ndarray | None = None
        self.thumb_ik_target_time: float | None = None
        self.manus_msg_count = 0
        self.joint_msg_count = 0
        self.align_offset: np.ndarray | None = None
        self.initial_thumb_tip: np.ndarray | None = None
        self.last_diag_log = 0.0

        joint_topic = joint_topic or f"/revo2_{hand_mode}/revo2_joint_state/joint_states"
        thumb_debug_topic = (
            thumb_debug_topic or f"/revo2_{hand_mode}/retarget/debug/thumb_ik_target"
        )
        self.create_subscription(JointState, joint_topic, self._joint_state_callback, 10)
        self.create_subscription(PointStamped, thumb_debug_topic, self._thumb_ik_target_callback, 10)
        for topic in manus_topics:
            self.create_subscription(
                ManusGlove,
                topic,
                lambda msg, topic=topic: self._manus_callback(msg, topic),
                10,
            )

        self.get_logger().info(f"Loaded MuJoCo model: {self.urdf_path}")
        self.get_logger().info(f"Listening to Revo2 JointState: {joint_topic}")
        self.get_logger().info(f"Listening to thumb IK debug target: {thumb_debug_topic}")
        self.get_logger().info(f"Listening to MANUS topics: {', '.join(manus_topics)}")
        self.get_logger().info(
            "MANUS overlay: transform=[-y,-x,z], "
            f"scale={self.manus_scale:.3f}, "
            f"offset={np.array2string(self.manual_manus_offset, precision=3)}, "
            f"relative_to_root={self.relative_to_root}, "
            f"align_on_first_frame={self.align_on_first_frame}"
        )

    def _build_joint_map(self) -> dict[str, dict[str, object]]:
        mapping: dict[str, dict[str, object]] = {}
        for short_name, suffix in zip(REVO2_JOINT_SHORT_NAMES, REVO2_JOINT_SUFFIXES):
            joint_name = f"{self.prefix}{suffix}"
            qposadr = _joint_qposadr(self.model, joint_name)
            joint_range = _joint_range(self.model, joint_name)
            if qposadr is None or joint_range is None:
                self.get_logger().warning(f"Joint not found in MuJoCo model: {joint_name}")
                continue
            info = {
                "joint_name": joint_name,
                "qposadr": qposadr,
                "range": joint_range,
            }
            names = {
                joint_name,
                suffix,
                f"{self.prefix}{short_name}",
                short_name,
            }
            for name in names:
                mapping[name] = info
        return mapping

    def _build_mimic_map(self) -> list[tuple[int, int, float, tuple[float, float]]]:
        mapping: list[tuple[int, int, float, tuple[float, float]]] = []
        for mimic_suffix, (source_suffix, multiplier) in MIMIC_SUFFIX.items():
            mimic_name = f"{self.prefix}{mimic_suffix}"
            source_name = f"{self.prefix}{source_suffix}"
            mimic_adr = _joint_qposadr(self.model, mimic_name)
            source_adr = _joint_qposadr(self.model, source_name)
            mimic_range = _joint_range(self.model, mimic_name)
            if mimic_adr is None or source_adr is None or mimic_range is None:
                continue
            mapping.append((mimic_adr, source_adr, float(multiplier), mimic_range))
        return mapping

    def _build_tip_body_ids(self) -> dict[str, int]:
        body_ids: dict[str, int] = {}
        for finger, candidates in REVO2_TIP_BODY_CANDIDATES.items():
            for suffix in candidates:
                body_name = f"{self.prefix}{suffix}"
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                if body_id >= 0:
                    body_ids[finger] = int(body_id)
                    break
        return body_ids

    def _joint_state_callback(self, msg: JointState) -> None:
        updated = False
        for name, value in zip(msg.name, msg.position):
            joint_info = self.joint_map.get(str(name))
            if joint_info is None:
                continue
            lo, hi = joint_info["range"]
            self.data.qpos[joint_info["qposadr"]] = float(np.clip(value, lo, hi))
            updated = True

        if not updated:
            if self.joint_msg_count == 0:
                self.get_logger().warning(
                    "Received JointState, but no Revo2 joint names matched: "
                    f"{list(msg.name)}"
                )
            return

        self.joint_msg_count += 1
        self._apply_mimic_and_forward()

    def _thumb_ik_target_callback(self, msg: PointStamped) -> None:
        self.thumb_ik_target = np.array(
            [float(msg.point.x), float(msg.point.y), float(msg.point.z)],
            dtype=float,
        )
        self.thumb_ik_target_time = time.monotonic()

    def _manus_callback(self, msg: ManusGlove, topic: str) -> None:
        side = _side_from_msg(msg)
        if side != self.hand_mode:
            return

        raw_points: dict[int, np.ndarray] = {}
        parents: dict[int, int] = {}
        for node in msg.raw_nodes:
            node_id = int(node.node_id)
            pos = node.pose.position
            raw_points[node_id] = np.array([float(pos.x), float(pos.y), float(pos.z)], dtype=float)
            parent_id = int(node.parent_node_id)
            if parent_id != node_id:
                parents[node_id] = parent_id

        if not raw_points:
            return

        origin = raw_points.get(0, np.zeros(3, dtype=float)) if self.relative_to_root else np.zeros(3, dtype=float)
        self.manus_points = {
            node_id: _transform_manus_point(point - origin, self.manus_scale)
            for node_id, point in raw_points.items()
        }
        self.manus_parents = parents
        self.manus_time = time.monotonic()
        self.manus_msg_count += 1
        if self.manus_msg_count == 1:
            self.get_logger().info(
                f"Received first {self.hand_mode} MANUS frame on {topic}: "
                f"raw_nodes={len(raw_points)}"
            )

    def _apply_mimic_and_forward(self) -> None:
        for mimic_adr, source_adr, multiplier, mimic_range in self.mimic_map:
            lo, hi = mimic_range
            self.data.qpos[mimic_adr] = np.clip(
                self.data.qpos[source_adr] * multiplier,
                lo,
                hi,
            )
        mujoco.mj_forward(self.model, self.data)

    def _revo2_tip_pos(self, finger: str) -> np.ndarray | None:
        body_id = self.tip_body_ids.get(finger)
        if body_id is None:
            return None
        return self.data.xpos[body_id].copy()

    def _ensure_alignment(self, points: dict[int, np.ndarray]) -> np.ndarray:
        if not self.align_on_first_frame:
            return self.manual_manus_offset
        if self.align_offset is not None:
            return self.align_offset
        if self.joint_msg_count == 0:
            return self.manual_manus_offset

        thumb_tip = points.get(MANUS_TIP_NODE_IDS["thumb"])
        revo2_thumb_tip = self._revo2_tip_pos("thumb")
        if thumb_tip is None or revo2_thumb_tip is None:
            return self.manual_manus_offset

        self.align_offset = revo2_thumb_tip - thumb_tip
        self.initial_thumb_tip = thumb_tip.copy()
        self.get_logger().info(
            "MANUS overlay aligned on first frame: "
            f"offset={np.array2string(self.align_offset, precision=4)}"
        )
        return self.align_offset

    def _draw_overlay(self, viewer) -> None:
        scn = viewer.user_scn
        scn.ngeom = 0
        now = time.monotonic()
        manus_live = (
            self.manus_time is not None
            and now - self.manus_time <= MANUS_OVERLAY_TIMEOUT_S
            and bool(self.manus_points)
        )
        debug_live = (
            self.thumb_ik_target_time is not None
            and now - self.thumb_ik_target_time <= MANUS_OVERLAY_TIMEOUT_S
            and self.thumb_ik_target is not None
        )
        if not manus_live and not debug_live:
            return

        offset = (
            self._ensure_alignment(self.manus_points)
            if manus_live
            else self.manual_manus_offset
        )
        points = (
            {node_id: point + offset for node_id, point in self.manus_points.items()}
            if manus_live
            else {}
        )

        bone_rgba = np.array([0.62, 0.66, 0.70, 0.34], dtype=float)
        node_rgba = np.array([0.72, 0.76, 0.80, 0.58], dtype=float)
        tip_link_rgba = np.array([1.0, 1.0, 1.0, 0.22], dtype=float)
        label_rgba = np.array([0.92, 0.96, 1.0, 1.0], dtype=float)
        ik_target_rgba = np.array([1.0, 1.0, 1.0, 0.98], dtype=float)

        for node_id, parent_id in self.manus_parents.items():
            p1 = points.get(parent_id)
            p2 = points.get(node_id)
            if p1 is None or p2 is None:
                continue
            _append_capsule(scn, p1, p2, 0.0011, bone_rgba)

        tip_ids = set(MANUS_TIP_NODE_IDS.values())
        for node_id, point in points.items():
            if node_id in tip_ids:
                continue
            finger = _finger_for_node(node_id)
            rgba = FINGER_RGBA.get(finger, node_rgba)
            _append_sphere(scn, point, 0.0024, rgba * np.array([1.0, 1.0, 1.0, 0.68], dtype=float))

        for finger, node_id in MANUS_TIP_NODE_IDS.items():
            manus_tip = points.get(node_id)
            if manus_tip is None:
                continue
            rgba = FINGER_RGBA[finger]
            _append_sphere(scn, manus_tip, 0.0050, rgba)
            _append_label(scn, manus_tip + np.array([0.0, 0.0, 0.012], dtype=float), finger, label_rgba)

            if not self.draw_revo2_tip_links:
                continue
            revo2_tip = self._revo2_tip_pos(finger)
            if revo2_tip is None:
                continue
            _append_sphere(scn, revo2_tip, 0.0036, rgba)
            _append_capsule(scn, manus_tip, revo2_tip, 0.0007, tip_link_rgba)

        if debug_live:
            ik_target = self.thumb_ik_target + offset
            _append_sphere(scn, ik_target, 0.0065, ik_target_rgba)
            _append_label(
                scn,
                ik_target + np.array([0.0, 0.0, 0.015], dtype=float),
                "IK target",
                label_rgba,
            )
            thumb_tip = points.get(MANUS_TIP_NODE_IDS["thumb"])
            if thumb_tip is not None:
                _append_capsule(scn, thumb_tip, ik_target, 0.0010, ik_target_rgba)

        _append_label(
            scn,
            np.array([0.0, 0.0, 0.18], dtype=float),
            "Revo2 + MANUS raw + thumb IK target",
            label_rgba,
        )
        self._maybe_log_diagnostics()

    def _maybe_log_diagnostics(self) -> None:
        now = time.monotonic()
        if now - self.last_diag_log < 1.0:
            return
        self.last_diag_log = now

        thumb_tip = self.manus_points.get(MANUS_TIP_NODE_IDS["thumb"])
        if thumb_tip is None:
            return
        if self.initial_thumb_tip is None:
            self.initial_thumb_tip = thumb_tip.copy()

        delta_mm = np.linalg.norm(thumb_tip - self.initial_thumb_tip) * 1000.0
        meta = self._joint_value("thumb_metacarpal_joint")
        prox = self._joint_value("thumb_proximal_joint")
        ik_delta_text = ""
        if self.thumb_ik_target is not None:
            ik_delta_mm = np.linalg.norm(self.thumb_ik_target - thumb_tip) * 1000.0
            ik_delta_text = f", ik_target_to_manus_thumb={ik_delta_mm:.1f} mm"
        self.get_logger().info(
            "Overlay diag: "
            f"manus_thumb_tip_delta={delta_mm:.1f} mm, "
            f"revo2_thumb_meta={meta:.3f} rad, "
            f"revo2_thumb_prox={prox:.3f} rad"
            f"{ik_delta_text}"
        )

    def _joint_value(self, suffix: str) -> float:
        joint_info = self.joint_map.get(f"{self.prefix}{suffix}")
        if joint_info is None:
            return float("nan")
        return float(self.data.qpos[joint_info["qposadr"]])


def parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Open a MuJoCo Revo2 viewer and overlay MANUS raw keypoints."
    )
    parser.add_argument(
        "--hand-mode",
        "--hand_mode",
        dest="hand_mode",
        default="right",
        choices=sorted(VALID_HAND_MODES),
        help="Which Revo2 hand model and MANUS side to visualize.",
    )
    parser.add_argument(
        "--joint-topic",
        "--joint_topic",
        dest="joint_topic",
        default=None,
        help=(
            "JointState topic for the Revo2 model. Defaults to "
            "/revo2_<hand>/revo2_joint_state/joint_states. Use "
            "/revo2_<hand>/revo2_pid_controller/target_joint_states to view retarget target instead."
        ),
    )
    parser.add_argument(
        "--thumb-debug-topic",
        "--thumb_debug_topic",
        dest="thumb_debug_topic",
        default=None,
        help="PointStamped topic for the retarget internal thumb IK target.",
    )
    parser.add_argument(
        "--manus-topics",
        "--manus_topics",
        dest="manus_topics",
        type=_parse_topics,
        default=DEFAULT_MANUS_TOPICS,
        help="Comma-separated ManusGlove topics.",
    )
    parser.add_argument(
        "--urdf",
        default=None,
        help="Optional explicit Revo2 URDF path.",
    )
    parser.add_argument(
        "--manus-scale",
        "--manus_scale",
        dest="manus_scale",
        default=DEFAULT_MANUS_SCALE,
        type=float,
        help="Scale applied to MANUS raw points after coordinate transform.",
    )
    parser.add_argument(
        "--manus-offset",
        "--manus_offset",
        dest="manus_offset",
        default=None,
        type=_parse_offset,
        help="Manual overlay offset x,y,z in meters. Defaults to a side-by-side MANUS offset.",
    )
    parser.add_argument(
        "--align-on-first-frame",
        "--align_on_first_frame",
        dest="align_on_first_frame",
        action="store_true",
        help="Align MANUS thumb_tip to Revo2 thumb tip on the first frame.",
    )
    parser.add_argument(
        "--no-relative-to-root",
        "--no_relative_to_root",
        dest="relative_to_root",
        action="store_false",
        help="Use MANUS absolute raw node positions instead of subtracting node 0.",
    )
    parser.add_argument(
        "--no-tip-links",
        "--no_tip_links",
        dest="draw_revo2_tip_links",
        action="store_false",
        help="Do not draw faint links between MANUS tips and Revo2 tips.",
    )
    parser.set_defaults(
        align_on_first_frame=False,
        relative_to_root=True,
        draw_revo2_tip_links=True,
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    cli_args, ros_args = parse_cli_args(argv)

    rclpy.init(args=ros_args)
    node = MujocoManusOverlayViewer(
        hand_mode=cli_args.hand_mode,
        joint_topic=cli_args.joint_topic,
        thumb_debug_topic=cli_args.thumb_debug_topic,
        manus_topics=cli_args.manus_topics,
        urdf_path=cli_args.urdf,
        manus_scale=cli_args.manus_scale,
        manus_offset=cli_args.manus_offset,
        align_on_first_frame=cli_args.align_on_first_frame,
        relative_to_root=cli_args.relative_to_root,
        draw_revo2_tip_links=cli_args.draw_revo2_tip_links,
    )

    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            viewer.cam.distance = 0.65
            viewer.cam.azimuth = 120 if cli_args.hand_mode == "right" else -120
            viewer.cam.elevation = -25
            while rclpy.ok() and viewer.is_running():
                loop_start = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.0)
                node._apply_mimic_and_forward()
                node._draw_overlay(viewer)
                viewer.sync()
                elapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, 1.0 / 60.0 - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
