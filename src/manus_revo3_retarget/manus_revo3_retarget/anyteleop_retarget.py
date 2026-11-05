from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml
from dex_retargeting.retargeting_config import RetargetingConfig



JOINT_NAMES = [
    "{side}_little_MPR_joint", "{side}_little_MCP_joint", "{side}_little_PIP_joint", "{side}_little_DIP_joint",
    "{side}_ring_MPR_joint", "{side}_ring_MCP_joint", "{side}_ring_PIP_joint", "{side}_ring_DIP_joint",
    "{side}_middle_MPR_joint", "{side}_middle_MCP_joint", "{side}_middle_PIP_joint", "{side}_middle_DIP_joint",
    "{side}_index_MPR_joint", "{side}_index_MCP_joint", "{side}_index_PIP_joint", "{side}_index_DIP_joint",
    "{side}_thumb_MCP_joint", "{side}_thumb_PIP_joint", "{side}_thumb_DIP_joint", "{side}_thumb_CMP_joint",
    "{side}_thumb_CMR_joint",
]
KEYPOINT_IDS = (0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24)
PIP_TRIPLES = {"index": (5, 6, 7), "middle": (9, 10, 11), "ring": (13, 14, 15), "little": (17, 18, 19)}


def _unit(vector: np.ndarray, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError(f"degenerate MANUS hand frame: {name}")
    return vector / norm


def _glove_points(message, side: str) -> np.ndarray:
    nodes = {int(node.node_id): node for node in message.raw_nodes}
    if len(nodes) != len(message.raw_nodes) or any(node_id not in nodes for node_id in KEYPOINT_IDS):
        raise ValueError("MANUS frame is missing hand keypoints")
    points = np.asarray(
        [
            [node.pose.position.x, node.pose.position.y, node.pose.position.z]
            for node in (nodes[i] for i in KEYPOINT_IDS)
        ],
        dtype=np.float64,
    )
    z_axis = _unit(points[9] - points[0], "wrist to middle knuckle")
    y_aux = _unit(points[5] - points[13], "index to ring knuckle")
    x_axis = _unit(np.cross(y_aux, z_axis), "palm normal")
    y_axis = np.cross(z_axis, x_axis)
    canonical = (points - points[0]) @ np.array([x_axis, y_axis, z_axis]).T
    base = (
        np.diag([-1.0, 1.0, 1.0])
        if side == "right"
        else np.diag([1.0, -1.0, 1.0])
    )
    result = canonical @ base
    if not np.isfinite(result).all():
        raise ValueError("MANUS frame contains non-finite keypoints")
    return result


def _pip_flexion(points: np.ndarray) -> dict[str, float]:
    output = {}
    for finger, (base, joint, distal) in PIP_TRIPLES.items():
        first = points[joint] - points[base]
        second = points[distal] - points[joint]
        denominator = np.linalg.norm(first) * np.linalg.norm(second)
        if denominator < 1e-9:
            raise ValueError(f"{finger} has a zero-length phalanx")
        output[finger] = float(
            np.arccos(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))
        )
    return output


def _rotate(vector: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]])


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _tip(angles, links):
    return sum((_rotate(link, angle) for link, angle in zip(links, np.cumsum(angles))), np.zeros(2))


def _redistribute_pip(angles, desired_pip: float, links: np.ndarray, limits: np.ndarray):
    angles = [float(a) for a in angles]
    target = _tip(angles, links)
    low, high = limits[1]
    t2 = min(max(float(desired_pip), low), high)
    fallback = (angles[0], t2, angles[2])

    c = links[0] + _rotate(links[1], t2)
    l1, l2 = float(np.linalg.norm(c)), float(np.linalg.norm(links[2]))
    if l1 < 1e-9 or l2 < 1e-9:
        return fallback

    reach = float(np.linalg.norm(target))
    if reach > l1 + l2 or reach < abs(l1 - l2):
        return fallback

    phi = math.atan2(c[1], c[0])
    distal = math.atan2(links[2][1], links[2][0])
    heading = math.atan2(target[1], target[0])
    cos_delta = (reach**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
    delta = math.acos(min(max(cos_delta, -1.0), 1.0))

    best = None
    for signed in (delta, -delta):
        distal_heading = phi + signed
        t3 = _wrap(distal_heading - distal - t2)
        whole = c + _rotate(links[2], t2 + t3)
        t1 = _wrap(heading - math.atan2(whole[1], whole[0]))
        candidate = (t1, t2, t3)
        if np.any(np.asarray(candidate) < limits[:, 0]) or np.any(np.asarray(candidate) > limits[:, 1]):
            continue
        if np.linalg.norm(_tip(candidate, links) - target) > 1e-6:
            continue
        cost = abs(_wrap(t1 - angles[0])) + abs(_wrap(t3 - angles[2]))
        if best is None or cost < best[0]:
            best = (cost, candidate)
    return fallback if best is None else best[1]


class AnyTeleopSolver:
    def __init__(self, side: str, config_path: Path, description_path: Path, filter_alpha: float = 0.4):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        document = yaml.safe_load(config_path.read_text(encoding="utf-8").replace("{side}", side))
        config = document["retargeting"]
        self.geometry = document["finger_geometry"][side]
        config["urdf_path"] = str(
            description_path
            / "urdf"
            / f"revo3_{side}_may3.urdf"
        )
        config["target_joint_names"] = [name.format(side=side) for name in JOINT_NAMES]
        config.update(
            add_dummy_free_joint=False,
            has_joint_limits=True,
            normal_delta=0.0,
            low_pass_alpha=-1.0,
        )
        self.retargeting = RetargetingConfig.from_dict(config).build()
        self.retargeting.optimizer.opt.set_ftol_abs(1e-10)
        self.joint_names = config["target_joint_names"]
        robot_names = self.retargeting.joint_names
        if set(robot_names) != set(self.joint_names):
            raise ValueError("Revo3 URDF joints do not match the AnyTeleop joint order")
        self.order = [robot_names.index(name) for name in self.joint_names]
        self.indices = self.retargeting.optimizer.target_link_human_indices
        self.limits = self.retargeting.joint_limits.copy()
        self.retargeting.optimizer.set_joint_limit(self.limits, epsilon=0.0)
        self.retargeting.last_qpos = np.clip(np.zeros(21), *self.limits.T)
        self.alpha = float(filter_alpha)
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("anyteleop_filter_alpha must be in (0, 1]")
        self.filtered: np.ndarray | None = None

    def retarget(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("expected finite [21, 3] keypoints")
        wanted = _pip_flexion(points)
        vectors = points[self.indices[1]] - points[self.indices[0]]
        if np.any(np.linalg.norm(vectors, axis=1) < 1e-6):
            raise ValueError("MANUS frame contains a zero-length target vector")
        previous = self.retargeting.last_qpos.copy()
        try:
            robot_q = self.retargeting.retarget(vectors)
            if self.retargeting.optimizer.opt.last_optimize_result() < 0:
                raise RuntimeError("AnyTeleop optimization failed")
            if robot_q.shape != (21,) or not np.isfinite(robot_q).all():
                raise ValueError("optimizer returned invalid joint positions")
        except (RuntimeError, ValueError):
            self.retargeting.last_qpos = previous
            raise
        joints = np.clip(robot_q[self.order], self.limits[:, 0], self.limits[:, 1])
        for finger, target in wanted.items():
            slots = [self.joint_names.index(f"{self.side}_{finger}_{joint}_joint")
                     for joint in ("MCP", "PIP", "DIP")]
            joints[slots] = _redistribute_pip(
                joints[slots], target, np.asarray(self.geometry[finger]), self.limits[slots]
            )
        device = joints.copy()
        for index, name in enumerate(self.joint_names):
            if "_MPR_" in name:
                device[index] *= -1.0
        self.filtered = (
            device
            if self.filtered is None
            else self.alpha * device + (1.0 - self.alpha) * self.filtered
        )
        for finger, target in wanted.items():
            slot = self.joint_names.index(f"{self.side}_{finger}_PIP_joint")
            self.filtered[slot] = np.clip(target, self.limits[slot, 0], self.limits[slot, 1])
        return self.filtered.copy()
