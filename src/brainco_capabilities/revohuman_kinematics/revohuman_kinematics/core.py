"""ROS-independent encoder mapping and URDF forward kinematics."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

FINGERS = ("index", "middle", "ring", "little", "thumb")
JOINT_KEYS = tuple(f"{f}_{j}" for f in FINGERS[:4] for j in ("dip", "pip", "mcp", "mpr")) + tuple(
    f"thumb_{j}" for j in ("dip", "pip", "mcp", "cmr", "cmp"))


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def rotation(axis, angle):
    axis = vector(axis, 3, "axis")
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError("zero rotation axis")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def transform(xyz, rpy):
    result = np.eye(4)
    roll, pitch, yaw = vector(rpy, 3, "rpy")
    result[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                      @ rotation([1, 0, 0], roll))
    result[:3, 3] = vector(xyz, 3, "xyz")
    return result


class URDFKinematics:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.parents = {}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            attr = {} if origin is None else origin.attrib
            pose = transform([float(v) for v in attr.get("xyz", "0 0 0").split()],
                             [float(v) for v in attr.get("rpy", "0 0 0").split()])
            kind = joint.get("type")
            if kind not in ("fixed", "continuous", "revolute"):
                raise ValueError(f"Unsupported joint type: {kind}")
            axis_element = joint.find("axis")
            axis = vector([float(v) for v in (
                axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0"
            ).split()], 3, "axis")
            if kind != "fixed":
                rotation(axis, 0.0)  # Validate before the first frame.
            child = joint.find("child").get("link")
            if child in self.parents:
                raise ValueError(f"Multiple parents for {child}")
            self.parents[child] = (joint.find("parent").get("link"), joint.get("name"),
                                   kind, pose, axis)

    def chain(self, base, tip):
        chain, seen = [], set()
        while tip != base:
            if tip in seen or tip not in self.parents:
                raise ValueError(f"No valid chain from {base} to {tip}")
            seen.add(tip)
            joint = self.parents[tip]
            chain.append(joint)
            tip = joint[0]
        return list(reversed(chain))

    @staticmethod
    def evaluate(chain, angles):
        result = np.eye(4)
        for _, name, kind, pose, axis in chain:
            result = result @ pose
            if kind != "fixed":
                motion = np.eye(4)
                motion[:3, :3] = rotation(axis, angles[name])  # Never assume missing q=0.
                result = result @ motion
        return result


class HandModel:
    def __init__(self, urdf, config, side):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        cfg = config["sides"][side]
        self.zero = vector(cfg["zero_deg"], 21, "zero_deg")
        self.sign = vector(cfg["sign"], 21, "sign")
        if not np.all(np.isin(self.sign, [-1, 1])):
            raise ValueError("sign values must be -1 or +1")
        self.offset = vector(cfg["offset_deg"], 21, "offset_deg")
        self.wrap = config.get("wrap_degrees", True)
        if not isinstance(self.wrap, bool):
            raise ValueError("wrap_degrees must be boolean")
        self.alignment = transform(cfg["palm_translation_m"], cfg["palm_rpy_rad"])
        self.offsets = {f: vector(cfg["tip_offsets_m"][f], 3, f"{f} tip offset") for f in FINGERS}
        self.joint_names = tuple(f"{side}_{key.rsplit('_', 1)[0]}_{key.rsplit('_', 1)[1].upper()}_joint"
                                 for key in JOINT_KEYS)
        self.fk = URDFKinematics(urdf)
        self.chains = {f: self.fk.chain(f"{side}_palm_link", f"{side}_{f}_DIP_Link") for f in FINGERS}
        self.flex = {}
        for finger in FINGERS[:4]:
            params = dict(config["four_finger"])
            params.update(cfg.get("four_finger", {}))
            params.update(params.get("fingers", {}).get(finger, {}))
            weights = vector(params["weights"], 3, "weights")
            opened = vector(params["open_deg"], 3, "open_deg")
            closed = vector(params["closed_deg"], 3, "closed_deg")
            span = float(params["output_range_rad"])
            if (np.any(weights < 0) or weights.sum() <= 0 or
                    np.any(np.abs(closed - opened) < 1e-9) or not np.isfinite(span) or span <= 0):
                raise ValueError("Invalid four-finger calibration range or weights")
            self.flex[finger] = (weights / weights.sum(), opened, closed, span)
        expected = set(self.joint_names)
        for chain in self.chains.values():
            if any(name not in expected for _, name, kind, _, _ in chain if kind != "fixed"):
                raise ValueError("URDF chain does not match SDK mapping")

    def compute(self, angles_deg):
        delta = vector(angles_deg, 21, "encoder_angles_deg") - self.zero
        if self.wrap:
            delta = (delta + 180.0) % 360.0 - 180.0
        corrected_deg = self.sign * delta + self.offset
        q = np.deg2rad(corrected_deg)
        joints = dict(zip(JOINT_KEYS, q.tolist()))
        for i, finger in enumerate(FINGERS[:4]):
            weights, opened, closed, span = self.flex[finger]
            normalized = np.clip((corrected_deg[i * 4:i * 4 + 3] - opened) / (closed - opened), 0, 1)
            joints[f"{finger}_flexion"] = float(weights @ normalized * span)
        urdf_angles = dict(zip(self.joint_names, q))
        points = {}
        for finger in FINGERS:
            pose = self.alignment @ self.fk.evaluate(self.chains[finger], urdf_angles)
            points[f"{finger}_tip"] = pose[:3, :3] @ self.offsets[finger] + pose[:3, 3]
        return q, joints, points


def load_config(path):
    with Path(path).open() as stream:
        return yaml.safe_load(stream)
