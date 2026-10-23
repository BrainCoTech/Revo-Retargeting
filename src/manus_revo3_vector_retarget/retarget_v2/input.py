"""SDK skeleton semantics, wrist-independent palm vectors and capture validation."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
FOUR = FINGERS[1:]
PARTS = ("Metacarpal", "Proximal", "Intermediate", "Distal", "Tip")
CHAINS = {f: tuple(p for p in PARTS if f != "thumb" or p != "Intermediate") for f in FINGERS}
# Current manus_ros2 JointTypeToString() abbreviates the SDK enum names.
# These are wire labels, not anatomical joint aliases: its PIP is Proximal
# and its IP is Intermediate. Keep coordinates and parent IDs unchanged.
_PUBLISHER_PARTS = dict(zip(("MCP", "PIP", "IP", "DIP", "TIP"), PARTS))


def unit(a):
    a = np.asarray(a, dtype=float)
    norm = np.linalg.norm(a, axis=-1, keepdims=True)
    if np.any(norm < 1e-6) or not np.isfinite(a).all():
        raise ValueError("Degenerate or non-finite skeleton vector")
    return a / norm


def normalize_hand_side(hand_side):
    if not isinstance(hand_side, str) or hand_side.lower() not in ("left", "right"):
        raise ValueError("hand_side must be 'left' or 'right'")
    return hand_side.lower()


def decode(msg, hand_side="right"):
    hand_side = normalize_hand_side(hand_side)
    if not isinstance(msg["side"], str) or msg["side"].lower() != hand_side:
        raise ValueError(f"This model is for the {hand_side} hand")
    nodes = msg["raw_nodes"]
    if len(nodes) != 25 or msg["raw_node_count"] != 25:
        raise ValueError("Expected the SDK 25-node skeleton")
    ids = [n["node_id"] for n in nodes]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate node IDs")
    points, by_key = {}, {}
    for node in nodes:
        finger = node["chain_type"].lower().removeprefix("finger")
        finger = {"little": "pinky"}.get(finger, finger)
        part = _PUBLISHER_PARTS.get(node["joint_type"], node["joint_type"])
        key = "wrist" if finger in ("hand", "wrist") and part == "Invalid" else f"{finger}_{part}"
        if key in by_key:
            raise ValueError("Duplicate semantic landmark")
        xyz = np.array([node["pose"]["position"][a] for a in "xyz"], dtype=float)
        if not np.isfinite(xyz).all():
            raise ValueError("Non-finite landmark")
        points[key], by_key[key] = xyz, node
    required = {"wrist"} | {f"{f}_{p}" for f, parts in CHAINS.items() for p in parts}
    if set(points) != required:
        raise ValueError(f"SDK labels do not match: {set(points)^required}")
    wrist = by_key["wrist"]
    if wrist["parent_node_id"] not in (0, wrist["node_id"]):
        raise ValueError("Invalid wrist parent")
    for f, parts in CHAINS.items():
        parent = wrist
        for p in parts:
            node = by_key[f"{f}_{p}"]
            if node["parent_node_id"] != parent["node_id"]:
                raise ValueError(f"Wrong parent for {f}_{p}")
            parent = node
    return points


def palm_coordinates(points, normal_sign=-1.):
    """Anatomical frame from THIS frame, no frozen world frame or wrist IMU.

    The SDK and Revo3 use opposite palm-normal flexion conventions.
    normal_sign explicitly declares this coordinate conversion; it is not a
    fitted position offset. Proper rigid motions cancel before this reflection.
    Mirroring a hand reverses its anatomical cross-product normal and the
    selected robot's palm normal together, so both sides use the same sign.
    """
    if normal_sign not in (-1., 1.):
        raise ValueError("normal_sign must explicitly be -1 or +1")
    anchors = np.array([points[f"{f}_Proximal"] for f in FOUR])
    origin = anchors.mean(axis=0)
    lateral = unit(anchors[0]-anchors[-1])
    distal = origin-points["wrist"]
    distal = unit(distal-lateral*np.dot(lateral, distal))
    normal = np.cross(lateral, distal)
    basis = np.column_stack((lateral, distal, normal))
    return {k: (basis.T @ (v-origin))*np.array([1., 1., normal_sign]) for k, v in points.items()}


def chains(points):
    return np.stack([np.stack([points[f"{f}_{p}"] for p in
                              (("Metacarpal", "Proximal", "Distal", "Tip") if f == "thumb"
                               else ("Proximal", "Intermediate", "Distal", "Tip"))]) for f in FINGERS])


@dataclass
class Calibration:
    lengths: np.ndarray
    normal_sign: float = -1.
    hand_side: str = "right"

    def __post_init__(self):
        self.hand_side = normalize_hand_side(self.hand_side)
        self.lengths = np.asarray(self.lengths, dtype=float)
        if self.lengths.shape != (5, 3) or not np.isfinite(self.lengths).all() or np.any(self.lengths <= 0):
            raise ValueError("Calibration lengths must be a finite positive 5x3 array")
        if self.normal_sign not in (-1., 1.):
            raise ValueError("normal_sign must explicitly be -1 or +1")

    @classmethod
    def from_frames(cls, messages, normal_sign=-1., hand_side="right"):
        hand_side = normalize_hand_side(hand_side)
        if len(messages) < 30:
            raise ValueError("At least 30 valid open-hand frames required")
        lengths = np.array([np.linalg.norm(np.diff(chains(decode(m, hand_side)), axis=1), axis=2)
                            for m in messages])
        median = np.median(lengths, axis=0)
        if np.any(median < .008) or np.any(median > .10):
            raise ValueError("Unexpected bone lengths: confirm metre units and SDK topology")
        if np.max(np.abs(lengths/median-1)) > .1:
            raise ValueError("Calibration bone length inconsistency >10%")
        # Natural-open means a stable reference, not zero angles. No pose subtraction.
        return cls(median, normal_sign, hand_side)

    def features(self, message):
        raw = decode(message, self.hand_side)
        local = chains(palm_coordinates(raw, self.normal_sign))
        vectors = np.diff(local, axis=1)
        lengths = np.linalg.norm(vectors, axis=2)
        if np.max(np.abs(lengths/self.lengths-1)) > .15:
            raise ValueError("Bone length changed by >15%")
        return dict(directions=unit(vectors), chain=local, tip_vectors=local[0, 3]-local[1:, 3],
                    tip_distances=np.linalg.norm(local[0, 3]-local[1:, 3], axis=1))


def read_capture(path):
    path = Path(path)
    meta = json.loads((path/"capture.json").read_text())
    if not meta.get("complete") or meta.get("synthetic", False):
        raise ValueError("A complete real capture is required for this evaluation")
    if meta.get("coordinate_space") != "common" or meta.get("wire_profile") != "sdk":
        raise ValueError("Expected SDK common-coordinate data")
    if hashlib.sha256((path/"raw.jsonl").read_bytes()).hexdigest() != meta["raw_sha256"]:
        raise ValueError("Raw capture checksum mismatch")
    rows = [json.loads(line) for line in (path/"raw.jsonl").read_text().splitlines()]
    times = np.array([r["t"] for r in rows])
    if not len(rows) or len(rows) != meta["frames"] or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Invalid frame count or timestamps")
    for row in rows:
        if not row["valid"] or row["msg"]["glove_id"] != meta["glove_id"] or row["topic"] != meta["raw_topic"]:
            raise ValueError("Invalid frame or changed stream identity")
    return meta, rows
