"""DV1 FK from named JointState radians; no encoder zero/sign conversion.

Uses the same origin-then-axis rotation chain as HumanDex joint_state_mux.
The SDK publisher has already converted firmware angles into URDF radians.
"""

import math

import numpy as np

from .core import FINGERS, JOINT_KEYS, URDFKinematics, vector


def quaternion_xyzw(rotation):
    """Convert a rotation matrix to a unit quaternion, including half turns."""
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s, s / 4.0]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0.0, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2.0
        q = np.zeros(4)
        q[i] = s / 4.0
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        q[3] = (r[k, j] - r[j, k]) / s
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


class JointStateFK:
    """Validate and optionally smooth one hand, then evaluate all five tips.

    Validation/filter state advances only after a complete successful frame.
    Timestamps are integer nanoseconds so paired outputs need no round trip
    through floating-point seconds.
    """

    def __init__(self, urdf_path, side, tip_offsets_m=None, ema_alpha=1.0,
                 max_age_sec=0.5):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        self.base_link = f"{side}_palm_link"
        self.joint_names = tuple(
            f"{side}_{key.rsplit('_', 1)[0]}_{key.rsplit('_', 1)[1].upper()}_joint"
            for key in JOINT_KEYS)
        self.ema_alpha = float(ema_alpha)
        if not math.isfinite(self.ema_alpha) or not 0 < self.ema_alpha <= 1:
            raise ValueError("ema_alpha must be in (0, 1]")
        if not math.isfinite(max_age_sec) or max_age_sec <= 0:
            raise ValueError("max_age_sec must be positive and finite")
        self.max_age_ns = int(max_age_sec * 1e9)
        self.offsets = vector([0.0] * 15 if tip_offsets_m is None else tip_offsets_m,
                              15, "tip_offsets_m").reshape(5, 3)
        self.fk = URDFKinematics(urdf_path)
        self.chains = [self.fk.chain(self.base_link, f"{side}_{f}_DIP_Link")
                       for f in FINGERS]
        required = {j[1] for chain in self.chains for j in chain if j[2] != "fixed"}
        if required != set(self.joint_names):
            raise ValueError("URDF chains do not match the DV1 21-joint layout")
        self.last_stamp = 0
        self.filtered = None

    def compute(self, names, positions, stamp_ns, now_ns, frame_id):
        if frame_id != self.base_link:
            raise ValueError(f"expected input frame {self.base_link}, got {frame_id!r}")
        if (len(names) != len(positions) or len(names) != len(set(names))
                or set(names) != set(self.joint_names)):
            raise ValueError("expected 21 unique joints for the selected hand")
        if (stamp_ns <= 0 or stamp_ns <= self.last_stamp
                or not 0 <= now_ns - stamp_ns <= self.max_age_ns):
            raise ValueError("stale, future, zero or reordered JointState timestamp")
        by_name = dict(zip(names, positions))
        q = vector([by_name[n] for n in self.joint_names], 21, "joint positions")
        if self.filtered is not None and stamp_ns - self.last_stamp <= self.max_age_ns:
            delta = (q - self.filtered + np.pi) % (2 * np.pi) - np.pi
            q = self.filtered + self.ema_alpha * delta
        angles = dict(zip(self.joint_names, q))
        poses = []
        for chain, offset in zip(self.chains, self.offsets):
            pose = self.fk.evaluate(chain, angles)
            pose[:3, 3] += pose[:3, :3] @ offset
            poses.append(pose)
        self.filtered = q.copy()
        self.last_stamp = stamp_ns
        return q, poses
