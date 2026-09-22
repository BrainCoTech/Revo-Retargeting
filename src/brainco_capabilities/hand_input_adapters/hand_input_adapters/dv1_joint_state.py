"""Normalize SDK wire names to the palm-local DV1 FK contract.

The source frame identifies the SDK stream. It is not a TF transform: FK is
evaluated from joint angles in the selected URDF palm frame, without trackers.
"""

import math

from revohuman_kinematics.core import JOINT_KEYS


class DV1JointStateInput:
    def __init__(self, side, layout="sdk_single", source_frame_id=""):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if layout not in ("sdk_single", "sdk_pair", "legacy"):
            raise ValueError("joint_state_layout must be sdk_single, sdk_pair or legacy")
        bare = tuple(f"{key.rsplit('_', 1)[0]}_{key.rsplit('_', 1)[1].upper()}_joint"
                     for key in JOINT_KEYS)
        self.joint_names = tuple(f"{side}_{name}" for name in bare)
        if layout == "sdk_single":
            self.wire_names = bare
            self.selected_names = bare
            default_frame = f"revohuman_{side}"
        elif layout == "sdk_pair":
            self.wire_names = tuple(f"{hand}_{name}" for hand in ("left", "right") for name in bare)
            self.selected_names = self.joint_names
            default_frame = "revohuman_pair"
        else:
            self.wire_names = self.joint_names
            self.selected_names = self.joint_names
            default_frame = f"{side}_palm_link"
        self.source_frame_id = source_frame_id or default_frame

    def normalize(self, names, positions, frame_id):
        if frame_id != self.source_frame_id:
            raise ValueError(f"expected SDK frame {self.source_frame_id}, got {frame_id!r}")
        if (len(names) != len(positions) or len(names) != len(set(names))
                or set(names) != set(self.wire_names)):
            raise ValueError(f"expected {len(self.wire_names)} unique joints for the input layout")
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("SDK joint positions must be finite")
        by_name = dict(zip(names, positions))
        # Do not remap sign/zero, convert units, or replace the sample timestamp.
        return self.joint_names, [by_name[name] for name in self.selected_names]
