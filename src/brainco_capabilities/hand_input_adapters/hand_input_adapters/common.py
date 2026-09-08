"""Shared naming and validation helpers for hand input adapters."""

from __future__ import annotations

import math
import re
from geometry_msgs.msg import Point
from hand_teleop_msgs.msg import HandKinematics


SIDES = ("left", "right")
FINGERS = ("thumb", "index", "middle", "ring", "little")

_HUMANDEX_JOINT_RE = re.compile(
    r"^(left|right)_(thumb|index|middle|ring|little)_([A-Za-z0-9]+)_joint$"
)
_ERGONOMICS_RE = re.compile(
    r"^(Thumb|Index|Middle|Ring|Pinky)(MCP|PIP|DIP)?(Stretch|Spread)$"
)


def side_value(side: str) -> int:
    if side == "left":
        return HandKinematics.LEFT
    if side == "right":
        return HandKinematics.RIGHT
    raise ValueError(f"unsupported hand side: {side}")


def stamp_key(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def humandex_joint_name(name: str, expected_side: str) -> str | None:
    match = _HUMANDEX_JOINT_RE.match(str(name))
    if match is None or match.group(1) != expected_side:
        return None
    finger = match.group(2)
    joint = match.group(3).lower()
    return f"{finger}_{joint}"


def ergonomics_joint_name(name: str) -> str | None:
    match = _ERGONOMICS_RE.match(str(name))
    if match is None:
        return None
    finger = match.group(1).lower().replace("pinky", "little")
    joint = (match.group(2) or "").lower()
    motion = match.group(3).lower()
    if motion == "spread":
        return f"{finger}_{joint + '_' if joint else ''}spread"
    if not joint:
        return None
    return f"{finger}_{joint}"


def finite_point(x: float, y: float, z: float) -> Point | None:
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def append_joint(message: HandKinematics, name: str, value_rad: float) -> None:
    if name and math.isfinite(value_rad):
        message.joint_names.append(name)
        message.joint_positions_rad.append(float(value_rad))


def append_landmark(message: HandKinematics, name: str, point: Point | None) -> None:
    if name and point is not None:
        message.landmark_names.append(name)
        message.landmarks_m.append(point)


def validate_message(message: HandKinematics) -> str | None:
    if message.side not in (HandKinematics.LEFT, HandKinematics.RIGHT):
        return "invalid side"
    if not message.header.frame_id:
        return "empty palm frame"
    if len(message.joint_names) != len(message.joint_positions_rad):
        return "joint name/value length mismatch"
    if len(message.landmark_names) != len(message.landmarks_m):
        return "landmark name/value length mismatch"
    if len(set(message.joint_names)) != len(message.joint_names):
        return "duplicate joint names"
    if len(set(message.landmark_names)) != len(message.landmark_names):
        return "duplicate landmark names"
    if not all(math.isfinite(value) for value in message.joint_positions_rad):
        return "non-finite joint value"
    for point in message.landmarks_m:
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return "non-finite landmark"
    return None


def selected_sides(value: str) -> tuple[str, ...]:
    normalized = str(value).strip().lower()
    if normalized == "both":
        return SIDES
    if normalized in SIDES:
        return (normalized,)
    raise ValueError("hand_mode must be left, right, or both")
