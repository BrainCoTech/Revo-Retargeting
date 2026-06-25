"""Shared Revo2 joint order and limits for ROS command adapters."""

REVO2_JOINT_SUFFIXES = (
    "thumb_proximal_joint",
    "thumb_metacarpal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)

REVO2_JOINT_SHORT_NAMES = (
    "thumb_prox",
    "thumb_meta",
    "index_prox",
    "middle_prox",
    "ring_prox",
    "pinky_prox",
)

REVO2_JOINT_LIMITS_RAD = (
    (0.0, 1.0472),
    (0.0, 1.5184),
    (0.0, 1.4661),
    (0.0, 1.4661),
    (0.0, 1.4661),
    (0.0, 1.4661),
)

REVO2_JOINT_UPPER_LIMITS_RAD = tuple(upper for _, upper in REVO2_JOINT_LIMITS_RAD)


def joint_names_for_side(side: str) -> tuple[str, ...]:
    prefix = "left" if side == "left" else "right"
    return tuple(f"{prefix}_{suffix}" for suffix in REVO2_JOINT_SUFFIXES)


def short_joint_names_for_side(side: str) -> tuple[str, ...]:
    prefix = "left" if side == "left" else "right"
    return tuple(f"{prefix}_{name}" for name in REVO2_JOINT_SHORT_NAMES)


def command_order_label() -> str:
    return ", ".join(name.removesuffix("_joint") for name in REVO2_JOINT_SUFFIXES)
