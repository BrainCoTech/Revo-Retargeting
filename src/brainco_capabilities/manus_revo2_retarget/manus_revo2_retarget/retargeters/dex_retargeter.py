"""Dex-retargeting based implementation.

Supports the algorithms provided by the ``dex_retargeting`` package:
- position
- DexPilot
- vector
"""

from pathlib import Path

import numpy as np
import yaml

from manus_revo2_retarget.retargeters import BaseRetargeter, RetargeterRegistry
from manus_revo2_retarget.revo2_joints import REVO2_JOINT_LIMITS_RAD, joint_names_for_side


VALID_SIDES = ("left", "right")


def _normalize_enabled_sides(enabled_sides):
    if enabled_sides is None:
        sides = VALID_SIDES
    elif isinstance(enabled_sides, str):
        sides = (enabled_sides,)
    else:
        sides = tuple(enabled_sides)

    normalized = []
    for side in sides:
        side = str(side).strip().lower()
        if side not in VALID_SIDES:
            raise ValueError(f"enabled_sides must contain only {VALID_SIDES}, got {side!r}")
        if side not in normalized:
            normalized.append(side)
    if not normalized:
        raise ValueError("enabled_sides must not be empty")
    return tuple(normalized)


def _resolve_path_fields(cfg, config_file_path: Path):
    """Resolve file paths in YAML relative to the config file when needed."""
    config_dir = config_file_path.expanduser().resolve().parent
    resolved_cfg = dict(cfg)

    for side in ("left", "right"):
        section = resolved_cfg.get(side)
        if not isinstance(section, dict):
            continue

        section = dict(section)
        for key in ("urdf_path", "urdf_file"):
            value = section.get(key)
            if not isinstance(value, str):
                continue

            path = Path(value).expanduser()
            if not path.is_absolute():
                candidates = (
                    Path.cwd() / path,
                    config_dir / path,
                )
                path = next((candidate for candidate in candidates if candidate.exists()), config_dir / path)
            section[key] = str(path.resolve())

        resolved_cfg[side] = section

    return resolved_cfg


class DexRetargeter(BaseRetargeter):
    """Retargeter built on top of ``dex_retargeting``."""

    def __init__(self, config_file_path: Path, enabled_sides=None):
        self.enabled_sides = _normalize_enabled_sides(enabled_sides)
        config_file_path = Path(config_file_path)
        with config_file_path.open("r") as f:
            cfg = yaml.safe_load(f)

        missing_sides = [side for side in self.enabled_sides if side not in cfg]
        if missing_sides:
            raise ValueError(
                "Configuration file is missing retargeting keys: "
                + ", ".join(missing_sides)
            )
        cfg = _resolve_path_fields(cfg, config_file_path)

        # Lazy-import dex_retargeting so the module is only required when this
        # retargeter is actually used.
        from dex_retargeting.retargeting_config import RetargetingConfig

        self._retargeting_by_side = {}
        self._indices_by_side = {}
        self._retargeting_to_hardware_by_side = {}
        self._brainco_api_joint_names_by_side = {}

        for side in self.enabled_sides:
            retargeting_config = RetargetingConfig.from_dict(cfg[side])
            retargeting = retargeting_config.build()
            joint_names = retargeting.joint_names
            brainco_api_joint_names = list(joint_names_for_side(side))
            hardware_mapping = [
                joint_names.index(name)
                for name in brainco_api_joint_names
            ]

            self._retargeting_by_side[side] = retargeting
            self._indices_by_side[side] = retargeting.optimizer.target_link_human_indices
            self._retargeting_to_hardware_by_side[side] = hardware_mapping
            self._brainco_api_joint_names_by_side[side] = brainco_api_joint_names

            setattr(self, f"{side}_retargeting", retargeting)
            setattr(self, f"{side}_retargeting_joint_names", joint_names)
            setattr(self, f"{side}_indices", self._indices_by_side[side])
            setattr(
                self,
                f"{side}_brainco_api_joint_names",
                brainco_api_joint_names,
            )
            setattr(self, f"{side}_dex_retargeting_to_hardware", hardware_mapping)

    def retarget_side(self, side, finger_tip_pos, **kwargs):
        del kwargs
        side = str(side).strip().lower()
        if side not in self._retargeting_by_side:
            raise ValueError(f"{side} retargeting is not enabled in this process")

        # DexPilot or vector type
        hand_data = np.zeros((25, 3))
        indices = self._indices_by_side[side]
        for i, finger_tip in enumerate(finger_tip_pos):
            hand_data[indices[1, i]] = finger_tip

        ref_value = (
            hand_data[indices[1, :]]
            - hand_data[indices[0, :]]
        )

        q_target = self._retargeting_by_side[side].retarget(ref_value)[
            self._retargeting_to_hardware_by_side[side]
        ]

        for idx, (lower, upper) in enumerate(REVO2_JOINT_LIMITS_RAD):
            q_target[idx] = np.clip(q_target[idx], lower, upper)

        return q_target

    def retarget_process(self, left_finger_tip_pos, right_finger_tip_pos, **kwargs):
        left_q_target = (
            self.retarget_side("left", left_finger_tip_pos, **kwargs)
            if "left" in self.enabled_sides
            else np.zeros(6, dtype=float)
        )
        right_q_target = (
            self.retarget_side("right", right_finger_tip_pos, **kwargs)
            if "right" in self.enabled_sides
            else np.zeros(6, dtype=float)
        )
        return left_q_target, right_q_target


def _build_dex_retargeter(config_path: Path, enabled_sides=None) -> BaseRetargeter:
    return DexRetargeter(config_path, enabled_sides=enabled_sides)


# Register under the legacy name "dex" as well as "dex_vector" for clarity.
RetargeterRegistry.register("dex", _build_dex_retargeter)
RetargeterRegistry.register("dex_vector", _build_dex_retargeter)
RetargeterRegistry.register("dex_position", _build_dex_retargeter)
RetargeterRegistry.register("dex_dexpilot", _build_dex_retargeter)
