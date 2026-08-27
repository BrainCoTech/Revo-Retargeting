"""Joint-semantics thumb retargeter for canonical hand kinematics."""

from pathlib import Path

import numpy as np

from revo2_hand_retarget.revo2_joints import REVO2_JOINT_LIMITS_RAD
from revo2_hand_retarget.retargeters import BaseRetargeter, RetargeterRegistry
from revo2_hand_retarget.retargeters.revo3_thumb_retargeter import (
    DEFAULT_REVO3_PARAMS,
    FOUR_FINGER_JOINT_OFFSET,
    FOUR_FINGER_NAMES,
    Revo3ThumbRetargeter,
    _apply_ema_to_landmark_targets,
    _four_finger_targets_from_joints,
)


DEFAULT_JOINT_THUMB_PARAMS = {
    "thumb_meta_sign": 1.0,
    "thumb_meta_zero_deg": 0.0,
    "thumb_meta_range_deg": 45.0,
    "thumb_prox_zero_deg": 0.0,
    "thumb_prox_range_deg": 80.0,
    "thumb_prox_mcp_weight": 0.45,
    "thumb_prox_pip_weight": 0.35,
    "thumb_prox_dip_weight": 0.20,
}


def _positive_joint(joint_positions, key: str) -> float:
    if not joint_positions or key not in joint_positions:
        return 0.0
    return max(0.0, float(joint_positions[key]))


def _signed_joint(joint_positions, key: str) -> float:
    if not joint_positions or key not in joint_positions:
        return 0.0
    return float(joint_positions[key])


def _normalized_signed_angle(value_rad: float, zero_deg: float, range_deg: float, sign: float) -> float:
    zero_rad = float(np.deg2rad(zero_deg))
    span_rad = max(1e-6, abs(float(np.deg2rad(range_deg))))
    return float(np.clip(float(sign) * (float(value_rad) - zero_rad) / span_rad, 0.0, 1.0))


def _normalized_positive_angle(value_rad: float, zero_deg: float, range_deg: float) -> float:
    zero_rad = float(np.deg2rad(zero_deg))
    span_rad = max(1e-6, abs(float(np.deg2rad(range_deg))))
    return float(np.clip((float(value_rad) - zero_rad) / span_rad, 0.0, 1.0))


def _thumb_targets_from_joints(joint_positions, params: dict[str, float]) -> tuple[float, float] | None:
    if not joint_positions:
        return None
    thumb_keys = (
        "thumb_mcp_spread",
        "thumb_spread",
        "thumb_mcp",
        "thumb_pip",
        "thumb_dip",
    )
    if not any(key in joint_positions for key in thumb_keys):
        return None

    meta_rad = _signed_joint(
        joint_positions,
        "thumb_mcp_spread" if "thumb_mcp_spread" in joint_positions else "thumb_spread",
    )
    meta_norm = _normalized_signed_angle(
        meta_rad,
        params["thumb_meta_zero_deg"],
        params["thumb_meta_range_deg"],
        params["thumb_meta_sign"],
    )

    flex_rad = (
        params["thumb_prox_mcp_weight"] * _positive_joint(joint_positions, "thumb_mcp")
        + params["thumb_prox_pip_weight"] * _positive_joint(joint_positions, "thumb_pip")
        + params["thumb_prox_dip_weight"] * _positive_joint(joint_positions, "thumb_dip")
    )
    prox_norm = _normalized_positive_angle(
        flex_rad,
        params["thumb_prox_zero_deg"],
        params["thumb_prox_range_deg"],
    )

    prox_lo, prox_hi = REVO2_JOINT_LIMITS_RAD[0]
    meta_lo, meta_hi = REVO2_JOINT_LIMITS_RAD[1]
    prox_q = prox_lo + prox_norm * (prox_hi - prox_lo)
    meta_q = meta_lo + meta_norm * (meta_hi - meta_lo)

    thumb_offset_rad = np.deg2rad(params["thumb_joint_offset_deg"])
    prox_q = prox_q * params["thumb_mcp_scale"]
    prox_q += thumb_offset_rad + np.deg2rad(params["thumb_mcp_offset_deg"])
    meta_q = meta_q * params["thumb_cmp_scale"]
    meta_q += thumb_offset_rad + np.deg2rad(params["thumb_cmp_offset_deg"])

    return (
        float(np.clip(prox_q, prox_lo, prox_hi)),
        float(np.clip(meta_q, meta_lo, meta_hi)),
    )


class JointThumbRetargeter(Revo3ThumbRetargeter):
    """Map canonical thumb joint semantics directly to Revo2 thumb joints."""

    @staticmethod
    def default_runtime_params():
        params = dict(DEFAULT_REVO3_PARAMS)
        params.update(DEFAULT_JOINT_THUMB_PARAMS)
        return params

    @staticmethod
    def _merge_runtime_params(current_params, new_params):
        merged = Revo3ThumbRetargeter._merge_runtime_params(current_params, new_params)
        for key, default_value in DEFAULT_JOINT_THUMB_PARAMS.items():
            if key in new_params:
                merged[key] = float(new_params[key])
            elif key not in merged:
                merged[key] = float(default_value)

        total_weight = (
            max(0.0, merged["thumb_prox_mcp_weight"])
            + max(0.0, merged["thumb_prox_pip_weight"])
            + max(0.0, merged["thumb_prox_dip_weight"])
        )
        if total_weight <= 1e-6:
            merged["thumb_prox_mcp_weight"] = DEFAULT_JOINT_THUMB_PARAMS["thumb_prox_mcp_weight"]
            merged["thumb_prox_pip_weight"] = DEFAULT_JOINT_THUMB_PARAMS["thumb_prox_pip_weight"]
            merged["thumb_prox_dip_weight"] = DEFAULT_JOINT_THUMB_PARAMS["thumb_prox_dip_weight"]
        else:
            merged["thumb_prox_mcp_weight"] = max(0.0, merged["thumb_prox_mcp_weight"]) / total_weight
            merged["thumb_prox_pip_weight"] = max(0.0, merged["thumb_prox_pip_weight"]) / total_weight
            merged["thumb_prox_dip_weight"] = max(0.0, merged["thumb_prox_dip_weight"]) / total_weight

        return merged

    def _cache_thumb_debug_target(self, side: str, finger_tip_pos, thumb_pip_pos, params):
        transformed = self._build_targets(finger_tip_pos)
        thumb_tip, thumb_pip = self._build_thumb_target(
            transformed,
            thumb_pip_pos,
            params["thumb_ik_position_scale"],
        )
        current = {"thumb": thumb_tip} if thumb_tip is not None else {}
        if side == "left":
            self._left_filtered = _apply_ema_to_landmark_targets(
                self._left_filtered,
                current,
                params["ema_prev"],
                params["ema_cur"],
            )
            filtered = self._left_filtered.get("thumb") if self._left_filtered else None
        else:
            self._right_filtered = _apply_ema_to_landmark_targets(
                self._right_filtered,
                current,
                params["ema_prev"],
                params["ema_cur"],
            )
            filtered = self._right_filtered.get("thumb") if self._right_filtered else None

        self._thumb_debug_targets[side] = {
            "thumb_tip_raw": thumb_tip,
            "thumb_tip_filtered": filtered,
            "thumb_pip": thumb_pip,
        }

    def retarget_process(
        self,
        left_finger_tip_pos,
        right_finger_tip_pos,
        left_thumb_dip_pos=None,
        left_thumb_pip_pos=None,
        right_thumb_dip_pos=None,
        right_thumb_pip_pos=None,
        left_joint_positions=None,
        right_joint_positions=None,
    ):
        del left_thumb_dip_pos, right_thumb_dip_pos
        if self._dex is not None:
            left_q_target, right_q_target = self._dex.retarget_process(
                left_finger_tip_pos,
                right_finger_tip_pos,
            )
        else:
            left_q_target = np.zeros(6, dtype=float)
            right_q_target = np.zeros(6, dtype=float)

        if "left" in self.enabled_sides:
            left_four = _four_finger_targets_from_joints(left_joint_positions)
            if left_four is not None:
                left_q_target[
                    FOUR_FINGER_JOINT_OFFSET:FOUR_FINGER_JOINT_OFFSET + len(FOUR_FINGER_NAMES)
                ] = left_four

            left_params = self._runtime_params["left"]
            self._cache_thumb_debug_target("left", left_finger_tip_pos, left_thumb_pip_pos, left_params)
            left_thumb = _thumb_targets_from_joints(left_joint_positions, left_params)
            if left_thumb is not None:
                left_q_target[0], left_q_target[1] = left_thumb

        if "right" in self.enabled_sides:
            right_four = _four_finger_targets_from_joints(right_joint_positions)
            if right_four is not None:
                right_q_target[
                    FOUR_FINGER_JOINT_OFFSET:FOUR_FINGER_JOINT_OFFSET + len(FOUR_FINGER_NAMES)
                ] = right_four

            right_params = self._runtime_params["right"]
            self._cache_thumb_debug_target("right", right_finger_tip_pos, right_thumb_pip_pos, right_params)
            right_thumb = _thumb_targets_from_joints(right_joint_positions, right_params)
            if right_thumb is not None:
                right_q_target[0], right_q_target[1] = right_thumb

        return left_q_target, right_q_target


def _build_joint_thumb_retargeter(config_path: Path, enabled_sides=None) -> BaseRetargeter:
    return JointThumbRetargeter(config_path, enabled_sides=enabled_sides)


RetargeterRegistry.register("joint_thumb", _build_joint_thumb_retargeter)
