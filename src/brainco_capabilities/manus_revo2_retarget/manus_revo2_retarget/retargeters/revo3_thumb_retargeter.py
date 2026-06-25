"""Revo3-style thumb retargeter for Revo2 hardware.

This retargeter improves thumb tracking by using a MuJoCo-based
Jacobian IK solver that fits both fingertip and proximal-link
(PIP equivalent) targets.
"""

from pathlib import Path
import logging
import shutil
import tempfile

import numpy as np

from manus_revo2_retarget.retargeters import BaseRetargeter, RetargeterRegistry
from manus_revo2_retarget.retargeters.dex_retargeter import (
    DexRetargeter,
    _normalize_enabled_sides,
)
from manus_revo2_retarget.revo2_joints import (
    REVO2_JOINT_LIMITS_RAD,
    REVO2_JOINT_UPPER_LIMITS_RAD,
)

logger = logging.getLogger(__name__)

try:
    import mujoco
except Exception as exc:  # pragma: no cover
    raise ImportError(
        "mujoco is required for Revo3ThumbRetargeter. "
        "Install it with: pip install mujoco>=3.0"
    ) from exc


DEFAULT_REVO3_PARAMS = {
    "thumb_ik_position_scale": 1.05,
    "thumb_joint_offset_deg": 6.0,
    "thumb_cmp_scale": 1.0,
    "thumb_mcp_scale": 0.6,
    "thumb_cmp_offset_deg": 0.0,
    "thumb_mcp_offset_deg": 0.0,
    "pip_constraint_weight": 0.10,
    "ema_prev": 0.10,
    "ema_cur": 0.90,
}

MANUS_Z_ROTATION_RAD = np.pi / 2.0
MANUS_SCALE_XZ = 1.13
FOUR_FINGER_NAMES = ("Index", "Middle", "Ring", "Pinky")
FOUR_FINGER_JOINT_OFFSET = 2
FOUR_FINGER_STRETCH_WEIGHTS = {
    "MCP": 0.50,
    "PIP": 0.35,
    "DIP": 0.15,
}
FOUR_FINGER_MAX_RAD = REVO2_JOINT_UPPER_LIMITS_RAD[FOUR_FINGER_JOINT_OFFSET]
FOUR_FINGER_MAX_STRETCH_DEG = float(np.rad2deg(FOUR_FINGER_MAX_RAD))

CALIBRATION_POSE_ORDER = ("open", "rotate", "pinch", "flex")
# Anchor order: (mcp, cmp)
CALIBRATION_ANCHORS = {
    "open": (0.12, 0.25),
    "rotate": (0.18, 0.88),
    "pinch": (0.72, 0.72),
    "flex": (0.90, 0.35),
}

AFFINE_A_MIN = 0.5
AFFINE_A_MAX = 1.6
AFFINE_B_MIN = -0.35
AFFINE_B_MAX = 0.35
AFFINE_REGULARIZATION = 0.1


# ---------------------------------------------------------------------------
# Coordinate utilities (ported from Revo3)
# ---------------------------------------------------------------------------
def _transform_manus_xyz(
    xyz,
    manus_xyz_scale: float,
    z_rotation_rad: float,
    out_x_sign: float,
    out_y_sign: float,
    out_z_sign: float,
):
    x = float(xyz[0])
    y = float(xyz[1])
    z = float(xyz[2])
    rot_x = np.cos(z_rotation_rad) * x - np.sin(z_rotation_rad) * y
    rot_y = np.sin(z_rotation_rad) * x + np.cos(z_rotation_rad) * y
    return np.array(
        [
            out_x_sign * rot_x * manus_xyz_scale,
            out_y_sign * rot_y * manus_xyz_scale,
            out_z_sign * z * manus_xyz_scale,
        ],
        dtype=float,
    )


def _apply_thumb_reach_scale(thumb_xyz, center_4, thumb_reach_scale):
    vec = np.asarray(thumb_xyz, dtype=float) - np.asarray(center_4, dtype=float)
    return center_4 + vec * thumb_reach_scale


def _apply_ema_to_manus_targets(previous_targets, current_targets, ema_prev, ema_cur):
    filtered_targets = {}
    previous_targets = previous_targets or {}
    for finger_cn, current_xyz in current_targets.items():
        previous_xyz = previous_targets.get(finger_cn)
        if previous_xyz is None:
            filtered_targets[finger_cn] = np.asarray(current_xyz, dtype=float).copy()
        else:
            filtered_targets[finger_cn] = (
                ema_prev * np.asarray(previous_xyz, dtype=float)
                + ema_cur * np.asarray(current_xyz, dtype=float)
            )
    return filtered_targets


def _four_finger_targets_from_ergonomics(ergonomics):
    if not ergonomics:
        return None

    targets = np.zeros(4, dtype=float)
    any_value = False
    for finger_index, finger_name in enumerate(FOUR_FINGER_NAMES):
        weighted_angle = 0.0
        for joint_name, weight in FOUR_FINGER_STRETCH_WEIGHTS.items():
            key = f"{finger_name}{joint_name}Stretch"
            if key not in ergonomics:
                continue
            any_value = True
            # MANUS ergonomics stretch values are degrees. Negative values mean
            # open/hyperextension on some setups, so keep them at the open end.
            weighted_angle += weight * max(0.0, float(ergonomics[key]))

        normalized = np.clip(weighted_angle / FOUR_FINGER_MAX_STRETCH_DEG, 0.0, 1.0)
        targets[finger_index] = normalized * FOUR_FINGER_MAX_RAD

    return targets if any_value else None


def _estimate_thumb_proximal_from_chain(thumb_tip_pos, thumb_dip_pos, thumb_pip_pos):
    if thumb_tip_pos is None or thumb_dip_pos is None or thumb_pip_pos is None:
        return None

    tip = np.asarray(thumb_tip_pos, dtype=float)
    dip = np.asarray(thumb_dip_pos, dtype=float)
    pip = np.asarray(thumb_pip_pos, dtype=float)
    if tip.shape != (3,) or dip.shape != (3,) or pip.shape != (3,):
        return None

    proximal_segment = dip - pip
    distal_segment = tip - dip
    proximal_norm = float(np.linalg.norm(proximal_segment))
    distal_norm = float(np.linalg.norm(distal_segment))
    if proximal_norm < 1e-6 or distal_norm < 1e-6:
        return None

    cosine = float(
        np.clip(
            np.dot(proximal_segment, distal_segment) / (proximal_norm * distal_norm),
            -1.0,
            1.0,
        )
    )
    angle = float(np.arccos(cosine))
    if not np.isfinite(angle):
        return None
    return angle


# ---------------------------------------------------------------------------
# MuJoCo helpers
# ---------------------------------------------------------------------------
def _get_joint_limits(model: mujoco.MjModel):
    jlow = np.full(model.nq, -np.pi)
    jhigh = np.full(model.nq, np.pi)
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        adr = model.jnt_qposadr[j]
        lo, hi = float(model.jnt_range[j, 0]), float(model.jnt_range[j, 1])
        if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
            jlow[adr] = lo
            jhigh[adr] = hi
    return jlow, jhigh


def _get_rest_pose(jlow, jhigh):
    q = np.zeros_like(jlow)
    for i in range(len(q)):
        q[i] = 0.0 if (jlow[i] <= 0.0 <= jhigh[i]) else jlow[i]
    return q


def _safe_normalize(value: float, lo: float, hi: float) -> float:
    if hi <= lo:
        return 0.0
    return float(np.clip((value - lo) / (hi - lo), 0.0, 1.0))


def _safe_denormalize(normalized: float, lo: float, hi: float) -> float:
    if hi <= lo:
        return lo
    return float(lo + np.clip(normalized, 0.0, 1.0) * (hi - lo))


def _fit_affine_with_bounds(raw_values, target_values, regularization=AFFINE_REGULARIZATION):
    """Fit n' = clip(a * n + b) with bounded grid search."""
    x = np.asarray(raw_values, dtype=float)
    y = np.asarray(target_values, dtype=float)
    if x.shape != y.shape:
        raise ValueError("raw_values and target_values must have the same shape")

    a_values = np.linspace(AFFINE_A_MIN, AFFINE_A_MAX, 111)
    b_values = np.linspace(AFFINE_B_MIN, AFFINE_B_MAX, 141)
    best = None

    for a in a_values:
        pred = a * x
        for b in b_values:
            clipped = np.clip(pred + b, 0.0, 1.0)
            residual = clipped - y
            loss = float(np.mean(residual * residual) + regularization * ((a - 1.0) ** 2 + b * b))
            if best is None or loss < best["loss"]:
                rmse = float(np.sqrt(np.mean(residual * residual)))
                best = {
                    "a": float(a),
                    "b": float(b),
                    "rmse": rmse,
                    "loss": loss,
                    "pred": clipped,
                }

    if best is None:  # pragma: no cover
        raise RuntimeError("Affine fitting failed")
    return best


def _solve_thumb_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    current_q: np.ndarray,
    thumb_target_xyz: np.ndarray,
    thumb_pip_target_xyz,
    metacarpal_adr: int,
    proximal_adr: int,
    distal_adr: int,
    metacarpal_dof: int,
    proximal_dof: int,
    tip_body_id: int,
    pip_body_id: int,
    jlow,
    jhigh,
    pip_constraint_weight: float,
    proximal_fixed_q=None,
):
    """MuJoCo Jacobian IK for the thumb with mimic distal coupling."""
    q = np.asarray(current_q, dtype=float).copy()
    proximal_is_fixed = proximal_fixed_q is not None
    if proximal_is_fixed:
        q[proximal_adr] = np.clip(
            float(proximal_fixed_q),
            jlow[proximal_adr],
            jhigh[proximal_adr],
        )
        thumb_dofs = [metacarpal_dof]
    else:
        thumb_dofs = [metacarpal_dof, proximal_dof]
    jacp_tip = np.zeros((3, model.nv))
    jacr_tip = np.zeros((3, model.nv))
    jacp_pip = np.zeros((3, model.nv))
    jacr_pip = np.zeros((3, model.nv))

    pip_weight = float(np.clip(pip_constraint_weight, 0.0, 1.0))

    for _ in range(40):
        if proximal_is_fixed:
            q[proximal_adr] = np.clip(
                float(proximal_fixed_q),
                jlow[proximal_adr],
                jhigh[proximal_adr],
            )
        q[distal_adr] = q[proximal_adr]
        data.qpos[:] = q
        mujoco.mj_forward(model, data)

        tip_curr = data.xpos[tip_body_id].copy()
        tip_err = thumb_target_xyz - tip_curr
        err_blocks = [2.0 * tip_err]
        mujoco.mj_jac(model, data, jacp_tip, jacr_tip, tip_curr, tip_body_id)
        jac_blocks = [2.0 * jacp_tip[:, thumb_dofs]]

        if thumb_pip_target_xyz is not None and pip_body_id >= 0 and pip_weight > 0.0:
            pip_curr = data.xpos[pip_body_id].copy()
            pip_err = thumb_pip_target_xyz - pip_curr
            err_blocks.append(pip_weight * pip_err)
            mujoco.mj_jac(model, data, jacp_pip, jacr_pip, pip_curr, pip_body_id)
            jac_blocks.append(pip_weight * jacp_pip[:, thumb_dofs])

        err = np.concatenate(err_blocks)
        if np.linalg.norm(err) < 5e-4:
            break

        J = np.vstack(jac_blocks)
        JJT = J @ J.T + (2e-2 * 2e-2) * np.eye(J.shape[0])
        dq = J.T @ np.linalg.solve(JJT, err)
        q[metacarpal_adr] = np.clip(
            q[metacarpal_adr] + 0.30 * dq[0],
            jlow[metacarpal_adr],
            jhigh[metacarpal_adr],
        )
        if not proximal_is_fixed:
            q[proximal_adr] = np.clip(
                q[proximal_adr] + 0.30 * dq[1],
                jlow[proximal_adr],
                jhigh[proximal_adr],
            )

    if proximal_is_fixed:
        q[proximal_adr] = np.clip(
            float(proximal_fixed_q),
            jlow[proximal_adr],
            jhigh[proximal_adr],
        )
    q[distal_adr] = q[proximal_adr]
    return q


# ---------------------------------------------------------------------------
# Retargeter class
# ---------------------------------------------------------------------------
class Revo3ThumbRetargeter(BaseRetargeter):
    """Retargeter that uses Revo3-style MuJoCo IK for the thumb.

    The four fingers prefer direct Manus ergonomics values. DexRetargeter is
    only an optional fallback when ergonomics data is unavailable.
    """

    def __init__(self, config_file_path: Path, enabled_sides=None):
        self.enabled_sides = _normalize_enabled_sides(enabled_sides)
        try:
            self._dex = DexRetargeter(
                config_file_path,
                enabled_sides=self.enabled_sides,
            )
        except Exception as exc:
            self._dex = None
            logger.debug(
                "DexRetargeter fallback unavailable; direct Manus ergonomics "
                "mapping remains active when ergonomics data is available: %s",
                exc,
            )

        pkg_root = Path(__file__).parent.parent

        self.left_model = None
        self.left_data = None
        self.right_model = None
        self.right_data = None
        self.left_q = None
        self.right_q = None
        self._left_filtered = None
        self._right_filtered = None
        self._thumb_debug_targets = {"left": {}, "right": {}}

        for side in self.enabled_sides:
            urdf = pkg_root / "brainco_hand" / f"brainco_{side}.urdf"
            model, data = self._load_mujoco(urdf)
            setattr(self, f"{side}_model", model)
            setattr(self, f"{side}_data", data)
            setattr(self, f"{side}_q", _get_rest_pose(*_get_joint_limits(model)))

        self._thumb_index_cache = {}
        self._joint_limit_cache = {}
        for side in self.enabled_sides:
            model = getattr(self, f"{side}_model")
            self._thumb_index_cache[side] = self._thumb_indices(model, side)
            self._joint_limit_cache[side] = _get_joint_limits(model)

        self._runtime_params = {
            side: self.default_runtime_params()
            for side in self.enabled_sides
        }

    def get_thumb_debug_targets(self, side: str) -> dict[str, np.ndarray | None]:
        targets = self._thumb_debug_targets.get(side, {})
        return {
            name: None if value is None else np.asarray(value, dtype=float).copy()
            for name, value in targets.items()
        }

    @staticmethod
    def default_runtime_params():
        return dict(DEFAULT_REVO3_PARAMS)

    def _ensure_side_enabled(self, side: str):
        side = str(side).strip().lower()
        if side not in self.enabled_sides:
            raise ValueError(f"{side} retargeting is not enabled in this process")
        return side

    @staticmethod
    def _load_mujoco(urdf_path: Path):
        urdf_path = Path(urdf_path).resolve()
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        mesh_dir = urdf_path.parent / "meshes"
        if mesh_dir.is_dir():
            # MuJoCo may strip the mesh subdirectory during URDF import, so
            # stage both basename and original "meshes/foo.STL" paths.
            with tempfile.TemporaryDirectory(prefix="revo3_mujoco_") as tmp:
                stage = Path(tmp)
                staged_urdf = stage / urdf_path.name
                staged_mesh_dir = stage / "meshes"
                staged_mesh_dir.mkdir(exist_ok=True)
                shutil.copy2(urdf_path, staged_urdf)
                for mesh_file in mesh_dir.glob("*.STL"):
                    mesh_target = mesh_file.resolve()
                    (stage / mesh_file.name).symlink_to(mesh_target)
                    (staged_mesh_dir / mesh_file.name).symlink_to(mesh_target)
                model = mujoco.MjModel.from_xml_path(str(staged_urdf))
        else:
            model = mujoco.MjModel.from_xml_path(str(urdf_path))

        # Retargeting only needs forward kinematics and Jacobians.  The CAD mesh
        # collision tree can overflow MuJoCo's stack during mj_forward.
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        data = mujoco.MjData(model)
        return model, data

    @staticmethod
    def _thumb_indices(model: mujoco.MjModel, side: str):
        prefix = "left" if side == "left" else "right"
        metacarpal_joint = f"{prefix}_thumb_metacarpal_joint"
        proximal_joint = f"{prefix}_thumb_proximal_joint"
        distal_joint = f"{prefix}_thumb_distal_joint"
        link_suffix = "Link" if side == "left" else "link"
        tip_body = f"{prefix}_thumb_distal_{link_suffix}"
        pip_body = f"{prefix}_thumb_proximal_{link_suffix}"

        def _qposadr(name):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return int(model.jnt_qposadr[jid])

        def _dofadr(name):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return int(model.jnt_dofadr[jid])

        def _bodyid(name):
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

        return {
            "metacarpal_adr": _qposadr(metacarpal_joint),
            "proximal_adr": _qposadr(proximal_joint),
            "distal_adr": _qposadr(distal_joint),
            "metacarpal_dof": _dofadr(metacarpal_joint),
            "proximal_dof": _dofadr(proximal_joint),
            "tip_body_id": _bodyid(tip_body),
            "pip_body_id": _bodyid(pip_body),
            "metacarpal_joint": metacarpal_joint,
            "proximal_joint": proximal_joint,
        }

    def _joint_range(self, model, joint_name: str):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        return float(model.jnt_range[joint_id][0]), float(model.jnt_range[joint_id][1])

    def _build_targets(self, finger_tip_pos):
        return {
            "thumb": np.asarray(finger_tip_pos[0], dtype=float) * MANUS_SCALE_XZ,
            "index": np.asarray(finger_tip_pos[1], dtype=float) * MANUS_SCALE_XZ,
            "middle": np.asarray(finger_tip_pos[2], dtype=float) * MANUS_SCALE_XZ,
            "ring": np.asarray(finger_tip_pos[3], dtype=float) * MANUS_SCALE_XZ,
            "pinky": np.asarray(finger_tip_pos[4], dtype=float) * MANUS_SCALE_XZ,
        }

    @staticmethod
    def _build_thumb_target(transformed, pip_pos, thumb_reach_scale):
        four = ["index", "middle", "ring", "pinky"]
        center_4 = (
            np.mean(np.vstack([transformed[f] for f in four]), axis=0)
            if all(f in transformed for f in four)
            else None
        )
        thumb_tip = None
        thumb_pip = None
        if "thumb" in transformed and center_4 is not None:
            thumb_tip = _apply_thumb_reach_scale(
                transformed["thumb"], center_4, thumb_reach_scale
            )
        if pip_pos is not None and center_4 is not None:
            pip_arr = np.asarray(pip_pos, dtype=float) * MANUS_SCALE_XZ
            thumb_pip = _apply_thumb_reach_scale(pip_arr, center_4, thumb_reach_scale)
        return thumb_tip, thumb_pip

    def _thumb_ik(
        self,
        model,
        data,
        current_q,
        thumb_tip_target,
        thumb_pip_target,
        side: str,
        pip_constraint_weight: float,
        proximal_fixed_q=None,
    ):
        idx = self._thumb_index_cache[side]
        jlow, jhigh = self._joint_limit_cache[side]
        return _solve_thumb_ik(
            model,
            data,
            current_q,
            thumb_tip_target,
            thumb_pip_target,
            idx["metacarpal_adr"],
            idx["proximal_adr"],
            idx["distal_adr"],
            idx["metacarpal_dof"],
            idx["proximal_dof"],
            idx["tip_body_id"],
            idx["pip_body_id"],
            jlow,
            jhigh,
            pip_constraint_weight,
            proximal_fixed_q=proximal_fixed_q,
        )

    @staticmethod
    def _merge_runtime_params(current_params, new_params):
        merged = dict(current_params)
        for key, value in new_params.items():
            if key in merged:
                merged[key] = float(value)

        ema_cur = float(np.clip(merged.get("ema_cur", 0.9), 0.0, 1.0))
        ema_prev = float(np.clip(merged.get("ema_prev", 0.1), 0.0, 1.0))
        ema_sum = ema_prev + ema_cur
        if ema_sum <= 1e-6:
            ema_cur = 0.9
            ema_prev = 0.1
        else:
            ema_cur = ema_cur / ema_sum
            ema_prev = ema_prev / ema_sum
        merged["ema_cur"] = float(np.clip(ema_cur, 0.0, 1.0))
        merged["ema_prev"] = float(np.clip(ema_prev, 0.0, 1.0))

        merged["pip_constraint_weight"] = float(
            np.clip(merged.get("pip_constraint_weight", 0.1), 0.0, 1.0)
        )
        merged["thumb_ik_position_scale"] = float(
            np.clip(merged.get("thumb_ik_position_scale", 1.05), 0.85, 1.30)
        )
        merged["thumb_cmp_scale"] = float(
            np.clip(merged.get("thumb_cmp_scale", 1.0), AFFINE_A_MIN, AFFINE_A_MAX)
        )
        merged["thumb_mcp_scale"] = float(
            np.clip(merged.get("thumb_mcp_scale", 0.6), AFFINE_A_MIN, AFFINE_A_MAX)
        )
        return merged

    def apply_calibration(self, params_by_side):
        """Apply calibrated runtime parameters.

        Args:
            params_by_side: dict with optional "left" / "right" keys.
        """
        if not isinstance(params_by_side, dict):
            raise TypeError("params_by_side must be a dict")

        for side in self.enabled_sides:
            side_params = params_by_side.get(side)
            if side_params is None:
                continue
            if not isinstance(side_params, dict):
                raise TypeError(f"Calibration params for {side} must be a dict")
            self._runtime_params[side] = self._merge_runtime_params(
                self._runtime_params[side], side_params
            )

    def _simulate_pose_sequence(self, side, ordered_poses, thumb_reach_scale, pip_weight):
        side = self._ensure_side_enabled(side)
        model = getattr(self, f"{side}_model")
        data = getattr(self, f"{side}_data")

        idx = self._thumb_index_cache[side]
        jlow, jhigh = self._joint_limit_cache[side]
        q = _get_rest_pose(jlow, jhigh)

        records = []
        tip_errors = []

        for pose in ordered_poses:
            transformed = self._build_targets(pose["finger_tips"])
            thumb_tip, thumb_pip = self._build_thumb_target(
                transformed,
                pose.get("thumb_pip"),
                thumb_reach_scale,
            )
            if thumb_tip is None:
                raise ValueError("Invalid pose: thumb target cannot be built")

            q = _solve_thumb_ik(
                model,
                data,
                q,
                thumb_tip,
                thumb_pip,
                idx["metacarpal_adr"],
                idx["proximal_adr"],
                idx["distal_adr"],
                idx["metacarpal_dof"],
                idx["proximal_dof"],
                idx["tip_body_id"],
                idx["pip_body_id"],
                jlow,
                jhigh,
                pip_weight,
            )

            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            tip_curr = data.xpos[idx["tip_body_id"]].copy()
            tip_err = float(np.linalg.norm(thumb_tip - tip_curr))
            tip_errors.append(tip_err)

            n_cmp_raw = _safe_normalize(
                q[idx["metacarpal_adr"]],
                jlow[idx["metacarpal_adr"]],
                jhigh[idx["metacarpal_adr"]],
            )
            n_mcp_raw = _safe_normalize(
                q[idx["proximal_adr"]],
                jlow[idx["proximal_adr"]],
                jhigh[idx["proximal_adr"]],
            )
            records.append(
                {
                    "n_cmp_raw": n_cmp_raw,
                    "n_mcp_raw": n_mcp_raw,
                    "tip_error": tip_err,
                }
            )

        mean_tip_error = float(np.mean(tip_errors)) if tip_errors else 1e9
        return records, mean_tip_error

    def _search_position_scale(self, side, ordered_poses, pip_weight):
        best = None
        for scale in np.linspace(0.85, 1.30, 46):
            records, mean_tip_error = self._simulate_pose_sequence(
                side,
                ordered_poses,
                thumb_reach_scale=float(scale),
                pip_weight=pip_weight,
            )
            candidate = {
                "scale": float(scale),
                "records": records,
                "mean_tip_error": mean_tip_error,
            }
            if best is None or candidate["mean_tip_error"] < best["mean_tip_error"]:
                best = candidate

        if best is None:  # pragma: no cover
            raise RuntimeError("Failed to search thumb_ik_position_scale")
        return best

    def _convert_affine_to_runtime(
        self,
        fitted,
        lo,
        hi,
        joint_offset_deg,
    ):
        a = float(fitted["a"])
        b = float(fitted["b"])
        delta = hi - lo
        joint_offset_rad = np.deg2rad(float(joint_offset_deg))
        offset_rad = ((1.0 - a) * lo + b * delta) - joint_offset_rad
        return a, float(np.rad2deg(offset_rad))

    def solve_calibration_for_side(self, side: str, pose_observations: dict):
        """Solve Revo3 runtime parameters from 4-pose observations."""
        side = self._ensure_side_enabled(side)

        ordered_poses = []
        for pose_name in CALIBRATION_POSE_ORDER:
            pose = pose_observations.get(pose_name)
            if not isinstance(pose, dict):
                raise ValueError(f"Missing pose '{pose_name}' for {side}")
            finger_tips = pose.get("finger_tips")
            thumb_pip = pose.get("thumb_pip")
            thumb_tip = pose.get("thumb_tip")
            if (
                not isinstance(finger_tips, list)
                or len(finger_tips) != 5
                or any(not isinstance(x, list) or len(x) != 3 for x in finger_tips)
            ):
                raise ValueError(f"Invalid finger_tips in pose '{pose_name}' for {side}")
            if thumb_pip is not None and (not isinstance(thumb_pip, list) or len(thumb_pip) != 3):
                raise ValueError(f"Invalid thumb_pip in pose '{pose_name}' for {side}")
            if thumb_tip is not None and (not isinstance(thumb_tip, list) or len(thumb_tip) != 3):
                raise ValueError(f"Invalid thumb_tip in pose '{pose_name}' for {side}")
            ordered_poses.append(
                {
                    "name": pose_name,
                    "finger_tips": finger_tips,
                    "thumb_pip": thumb_pip,
                    "thumb_tip": thumb_tip,
                    "thumb_tip_noise": float(max(0.0, pose.get("thumb_tip_noise", 0.0))),
                    "thumb_pip_noise": float(max(0.0, pose.get("thumb_pip_noise", 0.0))),
                }
            )

        pip_noises = [pose["thumb_pip_noise"] for pose in ordered_poses]
        tip_noises = [pose["thumb_tip_noise"] for pose in ordered_poses]
        mean_pip_noise = float(np.mean(pip_noises))
        mean_tip_noise = float(np.mean(tip_noises))

        pip_weight = float(np.clip(0.25 - 250.0 * np.sqrt(mean_pip_noise), 0.05, 0.25))

        best_scale = self._search_position_scale(side, ordered_poses, pip_weight)
        raw_cmp = np.array([r["n_cmp_raw"] for r in best_scale["records"]], dtype=float)
        raw_mcp = np.array([r["n_mcp_raw"] for r in best_scale["records"]], dtype=float)

        target_mcp = np.array(
            [CALIBRATION_ANCHORS[name][0] for name in CALIBRATION_POSE_ORDER], dtype=float
        )
        target_cmp = np.array(
            [CALIBRATION_ANCHORS[name][1] for name in CALIBRATION_POSE_ORDER], dtype=float
        )

        fitted_cmp = _fit_affine_with_bounds(raw_cmp, target_cmp)
        fitted_mcp = _fit_affine_with_bounds(raw_mcp, target_mcp)

        params = self.default_runtime_params()
        params["thumb_ik_position_scale"] = float(best_scale["scale"])

        joint_offset_deg = params["thumb_joint_offset_deg"]
        model = getattr(self, f"{side}_model")
        cmp_lo, cmp_hi = self._joint_range(
            model,
            self._thumb_index_cache[side]["metacarpal_joint"],
        )
        mcp_lo, mcp_hi = self._joint_range(
            model,
            self._thumb_index_cache[side]["proximal_joint"],
        )

        cmp_scale, cmp_offset_deg = self._convert_affine_to_runtime(
            fitted_cmp,
            cmp_lo,
            cmp_hi,
            joint_offset_deg,
        )
        mcp_scale, mcp_offset_deg = self._convert_affine_to_runtime(
            fitted_mcp,
            mcp_lo,
            mcp_hi,
            joint_offset_deg,
        )

        params["thumb_cmp_scale"] = float(cmp_scale)
        params["thumb_mcp_scale"] = float(mcp_scale)
        params["thumb_cmp_offset_deg"] = float(cmp_offset_deg)
        params["thumb_mcp_offset_deg"] = float(mcp_offset_deg)
        params["pip_constraint_weight"] = pip_weight

        ema_cur = float(np.clip(0.95 - 200.0 * np.sqrt(mean_tip_noise), 0.75, 0.95))
        params["ema_cur"] = ema_cur
        params["ema_prev"] = float(1.0 - ema_cur)

        params = self._merge_runtime_params(self.default_runtime_params(), params)

        fit_rmse = float(np.sqrt((fitted_cmp["rmse"] ** 2 + fitted_mcp["rmse"] ** 2) / 2.0))
        quality = {
            "fit_rmse": fit_rmse,
            "sampling_noise": {
                "thumb_tip_var": mean_tip_noise,
                "thumb_pip_var": mean_pip_noise,
            },
            "search_tip_rmse": float(best_scale["mean_tip_error"]),
        }

        return params, quality

    def retarget_side(
        self,
        side,
        finger_tip_pos,
        thumb_dip_pos=None,
        thumb_pip_pos=None,
        ergonomics=None,
        **kwargs,
    ):
        del kwargs
        side = self._ensure_side_enabled(side)
        if self._dex is not None:
            q_target = self._dex.retarget_side(side, finger_tip_pos)
        else:
            q_target = np.zeros(6, dtype=float)

        four = _four_finger_targets_from_ergonomics(ergonomics)
        if four is not None:
            q_target[
                FOUR_FINGER_JOINT_OFFSET:FOUR_FINGER_JOINT_OFFSET + len(FOUR_FINGER_NAMES)
            ] = four

        params = self._runtime_params[side]
        transformed = self._build_targets(finger_tip_pos)
        thumb_tip, thumb_pip = self._build_thumb_target(
            transformed,
            thumb_pip_pos,
            params["thumb_ik_position_scale"],
        )
        current = {"thumb": thumb_tip} if thumb_tip is not None else {}
        filtered_attr = f"_{side}_filtered"
        filtered = _apply_ema_to_manus_targets(
            getattr(self, filtered_attr),
            current,
            params["ema_prev"],
            params["ema_cur"],
        )
        setattr(self, filtered_attr, filtered)
        self._thumb_debug_targets[side] = {
            "thumb_tip_raw": thumb_tip,
            "thumb_tip_filtered": filtered.get("thumb") if filtered else None,
            "thumb_pip": thumb_pip,
        }

        if "thumb" in filtered:
            proximal_fixed_q = _estimate_thumb_proximal_from_chain(
                finger_tip_pos[0],
                thumb_dip_pos,
                thumb_pip_pos,
            )
            model = getattr(self, f"{side}_model")
            data = getattr(self, f"{side}_data")
            q_attr = f"{side}_q"
            q = self._thumb_ik(
                model,
                data,
                getattr(self, q_attr),
                filtered["thumb"],
                thumb_pip,
                side,
                params["pip_constraint_weight"],
                proximal_fixed_q=proximal_fixed_q,
            )
            setattr(self, q_attr, q)
            idx = self._thumb_index_cache[side]
            jlow, jhigh = self._joint_limit_cache[side]

            thumb_offset_rad = np.deg2rad(params["thumb_joint_offset_deg"])
            output_q = q.copy()

            cmp_adr = idx["metacarpal_adr"]
            cmp_q = output_q[cmp_adr] * params["thumb_cmp_scale"]
            cmp_q += thumb_offset_rad + np.deg2rad(params["thumb_cmp_offset_deg"])
            output_q[cmp_adr] = np.clip(cmp_q, float(jlow[cmp_adr]), float(jhigh[cmp_adr]))

            mcp_adr = idx["proximal_adr"]
            mcp_q = output_q[mcp_adr] * params["thumb_mcp_scale"]
            mcp_q += thumb_offset_rad + np.deg2rad(params["thumb_mcp_offset_deg"])
            output_q[mcp_adr] = np.clip(mcp_q, float(jlow[mcp_adr]), float(jhigh[mcp_adr]))

            mcp_lo, mcp_hi = REVO2_JOINT_LIMITS_RAD[0]
            cmp_lo, cmp_hi = REVO2_JOINT_LIMITS_RAD[1]
            q_target[0] = float(np.clip(output_q[mcp_adr], mcp_lo, mcp_hi))
            q_target[1] = float(np.clip(output_q[cmp_adr], cmp_lo, cmp_hi))

        return q_target

    def retarget_process(
        self,
        left_finger_tip_pos,
        right_finger_tip_pos,
        left_thumb_dip_pos=None,
        left_thumb_pip_pos=None,
        right_thumb_dip_pos=None,
        right_thumb_pip_pos=None,
        left_ergonomics=None,
        right_ergonomics=None,
    ):
        left_q_target = (
            self.retarget_side(
                "left",
                left_finger_tip_pos,
                thumb_dip_pos=left_thumb_dip_pos,
                thumb_pip_pos=left_thumb_pip_pos,
                ergonomics=left_ergonomics,
            )
            if "left" in self.enabled_sides
            else np.zeros(6, dtype=float)
        )
        right_q_target = (
            self.retarget_side(
                "right",
                right_finger_tip_pos,
                thumb_dip_pos=right_thumb_dip_pos,
                thumb_pip_pos=right_thumb_pip_pos,
                ergonomics=right_ergonomics,
            )
            if "right" in self.enabled_sides
            else np.zeros(6, dtype=float)
        )

        return left_q_target, right_q_target



def _build_revo3_thumb_retargeter(config_path: Path, enabled_sides=None) -> BaseRetargeter:
    return Revo3ThumbRetargeter(config_path, enabled_sides=enabled_sides)


RetargeterRegistry.register("revo3_thumb", _build_revo3_thumb_retargeter)
