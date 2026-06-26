import argparse
import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from manus_ros2_msgs.msg import ManusGlove
from manus_revo2_retarget.revo2_joints import (
    REVO2_JOINT_LIMITS_RAD,
    REVO2_JOINT_SUFFIXES,
    REVO2_JOINT_UPPER_LIMITS_RAD,
    command_order_label,
    joint_names_for_side,
    short_joint_names_for_side,
)
from manus_revo2_retarget.retargeters import RetargeterRegistry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(name)s] %(message)s")
logger = logging.getLogger(__name__)

end_effector = 'real_hand'  # real_hand  sim_hand
VALID_HAND_MODES = {"left", "right", "both"}
VALID_END_EFFECTORS = {"real_hand", "sim_hand"}
VALID_CONTROL_MODES = {"position_speed", "pd_velocity", "pd_position_speed"}
REVO3_CALIBRATION_PROTOCOL = "standard4_v1"
REVO3_SCHEMA_VERSION = 2
REVO3_POSE_SEQUENCE = [
    ("open", "五指张开"),
    ("rotate", "拇指外旋"),
    ("pinch", "拇食捏合"),
    ("flex", "拇指内收屈曲"),
]
REVO3_REQUIRED_PARAM_KEYS = {
    "thumb_ik_position_scale",
    "thumb_joint_offset_deg",
    "thumb_cmp_scale",
    "thumb_mcp_scale",
    "thumb_cmp_offset_deg",
    "thumb_mcp_offset_deg",
    "pip_constraint_weight",
    "ema_prev",
    "ema_cur",
}
REVO3_OPTIONAL_RUNTIME_PARAM_KEYS = {
    "thumb_meta_sign",
    "thumb_meta_zero_deg",
    "thumb_meta_range_deg",
    "thumb_prox_zero_deg",
    "thumb_prox_range_deg",
    "thumb_prox_mcp_weight",
    "thumb_prox_pip_weight",
    "thumb_prox_dip_weight",
}
REVO3_RUNTIME_PARAM_KEYS = REVO3_REQUIRED_PARAM_KEYS | REVO3_OPTIONAL_RUNTIME_PARAM_KEYS
CALIBRATION_COUNTDOWN_SEC = 3
CALIBRATION_SAMPLE_SEC = 1.2
CALIBRATION_SAMPLE_INTERVAL_SEC = 0.02


def quaternion_to_rotation_matrix(quat):
    """
    将四元数转换为旋转矩阵
    quat: [x, y, z, w] 格式的四元数
    返回: 3x3 旋转矩阵
    """
    x, y, z, w = quat

    # 标准化四元数
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm

    # 计算旋转矩阵
    rotation_matrix = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

    return rotation_matrix


def get_offset_position(pose, axis_offset_mm):
    """
    根据IMU的pose计算在某个轴偏移xxx毫米后的新position

    参数:
    pose: geometry_msgs.msg.Pose 对象
    axis_offset_mm: 轴偏移向量，单位毫米 [x_offset, y_offset, z_offset]
                   例如: [10, 0, 0] 表示在X轴正方向偏移10mm

    返回:
    新的position (x, y, z) 单位米
    """
    # 获取当前position (单位米)
    current_pos = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ])

    # 获取四元数 [x, y, z, w]
    quat = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])

    # 将偏移从毫米转换为米
    offset_m = np.array(axis_offset_mm) / 1000.0

    # 将四元数转换为旋转矩阵
    rotation_matrix = quaternion_to_rotation_matrix(quat)

    # 使用旋转矩阵变换偏移向量（从IMU局部坐标系转换到世界坐标系）
    world_offset = rotation_matrix @ offset_m

    # 计算新的position
    new_position = current_pos + world_offset

    return new_position


def similarity_calculation(a, b):
    # Relative Amplitude Difference
    rad = 0.
    for i in range(len(a)):
        sub = abs(a[i] - b[i])
        mean = (abs(a[i]) + abs(b[i])) / 2
        # sub over mean
        if mean != 0:
            rad += sub / mean

    rad /= len(a)
    if rad >= 1:
        rad = 1.

    # Correlation Coefficient
    cc = 0.
    if a != [0] * len(a) and b != [0] * len(b):
        cc = np.corrcoef(a, b)[0, 1]

    # Similarity
    cc_weight = 0.3
    sim = cc_weight * (cc + 1) / 2 + (1 - cc_weight) * (1 - rad)

    if sim == 0:
        sim = 0.01

    # print(f"cc: {cc}, rad: {rad}, sim: {sim}")
    return sim


# Derive the default config path from the package location so it works both
# from the source tree and from the install tree.
DEFAULT_CONFIG_PATH = (
    Path(__file__).parent / "brainco_hand" / "brainco.yml"
)


def _resolve_config_and_algorithm(config_file=None, algorithm=None):
    """Resolve config path and effective algorithm (CLI overrides YAML)."""
    config_path = Path(config_file or DEFAULT_CONFIG_PATH).expanduser()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    import yaml

    with config_path.open("r") as f:
        cfg = yaml.safe_load(f)

    cfg_algorithm = cfg.get("algorithm", "dex") if isinstance(cfg, dict) else "dex"
    resolved_algorithm = algorithm if algorithm is not None else cfg_algorithm
    return config_path, resolved_algorithm


def _is_dex_retargeter_algorithm(algorithm):
    return str(algorithm).lower().startswith("dex")


def _is_revo3_style_algorithm(algorithm):
    return str(algorithm).lower() in {"revo3_thumb", "joint_thumb"}


def _parse_six_float_vector(value, name):
    if value is None:
        return None
    if isinstance(value, str):
        parts = [item.strip() for item in value.split(",") if item.strip()]
    else:
        parts = list(value)
    if len(parts) != 6:
        raise ValueError(f"{name} must contain exactly 6 comma-separated values.")
    return np.asarray([float(item) for item in parts], dtype=float)


def _parse_joint_indices(value, name):
    if value is None:
        return None
    if isinstance(value, str):
        parts = [item.strip() for item in value.split(",") if item.strip()]
    else:
        parts = list(value)
    if not parts:
        raise ValueError(f"{name} must contain at least one joint index.")

    indices = []
    for item in parts:
        index = int(item)
        if index < 0 or index > 5:
            raise ValueError(f"{name} joint indices must be in [0, 5].")
        if index not in indices:
            indices.append(index)
    return tuple(indices)


def _cli_destinations_supplied(argv, parser):
    supplied = set()
    tokens = list(argv or [])
    for action in parser._actions:
        if not action.option_strings:
            continue
        for option in action.option_strings:
            if option in tokens or any(token.startswith(option + "=") for token in tokens):
                supplied.add(action.dest)
                break
    return supplied


def _nested_config_value(config, *paths):
    if not isinstance(config, dict):
        return None
    for path in paths:
        node = config
        found = True
        for key in path:
            if not isinstance(node, dict) or key not in node:
                found = False
                break
            node = node[key]
        if found:
            return node
    return None


def _normalize_revo3_thumb_params(params, hand_mode):
    if not params:
        return {}
    if not isinstance(params, dict):
        raise ValueError("revo3_thumb config must be a YAML mapping.")

    valid_sides = ("left", "right")
    enabled_sides = tuple(side for side in valid_sides if hand_mode in (side, "both"))
    side_params = {}
    shared = {}

    for key, value in params.items():
        if key in valid_sides:
            if value is None:
                continue
            if not isinstance(value, dict):
                raise ValueError(f"revo3_thumb.{key} must be a YAML mapping.")
            side_params[key] = {
                name: value[name]
                for name in REVO3_RUNTIME_PARAM_KEYS
                if name in value
            }
        elif key in REVO3_RUNTIME_PARAM_KEYS:
            shared[key] = value
        elif key in {"description", "comment", "notes"}:
            continue
        else:
            logger.warning("Ignoring unknown revo3_thumb parameter: %s", key)

    normalized = {}
    for side in enabled_sides:
        merged = dict(shared)
        merged.update(side_params.get(side, {}))
        if merged:
            normalized[side] = merged

    return normalized


def _merge_revo3_thumb_params(base_params, override_params):
    merged = {}
    for side in ("left", "right"):
        side_merged = {}
        if isinstance(base_params, dict) and isinstance(base_params.get(side), dict):
            side_merged.update(base_params[side])
        if isinstance(override_params, dict) and isinstance(override_params.get(side), dict):
            side_merged.update(override_params[side])
        if side_merged:
            merged[side] = side_merged
    return merged


_CONTROL_CONFIG_FIELDS = {
    "hand_mode": (("hand_mode",),),
    "skip_calibration": (("skip_calibration",),),
    "calibration_file": (("calibration_file",),),
    "use_default_revo3_calibration": (("use_default_revo3_calibration",),),
    "revo3_thumb_params": (("revo3_thumb",), ("revo3_thumb_params",)),
    "config_file": (("retarget_config_file",), ("config_file",)),
    "algorithm": (("algorithm",),),
    "end_effector": (("end_effector",),),
    "pd_speed_control": (("position_speed", "pd_speed_control"), ("pd_speed_control",)),
    "pd_kp": (("position_speed", "pd_kp"), ("pd_kp",)),
    "pd_kd": (("position_speed", "pd_kd"), ("pd_kd",)),
    "pd_derivative_alpha": (("position_speed", "pd_derivative_alpha"), ("pd_derivative_alpha",)),
    "target_filter_alpha": (("target_filter", "alpha"), ("pd_velocity", "target_filter_alpha"), ("target_filter_alpha",)),
    "target_filter_fast_alpha": (("target_filter", "fast_alpha"), ("pd_velocity", "target_filter_fast_alpha"), ("target_filter_fast_alpha",)),
    "target_filter_fast_threshold": (("target_filter", "fast_threshold"), ("pd_velocity", "target_filter_fast_threshold"), ("target_filter_fast_threshold",)),
    "dead_zone": (("dead_zone",),),
    "control_mode": (("control_mode",),),
    "velocity_kp": (("pd_velocity", "velocity_kp"), ("velocity_kp",)),
    "velocity_kd": (("pd_velocity", "velocity_kd"), ("velocity_kd",)),
    "velocity_deadband": (("pd_velocity", "velocity_deadband"), ("velocity_deadband",)),
    "thumb_velocity_deadband": (("pd_velocity", "thumb_velocity_deadband"), ("thumb_velocity_deadband",)),
    "thumb_velocity_min": (("pd_velocity", "thumb_velocity_min"), ("thumb_velocity_min",)),
    "thumb_velocity_kp_scale": (("pd_velocity", "thumb_velocity_kp_scale"), ("thumb_velocity_kp_scale",)),
    "thumb_velocity_brake_zone": (("pd_velocity", "thumb_velocity_brake_zone"), ("thumb_velocity_brake_zone",)),
    "thumb_velocity_brake_max": (("pd_velocity", "thumb_velocity_brake_max"), ("thumb_velocity_brake_max",)),
    "ring_velocity_deadband": (("pd_velocity", "ring_velocity_deadband"), ("ring_velocity_deadband",)),
    "ring_velocity_min": (("pd_velocity", "ring_velocity_min"), ("ring_velocity_min",)),
    "ring_velocity_kp_scale": (("pd_velocity", "ring_velocity_kp_scale"), ("ring_velocity_kp_scale",)),
    "four_finger_extension_velocity_scale": (
        ("pd_velocity", "four_finger_extension_velocity_scale"),
        ("four_finger_extension_velocity_scale",),
    ),
    "four_finger_extension_joints": (
        ("pd_velocity", "four_finger_extension_joints"),
        ("four_finger_extension_joints",),
    ),
    "velocity_max": (("pd_velocity", "velocity_max"), ("velocity_max",)),
    "velocity_slew_rate": (("pd_velocity", "velocity_slew_rate"), ("velocity_slew_rate",)),
    "velocity_feedback_timeout": (("pd_velocity", "velocity_feedback_timeout"), ("velocity_feedback_timeout",)),
    "feedback_position_scale": (("feedback", "position_scale"), ("motor_feedback", "position_scale"), ("feedback_position_scale",), ("motor_status_position_scale",)),
    "feedback_position_scales": (("feedback", "position_scales"), ("motor_feedback", "position_scales"), ("feedback_position_scales",), ("motor_status_position_scales",)),
    "feedback_position_offsets": (("feedback", "position_offsets"), ("motor_feedback", "position_offsets"), ("feedback_position_offsets",), ("motor_status_position_offsets",)),
    "pd_debug": (("debug", "pd_debug"), ("pd_debug",)),
    "pd_debug_joint": (("debug", "pd_debug_joint"), ("pd_debug_joint",)),
    "pd_debug_joints": (("debug", "pd_debug_joints"), ("pd_debug_joints",)),
    "pd_debug_interval": (("debug", "pd_debug_interval"), ("pd_debug_interval",)),
    "pd_debug_file": (("debug", "pd_debug_file"), ("pd_debug_file",)),
    "revo2_position_command_scale": (("revo2_driver", "position_command_scale"), ("revo2_position_command_scale",)),
    "revo2_velocity_command_scale": (("revo2_driver", "velocity_command_scale"), ("revo2_velocity_command_scale",)),
    "left_position_command_topic": (("revo2_driver", "left_position_command_topic"), ("left_position_command_topic",)),
    "right_position_command_topic": (("revo2_driver", "right_position_command_topic"), ("right_position_command_topic",)),
    "left_velocity_command_topic": (("revo2_driver", "left_velocity_command_topic"), ("left_velocity_command_topic",)),
    "right_velocity_command_topic": (("revo2_driver", "right_velocity_command_topic"), ("right_velocity_command_topic",)),
    "left_joint_state_topic": (("revo2_driver", "left_joint_state_topic"), ("left_joint_state_topic",)),
    "right_joint_state_topic": (("revo2_driver", "right_joint_state_topic"), ("right_joint_state_topic",)),
    "mirror_sim_output": (
        ("retarget_output", "publish_target_joint_states"),
        ("sim_debug", "mirror_sim_output"),
        ("mirror_sim_output",),
    ),
    "left_sim_command_topic": (
        ("retarget_output", "left_joint_state_topic"),
        ("sim_debug", "left_joint_topic"),
        ("left_sim_command_topic",),
    ),
    "right_sim_command_topic": (
        ("retarget_output", "right_joint_state_topic"),
        ("sim_debug", "right_joint_topic"),
        ("right_sim_command_topic",),
    ),
}


def _apply_control_config(args, argv, parser):
    control_config = getattr(args, "control_config", None)
    if not control_config:
        return

    import yaml

    config_path = _resolve_control_config_path(control_config)
    if not config_path.exists():
        raise FileNotFoundError(f"Control config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise ValueError(f"Control config must be a YAML mapping: {config_path}")

    supplied = _cli_destinations_supplied(argv, parser)
    for dest, paths in _CONTROL_CONFIG_FIELDS.items():
        if dest in supplied:
            continue
        value = _nested_config_value(config, *paths)
        if value is None:
            continue
        if dest in {"feedback_position_scales", "feedback_position_offsets"} and isinstance(value, (list, tuple)):
            value = ",".join(str(item) for item in value)
        setattr(args, dest, value)


def _resolve_control_config_path(control_config):
    config_path = Path(control_config).expanduser()
    if config_path.is_absolute():
        return config_path

    candidates = [Path.cwd() / config_path]
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = Path(get_package_share_directory("manus_revo2_retarget"))
        candidates.extend([
            package_share / config_path,
            package_share / "config" / config_path,
        ])
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return config_path


class ManusRevo2Node(Node):
    """单节点架构，避免多线程问题"""

    def __init__(
        self,
        hand_mode='both',
        skip_calibration=False,
        calibration_file=None,
        use_default_revo3_calibration=False,
        revo3_thumb_params=None,
        config_file=None,
        algorithm=None,
        end_effector_mode=None,
        pd_speed_control=False,
        pd_kp=2.0,
        pd_kd=0.002,
        pd_derivative_alpha=0.25,
        target_filter_alpha=0.35,
        target_filter_fast_alpha=None,
        target_filter_fast_threshold=0.0,
        dead_zone=0.003,
        control_mode="position_speed",
        velocity_kp=0.8,
        velocity_kd=0.01,
        velocity_deadband=0.01,
        thumb_velocity_deadband=None,
        thumb_velocity_min=0.0,
        thumb_velocity_kp_scale=1.0,
        thumb_velocity_brake_zone=0.0,
        thumb_velocity_brake_max=0.0,
        ring_velocity_deadband=None,
        ring_velocity_min=0.0,
        ring_velocity_kp_scale=1.0,
        four_finger_extension_velocity_scale=1.0,
        four_finger_extension_joints=None,
        velocity_max=1.2,
        velocity_slew_rate=0.2,
        velocity_feedback_timeout=0.3,
        feedback_position_scale=1.0,
        feedback_position_scales=None,
        feedback_position_offsets=None,
        revo2_position_command_scale=572.9577951308232,
        revo2_velocity_command_scale=1.0,
        left_position_command_topic="/revo2_left/joint_forward_pos_controller/commands",
        right_position_command_topic="/revo2_right/joint_forward_pos_controller/commands",
        left_velocity_command_topic="/revo2_left/joint_forward_vel_controller/commands",
        right_velocity_command_topic="/revo2_right/joint_forward_vel_controller/commands",
        left_joint_state_topic="/revo2_left/revo2_joint_state/joint_states",
        right_joint_state_topic="/revo2_right/revo2_joint_state/joint_states",
        mirror_sim_output=False,
        left_sim_command_topic="/revo2_left/retarget/joint_states",
        right_sim_command_topic="/revo2_right/retarget/joint_states",
        target_only=False,
        left_target_joint_state_topic="/revo2_left/revo2_pid_controller/target_joint_states",
        right_target_joint_state_topic="/revo2_right/revo2_pid_controller/target_joint_states",
        pd_debug=False,
        pd_debug_joint=2,
        pd_debug_joints=None,
        pd_debug_interval=0.2,
        pd_debug_file=None,
    ):
        self.config_path, algorithm = _resolve_config_and_algorithm(
            config_file=config_file,
            algorithm=algorithm,
        )
        node_name = "manus_revo2_controller_old" if _is_dex_retargeter_algorithm(algorithm) else "manus_revo2_controller"
        super().__init__(node_name)
        self.hand_mode = hand_mode.lower()
        if self.hand_mode not in VALID_HAND_MODES:
            raise ValueError(
                f"Invalid hand_mode: {self.hand_mode}, expected one of {sorted(VALID_HAND_MODES)}."
            )
        self.enable_left = self.hand_mode in ('left', 'both')
        self.enable_right = self.hand_mode in ('right', 'both')
        self.enabled_sides = tuple(
            side
            for side, enabled in (("left", self.enable_left), ("right", self.enable_right))
            if enabled
        )
        self.end_effector = (end_effector_mode or end_effector).lower()
        if self.end_effector not in VALID_END_EFFECTORS:
            raise ValueError(
                f"Invalid end_effector: {self.end_effector}, "
                f"expected one of {sorted(VALID_END_EFFECTORS)}."
            )
        self.control_mode = str(control_mode).lower()
        if self.control_mode not in VALID_CONTROL_MODES:
            raise ValueError(
                f"Invalid control_mode: {self.control_mode}, "
                f"expected one of {sorted(VALID_CONTROL_MODES)}."
            )
        if self.control_mode in {"pd_velocity", "pd_position_speed"} and self.end_effector != "real_hand":
            raise ValueError(f"{self.control_mode} control mode only supports real_hand output.")
        self.skip_calibration = skip_calibration
        self.use_default_revo3_calibration = use_default_revo3_calibration
        self.revo3_thumb_param_overrides = _normalize_revo3_thumb_params(
            revo3_thumb_params,
            self.hand_mode,
        )
        self.calibration_file = Path(calibration_file).expanduser() if calibration_file else None

        # Effective algorithm is already resolved by CLI/YAML at node init.
        logger.info(f"Loading retargeting algorithm: {algorithm} (config: {self.config_path})")

        # Instantiate retargeter via registry
        self.algorithm = algorithm
        self.hand_retargeting = RetargeterRegistry.create(
            algorithm,
            self.config_path,
            enabled_sides=self.enabled_sides,
        )

        logger.info(f"当前启动模式: {self.hand_mode}")

        # 手套数据
        self.finger_tip_node_ids = [4, 9, 14, 19, 24]
        self.thumb_dip_node_id = 3
        self.thumb_pip_node_id = 2
        self.left_finger_tip_pos = [[0, 0, 0] for _ in range(5)]
        self.right_finger_tip_pos = [[0, 0, 0] for _ in range(5)]
        self.left_thumb_dip_pos = None
        self.left_thumb_pip_pos = None
        self.right_thumb_dip_pos = None
        self.right_thumb_pip_pos = None
        self.left_ergonomics = {}
        self.right_ergonomics = {}
        # 仅在收到有效手套数据后才允许对应手下发电机命令，避免启动时默认值冲击
        self.left_data_ready = not self.enable_left
        self.right_data_ready = not self.enable_right
        self._glove_msg_counts = {"left": 0, "right": 0, "unknown": 0}
        self._last_glove_side = None
        self._last_glove_raw_node_count = 0
        self._last_glove_ergonomics_count = 0
        self._last_valid_tip_count_by_side = {"left": 0, "right": 0}
        self._last_missing_tip_ids_by_side = {"left": list(self.finger_tip_node_ids), "right": list(self.finger_tip_node_ids)}
        self._last_glove_status_log_ts = 0.0
        self._last_waiting_hand_log_ts = 0.0

        # 目标位置
        self.left_position = [0, 0, 0, 0, 0, 0]
        self.right_position = [0, 0, 0, 0, 0, 0]

        # 记录上一次的位置，用于比较
        self.last_left_position = [0, 0, 0, 0, 0, 0]
        self.last_right_position = [0, 0, 0, 0, 0, 0]

        # 五次多项式轨迹参数（参考 Revo3 仓库的 quintic trajectory 策略）
        self.quintic_duration = 0.003   # 缩短轨迹时长，降低平滑带来的体感延迟
        self.left_quintic = None        # dict: coeffs(6,6), t_start, t_end, p_end
        self.right_quintic = None
        self.left_interp = np.zeros((3, 6), dtype=float)   # [pos, vel, acc]
        self.right_interp = np.zeros((3, 6), dtype=float)
        self.last_left_target = np.zeros(6, dtype=float)
        self.last_right_target = np.zeros(6, dtype=float)
        self.replan_threshold = 0.003   # 目标变化超过该阈值(rad)才重规划轨迹
        self.dead_zone = float(max(0.0, dead_zone))
        self.target_filter_alpha = float(np.clip(target_filter_alpha, 0.0, 1.0))
        if target_filter_fast_alpha is None:
            self.target_filter_fast_alpha = self.target_filter_alpha
        else:
            self.target_filter_fast_alpha = float(np.clip(target_filter_fast_alpha, 0.0, 1.0))
        self.target_filter_fast_alpha = max(
            self.target_filter_alpha,
            self.target_filter_fast_alpha,
        )
        self.target_filter_fast_threshold = float(max(0.0, target_filter_fast_threshold))
        self.left_filtered_target = np.zeros(6, dtype=float)
        self.right_filtered_target = np.zeros(6, dtype=float)
        self.left_filter_initialized = False
        self.right_filter_initialized = False
        self.speed_gain = 4.0           # 按当前位置差值放大速度，保证大位移快速响应
        self.pd_speed_control = pd_speed_control
        self.pd_kp = float(pd_kp)
        self.pd_kd = float(pd_kd)
        self.pd_derivative_alpha = float(np.clip(pd_derivative_alpha, 0.0, 1.0))
        self.last_left_error = np.zeros(6, dtype=float)
        self.last_right_error = np.zeros(6, dtype=float)
        self.left_error_derivative = np.zeros(6, dtype=float)
        self.right_error_derivative = np.zeros(6, dtype=float)
        self.last_left_pd_time = None
        self.last_right_pd_time = None
        # 电机速度限制：仅拇指侧摆关节单独设置，其余（含拇指另一个关节）共用四指参数
        self.thumb_splay_joint_index = 0
        self.thumb_splay_speed_min = 200.0
        self.thumb_splay_speed_max = 900.0
        self.finger_speed_min = 350.0
        self.finger_speed_max = 1000.0
        self.revo2_joint_suffixes = REVO2_JOINT_SUFFIXES
        self.revo2_joint_limits_rad = REVO2_JOINT_LIMITS_RAD
        self.revo2_joint_upper_limits_rad = np.asarray(REVO2_JOINT_UPPER_LIMITS_RAD, dtype=float)

        self.velocity_kp = float(velocity_kp)
        self.velocity_kd = float(velocity_kd)
        self.velocity_deadband = float(max(0.0, velocity_deadband))
        if thumb_velocity_deadband is None:
            self.thumb_velocity_deadband = self.velocity_deadband
        else:
            self.thumb_velocity_deadband = float(max(0.0, thumb_velocity_deadband))
        self.thumb_velocity_min = float(max(0.0, thumb_velocity_min))
        self.thumb_velocity_kp_scale = float(max(0.0, thumb_velocity_kp_scale))
        self.thumb_velocity_brake_zone = float(max(0.0, thumb_velocity_brake_zone))
        self.thumb_velocity_brake_max = float(max(0.0, thumb_velocity_brake_max))
        if ring_velocity_deadband is None:
            self.ring_velocity_deadband = self.velocity_deadband
        else:
            self.ring_velocity_deadband = float(max(0.0, ring_velocity_deadband))
        self.ring_velocity_min = float(max(0.0, ring_velocity_min))
        self.ring_velocity_kp_scale = float(max(0.0, ring_velocity_kp_scale))
        self.four_finger_extension_velocity_scale = float(
            max(0.0, four_finger_extension_velocity_scale)
        )
        parsed_extension_joints = _parse_joint_indices(
            four_finger_extension_joints,
            "four_finger_extension_joints",
        )
        self.four_finger_extension_joints = (
            parsed_extension_joints if parsed_extension_joints is not None else (2, 3, 4, 5)
        )
        self.velocity_max = float(max(0.0, velocity_max))
        self.velocity_slew_rate = float(max(0.0, velocity_slew_rate))
        self.velocity_zero_epsilon = min(0.002, max(1e-6, self.velocity_max * 1e-3))
        self.velocity_feedback_timeout = float(max(0.01, velocity_feedback_timeout))
        self.feedback_position_scale = float(feedback_position_scale)
        self.feedback_position_scales = _parse_six_float_vector(
            feedback_position_scales,
            "feedback_position_scales",
        )
        if self.feedback_position_scales is None:
            self.feedback_position_scales = np.full(
                6, self.feedback_position_scale, dtype=float
            )
        self.feedback_position_offsets = _parse_six_float_vector(
            feedback_position_offsets,
            "feedback_position_offsets",
        )
        if self.feedback_position_offsets is None:
            self.feedback_position_offsets = np.zeros(6, dtype=float)
        self.left_position_command_topic = str(left_position_command_topic)
        self.right_position_command_topic = str(right_position_command_topic)
        self.left_velocity_command_topic = str(left_velocity_command_topic)
        self.right_velocity_command_topic = str(right_velocity_command_topic)
        self.left_joint_state_topic = str(left_joint_state_topic)
        self.right_joint_state_topic = str(right_joint_state_topic)
        self.mirror_sim_output = bool(mirror_sim_output)
        self.left_sim_command_topic = str(left_sim_command_topic)
        self.right_sim_command_topic = str(right_sim_command_topic)
        self.target_only = bool(target_only)
        self.left_target_joint_state_topic = str(left_target_joint_state_topic)
        self.right_target_joint_state_topic = str(right_target_joint_state_topic)
        if self.control_mode in {"pd_velocity", "pd_position_speed"}:
            logger.info(
                f"{self.control_mode} Revo2 velocity command contract: "
                f"Float64MultiArray order=[{command_order_label()}], unit=rad/s"
            )
            logger.info(
                f"{self.control_mode} Revo2 joint limits rad: "
                f"{[(round(lo, 4), round(hi, 4)) for lo, hi in self.revo2_joint_limits_rad]}"
            )
            logger.info(
                f"{self.control_mode} target/feedback contract: "
                "retarget target=rad, JointState feedback=rad, command velocity=rad/s"
            )
            if self.control_mode == "pd_position_speed":
                logger.info(
                    "pd_position_speed publishes target position + positive speed percentage: "
                    f"left_pos={self.left_position_command_topic}, "
                    f"left_speed={self.left_velocity_command_topic}, "
                    f"right_pos={self.right_position_command_topic}, "
                    f"right_speed={self.right_velocity_command_topic}"
                )
            logger.info(
                f"{self.control_mode} feedback correction in rad: scales="
                f"{np.round(self.feedback_position_scales, 4).tolist()}, "
                f"offsets={np.round(self.feedback_position_offsets, 5).tolist()}"
            )
            logger.info(
                f"{self.control_mode} thumb tuning in rad/rad_s: deadband="
                f"{self.thumb_velocity_deadband:.5f}, "
                f"min_speed={self.thumb_velocity_min:.5f}, "
                f"kp_scale={self.thumb_velocity_kp_scale:.2f}, "
                f"brake_zone={self.thumb_velocity_brake_zone:.5f}, "
                f"brake_max={self.thumb_velocity_brake_max:.5f}"
            )
            logger.info(
                f"{self.control_mode} ring tuning in rad/rad_s: deadband="
                f"{self.ring_velocity_deadband:.5f}, "
                f"min_speed={self.ring_velocity_min:.5f}, "
                f"kp_scale={self.ring_velocity_kp_scale:.2f}"
            )
            logger.info(
                f"{self.control_mode} extension tuning: negative velocity scale="
                f"{self.four_finger_extension_velocity_scale:.2f}, "
                f"joints={self.four_finger_extension_joints}"
            )
        self.left_actual_raw_position = np.zeros(6, dtype=float)
        self.right_actual_raw_position = np.zeros(6, dtype=float)
        self.left_actual_position = np.zeros(6, dtype=float)
        self.right_actual_position = np.zeros(6, dtype=float)
        self.left_feedback_ready = not self.enable_left
        self.right_feedback_ready = not self.enable_right
        self.left_feedback_seq = 0
        self.right_feedback_seq = 0
        self.left_feedback_time = None
        self.right_feedback_time = None
        self.last_left_feedback_seq_used = -1
        self.last_right_feedback_seq_used = -1
        self.last_left_velocity_error = np.zeros(6, dtype=float)
        self.last_right_velocity_error = np.zeros(6, dtype=float)
        self.left_velocity_derivative = np.zeros(6, dtype=float)
        self.right_velocity_derivative = np.zeros(6, dtype=float)
        self.last_left_velocity_pd_time = None
        self.last_right_velocity_pd_time = None
        self.left_target_velocity = np.zeros(6, dtype=float)
        self.right_target_velocity = np.zeros(6, dtype=float)
        self.left_command_velocity = np.zeros(6, dtype=float)
        self.right_command_velocity = np.zeros(6, dtype=float)
        self._last_feedback_warn_ts = 0.0
        self.pd_debug = bool(pd_debug)
        parsed_debug_joints = _parse_joint_indices(pd_debug_joints, "pd_debug_joints")
        if parsed_debug_joints is None:
            parsed_debug_joints = (int(np.clip(pd_debug_joint, 0, 5)),)
        self.pd_debug_joints = parsed_debug_joints
        self.pd_debug_joint = self.pd_debug_joints[0]
        self.pd_debug_interval = float(max(0.02, pd_debug_interval))
        self.pd_debug_file = Path(pd_debug_file).expanduser() if pd_debug_file else None
        self._last_pd_debug_ts = 0.0
        self.direct_follow = True       # 直跟模式：不过轨迹段，直接跟随最新目标
        self.enable_debug_print = False
        self.enable_timing_print = False
        self.debug_print_interval = 0.2
        self._last_debug_print_ts = 0.0

        # 采集大拇指校准位置标志位
        self.record_thumb_calibration_flag = True
        # 用来校准大拇指位置的手套数据
        self.thumb_open_rotate_tip_pos = [[0, 0, 0] for _ in range(4)]
        self.revo3_calibration = None
        # 相似度数组
        self.thumb_sim_value = [0] * 4

        # 重定向处理器 (已在上方初始化)

        # 始终监听两个手套话题，并按 msg.side 分流，兼容单手时 topic 索引变化
        self.manus_glove_sub_0 = self.create_subscription(
            ManusGlove, "/manus_glove_0", self.glove_callback, 10
        )
        self.manus_glove_sub_1 = self.create_subscription(
            ManusGlove, "/manus_glove_1", self.glove_callback, 10
        )

        # 发布控制命令
        self.left_target_publisher = None
        self.right_target_publisher = None
        if self.target_only:
            if self.enable_left:
                self.left_target_publisher = self.create_publisher(
                    JointState, self.left_target_joint_state_topic, 10
                )
                logger.info(f"Left retarget target topic: {self.left_target_joint_state_topic}")
            if self.enable_right:
                self.right_target_publisher = self.create_publisher(
                    JointState, self.right_target_joint_state_topic, 10
                )
                logger.info(f"Right retarget target topic: {self.right_target_joint_state_topic}")

        self.left_motor_publisher = None
        self.right_motor_publisher = None
        self.left_speed_publisher = None
        self.right_speed_publisher = None
        self.left_sim_publisher = None
        self.right_sim_publisher = None
        self.left_thumb_ik_target_debug_publisher = None
        self.right_thumb_ik_target_debug_publisher = None
        if self.enable_left:
            self.left_thumb_ik_target_debug_publisher = self.create_publisher(
                PointStamped, "/revo2_left/retarget/debug/thumb_ik_target", 10
            )
            logger.info("Left thumb IK debug target topic: /revo2_left/retarget/debug/thumb_ik_target")
        if self.enable_right:
            self.right_thumb_ik_target_debug_publisher = self.create_publisher(
                PointStamped, "/revo2_right/retarget/debug/thumb_ik_target", 10
            )
            logger.info("Right thumb IK debug target topic: /revo2_right/retarget/debug/thumb_ik_target")
        if self.target_only:
            logger.info("Target-only mode: Revo2 command publishing is disabled.")
        elif self.end_effector == 'real_hand':
            if self.control_mode not in {"pd_velocity", "pd_position_speed"}:
                raise ValueError(
                    "real_hand output in this repository is wired to revo2_driver "
                    "PD speed control and requires --control-mode pd_velocity or pd_position_speed."
                )
            if self.enable_left:
                if self.control_mode == "pd_position_speed":
                    self.left_motor_publisher = self.create_publisher(
                        Float64MultiArray, self.left_position_command_topic, 10
                    )
                    self.left_speed_publisher = self.create_publisher(
                        Float64MultiArray, self.left_velocity_command_topic, 10
                    )
                    logger.info(f"Left Revo2 position command topic: {self.left_position_command_topic}")
                    logger.info(f"Left Revo2 speed command topic: {self.left_velocity_command_topic}")
                else:
                    self.left_motor_publisher = self.create_publisher(
                        Float64MultiArray, self.left_velocity_command_topic, 10
                    )
                    logger.info(f"Left Revo2 velocity command topic: {self.left_velocity_command_topic}")
            if self.enable_right:
                if self.control_mode == "pd_position_speed":
                    self.right_motor_publisher = self.create_publisher(
                        Float64MultiArray, self.right_position_command_topic, 10
                    )
                    self.right_speed_publisher = self.create_publisher(
                        Float64MultiArray, self.right_velocity_command_topic, 10
                    )
                    logger.info(f"Right Revo2 position command topic: {self.right_position_command_topic}")
                    logger.info(f"Right Revo2 speed command topic: {self.right_velocity_command_topic}")
                else:
                    self.right_motor_publisher = self.create_publisher(
                        Float64MultiArray, self.right_velocity_command_topic, 10
                    )
                    logger.info(f"Right Revo2 velocity command topic: {self.right_velocity_command_topic}")
            if self.mirror_sim_output:
                if self.enable_left:
                    self.left_sim_publisher = self.create_publisher(
                        JointState, self.left_sim_command_topic, 10
                    )
                    logger.info(f"Left retarget target JointState topic: {self.left_sim_command_topic}")
                if self.enable_right:
                    self.right_sim_publisher = self.create_publisher(
                        JointState, self.right_sim_command_topic, 10
                    )
                    logger.info(f"Right retarget target JointState topic: {self.right_sim_command_topic}")
        elif self.end_effector == 'sim_hand':
            if self.enable_left:
                self.left_motor_publisher = self.create_publisher(
                    JointState, self.left_sim_command_topic, 10
                )
            if self.enable_right:
                self.right_motor_publisher = self.create_publisher(
                    JointState, self.right_sim_command_topic, 10
                )

        self.feedback_subscribers = []
        if (
            not self.target_only
            and self.end_effector == 'real_hand'
            and self.control_mode in {"pd_velocity", "pd_position_speed"}
        ):
            if self.enable_left:
                self.feedback_subscribers.append(
                    self.create_subscription(
                        JointState,
                        self.left_joint_state_topic,
                        lambda msg: self.joint_state_feedback_callback(msg, "left"),
                        10,
                    )
                )
                logger.info(f"Left Revo2 joint feedback topic: {self.left_joint_state_topic}")
            if self.enable_right:
                self.feedback_subscribers.append(
                    self.create_subscription(
                        JointState,
                        self.right_joint_state_topic,
                        lambda msg: self.joint_state_feedback_callback(msg, "right"),
                        10,
                    )
                )
                logger.info(f"Right Revo2 joint feedback topic: {self.right_joint_state_topic}")

        # 定时器：重定向计算
        self.retarget_timer = self.create_timer(0.01, self.retarget_callback)
        # 定时器：大拇指相似度计算
        if self.skip_calibration:
            if _is_revo3_style_algorithm(self.algorithm):
                if self.use_default_revo3_calibration:
                    params_by_side = {}
                    logger.info("已跳过标定文件，使用 revo3 默认参数进入控制模式")
                else:
                    params_by_side = self.load_revo3_calibration_data()
                params_by_side = _merge_revo3_thumb_params(
                    params_by_side,
                    self.revo3_thumb_param_overrides,
                )
                if not hasattr(self.hand_retargeting, "apply_calibration"):
                    raise RuntimeError("revo3_thumb retargeter 缺少 apply_calibration 接口")
                self.hand_retargeting.apply_calibration(params_by_side)
                if self.revo3_thumb_param_overrides:
                    logger.info(
                        "已应用 revo3 风格拇指 YAML 参数覆盖: %s",
                        self.revo3_thumb_param_overrides,
                    )
                self.record_thumb_calibration_flag = False
                if not self.use_default_revo3_calibration:
                    logger.info("已跳过标定，加载 revo3 标定参数进入控制模式")
            else:
                if self.use_default_revo3_calibration:
                    logger.warning("--use-default-revo3-calibration 只对 revo3_thumb 算法生效，当前算法将继续读取标定文件")
                self.load_thumb_calibration_data()
                self.record_thumb_calibration_flag = False
                logger.info("已跳过标定，加载标定数据进入控制模式")
        else:
            thread_record_thumb_calibration_data = threading.Thread(target=self.record_thumb_calibration_data)
            thread_record_thumb_calibration_data.daemon = True
            thread_record_thumb_calibration_data.start()
        if not self.target_only:
            self.control_timer = self.create_timer(0.01, self.thumb_sim_callback)
            # 定时器：电机控制
            self.control_timer = self.create_timer(0.01, self.control_callback)

    def _update_finger_tip_pos(self, msg: ManusGlove, target_finger_tip_pos, thumb_dip_store, thumb_pip_store, hand: str = ""):
        updated_tip_indices = set()
        try:
            for tip_node in msg.raw_nodes:
                pose = tip_node.pose
                node_id = tip_node.node_id

                if node_id in self.finger_tip_node_ids:
                    idx = self.finger_tip_node_ids.index(node_id)
                    if idx == 0:
                        axis_offset_mm = [0, 0, 0]
                    elif idx == 1:
                        axis_offset_mm = [0, 0, 0]
                    else:
                        axis_offset_mm = [0, 0, 0]
                    new_pos = get_offset_position(pose, axis_offset_mm)
                    if idx == 0:
                        new_pos += np.array([0, 0, 0]) * 0.001
                    pos = [-new_pos[1], -new_pos[0], new_pos[2]]
                    target_finger_tip_pos[idx] = pos
                    updated_tip_indices.add(idx)
                elif node_id == self.thumb_dip_node_id:
                    new_pos = get_offset_position(pose, [0, 0, 0])
                    thumb_dip_store[0] = [-new_pos[1], -new_pos[0], new_pos[2]]
                elif node_id == self.thumb_pip_node_id:
                    new_pos = get_offset_position(pose, [0, 0, 0])
                    thumb_pip_store[0] = [-new_pos[1], -new_pos[0], new_pos[2]]
        except Exception as e:
            logger.error(f"处理手套数据时出错: {e}")
            return False

        if hand in self._last_valid_tip_count_by_side:
            self._last_valid_tip_count_by_side[hand] = len(updated_tip_indices)
            self._last_missing_tip_ids_by_side[hand] = [
                node_id
                for idx, node_id in enumerate(self.finger_tip_node_ids)
                if idx not in updated_tip_indices
            ]

        if not updated_tip_indices:
            return False

        return any(
            np.linalg.norm(target_finger_tip_pos[idx]) > 1e-6
            for idx in updated_tip_indices
        )

    def _ergonomics_dict(self, msg: ManusGlove):
        ergonomics = {}
        for ergo in msg.ergonomics:
            ergonomics[str(ergo.type)] = float(ergo.value)
        return ergonomics

    def _mark_hand_data_ready(self, hand: str):
        if hand == "left" and not self.left_data_ready:
            self.left_data_ready = True
            logger.info("检测到左手手套有效数据，开始发送左手控制。")
        elif hand == "right" and not self.right_data_ready:
            self.right_data_ready = True
            logger.info("检测到右手手套有效数据，开始发送右手控制。")

    def _record_glove_status(self, msg: ManusGlove, side: str):
        count_key = side if side in ("left", "right") else "unknown"
        self._glove_msg_counts[count_key] += 1
        self._last_glove_side = side or "(empty)"
        self._last_glove_raw_node_count = len(msg.raw_nodes)
        self._last_glove_ergonomics_count = len(msg.ergonomics)

        now = time.time()
        total_count = sum(self._glove_msg_counts.values())
        if total_count == 1 or now - self._last_glove_status_log_ts >= 5.0:
            self._last_glove_status_log_ts = now
            self.get_logger().info(
                "MANUS glove msg: "
                f"side={self._last_glove_side} "
                f"raw_nodes={self._last_glove_raw_node_count} "
                f"ergonomics={self._last_glove_ergonomics_count} "
                f"counts={self._glove_msg_counts}"
            )

    def _log_waiting_for_hand_data(self):
        now = time.time()
        if now - self._last_waiting_hand_log_ts < 2.0:
            return
        self._last_waiting_hand_log_ts = now
        self.get_logger().warning(
            "Waiting for valid MANUS data before publishing target. "
            f"enabled=(left:{self.enable_left} right:{self.enable_right}), "
            f"ready=(left:{self.left_data_ready} right:{self.right_data_ready}), "
            f"glove_counts={self._glove_msg_counts}, "
            f"last_side={self._last_glove_side}, "
            f"last_raw_nodes={self._last_glove_raw_node_count}, "
            f"last_ergonomics={self._last_glove_ergonomics_count}, "
            f"right_tip_count={self._last_valid_tip_count_by_side['right']}, "
            f"right_missing_tip_node_ids={self._last_missing_tip_ids_by_side['right']}"
        )

    def _coerce_revo2_vector(self, values, label: str):
        array = np.asarray(values, dtype=float).reshape(-1)
        if array.size != len(self.revo2_joint_suffixes):
            raise ValueError(
                f"{label} must have {len(self.revo2_joint_suffixes)} values, got {array.size}"
            )
        return np.nan_to_num(array, nan=0.0, posinf=0.0, neginf=0.0)

    def _joint_positions_rad(self, target_positions):
        target_rad = self._coerce_revo2_vector(target_positions, "Revo2 target position")
        return np.clip(target_rad, 0.0, self.revo2_joint_upper_limits_rad)

    def _velocity_command_msg(self, command_velocity_rad_s):
        velocity = self._coerce_revo2_vector(command_velocity_rad_s, "Revo2 velocity command")
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocity]
        return msg

    def _joint_state_to_rad_positions(self, msg: JointState, side: str):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        raw_positions = np.zeros(6, dtype=float)
        prefix = "left" if side == "left" else "right"
        for i, suffix in enumerate(self.revo2_joint_suffixes):
            candidates = (f"{prefix}_{suffix}", suffix)
            index = next((name_to_index[name] for name in candidates if name in name_to_index), None)
            if index is None or index >= len(msg.position):
                raise ValueError(
                    f"{side} JointState missing joint {prefix}_{suffix}; names={list(msg.name)}"
                )
            raw_positions[i] = float(msg.position[index])
        corrected_positions = raw_positions * self.feedback_position_scales + self.feedback_position_offsets
        return raw_positions, corrected_positions

    def joint_state_feedback_callback(self, msg: JointState, side: str):
        """Cache revo2_driver JointState feedback for PD speed modes."""
        try:
            actual_raw_position, actual_position = self._joint_state_to_rad_positions(msg, side)
        except Exception as e:
            now = time.time()
            if now - self._last_feedback_warn_ts > 1.0:
                self._last_feedback_warn_ts = now
                logger.warning(f"Failed to parse {side} Revo2 JointState feedback: {e}")
            return

        now = time.time()
        if side == "left":
            self.left_actual_raw_position = actual_raw_position
            self.left_actual_position = actual_position
            self.left_feedback_seq += 1
            self.left_feedback_time = now
            if not self.left_feedback_ready:
                self.left_feedback_ready = True
                logger.info("收到左手 revo2_driver JointState，PD 速度闭环已启用。")
        elif side == "right":
            self.right_actual_raw_position = actual_raw_position
            self.right_actual_position = actual_position
            self.right_feedback_seq += 1
            self.right_feedback_time = now
            if not self.right_feedback_ready:
                self.right_feedback_ready = True
                logger.info("收到右手 revo2_driver JointState，PD 速度闭环已启用。")

    def glove_callback(self, msg: ManusGlove):
        """按手套消息中的 side 字段分流左右手数据"""
        t0 = time.time() if self.enable_timing_print else None
        side = str(msg.side).strip().lower()
        self._record_glove_status(msg, side)
        if side == "left":
            if self.enable_left:
                self.left_ergonomics = self._ergonomics_dict(msg)
                left_dip = [None]
                left_pip = [None]
                if self._update_finger_tip_pos(msg, self.left_finger_tip_pos, left_dip, left_pip, "left"):
                    self._mark_hand_data_ready("left")
                if left_dip[0] is not None:
                    self.left_thumb_dip_pos = left_dip[0]
                if left_pip[0] is not None:
                    self.left_thumb_pip_pos = left_pip[0]
        elif side == "right":
            if self.enable_right:
                self.right_ergonomics = self._ergonomics_dict(msg)
                right_dip = [None]
                right_pip = [None]
                if self._update_finger_tip_pos(msg, self.right_finger_tip_pos, right_dip, right_pip, "right"):
                    self._mark_hand_data_ready("right")
                if right_dip[0] is not None:
                    self.right_thumb_dip_pos = right_dip[0]
                if right_pip[0] is not None:
                    self.right_thumb_pip_pos = right_pip[0]
        else:
            # side 不可用时，为单手模式提供兜底
            if self.enable_left and not self.enable_right:
                self.left_ergonomics = self._ergonomics_dict(msg)
                left_dip = [None]
                left_pip = [None]
                if self._update_finger_tip_pos(msg, self.left_finger_tip_pos, left_dip, left_pip, "left"):
                    self._mark_hand_data_ready("left")
                if left_dip[0] is not None:
                    self.left_thumb_dip_pos = left_dip[0]
                if left_pip[0] is not None:
                    self.left_thumb_pip_pos = left_pip[0]
            elif self.enable_right and not self.enable_left:
                self.right_ergonomics = self._ergonomics_dict(msg)
                right_dip = [None]
                right_pip = [None]
                if self._update_finger_tip_pos(msg, self.right_finger_tip_pos, right_dip, right_pip, "right"):
                    self._mark_hand_data_ready("right")
                if right_dip[0] is not None:
                    self.right_thumb_dip_pos = right_dip[0]
                if right_pip[0] is not None:
                    self.right_thumb_pip_pos = right_pip[0]
        if self.enable_timing_print:
            dt = time.time() - t0
            if dt > 0.005:
                print(f"[TIMING] glove_callback: {dt*1000:.1f} ms")

    def _publish_sim_mirror(self, positions, side: str):
        if not self.mirror_sim_output:
            return
        if side == "left":
            publisher = self.left_sim_publisher
            names = short_joint_names_for_side("left")
        elif side == "right":
            publisher = self.right_sim_publisher
            names = short_joint_names_for_side("right")
        else:
            return
        if publisher is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(c) for c in positions]
        publisher.publish(msg)

    def _publish_target_joint_state(self, positions, side: str):
        if not self.target_only or self.record_thumb_calibration_flag:
            return
        if side == "left":
            publisher = self.left_target_publisher
        elif side == "right":
            publisher = self.right_target_publisher
        else:
            return
        if publisher is None:
            return

        target = self._filter_joint_target(positions, side)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names_for_side(side))
        msg.position = [float(c) for c in target]
        publisher.publish(msg)

    def _publish_thumb_ik_debug_target(self, side: str):
        if side == "left":
            publisher = self.left_thumb_ik_target_debug_publisher
        elif side == "right":
            publisher = self.right_thumb_ik_target_debug_publisher
        else:
            return
        if publisher is None:
            return

        getter = getattr(self.hand_retargeting, "get_thumb_debug_targets", None)
        if not callable(getter):
            return
        targets = getter(side) or {}
        point = targets.get("thumb_tip_filtered")
        if point is None:
            return
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            return

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"revo2_{side}_retarget"
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = float(point[2])
        publisher.publish(msg)

    def retarget_callback(self):
        """重定向计算回调"""
        if not (
            (self.enable_left and self.left_data_ready)
            or (self.enable_right and self.right_data_ready)
        ):
            self._log_waiting_for_hand_data()
            return

        t0 = time.time() if self.enable_timing_print else None
        try:
            t1 = time.time() if self.enable_timing_print else None
            [left_target, right_target] = self.hand_retargeting.retarget_process(
                self.left_finger_tip_pos,
                self.right_finger_tip_pos,
                left_thumb_dip_pos=self.left_thumb_dip_pos,
                left_thumb_pip_pos=self.left_thumb_pip_pos,
                right_thumb_dip_pos=self.right_thumb_dip_pos,
                right_thumb_pip_pos=self.right_thumb_pip_pos,
                left_ergonomics=self.left_ergonomics,
                right_ergonomics=self.right_ergonomics,
            )
            t2 = time.time() if self.enable_timing_print else None

            # 直接更新位置（无需锁，单线程安全）
            self.left_position = left_target
            self.right_position = right_target
            if self.enable_left and self.left_data_ready:
                self._publish_thumb_ik_debug_target("left")
            if self.enable_right and self.right_data_ready:
                self._publish_thumb_ik_debug_target("right")
            if self.target_only:
                if self.enable_left and self.left_data_ready:
                    self._publish_target_joint_state(left_target, "left")
                if self.enable_right and self.right_data_ready:
                    self._publish_target_joint_state(right_target, "right")

            if self.enable_timing_print:
                dt_total = time.time() - t0
                dt_retarget = t2 - t1
                if dt_total > 0.005 or dt_retarget > 0.005:
                    print(f"[TIMING] retarget_callback total: {dt_total*1000:.1f} ms, retarget_process: {dt_retarget*1000:.1f} ms")
        except Exception as e:
            logger.error(f"重定向处理时出错: {e}")

    def control_callback(self):
        """控制回调"""
        if not self.record_thumb_calibration_flag:
            if not (
                (self.enable_left and self.left_data_ready)
                or (self.enable_right and self.right_data_ready)
            ):
                return

            t_ctrl_0 = time.time() if self.enable_timing_print else None
            try:
                # 旧算法 dex_vector 需要靠标定+相似度混合来修补拇指旋转；
                # revo3 风格算法已经自带 MuJoCo IK，跳过相似度覆盖。
                if not _is_revo3_style_algorithm(self.algorithm):
                    sim_threshold = 0.8
                    # left hand
                    if (
                        self.enable_left
                        and self.left_data_ready
                        and self.thumb_sim_value[1] > sim_threshold
                    ):
                        coefficient_retarget = (1 - self.thumb_sim_value[1]) / (1 - sim_threshold)
                        self.left_position[0] = self.left_position[0] * coefficient_retarget
                        self.left_position[1] = (
                            self.revo2_joint_upper_limits_rad[1] * (1 - coefficient_retarget)
                            + self.left_position[1] * coefficient_retarget
                        )
                    # right hand
                    if (
                        self.enable_right
                        and self.right_data_ready
                        and self.thumb_sim_value[3] > sim_threshold
                    ):
                        coefficient_retarget = (1 - self.thumb_sim_value[3]) / (1 - sim_threshold)
                        self.right_position[0] = self.right_position[0] * coefficient_retarget
                        self.right_position[1] = (
                            self.revo2_joint_upper_limits_rad[1] * (1 - coefficient_retarget)
                            + self.right_position[1] * coefficient_retarget
                        )

                now = time.time()

                # ---- 左手五次多项式轨迹生成与评估 ----
                if self.enable_left and self.left_data_ready:
                    left_target = self._filter_joint_target(self.left_position, "left")
                    if self.direct_follow:
                        pos = left_target.copy()
                        vel = np.zeros(6)
                        acc = np.zeros(6)
                        self.left_interp = np.stack([pos, vel, acc])
                        left_positions = left_target.copy()
                    else:
                        # 目标变化超过阈值才重新规划轨迹，避免噪声导致频繁重规划
                        if np.max(np.abs(left_target - self.last_left_target)) > self.replan_threshold:
                            p0 = self.left_interp[0].copy()
                            v0 = self.left_interp[1].copy()
                            a0 = self.left_interp[2].copy()
                            self.left_quintic = self._make_quintic_segment(
                                p0, v0, a0, left_target,
                                v0 * 0.3, np.zeros(6),
                                now, self.quintic_duration,
                            )
                            self.last_left_target = left_target.copy()

                        if self.left_quintic is not None:
                            pos, vel, acc = self._eval_quintic(self.left_quintic, now)
                            if now >= self.left_quintic['t_end']:
                                pos = self.left_quintic['p_end'].copy()
                                vel = np.zeros(6)
                                acc = np.zeros(6)
                            self.left_interp = np.stack([pos, vel, acc])
                            left_positions = pos.copy()
                        else:
                            left_positions = left_target.copy()

                    if self.end_effector == 'real_hand' and self.control_mode in {"pd_velocity", "pd_position_speed"}:
                        self._publish_sim_mirror(left_positions, "left")
                        if self.control_mode == "pd_position_speed":
                            self._publish_pd_position_speed_command(left_positions, "left", now)
                        else:
                            self._publish_pd_velocity_command(left_positions, "left", now)
                        self.last_left_position = left_positions.copy()
                    else:
                        left_changed = any(
                            abs(left_positions[i] - self.last_left_position[i]) >= self.dead_zone
                            for i in range(6)
                        )

                        if left_changed:
                            if self.pd_speed_control:
                                left_speeds = self._compute_pd_motor_speeds(
                                    left_positions,
                                    self.last_left_position,
                                    "left",
                                    now,
                                )
                            else:
                                # 速度按本次指令位移计算；位移越大速度越高，避免“大位移却低速”
                                left_delta = [
                                    abs(left_positions[i] - self.last_left_position[i])
                                    for i in range(6)
                                ]
                                left_speeds = self._compute_motor_speeds(left_delta)

                            if self.end_effector == 'sim_hand':
                                left_msg = JointState()
                                left_msg.header.stamp = self.get_clock().now().to_msg()
                                left_msg.name = list(short_joint_names_for_side("left"))
                                left_msg.position = [float(c) for c in left_positions]

                                self.left_motor_publisher.publish(left_msg)
                                self.last_left_position = left_positions.copy()

                # ---- 右手五次多项式轨迹生成与评估 ----
                if self.enable_right and self.right_data_ready:
                    right_target = self._filter_joint_target(self.right_position, "right")
                    if self.direct_follow:
                        pos = right_target.copy()
                        vel = np.zeros(6)
                        acc = np.zeros(6)
                        self.right_interp = np.stack([pos, vel, acc])
                        right_positions = right_target.copy()
                    else:
                        if np.max(np.abs(right_target - self.last_right_target)) > self.replan_threshold:
                            p0 = self.right_interp[0].copy()
                            v0 = self.right_interp[1].copy()
                            a0 = self.right_interp[2].copy()
                            self.right_quintic = self._make_quintic_segment(
                                p0, v0, a0, right_target,
                                v0 * 0.3, np.zeros(6),
                                now, self.quintic_duration,
                            )
                            self.last_right_target = right_target.copy()

                        if self.right_quintic is not None:
                            pos, vel, acc = self._eval_quintic(self.right_quintic, now)
                            if now >= self.right_quintic['t_end']:
                                pos = self.right_quintic['p_end'].copy()
                                vel = np.zeros(6)
                                acc = np.zeros(6)
                            self.right_interp = np.stack([pos, vel, acc])
                            right_positions = pos.copy()
                        else:
                            right_positions = right_target.copy()

                    if self.end_effector == 'real_hand' and self.control_mode in {"pd_velocity", "pd_position_speed"}:
                        self._publish_sim_mirror(right_positions, "right")
                        if self.control_mode == "pd_position_speed":
                            self._publish_pd_position_speed_command(right_positions, "right", now)
                        else:
                            self._publish_pd_velocity_command(right_positions, "right", now)
                        self.last_right_position = right_positions.copy()
                    else:
                        right_changed = any(
                            abs(right_positions[i] - self.last_right_position[i]) >= self.dead_zone
                            for i in range(6)
                        )

                        if right_changed:
                            if self.pd_speed_control:
                                right_speeds = self._compute_pd_motor_speeds(
                                    right_positions,
                                    self.last_right_position,
                                    "right",
                                    now,
                                )
                            else:
                                right_delta = [
                                    abs(right_positions[i] - self.last_right_position[i])
                                    for i in range(6)
                                ]
                                right_speeds = self._compute_motor_speeds(right_delta)

                            if self.end_effector == 'sim_hand':
                                right_msg = JointState()
                                right_msg.header.stamp = self.get_clock().now().to_msg()
                                right_msg.name = list(short_joint_names_for_side("right"))
                                right_msg.position = [float(c) for c in right_positions]

                                self.right_motor_publisher.publish(right_msg)
                                self.last_right_position = right_positions.copy()

                # 高频控制回路中默认关闭打印，避免终端 IO 拉低跟手性。
                if self.enable_debug_print and (now - self._last_debug_print_ts) >= self.debug_print_interval:
                    self._last_debug_print_ts = now
                    if self.enable_left and self.left_data_ready and self.enable_right and self.right_data_ready:
                        print(
                            f"left_positions_rad: {np.round(self.last_left_position, 3)}, "
                            f"right_positions_rad: {np.round(self.last_right_position, 3)}, "
                            f"sim: {[round(float(xx), 2) for xx in self.thumb_sim_value]}"
                        )
                    elif self.enable_left and self.left_data_ready:
                        print(
                            f"left_positions_rad: {np.round(self.last_left_position, 3)}, "
                            f"sim_left: {[round(float(self.thumb_sim_value[i]), 2) for i in [0, 1]]}"
                        )
                    elif self.enable_right and self.right_data_ready:
                        print(
                            f"right_positions_rad: {np.round(self.last_right_position, 3)}, "
                            f"sim_right: {[round(float(self.thumb_sim_value[i]), 2) for i in [2, 3]]}"
                        )

                if self.enable_timing_print:
                    dt_ctrl = time.time() - t_ctrl_0
                    if dt_ctrl > 0.005:
                        print(f"[TIMING] control_callback: {dt_ctrl*1000:.1f} ms")
            except Exception as e:
                logger.error(f"控制出错: {e}")

    def record_thumb_calibration_data(self):
        try:
            if _is_revo3_style_algorithm(self.algorithm):
                self._record_revo3_calibration_data()
            else:
                self._record_legacy_thumb_calibration_data()
        except Exception as e:
            logger.error(f"标定流程失败: {e}")
            print("**** 标定失败，未进入控制模式，请重试 ****\n")

    def _record_legacy_thumb_calibration_data(self):
        print(f"**** 开始手势校准（模式: {self.hand_mode}）****\n")
        time.sleep(2)

        calibration_steps = []
        if self.enable_left:
            calibration_steps.extend([
                (0, "左手五指张开", "left"),
                (1, "左手拇指旋转", "left"),
            ])
        if self.enable_right:
            calibration_steps.extend([
                (2, "右手五指张开", "right"),
                (3, "右手拇指旋转", "right"),
            ])

        for index, gesture, hand in calibration_steps:
            print(f"准备采集手势数据: {gesture}")
            time.sleep(3)
            if hand == "left":
                self.thumb_open_rotate_tip_pos[index] = self.left_finger_tip_pos[0].copy()
            else:
                self.thumb_open_rotate_tip_pos[index] = self.right_finger_tip_pos[0].copy()

            print("手势数据采集完毕\n")
            time.sleep(2)

        save_ok = self.save_thumb_calibration_data()
        if not save_ok:
            logger.error("标定文件保存失败，已阻止进入控制模式。")
            print("**** 标定已完成，但保存失败，未进入控制模式 ****\n")
            return

        print("**** 标定文件已保存，开始实时控制 ****\n")
        time.sleep(1)
        self.record_thumb_calibration_flag = False

    def _is_hand_data_ready(self, hand: str) -> bool:
        if hand == "left":
            return self.left_data_ready
        if hand == "right":
            return self.right_data_ready
        return False

    def _wait_for_hand_data_ready(self, hand: str, timeout_sec: float = 20.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._is_hand_data_ready(hand):
                return
            time.sleep(0.05)
        raise TimeoutError(f"{hand} 手在 {timeout_sec:.1f}s 内未收到有效手套数据")

    def _get_hand_snapshot(self, hand: str):
        if hand == "left":
            return {
                "finger_tips": [list(v) for v in self.left_finger_tip_pos],
                "thumb_tip": list(self.left_finger_tip_pos[0]),
                "thumb_pip": list(self.left_thumb_pip_pos) if self.left_thumb_pip_pos is not None else None,
            }
        return {
            "finger_tips": [list(v) for v in self.right_finger_tip_pos],
            "thumb_tip": list(self.right_finger_tip_pos[0]),
            "thumb_pip": list(self.right_thumb_pip_pos) if self.right_thumb_pip_pos is not None else None,
        }

    def _sample_hand_pose_median(
        self,
        hand: str,
        sample_duration_sec: float = CALIBRATION_SAMPLE_SEC,
        sample_interval_sec: float = CALIBRATION_SAMPLE_INTERVAL_SEC,
    ):
        thumb_tip_samples = []
        thumb_pip_samples = []
        finger_tip_samples = []

        start = time.time()
        while time.time() - start < sample_duration_sec:
            snapshot = self._get_hand_snapshot(hand)
            thumb_pip = snapshot["thumb_pip"]
            if thumb_pip is not None:
                thumb_tip_samples.append(np.asarray(snapshot["thumb_tip"], dtype=float))
                thumb_pip_samples.append(np.asarray(thumb_pip, dtype=float))
                finger_tip_samples.append(np.asarray(snapshot["finger_tips"], dtype=float))
            time.sleep(sample_interval_sec)

        if len(thumb_tip_samples) < 3:
            raise RuntimeError(f"{hand} 手标定采样不足，请保持手势稳定并确保手套数据正常")

        tip_arr = np.asarray(thumb_tip_samples, dtype=float)
        pip_arr = np.asarray(thumb_pip_samples, dtype=float)
        finger_arr = np.asarray(finger_tip_samples, dtype=float)

        return {
            "thumb_tip": np.median(tip_arr, axis=0).tolist(),
            "thumb_pip": np.median(pip_arr, axis=0).tolist(),
            "finger_tips": np.median(finger_arr, axis=0).tolist(),
            "thumb_tip_noise": float(np.mean(np.var(tip_arr, axis=0))),
            "thumb_pip_noise": float(np.mean(np.var(pip_arr, axis=0))),
        }

    def _collect_revo3_pose(self, hand: str, gesture_desc: str):
        side_text = "左手" if hand == "left" else "右手"
        for sec in range(CALIBRATION_COUNTDOWN_SEC, 0, -1):
            print(f"[{side_text}] 请保持“{gesture_desc}”，{sec}s 后开始采样...")
            time.sleep(1)
        pose_data = self._sample_hand_pose_median(hand)
        print(f"[{side_text}] “{gesture_desc}”采样完成\n")
        return pose_data

    def _record_revo3_calibration_data(self):
        if not hasattr(self.hand_retargeting, "solve_calibration_for_side"):
            raise RuntimeError("revo3_thumb retargeter 缺少 solve_calibration_for_side 接口")
        if not hasattr(self.hand_retargeting, "apply_calibration"):
            raise RuntimeError("revo3_thumb retargeter 缺少 apply_calibration 接口")

        print(f"**** 开始 Revo3 手势校准（模式: {self.hand_mode}）****\n")
        time.sleep(1)

        revo3_payload = {"protocol": REVO3_CALIBRATION_PROTOCOL}

        for hand in ("left", "right"):
            if hand == "left" and not self.enable_left:
                continue
            if hand == "right" and not self.enable_right:
                continue

            side_text = "左手" if hand == "left" else "右手"
            self._wait_for_hand_data_ready(hand)
            print(f"---- 开始 {side_text} Revo3 四姿态标定 ----")

            side_poses = {}
            for pose_name, gesture_desc in REVO3_POSE_SEQUENCE:
                side_poses[pose_name] = self._collect_revo3_pose(hand, gesture_desc)

            side_params, side_quality = self.hand_retargeting.solve_calibration_for_side(
                hand,
                side_poses,
            )
            if self.revo3_thumb_param_overrides.get(hand):
                side_params.update(self.revo3_thumb_param_overrides[hand])
            self.hand_retargeting.apply_calibration({hand: side_params})

            revo3_payload[hand] = {
                "poses": side_poses,
                "params": side_params,
                "quality": side_quality,
            }

            if hand == "left":
                self.thumb_open_rotate_tip_pos[0] = side_poses["open"]["thumb_tip"]
                self.thumb_open_rotate_tip_pos[1] = side_poses["rotate"]["thumb_tip"]
            else:
                self.thumb_open_rotate_tip_pos[2] = side_poses["open"]["thumb_tip"]
                self.thumb_open_rotate_tip_pos[3] = side_poses["rotate"]["thumb_tip"]

            fit_rmse = side_quality.get("fit_rmse", -1.0)
            print(f"{side_text} 参数拟合完成，fit_rmse={fit_rmse:.5f}\n")

        self.revo3_calibration = revo3_payload

        save_ok = self.save_thumb_calibration_data()
        if not save_ok:
            logger.error("标定文件保存失败，已阻止进入控制模式。")
            print("**** 标定已完成，但保存失败，未进入控制模式 ****\n")
            return

        print("**** Revo3 标定文件已保存，开始实时控制 ****\n")
        time.sleep(1)
        self.record_thumb_calibration_flag = False

    def save_thumb_calibration_data(self):
        if self.calibration_file is None:
            return True
        try:
            self.calibration_file.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "schema_version": REVO3_SCHEMA_VERSION,
                "hand_mode": self.hand_mode,
                "thumb_open_rotate_tip_pos": self.thumb_open_rotate_tip_pos,
                "saved_at_unix": time.time(),
            }
            if isinstance(self.revo3_calibration, dict):
                payload["revo3_calibration"] = self.revo3_calibration
            with self.calibration_file.open("w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            logger.info(f"标定数据已保存: {self.calibration_file}")
            return True
        except Exception as e:
            logger.error(f"保存标定数据失败: {e}")
            return False

    def _calibration_hand_mode_matches(self, saved_hand_mode):
        if saved_hand_mode == self.hand_mode:
            return True
        return saved_hand_mode == "both" and self.hand_mode in ("left", "right")

    def load_thumb_calibration_data(self):
        if self.calibration_file is None:
            raise ValueError("skip_calibration 模式下必须传入 --calibration-file")
        if not self.calibration_file.exists():
            raise FileNotFoundError(f"标定文件不存在: {self.calibration_file}")
        with self.calibration_file.open("r", encoding="utf-8") as f:
            payload = json.load(f)
        saved_hand_mode = payload.get("hand_mode")
        if not self._calibration_hand_mode_matches(saved_hand_mode):
            raise ValueError(
                f"标定文件 hand_mode 不匹配: 文件为 '{saved_hand_mode}', 当前为 '{self.hand_mode}'"
            )
        calibration = payload.get("thumb_open_rotate_tip_pos")
        if (
            not isinstance(calibration, list)
            or len(calibration) != 4
            or any(not isinstance(item, list) or len(item) != 3 for item in calibration)
        ):
            raise ValueError(f"标定文件格式错误: {self.calibration_file}")
        self.thumb_open_rotate_tip_pos = calibration
        logger.info(f"标定数据已加载: {self.calibration_file}")

    def load_revo3_calibration_data(self):
        if self.calibration_file is None:
            raise ValueError("skip_calibration 模式下必须传入 --calibration-file")
        if not self.calibration_file.exists():
            raise FileNotFoundError(f"标定文件不存在: {self.calibration_file}")
        with self.calibration_file.open("r", encoding="utf-8") as f:
            payload = json.load(f)

        schema_raw = payload.get("schema_version", 1)
        try:
            schema_version = int(schema_raw)
        except (TypeError, ValueError):
            schema_version = 1
            logger.warning(
                f"标定文件 schema_version 非法({schema_raw})，已按旧版文件(schema_version=1)兼容处理。"
            )
        if schema_version < REVO3_SCHEMA_VERSION:
            logger.warning(
                "revo3 标定文件版本较旧(schema_version="
                f"{schema_version})，将尝试兼容加载；建议重新执行标定生成最新文件。"
            )

        saved_hand_mode = payload.get("hand_mode")
        if not self._calibration_hand_mode_matches(saved_hand_mode):
            raise ValueError(
                f"标定文件 hand_mode 不匹配: 文件为 '{saved_hand_mode}', 当前为 '{self.hand_mode}'"
            )

        # 兼容旧文件：如果存在旧字段，优先恢复用于相似度计算的 open/rotate 参考点。
        legacy_calibration = payload.get("thumb_open_rotate_tip_pos")
        if (
            isinstance(legacy_calibration, list)
            and len(legacy_calibration) == 4
            and all(isinstance(item, list) and len(item) == 3 for item in legacy_calibration)
        ):
            self.thumb_open_rotate_tip_pos = legacy_calibration

        revo3_payload = payload.get("revo3_calibration")
        if not isinstance(revo3_payload, dict):
            if schema_version < REVO3_SCHEMA_VERSION:
                logger.warning(
                    "旧版标定文件缺少 revo3_calibration 字段，将使用默认 revo3 参数启动；"
                    "建议尽快重新标定以获得最佳效果。"
                )
                self.revo3_calibration = None
                return {}
            raise ValueError(
                f"revo3 模式要求标定文件包含 revo3_calibration 字段: {self.calibration_file}"
            )

        protocol = revo3_payload.get("protocol")
        if protocol is None and schema_version < REVO3_SCHEMA_VERSION:
            protocol = REVO3_CALIBRATION_PROTOCOL
            revo3_payload["protocol"] = protocol
            logger.warning(
                "旧版 revo3 标定文件缺少 protocol 字段，已按 "
                f"{REVO3_CALIBRATION_PROTOCOL} 兼容处理。"
            )
        if protocol != REVO3_CALIBRATION_PROTOCOL:
            raise ValueError(
                "revo3 标定协议不匹配: "
                f"expect {REVO3_CALIBRATION_PROTOCOL}, got {protocol}"
            )

        params_by_side = {}
        for side in ("left", "right"):
            if side == "left" and not self.enable_left:
                continue
            if side == "right" and not self.enable_right:
                continue

            side_payload = revo3_payload.get(side)
            if not isinstance(side_payload, dict):
                if schema_version < REVO3_SCHEMA_VERSION:
                    logger.warning(
                        f"旧版 revo3 标定缺少 {side} 数据，将对该手使用默认参数。"
                    )
                    continue
                raise ValueError(f"revo3 标定缺少 {side} 数据")

            params = side_payload.get("params")
            if not isinstance(params, dict):
                if schema_version < REVO3_SCHEMA_VERSION:
                    logger.warning(
                        f"旧版 revo3 标定缺少 {side}.params，将对该手使用默认参数。"
                    )
                    continue
                raise ValueError(f"revo3 标定缺少 {side}.params")
            missing = sorted(REVO3_REQUIRED_PARAM_KEYS - set(params.keys()))
            if missing:
                if schema_version < REVO3_SCHEMA_VERSION:
                    logger.warning(
                        f"旧版 revo3 标定缺少 {side}.params 字段 {missing}，"
                        "将使用默认值补齐。"
                    )
                else:
                    raise ValueError(f"revo3 标定缺少 {side}.params 字段: {missing}")

            params_by_side[side] = params

            poses = side_payload.get("poses")
            if isinstance(poses, dict):
                open_pose = poses.get("open")
                rotate_pose = poses.get("rotate")
                if isinstance(open_pose, dict) and isinstance(open_pose.get("thumb_tip"), list):
                    if side == "left":
                        self.thumb_open_rotate_tip_pos[0] = open_pose["thumb_tip"]
                    else:
                        self.thumb_open_rotate_tip_pos[2] = open_pose["thumb_tip"]
                if isinstance(rotate_pose, dict) and isinstance(rotate_pose.get("thumb_tip"), list):
                    if side == "left":
                        self.thumb_open_rotate_tip_pos[1] = rotate_pose["thumb_tip"]
                    else:
                        self.thumb_open_rotate_tip_pos[3] = rotate_pose["thumb_tip"]

        self.revo3_calibration = revo3_payload
        logger.info(f"revo3 标定参数已加载: {self.calibration_file}")
        return params_by_side

    def thumb_sim_callback(self):
        t0 = time.time() if self.enable_timing_print else None
        thumb_sim_value = [0.0] * 4
        if self.enable_left and self.left_data_ready:
            for index in [0, 1]:
                tip_pos = self.thumb_open_rotate_tip_pos[index]
                thumb_sim_value[index] = similarity_calculation(tip_pos, self.left_finger_tip_pos[0])
        if self.enable_right and self.right_data_ready:
            for index in [2, 3]:
                tip_pos = self.thumb_open_rotate_tip_pos[index]
                thumb_sim_value[index] = similarity_calculation(tip_pos, self.right_finger_tip_pos[0])

        self.thumb_sim_value = thumb_sim_value.copy()
        if self.enable_timing_print:
            dt = time.time() - t0
            if dt > 0.005:
                print(f"[TIMING] thumb_sim_callback: {dt*1000:.1f} ms")

    def _compute_motor_speeds(self, deltas):
        """Compute per-joint speed with dedicated limits for thumb splay joint only."""
        speeds = []
        for i in range(6):
            if i == self.thumb_splay_joint_index:
                speed_min = self.thumb_splay_speed_min
                speed_max = self.thumb_splay_speed_max
            else:
                speed_min = self.finger_speed_min
                speed_max = self.finger_speed_max
            speed = float(deltas[i]) * self.speed_gain + speed_min
            speed = max(speed_min, min(speed_max, speed))
            speeds.append(int(speed))
        return speeds

    def _speed_limits_for_joint(self, joint_index):
        if joint_index == self.thumb_splay_joint_index:
            return self.thumb_splay_speed_min, self.thumb_splay_speed_max
        return self.finger_speed_min, self.finger_speed_max

    def _filter_joint_target(self, target_positions, side):
        target = self._joint_positions_rad(target_positions)

        def filter_once(previous):
            if self.target_filter_fast_threshold <= 0.0:
                alpha = self.target_filter_alpha
            else:
                delta = np.abs(target - previous)
                alpha = np.where(
                    delta >= self.target_filter_fast_threshold,
                    self.target_filter_fast_alpha,
                    self.target_filter_alpha,
                )
            return (1.0 - alpha) * previous + alpha * target

        if side == "left":
            if not self.left_filter_initialized:
                self.left_filtered_target = target.copy()
                self.left_filter_initialized = True
            else:
                self.left_filtered_target = filter_once(self.left_filtered_target)
            return self.left_filtered_target.copy()

        if side == "right":
            if not self.right_filter_initialized:
                self.right_filtered_target = target.copy()
                self.right_filter_initialized = True
            else:
                self.right_filtered_target = filter_once(self.right_filtered_target)
            return self.right_filtered_target.copy()

        raise ValueError(f"Invalid hand side for target filtering: {side}")

    def _compute_pd_motor_speeds(self, target_positions, estimated_positions, side, now):
        """Shape mode-6 speed commands from command-side PD error."""
        target = np.asarray(target_positions, dtype=float)
        estimated = np.asarray(estimated_positions, dtype=float)
        error = target - estimated

        if side == "left":
            last_error = self.last_left_error
            filtered_derivative = self.left_error_derivative
            last_time = self.last_left_pd_time
        elif side == "right":
            last_error = self.last_right_error
            filtered_derivative = self.right_error_derivative
            last_time = self.last_right_pd_time
        else:
            raise ValueError(f"Invalid hand side for PD speed control: {side}")

        dt = 0.01 if last_time is None else max(now - last_time, 1e-3)
        raw_derivative = (error - last_error) / dt
        alpha = self.pd_derivative_alpha
        filtered_derivative = (
            (1.0 - alpha) * filtered_derivative
            + alpha * raw_derivative
        )

        speeds = []
        for i in range(6):
            speed_min, speed_max = self._speed_limits_for_joint(i)
            speed = (
                speed_min
                + self.pd_kp * abs(error[i])
                + self.pd_kd * abs(filtered_derivative[i])
            )
            speed = max(speed_min, min(speed_max, speed))
            speeds.append(int(speed))

        if side == "left":
            self.last_left_error = error
            self.left_error_derivative = filtered_derivative
            self.last_left_pd_time = now
        else:
            self.last_right_error = error
            self.right_error_derivative = filtered_derivative
            self.last_right_pd_time = now

        return speeds

    def _update_pd_velocity_target(self, target_positions, side, now):
        target = self._joint_positions_rad(target_positions)
        if side == "left":
            actual = self.left_actual_position
            last_error = self.last_left_velocity_error
            filtered_derivative = self.left_velocity_derivative
            last_time = self.last_left_velocity_pd_time
        elif side == "right":
            actual = self.right_actual_position
            last_error = self.last_right_velocity_error
            filtered_derivative = self.right_velocity_derivative
            last_time = self.last_right_velocity_pd_time
        else:
            raise ValueError(f"Invalid hand side for PD velocity control: {side}")

        error = target - actual
        dt = 0.1 if last_time is None else max(now - last_time, 1e-3)
        raw_derivative = (error - last_error) / dt
        alpha = self.pd_derivative_alpha
        filtered_derivative = (
            (1.0 - alpha) * filtered_derivative
            + alpha * raw_derivative
        )
        target_velocity = self.velocity_kp * error + self.velocity_kd * filtered_derivative
        if self.thumb_velocity_kp_scale != 1.0:
            target_velocity[:2] *= self.thumb_velocity_kp_scale
        if self.ring_velocity_kp_scale != 1.0:
            target_velocity[4] *= self.ring_velocity_kp_scale

        deadbands = np.full(6, self.velocity_deadband, dtype=float)
        deadbands[:2] = self.thumb_velocity_deadband
        deadbands[4] = self.ring_velocity_deadband
        target_velocity[np.abs(error) <= deadbands] = 0.0
        if self.four_finger_extension_velocity_scale != 1.0:
            for joint_index in self.four_finger_extension_joints:
                if target_velocity[joint_index] < 0.0:
                    target_velocity[joint_index] *= self.four_finger_extension_velocity_scale
        target_velocity = np.clip(target_velocity, -self.velocity_max, self.velocity_max)

        if self.thumb_velocity_min > 0.0:
            thumb_min = min(self.thumb_velocity_min, self.velocity_max)
            thumb_error = error[:2]
            thumb_velocity = target_velocity[:2]
            thumb_active = (
                (np.abs(thumb_error) > self.thumb_velocity_deadband)
                & (np.abs(thumb_velocity) < thumb_min)
            )
            if np.any(thumb_active):
                thumb_velocity[thumb_active] = np.sign(thumb_error[thumb_active]) * thumb_min
                target_velocity[:2] = thumb_velocity

        if (
            self.ring_velocity_min > 0.0
            and abs(error[4]) > self.ring_velocity_deadband
            and abs(target_velocity[4]) < self.ring_velocity_min
        ):
            target_velocity[4] = np.sign(error[4]) * min(
                self.ring_velocity_min,
                self.velocity_max,
            )

        if (
            self.thumb_velocity_brake_zone > 0.0
            and self.thumb_velocity_brake_max > 0.0
        ):
            thumb_error_abs = np.abs(error[:2])
            thumb_brake_active = thumb_error_abs < self.thumb_velocity_brake_zone
            if np.any(thumb_brake_active):
                thumb_velocity = target_velocity[:2]
                thumb_brake_cap = (
                    min(self.thumb_velocity_brake_max, self.velocity_max)
                    * thumb_error_abs[thumb_brake_active]
                    / self.thumb_velocity_brake_zone
                )
                thumb_velocity[thumb_brake_active] = np.clip(
                    thumb_velocity[thumb_brake_active],
                    -thumb_brake_cap,
                    thumb_brake_cap,
                )
                target_velocity[:2] = thumb_velocity

        if side == "left":
            self.last_left_velocity_error = error
            self.left_velocity_derivative = filtered_derivative
            self.last_left_velocity_pd_time = now
            self.left_target_velocity = target_velocity
        else:
            self.last_right_velocity_error = error
            self.right_velocity_derivative = filtered_derivative
            self.last_right_velocity_pd_time = now
            self.right_target_velocity = target_velocity

    def _publish_pd_velocity_command(self, target_positions, side, now):
        if side == "left":
            feedback_ready = self.left_feedback_ready
            feedback_seq = self.left_feedback_seq
            feedback_time = self.left_feedback_time
            actual_raw_position = self.left_actual_raw_position
            actual_position = self.left_actual_position
            last_feedback_seq_used = self.last_left_feedback_seq_used
            target_velocity = self.left_target_velocity
            command_velocity = self.left_command_velocity
            publisher = self.left_motor_publisher
        elif side == "right":
            feedback_ready = self.right_feedback_ready
            feedback_seq = self.right_feedback_seq
            feedback_time = self.right_feedback_time
            actual_raw_position = self.right_actual_raw_position
            actual_position = self.right_actual_position
            last_feedback_seq_used = self.last_right_feedback_seq_used
            target_velocity = self.right_target_velocity
            command_velocity = self.right_command_velocity
            publisher = self.right_motor_publisher
        else:
            raise ValueError(f"Invalid hand side for PD velocity control: {side}")

        if not feedback_ready:
            if now - self._last_feedback_warn_ts > 1.0:
                self._last_feedback_warn_ts = now
                logger.warning(f"等待 {side} JointState 反馈，暂不发送 pd_velocity 命令。")
            return

        feedback_stale = (
            feedback_time is None
            or now - feedback_time > self.velocity_feedback_timeout
        )
        if feedback_stale:
            target_velocity = np.zeros(6, dtype=float)
            if side == "left":
                self.left_target_velocity = target_velocity
            else:
                self.right_target_velocity = target_velocity
            if now - self._last_feedback_warn_ts > 1.0:
                self._last_feedback_warn_ts = now
                logger.warning(f"{side} JointState 反馈超时，pd_velocity 目标速度归零。")

        if not feedback_stale:
            self._update_pd_velocity_target(target_positions, side, now)
            if side == "left":
                self.last_left_feedback_seq_used = feedback_seq
                target_velocity = self.left_target_velocity
            else:
                self.last_right_feedback_seq_used = feedback_seq
                target_velocity = self.right_target_velocity

        max_step = self.velocity_slew_rate
        zero_target = np.abs(target_velocity) < self.velocity_zero_epsilon
        reversing = (target_velocity * command_velocity) < 0.0
        command_velocity[zero_target | reversing] = 0.0
        command_velocity = command_velocity + np.clip(
            target_velocity - command_velocity,
            -max_step,
            max_step,
        )
        command_velocity[np.abs(command_velocity) < self.velocity_zero_epsilon] = 0.0

        if side == "left":
            self.left_command_velocity = command_velocity
        else:
            self.right_command_velocity = command_velocity

        publisher.publish(self._velocity_command_msg(command_velocity))

        if self.pd_debug and now - self._last_pd_debug_ts >= self.pd_debug_interval:
            self._last_pd_debug_ts = now
            target_array = self._joint_positions_rad(target_positions)
            if self.pd_debug_file is not None:
                self.pd_debug_file.parent.mkdir(parents=True, exist_ok=True)
            for j in self.pd_debug_joints:
                error = float(target_array[j] - actual_position[j])
                debug_line = (
                    f"[pd_velocity {side} j{j}] "
                    f"t={now:.3f} "
                    f"target_rad={float(target_array[j]):.5f} "
                    f"actual_raw_rad={float(actual_raw_position[j]):.5f} "
                    f"actual_rad={float(actual_position[j]):.5f} "
                    f"err_rad={error:.5f} "
                    f"target_v_rad_s={float(target_velocity[j]):.5f} "
                    f"cmd_v_rad_s={float(command_velocity[j]):.5f} "
                    f"seq={feedback_seq}"
                )
                logger.info(debug_line)
                if self.pd_debug_file is not None:
                    with self.pd_debug_file.open("a", encoding="utf-8") as f:
                        f.write(debug_line + "\n")

    def _publish_pd_position_speed_command(self, target_positions, side, now):
        if side == "left":
            feedback_ready = self.left_feedback_ready
            feedback_seq = self.left_feedback_seq
            feedback_time = self.left_feedback_time
            actual_raw_position = self.left_actual_raw_position
            actual_position = self.left_actual_position
            target_velocity = self.left_target_velocity
            command_velocity = self.left_command_velocity
            position_publisher = self.left_motor_publisher
            speed_publisher = self.left_speed_publisher
        elif side == "right":
            feedback_ready = self.right_feedback_ready
            feedback_seq = self.right_feedback_seq
            feedback_time = self.right_feedback_time
            actual_raw_position = self.right_actual_raw_position
            actual_position = self.right_actual_position
            target_velocity = self.right_target_velocity
            command_velocity = self.right_command_velocity
            position_publisher = self.right_motor_publisher
            speed_publisher = self.right_speed_publisher
        else:
            raise ValueError(f"Invalid hand side for PD position-speed control: {side}")

        if position_publisher is None or speed_publisher is None:
            return

        if not feedback_ready:
            if now - self._last_feedback_warn_ts > 1.0:
                self._last_feedback_warn_ts = now
                logger.warning(f"等待 {side} JointState 反馈，暂不发送 pd_position_speed 命令。")
            return

        feedback_stale = (
            feedback_time is None
            or now - feedback_time > self.velocity_feedback_timeout
        )
        if feedback_stale:
            target_velocity = np.zeros(6, dtype=float)
            command_velocity = np.zeros(6, dtype=float)
            target_array = actual_position.copy()
            if side == "left":
                self.left_target_velocity = target_velocity
                self.left_command_velocity = command_velocity
            else:
                self.right_target_velocity = target_velocity
                self.right_command_velocity = command_velocity
            if now - self._last_feedback_warn_ts > 1.0:
                self._last_feedback_warn_ts = now
                logger.warning(f"{side} JointState 反馈超时，pd_position_speed 速度归零并保持当前位置。")
        else:
            self._update_pd_velocity_target(target_positions, side, now)
            target_array = self._joint_positions_rad(target_positions)
            if side == "left":
                self.last_left_feedback_seq_used = feedback_seq
                target_velocity = self.left_target_velocity
                command_velocity = self.left_command_velocity
            else:
                self.last_right_feedback_seq_used = feedback_seq
                target_velocity = self.right_target_velocity
                command_velocity = self.right_command_velocity

            max_step = self.velocity_slew_rate
            zero_target = np.abs(target_velocity) < self.velocity_zero_epsilon
            reversing = (target_velocity * command_velocity) < 0.0
            command_velocity[zero_target | reversing] = 0.0
            command_velocity = command_velocity + np.clip(
                target_velocity - command_velocity,
                -max_step,
                max_step,
            )
            command_velocity[np.abs(command_velocity) < self.velocity_zero_epsilon] = 0.0

            if side == "left":
                self.left_command_velocity = command_velocity
            else:
                self.right_command_velocity = command_velocity

        position_msg = Float64MultiArray()
        position_msg.data = [
            float(v)
            for v in np.clip(target_array, 0.0, self.revo2_joint_upper_limits_rad)
        ]
        position_publisher.publish(position_msg)

        speed_msg = Float64MultiArray()
        speed_percent_scale = 100.0 / max(self.velocity_max, 1e-9)
        speed_msg.data = [
            float(np.clip(abs(v) * speed_percent_scale, 0.0, 100.0))
            for v in command_velocity
        ]
        speed_publisher.publish(speed_msg)

        if self.pd_debug and now - self._last_pd_debug_ts >= self.pd_debug_interval:
            self._last_pd_debug_ts = now
            if self.pd_debug_file is not None:
                self.pd_debug_file.parent.mkdir(parents=True, exist_ok=True)
            for j in self.pd_debug_joints:
                error = float(target_array[j] - actual_position[j])
                debug_line = (
                    f"[pd_position_speed {side} j{j}] "
                    f"t={now:.3f} "
                    f"target_rad={float(target_array[j]):.5f} "
                    f"actual_raw_rad={float(actual_raw_position[j]):.5f} "
                    f"actual_rad={float(actual_position[j]):.5f} "
                    f"err_rad={error:.5f} "
                    f"target_v_rad_s={float(target_velocity[j]):.5f} "
                    f"cmd_v_rad_s={float(command_velocity[j]):.5f} "
                    f"speed_pct={float(speed_msg.data[j]):.1f} "
                    f"seq={feedback_seq}"
                )
                logger.info(debug_line)
                if self.pd_debug_file is not None:
                    with self.pd_debug_file.open("a", encoding="utf-8") as f:
                        f.write(debug_line + "\n")

    def _make_quintic_segment(self, p0, v0, a0, p1, v1, a1, t_start, T):
        """生成五次多项式轨迹段，匹配起点/终点的位置、速度、加速度。
        与 Revo3 仓库的 _make_quintic_segment 保持一致。
        """
        T = max(T, 1e-6)
        c0 = p0.copy()
        c1 = v0 * T
        c2 = a0 * (T ** 2) / 2.0
        r0 = p1 - c0 - c1 - c2
        r1 = v1 * T - c1 - 2.0 * c2
        r2 = a1 * (T ** 2) - 2.0 * c2
        A = np.array([[1.0, 1.0, 1.0],
                      [3.0, 4.0, 5.0],
                      [6.0, 12.0, 20.0]], dtype=float)
        R = np.stack([r0, r1, r2], axis=0)   # (3, 6)
        X = np.linalg.solve(A, R)           # (3, 6)
        coeffs = np.stack([c0, c1, c2, X[0], X[1], X[2]], axis=-1)  # (6, 6)
        return {
            'coeffs': coeffs,
            't_start': t_start,
            't_end': t_start + T,
            'p_end': p1.copy(),
        }

    def _eval_quintic(self, seg, t):
        """评估 quintic segment，返回 (pos, vel, acc)。
        与 Revo3 仓库的 _eval_quintic 保持一致。
        """
        T = seg['t_end'] - seg['t_start']
        if T <= 0.0:
            return seg['p_end'].copy(), np.zeros(6), np.zeros(6)
        s = np.clip((t - seg['t_start']) / T, 0.0, 1.0)
        c = seg['coeffs']
        pos = c @ np.array([1.0, s, s**2, s**3, s**4, s**5])
        vel = (c @ np.array([0.0, 1.0, 2*s, 3*s**2, 4*s**3, 5*s**4])) / T
        acc = (c @ np.array([0.0, 0.0, 2.0, 6*s, 12*s**2, 20*s**3])) / (T ** 2)
        return pos, vel, acc


class ManusRevo2Retargeter:

    def __init__(
        self,
        hand_mode='both',
        ros_args=None,
        skip_calibration=False,
        calibration_file=None,
        use_default_revo3_calibration=False,
        revo3_thumb_params=None,
        config_file=None,
        algorithm=None,
        end_effector_mode=None,
        pd_speed_control=False,
        pd_kp=2.0,
        pd_kd=0.002,
        pd_derivative_alpha=0.25,
        target_filter_alpha=0.35,
        target_filter_fast_alpha=None,
        target_filter_fast_threshold=0.0,
        dead_zone=0.003,
        control_mode="position_speed",
        velocity_kp=0.8,
        velocity_kd=0.01,
        velocity_deadband=0.01,
        thumb_velocity_deadband=None,
        thumb_velocity_min=0.0,
        thumb_velocity_kp_scale=1.0,
        thumb_velocity_brake_zone=0.0,
        thumb_velocity_brake_max=0.0,
        ring_velocity_deadband=None,
        ring_velocity_min=0.0,
        ring_velocity_kp_scale=1.0,
        four_finger_extension_velocity_scale=1.0,
        four_finger_extension_joints=None,
        velocity_max=1.2,
        velocity_slew_rate=0.2,
        velocity_feedback_timeout=0.3,
        feedback_position_scale=1.0,
        feedback_position_scales=None,
        feedback_position_offsets=None,
        revo2_position_command_scale=572.9577951308232,
        revo2_velocity_command_scale=1.0,
        left_position_command_topic="/revo2_left/joint_forward_pos_controller/commands",
        right_position_command_topic="/revo2_right/joint_forward_pos_controller/commands",
        left_velocity_command_topic="/revo2_left/joint_forward_vel_controller/commands",
        right_velocity_command_topic="/revo2_right/joint_forward_vel_controller/commands",
        left_joint_state_topic="/revo2_left/revo2_joint_state/joint_states",
        right_joint_state_topic="/revo2_right/revo2_joint_state/joint_states",
        mirror_sim_output=False,
        left_sim_command_topic="/revo2_left/retarget/joint_states",
        right_sim_command_topic="/revo2_right/retarget/joint_states",
        target_only=False,
        left_target_joint_state_topic="/revo2_left/revo2_pid_controller/target_joint_states",
        right_target_joint_state_topic="/revo2_right/revo2_pid_controller/target_joint_states",
        pd_debug=False,
        pd_debug_joint=2,
        pd_debug_joints=None,
        pd_debug_interval=0.2,
        pd_debug_file=None,
    ):
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)

        # 初始化ROS2
        rclpy.init(args=ros_args)

        # 创建单一节点
        self.node = ManusRevo2Node(
            hand_mode=hand_mode,
            skip_calibration=skip_calibration,
            calibration_file=calibration_file,
            use_default_revo3_calibration=use_default_revo3_calibration,
            revo3_thumb_params=revo3_thumb_params,
            config_file=config_file,
            algorithm=algorithm,
            end_effector_mode=end_effector_mode,
            pd_speed_control=pd_speed_control,
            pd_kp=pd_kp,
            pd_kd=pd_kd,
            pd_derivative_alpha=pd_derivative_alpha,
            target_filter_alpha=target_filter_alpha,
            target_filter_fast_alpha=target_filter_fast_alpha,
            target_filter_fast_threshold=target_filter_fast_threshold,
            dead_zone=dead_zone,
            control_mode=control_mode,
            velocity_kp=velocity_kp,
            velocity_kd=velocity_kd,
            velocity_deadband=velocity_deadband,
            thumb_velocity_deadband=thumb_velocity_deadband,
            thumb_velocity_min=thumb_velocity_min,
            thumb_velocity_kp_scale=thumb_velocity_kp_scale,
            thumb_velocity_brake_zone=thumb_velocity_brake_zone,
            thumb_velocity_brake_max=thumb_velocity_brake_max,
            ring_velocity_deadband=ring_velocity_deadband,
            ring_velocity_min=ring_velocity_min,
            ring_velocity_kp_scale=ring_velocity_kp_scale,
            four_finger_extension_velocity_scale=four_finger_extension_velocity_scale,
            four_finger_extension_joints=four_finger_extension_joints,
            velocity_max=velocity_max,
            velocity_slew_rate=velocity_slew_rate,
            velocity_feedback_timeout=velocity_feedback_timeout,
            feedback_position_scale=feedback_position_scale,
            feedback_position_scales=feedback_position_scales,
            feedback_position_offsets=feedback_position_offsets,
            revo2_position_command_scale=revo2_position_command_scale,
            revo2_velocity_command_scale=revo2_velocity_command_scale,
            left_position_command_topic=left_position_command_topic,
            right_position_command_topic=right_position_command_topic,
            left_velocity_command_topic=left_velocity_command_topic,
            right_velocity_command_topic=right_velocity_command_topic,
            left_joint_state_topic=left_joint_state_topic,
            right_joint_state_topic=right_joint_state_topic,
            mirror_sim_output=mirror_sim_output,
            left_sim_command_topic=left_sim_command_topic,
            right_sim_command_topic=right_sim_command_topic,
            target_only=target_only,
            left_target_joint_state_topic=left_target_joint_state_topic,
            right_target_joint_state_topic=right_target_joint_state_topic,
            pd_debug=pd_debug,
            pd_debug_joint=pd_debug_joint,
            pd_debug_joints=pd_debug_joints,
            pd_debug_interval=pd_debug_interval,
            pd_debug_file=pd_debug_file,
        )

        self.running = True

    def signal_handler(self, signum, frame):
        """处理Ctrl+C信号"""
        logger.info("接收到退出信号，正在关闭...")
        self.running = False

    def run(self):
        """运行主循环"""
        try:
            while self.running:
                t0 = time.time() if self.node.enable_timing_print else None
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if self.node.enable_timing_print:
                    dt = time.time() - t0
                    if dt > 0.02:
                        print(f"[TIMING] spin_once blocked: {dt*1000:.1f} ms")
        except Exception as e:
            logger.error(f"运行时出错: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        try:
            if hasattr(self, 'node'):
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            logger.error(f"清理资源时出错: {e}")


def parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Manus Revo2 retarget node launcher."
    )
    parser.add_argument(
        "--control-config",
        "--control_config",
        dest="control_config",
        default=None,
        help="YAML file with control tuning defaults. Explicit CLI flags override YAML values.",
    )
    parser.add_argument(
        "--hand-mode",
        "--hand_mode",
        dest="hand_mode",
        default="both",
        choices=sorted(VALID_HAND_MODES),
        help="Control hand mode: left, right, or both.",
    )
    parser.add_argument(
        "--skip-calibration",
        action="store_true",
        help="Skip gesture calibration and load from --calibration-file.",
    )
    parser.add_argument(
        "--calibration-file",
        default=None,
        help="Calibration file path for saving/loading thumb calibration data. "
             "Defaults to ~/.manus_revo2/thumb_calibration_<hand_mode>.json",
    )
    parser.add_argument(
        "--use-default-revo3-calibration",
        action="store_true",
        help="With --skip-calibration and revo3-style thumb algorithms, do not load a calibration file; "
             "use the built-in revo3 default runtime parameters.",
    )
    parser.add_argument(
        "--config-file",
        "--config_file",
        dest="config_file",
        default=None,
        help="Path to the retargeting configuration YAML file. "
             f"Defaults to {DEFAULT_CONFIG_PATH}",
    )
    parser.add_argument(
        "--algorithm",
        dest="algorithm",
        default=None,
        choices=RetargeterRegistry.list_algorithms(),
        help="Override the retargeting algorithm defined in the config file. "
             "Available: " + ", ".join(RetargeterRegistry.list_algorithms()),
    )
    parser.add_argument(
        "--end-effector",
        "--end_effector",
        dest="end_effector",
        default=end_effector,
        choices=sorted(VALID_END_EFFECTORS),
        help="Output target: real_hand publishes revo2_driver velocity commands; "
             "sim_hand publishes sensor_msgs/JointState.",
    )
    parser.add_argument(
        "--mirror-sim-output",
        "--mirror_sim_output",
        dest="mirror_sim_output",
        action="store_true",
        help="In real_hand mode, also publish retarget target JointState.",
    )
    parser.add_argument(
        "--left-sim-command-topic",
        "--left_sim_command_topic",
        dest="left_sim_command_topic",
        default="/revo2_left/retarget/joint_states",
        help="JointState topic for left retarget target output.",
    )
    parser.add_argument(
        "--right-sim-command-topic",
        "--right_sim_command_topic",
        dest="right_sim_command_topic",
        default="/revo2_right/retarget/joint_states",
        help="JointState topic for right retarget target output.",
    )
    parser.add_argument(
        "--target-only",
        "--target_only",
        dest="target_only",
        action="store_true",
        help=(
            "Publish retarget target JointState topics only. "
            "Disables Revo2 feedback subscriptions and velocity command publishing."
        ),
    )
    parser.add_argument(
        "--left-target-joint-state-topic",
        "--left_target_joint_state_topic",
        dest="left_target_joint_state_topic",
        default="/revo2_left/revo2_pid_controller/target_joint_states",
        help="Left target JointState topic used by the split teleop controller.",
    )
    parser.add_argument(
        "--right-target-joint-state-topic",
        "--right_target_joint_state_topic",
        dest="right_target_joint_state_topic",
        default="/revo2_right/revo2_pid_controller/target_joint_states",
        help="Right target JointState topic used by the split teleop controller.",
    )
    parser.add_argument(
        "--pd-speed-control",
        action="store_true",
        help="Enable legacy command-side PD shaping for position_speed simulation output.",
    )
    parser.add_argument(
        "--pd-kp",
        type=float,
        default=2.0,
        help="P gain for legacy command-side PD speed shaping.",
    )
    parser.add_argument(
        "--pd-kd",
        type=float,
        default=0.002,
        help="D gain for legacy command-side PD speed shaping.",
    )
    parser.add_argument(
        "--pd-derivative-alpha",
        type=float,
        default=0.25,
        help="Low-pass alpha for the PD derivative term, in [0, 1].",
    )
    parser.add_argument(
        "--target-filter-alpha",
        type=float,
        default=0.35,
        help="Low-pass alpha for target positions in rad, in [0, 1].",
    )
    parser.add_argument(
        "--target-filter-fast-alpha",
        type=float,
        default=None,
        help=(
            "Optional adaptive-filter alpha for large target jumps. "
            "When unset, target filtering stays fixed at --target-filter-alpha."
        ),
    )
    parser.add_argument(
        "--target-filter-fast-threshold",
        type=float,
        default=0.0,
        help=(
            "Target-position delta in rad that switches a joint to "
            "--target-filter-fast-alpha. 0 disables adaptive filtering."
        ),
    )
    parser.add_argument(
        "--dead-zone",
        type=float,
        default=0.003,
        help="Minimum target-position change in rad required before publishing a command.",
    )
    parser.add_argument(
        "--control-mode",
        default="position_speed",
        choices=sorted(VALID_CONTROL_MODES),
        help=(
            "Control output mode. pd_position_speed keeps the PD speed loop but "
            "publishes target positions plus positive speed percentages for Revo2."
        ),
    )
    parser.add_argument(
        "--velocity-kp",
        type=float,
        default=0.8,
        help="P gain for pd_velocity speed commands.",
    )
    parser.add_argument(
        "--velocity-kd",
        type=float,
        default=0.01,
        help="D gain for pd_velocity speed commands.",
    )
    parser.add_argument(
        "--velocity-deadband",
        type=float,
        default=0.01,
        help="Position error band in rad where pd_velocity sends zero speed.",
    )
    parser.add_argument(
        "--thumb-velocity-deadband",
        type=float,
        default=None,
        help=(
            "Thumb-only pd_velocity deadband for joints 0 and 1. "
            "Defaults to --velocity-deadband."
        ),
    )
    parser.add_argument(
        "--thumb-velocity-min",
        type=float,
        default=0.0,
        help=(
            "Minimum absolute speed for thumb joints in pd_velocity mode "
            "while their error is outside --thumb-velocity-deadband, in rad/s."
        ),
    )
    parser.add_argument(
        "--thumb-velocity-kp-scale",
        type=float,
        default=1.0,
        help="Multiplier for thumb joint P velocity in pd_velocity mode.",
    )
    parser.add_argument(
        "--thumb-velocity-brake-zone",
        type=float,
        default=0.0,
        help=(
            "Thumb-only terminal braking zone in rad. "
            "When positive, thumb speed is capped linearly near the target."
        ),
    )
    parser.add_argument(
        "--thumb-velocity-brake-max",
        type=float,
        default=0.0,
        help=(
            "Maximum thumb speed allowed at the outer edge of "
            "--thumb-velocity-brake-zone, in rad/s."
        ),
    )
    parser.add_argument(
        "--ring-velocity-deadband",
        type=float,
        default=None,
        help=(
            "Ring-finger pd_velocity deadband for joint 4. "
            "Defaults to --velocity-deadband."
        ),
    )
    parser.add_argument(
        "--ring-velocity-min",
        type=float,
        default=0.0,
        help=(
            "Minimum absolute speed for the ring-finger joint in pd_velocity "
            "mode while its error is outside --ring-velocity-deadband, in rad/s."
        ),
    )
    parser.add_argument(
        "--ring-velocity-kp-scale",
        type=float,
        default=1.0,
        help="Multiplier for ring-finger P velocity in pd_velocity mode.",
    )
    parser.add_argument(
        "--four-finger-extension-velocity-scale",
        type=float,
        default=1.0,
        help=(
            "Multiplier applied only to negative pd_velocity commands for the "
            "selected four-finger joints. Values above 1 speed up extension "
            "without changing positive grasp commands."
        ),
    )
    parser.add_argument(
        "--four-finger-extension-joints",
        default=None,
        help=(
            "Comma-separated joint indices that use "
            "--four-finger-extension-velocity-scale. Defaults to '2,3,4,5'."
        ),
    )
    parser.add_argument(
        "--velocity-max",
        type=float,
        default=1.2,
        help="Absolute max speed command in rad/s for pd_velocity mode.",
    )
    parser.add_argument(
        "--velocity-slew-rate",
        type=float,
        default=0.2,
        help="Max rad/s command change per control tick in pd_velocity mode.",
    )
    parser.add_argument(
        "--velocity-feedback-timeout",
        type=float,
        default=0.3,
        help="Seconds before stale JointState feedback forces pd_velocity target speed to zero.",
    )
    parser.add_argument(
        "--feedback-position-scale",
        "--motor-status-position-scale",
        dest="feedback_position_scale",
        type=float,
        default=1.0,
        help=(
            "Fallback scalar correction applied to Revo2 JointState positions in rad. "
            "Ignored per joint when --feedback-position-scales is provided."
        ),
    )
    parser.add_argument(
        "--feedback-position-scales",
        "--motor-status-position-scales",
        dest="feedback_position_scales",
        default=None,
        help=(
            "Six comma-separated per-joint scales applied to JointState positions in rad, "
            "e.g. '1,1,1,1,1,1'."
        ),
    )
    parser.add_argument(
        "--feedback-position-offsets",
        "--motor-status-position-offsets",
        dest="feedback_position_offsets",
        default=None,
        help=(
            "Six comma-separated rad offsets for JointState correction, "
            "e.g. '0,0,0,0,0,0'."
        ),
    )
    parser.add_argument(
        "--revo2-position-command-scale",
        type=float,
        default=572.9577951308232,
        help="Deprecated; rad-native retarget control no longer uses this scale.",
    )
    parser.add_argument(
        "--revo2-velocity-command-scale",
        type=float,
        default=1.0,
        help=(
            "Deprecated; rad-native pd_velocity publishes rad/s directly."
        ),
    )
    parser.add_argument(
        "--left-position-command-topic",
        default="/revo2_left/joint_forward_pos_controller/commands",
        help="Left Revo2 position controller command topic used by pd_position_speed.",
    )
    parser.add_argument(
        "--right-position-command-topic",
        default="/revo2_right/joint_forward_pos_controller/commands",
        help="Right Revo2 position controller command topic used by pd_position_speed.",
    )
    parser.add_argument(
        "--left-velocity-command-topic",
        default="/revo2_left/joint_forward_vel_controller/commands",
        help="Left Revo2 velocity controller command topic.",
    )
    parser.add_argument(
        "--right-velocity-command-topic",
        default="/revo2_right/joint_forward_vel_controller/commands",
        help="Right Revo2 velocity controller command topic.",
    )
    parser.add_argument(
        "--left-joint-state-topic",
        default="/revo2_left/revo2_joint_state/joint_states",
        help="Left Revo2 joint state feedback topic.",
    )
    parser.add_argument(
        "--right-joint-state-topic",
        default="/revo2_right/revo2_joint_state/joint_states",
        help="Right Revo2 joint state feedback topic.",
    )
    parser.add_argument(
        "--pd-debug",
        action="store_true",
        help="Print pd_velocity target/feedback/error/speed debug values.",
    )
    parser.add_argument(
        "--pd-debug-joint",
        type=int,
        default=2,
        help="Joint index to print for --pd-debug, 0-5.",
    )
    parser.add_argument(
        "--pd-debug-joints",
        default=None,
        help=(
            "Comma-separated joint indices to print for --pd-debug, e.g. "
            "'0,1,4'. Overrides --pd-debug-joint when provided."
        ),
    )
    parser.add_argument(
        "--pd-debug-interval",
        type=float,
        default=0.2,
        help="Seconds between --pd-debug prints.",
    )
    parser.add_argument(
        "--pd-debug-file",
        default=None,
        help="Optional text file path to append --pd-debug output.",
    )
    raw_argv = list(argv) if argv is not None else sys.argv[1:]
    args, ros_args = parser.parse_known_args(raw_argv)
    _apply_control_config(args, raw_argv, parser)
    if args.calibration_file is None:
        args.calibration_file = str(
            Path.home() / f".manus_revo2/thumb_calibration_{args.hand_mode}.json"
        )
    return args, ros_args


def main(argv=None):
    try:
        if argv is None:
            argv = sys.argv[1:]
        cli_args, ros_args = parse_cli_args(argv)
        retargeter = ManusRevo2Retargeter(
            hand_mode=cli_args.hand_mode,
            ros_args=ros_args,
            skip_calibration=cli_args.skip_calibration,
            calibration_file=cli_args.calibration_file,
            use_default_revo3_calibration=cli_args.use_default_revo3_calibration,
            revo3_thumb_params=getattr(cli_args, "revo3_thumb_params", None),
            config_file=cli_args.config_file,
            algorithm=cli_args.algorithm,
            end_effector_mode=cli_args.end_effector,
            pd_speed_control=cli_args.pd_speed_control,
            pd_kp=cli_args.pd_kp,
            pd_kd=cli_args.pd_kd,
            pd_derivative_alpha=cli_args.pd_derivative_alpha,
            target_filter_alpha=cli_args.target_filter_alpha,
            target_filter_fast_alpha=cli_args.target_filter_fast_alpha,
            target_filter_fast_threshold=cli_args.target_filter_fast_threshold,
            dead_zone=cli_args.dead_zone,
            control_mode=cli_args.control_mode,
            velocity_kp=cli_args.velocity_kp,
            velocity_kd=cli_args.velocity_kd,
            velocity_deadband=cli_args.velocity_deadband,
            thumb_velocity_deadband=cli_args.thumb_velocity_deadband,
            thumb_velocity_min=cli_args.thumb_velocity_min,
            thumb_velocity_kp_scale=cli_args.thumb_velocity_kp_scale,
            thumb_velocity_brake_zone=cli_args.thumb_velocity_brake_zone,
            thumb_velocity_brake_max=cli_args.thumb_velocity_brake_max,
            ring_velocity_deadband=cli_args.ring_velocity_deadband,
            ring_velocity_min=cli_args.ring_velocity_min,
            ring_velocity_kp_scale=cli_args.ring_velocity_kp_scale,
            four_finger_extension_velocity_scale=cli_args.four_finger_extension_velocity_scale,
            four_finger_extension_joints=cli_args.four_finger_extension_joints,
            velocity_max=cli_args.velocity_max,
            velocity_slew_rate=cli_args.velocity_slew_rate,
            velocity_feedback_timeout=cli_args.velocity_feedback_timeout,
            feedback_position_scale=cli_args.feedback_position_scale,
            feedback_position_scales=cli_args.feedback_position_scales,
            feedback_position_offsets=cli_args.feedback_position_offsets,
            revo2_position_command_scale=cli_args.revo2_position_command_scale,
            revo2_velocity_command_scale=cli_args.revo2_velocity_command_scale,
            left_position_command_topic=cli_args.left_position_command_topic,
            right_position_command_topic=cli_args.right_position_command_topic,
            left_velocity_command_topic=cli_args.left_velocity_command_topic,
            right_velocity_command_topic=cli_args.right_velocity_command_topic,
            left_joint_state_topic=cli_args.left_joint_state_topic,
            right_joint_state_topic=cli_args.right_joint_state_topic,
            mirror_sim_output=cli_args.mirror_sim_output,
            left_sim_command_topic=cli_args.left_sim_command_topic,
            right_sim_command_topic=cli_args.right_sim_command_topic,
            target_only=cli_args.target_only,
            left_target_joint_state_topic=cli_args.left_target_joint_state_topic,
            right_target_joint_state_topic=cli_args.right_target_joint_state_topic,
            pd_debug=cli_args.pd_debug,
            pd_debug_joint=cli_args.pd_debug_joint,
            pd_debug_joints=cli_args.pd_debug_joints,
            pd_debug_interval=cli_args.pd_debug_interval,
            pd_debug_file=cli_args.pd_debug_file,
        )
        retargeter.run()
    except Exception as e:
        logger.error(f"程序运行出错: {e}")
    finally:
        logger.info("程序已退出")


if __name__ == "__main__":
    main()
