"""Named model/ROS hardware boundary, independent of any legacy retarget solver.

Model-specific profiles preserve the archived motor-channel command envelope.
The May3 profile retains this workspace's ROS joint coordinates, not a physical
calibration; thumb slot names retain their historical ROS/SDK aliases.
"""
from dataclasses import dataclass, fields
from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from .paths import default_profile


@dataclass(frozen=True)
class CommandConfig:
    publish_hz: float = 200.0
    startup_duration_s: float = 2.0
    target_timeout_s: float = 0.25
    max_velocity_rad_s: float = 0.9
    max_acceleration_rad_s2: float = 1.92
    max_timer_gap_s: float = 0.1
    velocity_feedforward: bool = True

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == "velocity_feedforward":
                if type(value) is not bool:
                    raise ValueError("velocity_feedforward must be a boolean")
                continue
            _number(value, field.name)
            if value <= 0:
                raise ValueError(f"{field.name} must be positive")


class _UniqueLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ValueError(f"Profile keys must be unique strings: {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def _mapping(value, expected, context):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(f"{context} must contain exactly {sorted(expected)}")
    return value


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError(f"{name} must be a finite number")
    if not np.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _vector(value, name):
    try:
        result = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must contain 21 finite numbers") from exc
    if result.shape != (21,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain 21 finite numbers")
    return result


# Archived May3 motor-channel bounds (radians), independent of an editable YAML.
# Source URDF SHA256: eacb397ba358e347e691c6b3ed31bb76a33b6231ff6cf2efd003232b0d6898f4.
# The workspace May3 model and its ROS channels use these same named bounds.
_HARDWARE_STEMS = tuple(
    f"{finger}_{joint}_joint"
    for finger in ("little", "ring", "middle", "index")
    for joint in ("MPR", "MCP", "PIP", "DIP")
) + tuple(f"thumb_{joint}_joint" for joint in ("MCP", "PIP", "DIP", "CMP", "CMR"))
_ARCHIVED_HARDWARE_BOUNDS = np.array(
    [(-.2618, .2618), (0., 1.4835), (0., 1.4835), (0., 1.4835)] * 4
    + [(0., .8727), (0., 1.4835), (0., 1.4835), (0., 1.9199), (0., 2.0071)]
)


class HardwareProfile:
    """Named model/ROS mapping with a model-specific trial command envelope.

    ``lower``/``upper`` are hardware-channel bounds; ``model_lower``/
    ``model_upper`` are their inverse in model order for constrained solving.
    RobotModel's source URDF bounds are never changed. Feedback conversion
    retains excursions so diagnostics can report measured boundary crossings.

    Velocity and acceleration limits come from the editable profile or explicit
    overrides, with no hardcoded ceiling; they must be finite and positive.
    Explicit motion_scale scales velocity linearly and acceleration quadratically;
    independent overrides cannot be combined with an explicit motion_scale.
    """

    def __init__(self, robot, path=None, motion_scale=None,
                 max_velocity_rad_s=None, max_acceleration_rad_s2=None):
        self.model_version = getattr(robot, "model_version", None)
        self.hand_side = getattr(robot, "hand_side", "right")
        if self.hand_side not in ("left", "right"):
            raise ValueError("HardwareProfile hand_side must be left or right")
        if self.model_version != "may3":
            raise ValueError("HardwareProfile requires model_version='may3' from current revo3_description")
        self.path = Path(path) if path is not None else default_profile(self.model_version, self.hand_side)
        document = yaml.load(self.path.read_text(), Loader=_UniqueLoader)
        schema = {"command", "gains", "output_calibration"}
        if self.model_version == "may3":
            schema |= {"model_version", "urdf_sha256", "mapping_metadata", "hardware_channel_bounds"}
            if isinstance(document, dict) and "hand_side" in document:
                schema.add("hand_side")
        _mapping(document, schema, f"{self.model_version} profile")
        # Existing right-hand profiles predate the explicit side field.
        if document.get("hand_side", "right") != self.hand_side:
            raise ValueError("Profile hand_side does not match the robot")
        self.mapping_metadata = {"status": "legacy_direction_acceptance_pending"}
        if self.model_version == "may3":
            if document["model_version"] != self.model_version:
                raise ValueError("Profile model_version does not match the robot")
            actual_hash = sha256(Path(robot.urdf_path).read_bytes()).hexdigest()
            if actual_hash != document["urdf_sha256"]:
                raise ValueError("Profile URDF SHA256 does not match current revo3_description; update the profile after reviewing model changes")
            metadata = _mapping(document["mapping_metadata"],
                                ("status", "mpr_sign_basis", "encoder_zero_basis", "thumb_alias_policy"),
                                "mapping_metadata")
            if metadata["status"] != "provisional_not_physically_calibrated":
                raise ValueError("May3 mapping must remain provisional_not_physically_calibrated")
            if any(not isinstance(value, str) or not value.strip() for value in metadata.values()):
                raise ValueError("Mapping metadata must contain nonempty strings")
            self.mapping_metadata = dict(metadata)

        if motion_scale is not None and (max_velocity_rad_s is not None or max_acceleration_rad_s2 is not None):
            raise ValueError("Explicit motion_scale cannot be combined with independent motion-limit overrides")
        self.motion_scale = _number(
            (.2 if self.model_version == "legacy" else 1.) if motion_scale is None else motion_scale,
            "motion_scale")
        if not 0 < self.motion_scale <= 1:
            raise ValueError("motion_scale must be in (0, 1]")
        command = _mapping(document["command"], [field.name for field in fields(CommandConfig)], "command")
        # Validate unscaled YAML as well as final values; scaling cannot hide bad data.
        CommandConfig(**command)
        command = dict(command)
        command["max_velocity_rad_s"] *= self.motion_scale
        command["max_acceleration_rad_s2"] *= self.motion_scale ** 2
        for key, value in (("max_velocity_rad_s", max_velocity_rad_s),
                           ("max_acceleration_rad_s2", max_acceleration_rad_s2)):
            if value is not None:
                command[key] = _number(value, key)
        self.command = CommandConfig(**command)

        self.hardware_names = tuple(f"{self.hand_side}_{stem}" for stem in _HARDWARE_STEMS)
        self.names = self.hardware_names
        self.model_names = tuple(robot.joint_names)
        if len(self.model_names) != 21 or len(set(self.model_names)) != 21 or set(self.model_names) != set(self.names):
            raise ValueError(f"Model must contain the same 21 named {self.hand_side}-hand joints as the hardware")
        model_index = {name: index for index, name in enumerate(self.model_names)}
        self._hardware_to_model = np.array([model_index[name] for name in self.names])
        source_lower = _vector(robot.lower, "Model lower limits")[self._hardware_to_model]
        source_upper = _vector(robot.upper, "Model upper limits")[self._hardware_to_model]
        urdf_joints = {joint.attrib["name"]: joint for joint in ET.parse(robot.urdf_path).getroot().findall("joint")}
        for index, name in enumerate(self.names):
            joint = urdf_joints.get(name)
            if joint is None or joint.get("type") != "revolute" or joint.find("limit") is None:
                raise ValueError(f"Missing revolute URDF limit for {name}")
            bounds = joint.find("limit").attrib
            expected = [float(bounds["lower"]), float(bounds["upper"])]
            if not np.isfinite(expected).all() or expected[0] >= expected[1] or not np.allclose(
                    [source_lower[index], source_upper[index]], expected, atol=1e-12, rtol=0.):
                raise ValueError(f"Model and source URDF limits differ for {name}")

        gains = _mapping(document["gains"], ("default_kp", "default_kd", "by_joint"), "gains")
        self.kp = np.full(21, _number(gains["default_kp"], "default_kp"))
        self.kd = np.full(21, _number(gains["default_kd"], "default_kd"))
        overrides = gains["by_joint"]
        calibration = _mapping(document["output_calibration"], ("left", "right"), "output_calibration")
        other_side = "left" if self.hand_side == "right" else "right"
        if calibration[other_side] != {}:
            raise ValueError(f"The {self.hand_side}-hand profile must not contain {other_side}-hand calibration")
        calibration = calibration[self.hand_side]
        stems = [name.removeprefix(f"{self.hand_side}_") for name in self.names]
        if not isinstance(overrides, dict) or not isinstance(calibration, dict):
            raise ValueError("Joint gains and calibration must be mappings")
        if set(overrides) - set(stems) or set(calibration) - set(stems):
            raise ValueError("Unknown joint in gains/output calibration")
        if self.model_version == "may3" and set(calibration) != set(stems):
            raise ValueError("May3 output calibration must explicitly map all 21 named channels")
        self.scale, self.offset = np.ones(21), np.zeros(21)
        for index, stem in enumerate(stems):
            if stem in overrides:
                item = _mapping(overrides[stem], ("kp", "kd"), f"gains.{stem}")
                self.kp[index] = _number(item["kp"], f"{stem}.kp")
                self.kd[index] = _number(item["kd"], f"{stem}.kd")
            if stem in calibration:
                item = _mapping(calibration[stem], ("scale", "offset_deg"), f"calibration.{stem}")
                self.scale[index] = _number(item["scale"], f"{stem}.scale")
                if self.scale[index] not in (-1., 1.):
                    raise ValueError(f"{stem}.scale must be a sign, exactly -1 or +1")
                self.offset[index] = np.deg2rad(_number(item["offset_deg"], f"{stem}.offset_deg"))
        if np.any(self.kp < 0) or np.any(self.kd < 0):
            raise ValueError("Gains must be nonnegative")
        endpoints = np.stack((source_lower * self.scale + self.offset,
                              source_upper * self.scale + self.offset))
        mapped_lower, mapped_upper = endpoints.min(axis=0), endpoints.max(axis=0)
        if self.model_version == "legacy":
            if not (np.allclose(mapped_lower, source_lower, atol=1e-12, rtol=0.)
                    and np.allclose(mapped_upper, source_upper, atol=1e-12, rtol=0.)):
                raise ValueError("Output calibration must preserve the same named URDF bounds")
            self.lower, self.upper = source_lower.copy(), source_upper.copy()
        else:
            # A custom profile may narrow the archive, but may never expand it.
            bounds = _mapping(document["hardware_channel_bounds"], self.names, "hardware_channel_bounds")
            archived_lower, archived_upper = _ARCHIVED_HARDWARE_BOUNDS.T
            requested_lower, requested_upper = np.empty(21), np.empty(21)
            for index, name in enumerate(self.names):
                item = _mapping(bounds[name], ("lower_rad", "upper_rad"), f"bounds.{name}")
                requested_lower[index] = _number(item["lower_rad"], f"{name}.lower_rad")
                requested_upper[index] = _number(item["upper_rad"], f"{name}.upper_rad")
            if np.any(requested_lower >= requested_upper):
                raise ValueError("Hardware channel bounds must be nonempty and increasing")
            if np.any(requested_lower < archived_lower) or np.any(requested_upper > archived_upper):
                raise ValueError("Official hardware bounds must not expand the archived motor-channel envelope")
            self.lower = np.maximum(mapped_lower, requested_lower)
            self.upper = np.minimum(mapped_upper, requested_upper)
            if np.any(self.lower >= self.upper):
                raise ValueError("Mapped URDF and hardware command envelopes have an empty intersection")
        inverse_endpoints = np.stack(((self.lower - self.offset) / self.scale,
                                      (self.upper - self.offset) / self.scale))
        self.model_lower, self.model_upper = np.empty(21), np.empty(21)
        self.model_lower[self._hardware_to_model] = inverse_endpoints.min(axis=0)
        self.model_upper[self._hardware_to_model] = inverse_endpoints.max(axis=0)

    def ordered(self, names, values):
        """Reorder a complete, uniquely named feedback array to hardware order."""
        names = tuple(names)
        if len(names) != 21 or any(not isinstance(name, str) for name in names) or len(set(names)) != 21:
            raise ValueError("Expected 21 unique joint names")
        if set(names) != set(self.names):
            raise ValueError("Feedback joint names do not match the 21 hardware joints")
        values = _vector(values, "Named feedback")
        index = {name: index for index, name in enumerate(names)}
        return values[[index[name] for name in self.names]].copy()

    def model_to_hardware(self, q):
        q = _vector(q, "Model position")
        result = q[self._hardware_to_model] * self.scale + self.offset
        if np.any(result < self.lower) or np.any(result > self.upper):
            raise ValueError("Model target maps outside hardware command envelope")
        return result

    to_hardware = model_to_hardware

    def feedback_to_model(self, q):
        q = _vector(q, "Hardware feedback")
        result = np.empty(21)
        result[self._hardware_to_model] = (q - self.offset) / self.scale
        return result
