"""Calibrated HumanDex flexion from DIP/PIP/MCP, in upstream joint units."""

import math


FINGERS = ("index", "middle", "ring", "little")
JOINTS = tuple(f"{finger}_{joint}" for finger in FINGERS for joint in ("dip", "pip", "mcp"))


class CalibratedFingerFlexion:
    def __init__(self, opened, closed, weights, output_range, wrap_angles=False):
        self.opened = tuple(float(v) for v in opened)
        self.closed = tuple(float(v) for v in closed)
        self.weights = tuple(float(v) for v in weights)
        self.output_range = float(output_range)
        self.wrap_angles = bool(wrap_angles)
        values = self.opened + self.closed + self.weights + (self.output_range,)
        if (len(self.opened) != 12 or len(self.closed) != 12 or len(self.weights) != 3
                or not all(math.isfinite(v) for v in values)
                or self.output_range <= 0 or any(w < 0 for w in self.weights)
                or sum(self.weights) <= 0
                or any(abs(c - o) < 1e-9 for o, c in zip(self.opened, self.closed))):
            raise ValueError("Expected 12 finite open/closed angles with nonzero spans, "
                             "3 nonnegative weights and a positive output range")
        self.spans = tuple(self._delta(c, o) for o, c in zip(self.opened, self.closed))
        if any(abs(s) < 1e-9 for s in self.spans):
            raise ValueError("open/closed endpoints must differ after angle wrapping")

    def _delta(self, value, reference):
        delta = value - reference
        return (delta + math.pi) % (2 * math.pi) - math.pi if self.wrap_angles else delta

    def compute(self, measured):
        # Require all bending joints; never silently substitute a missing joint.
        values = [measured[name] for name in JOINTS]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite HumanDex bending joint")
        normalized = [min(1.0, max(0.0, self._delta(v, o) / span))
                      for v, o, span in zip(values, self.opened, self.spans)]
        total = sum(self.weights)
        return {f"{finger}_flexion": self.output_range * sum(
                    w * normalized[3 * i + j] for j, w in enumerate(self.weights)) / total
                for i, finger in enumerate(FINGERS)}


class Revo2FingerMapping:
    """Explicit robot-side mapping; never infer the algorithm from message.source."""
    def __init__(self, config):
        self.config = dict(config)
        self.mode = self.config.get("four_finger_mapping", "passthrough")
        if self.mode not in ("passthrough", "manus", "pip", "calibrated"):
            raise ValueError("Unknown Revo2 four_finger_mapping: " + str(self.mode))
        self.calibrations = {}
        self.span = float(self.config.get("four_finger_flexion_range_rad", 1.4661))
        self.opened = float(self.config.get("four_finger_open_rad", math.radians(12)))
        self.closed = float(self.config.get("four_finger_closed_rad", math.radians(-12)))
        if not all(math.isfinite(v) for v in (self.span, self.opened, self.closed)) or self.span <= 0 or abs(self.opened-self.closed) < 1e-9:
            raise ValueError("Invalid Revo2 flexion range")
        if self.mode == "calibrated":
            for side in ("left", "right"):
                key = f"{side}_four_finger_open_rad"
                if key in self.config:
                    self.calibrations[side] = CalibratedFingerFlexion(
                        self.config[key], self.config[f"{side}_four_finger_closed_rad"],
                        self.config.get("four_finger_weights", [1., 1., 1.]), self.span,
                        self.config.get("four_finger_wrap_angles", False))
            if not self.calibrations:
                raise ValueError("Calibrated mapping needs side-specific endpoints")

    @classmethod
    def from_file(cls, path):
        import yaml
        from pathlib import Path
        return cls(yaml.safe_load(Path(path).read_text()) or {})

    def apply(self, measured, side):
        result = dict(measured)
        if self.mode == "passthrough":
            return result
        # Remove any legacy aggregate supplied by the input; this profile owns it.
        for finger in FINGERS:
            result.pop(f"{finger}_flexion", None)
        if self.mode == "calibrated":
            if side not in self.calibrations:
                raise ValueError(f"No calibrated flexion endpoints for {side}")
            result.update(self.calibrations[side].compute(measured))
        elif self.mode == "pip":
            for finger in FINGERS:
                value = measured[f"{finger}_pip"]
                if not math.isfinite(value):
                    raise ValueError("Non-finite PIP")
                result[f"{finger}_flexion"] = self.span * min(1., max(0., (value-self.opened)/(self.closed-self.opened)))
        else:
            for finger in FINGERS:
                entries = [(measured[f"{finger}_{joint}"], weight) for joint, weight in (("mcp", .50), ("pip", .35), ("dip", .15)) if f"{finger}_{joint}" in measured]
                if entries:
                    if not all(math.isfinite(v) for v, _ in entries):
                        raise ValueError("Non-finite MANUS bending angle")
                    result[f"{finger}_flexion"] = sum(w * max(0., v) for v, w in entries)
        return result
