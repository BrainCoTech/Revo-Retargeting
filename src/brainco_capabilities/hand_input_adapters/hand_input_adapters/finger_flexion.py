"""Calibrated HumanDex flexion from DIP/PIP/MCP, in upstream joint units."""

import math


FINGERS = ("index", "middle", "ring", "little")
JOINTS = tuple(f"{finger}_{joint}" for finger in FINGERS for joint in ("dip", "pip", "mcp"))


class CalibratedFingerFlexion:
    def __init__(self, opened, closed, weights, output_range):
        self.opened = tuple(float(v) for v in opened)
        self.closed = tuple(float(v) for v in closed)
        self.weights = tuple(float(v) for v in weights)
        self.output_range = float(output_range)
        values = self.opened + self.closed + self.weights + (self.output_range,)
        if (len(self.opened) != 12 or len(self.closed) != 12 or len(self.weights) != 3
                or not all(math.isfinite(v) for v in values)
                or self.output_range <= 0 or any(w < 0 for w in self.weights)
                or sum(self.weights) <= 0
                or any(abs(c - o) < 1e-9 for o, c in zip(self.opened, self.closed))):
            raise ValueError("Expected 12 finite open/closed angles with nonzero spans, "
                             "3 nonnegative weights and a positive output range")

    def compute(self, measured):
        # Require all bending joints; never silently substitute a missing joint.
        values = [measured[name] for name in JOINTS]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite HumanDex bending joint")
        normalized = [min(1.0, max(0.0, (v - o) / (c - o)))
                      for v, o, c in zip(values, self.opened, self.closed)]
        total = sum(self.weights)
        return {f"{finger}_flexion": self.output_range * sum(
                    w * normalized[3 * i + j] for j, w in enumerate(self.weights)) / total
                for i, finger in enumerate(FINGERS)}
