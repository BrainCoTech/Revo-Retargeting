"""Rigid palm-coordinate conversion; no morphology scaling or joint mapping."""

import math


class PalmTransform:
    def __init__(self, xyz, rpy):
        if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in (*xyz, *rpy)):
            raise ValueError("palm xyz/rpy must each contain three finite values")
        self.xyz = tuple(xyz)
        r, p, y = rpy
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        self.rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                         (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                         (-sp, cp*sr, cp*cr))

    def apply(self, xyz):
        return tuple(sum(a*b for a, b in zip(row, xyz)) + offset
                     for row, offset in zip(self.rotation, self.xyz))
