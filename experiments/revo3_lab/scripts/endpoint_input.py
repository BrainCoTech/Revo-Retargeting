"""Source adapters for five fingertip targets in robot world coordinates.

Finger order is thumb, index, middle, ring, little. Missing observations stay
missing; holding or returning to neutral belongs to the caller's controller.
The optional bone_scaled adapter anchors each source chain's unit directions
at the robot's neutral finger root and uses robot bone lengths. The solver
still receives only endpoints and optional distal directions.
"""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass
class EndpointTargets:
    timestamp_s: float
    positions_m: np.ndarray
    valid: np.ndarray
    directions: np.ndarray | None = None
    direction_valid: np.ndarray | None = None
    source_kind: str = 'unknown'

    def __post_init__(self):
        self.timestamp_s = float(self.timestamp_s)
        if not np.isfinite(self.timestamp_s):
            raise ValueError('timestamp_s must be finite')
        self.positions_m = np.array(self.positions_m, dtype=float, copy=True)
        self.valid = np.array(self.valid, dtype=bool, copy=True)
        if self.positions_m.shape != (5, 3) or self.valid.shape != (5,):
            raise ValueError('positions_m and valid must have shapes (5, 3) and (5,)')
        if not np.isfinite(self.positions_m[self.valid]).all():
            raise ValueError('valid positions must be finite')
        self.positions_m[~self.valid] = 0.
        if self.directions is None:
            if self.direction_valid is not None:
                raise ValueError('direction_valid requires directions')
            return
        self.directions = np.array(self.directions, dtype=float, copy=True)
        self.direction_valid = (self.valid.copy() if self.direction_valid is None
                                else np.array(self.direction_valid, dtype=bool, copy=True))
        if self.directions.shape != (5, 3) or self.direction_valid.shape != (5,):
            raise ValueError('directions and direction_valid must have shapes (5, 3) and (5,)')
        if np.any(self.direction_valid & ~self.valid):
            raise ValueError('A valid direction requires a valid fingertip position')
        if not np.isfinite(self.directions[self.direction_valid]).all():
            raise ValueError('valid directions must be finite')
        lengths = np.linalg.norm(self.directions[self.direction_valid], axis=1)
        if np.any(lengths <= 1e-12) or not np.isfinite(lengths).all():
            raise ValueError('valid directions must have nonzero finite length')
        self.directions[self.direction_valid] /= lengths[:, None]
        self.directions[~self.direction_valid] = 0.


def _transform(basis, wrist, scale):
    basis = np.asarray(basis, dtype=float)
    wrist = np.asarray(wrist, dtype=float)
    scale = float(scale)
    if basis.shape != (3, 3) or wrist.shape != (3,):
        raise ValueError('basis and wrist must have shapes (3, 3) and (3,)')
    if not np.isfinite(basis).all() or not np.isfinite(wrist).all():
        raise ValueError('basis and wrist must be finite')
    if not np.allclose(basis.T @ basis, np.eye(3), atol=1e-6):
        raise ValueError('basis must be orthonormal')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be finite and positive')
    return basis, wrist, scale


def _missing(timestamp_s, source_kind, include_directions):
    return EndpointTargets(timestamp_s, np.zeros((5, 3)), np.zeros(5, dtype=bool),
                           np.zeros((5, 3)) if include_directions else None,
                           np.zeros(5, dtype=bool) if include_directions else None,
                           source_kind)


def _extract(points, tips, dips, timestamp_s, basis, wrist, scale,
             source_kind, include_directions, target_mode='wrist_scaled', robot_rest_chains_m=None,
             chains=None):
    if target_mode == 'bone_scaled':
        rest = _reference_chains(robot_rest_chains_m)
        vectors = np.diff(points[chains], axis=1)
        lengths = np.linalg.norm(vectors, axis=2, keepdims=True)
        good = np.isfinite(vectors).all(axis=2, keepdims=True) & np.isfinite(lengths) & (lengths > 1e-8)
        directions = np.divide(vectors, lengths, out=np.zeros_like(vectors), where=good)
        robot_lengths = np.linalg.norm(np.diff(rest, axis=1), axis=2, keepdims=True)
        local_tips = rest[:,0] + scale*np.sum(directions*robot_lengths, axis=1)
        positions = local_tips @ basis.T + wrist
        positions[~good.all(axis=(1,2))] = np.nan
    elif target_mode == 'wrist_scaled':
        positions = points[tips] * scale @ basis.T + wrist
    else:
        raise ValueError('target_mode must be wrist_scaled or bone_scaled')
    valid = np.isfinite(positions).all(axis=1)
    directions = direction_valid = None
    if include_directions:
        directions = (points[tips] - points[dips]) @ basis.T
        lengths = np.linalg.norm(directions, axis=1)
        direction_valid = (valid & np.isfinite(directions).all(axis=1)
                           & np.isfinite(lengths) & (lengths > 1e-12))
    return EndpointTargets(timestamp_s, positions, valid, directions,
                           direction_valid, source_kind)


def _reference_chains(rest):
    rest = np.asarray(rest, dtype=float)
    if rest.shape != (5,4,3) or not np.isfinite(rest).all():
        raise ValueError('bone_scaled requires finite robot_rest_chains_m with shape (5,4,3) in palm coordinates')
    if np.any(np.linalg.norm(np.diff(rest,axis=1),axis=2) <= 1e-8):
        raise ValueError('Robot reference chain contains a zero-length bone')
    return rest


def from_manus(points, timestamp_s, *, basis, wrist, scale,
               source_kind='recorded_manus', include_directions=False,
               target_mode='wrist_scaled', robot_rest_chains_m=None):
    """Extract endpoints from already canonicalized/reflected MANUS 25 points."""
    basis, wrist, scale = _transform(basis, wrist, scale)
    points = np.asarray(points, dtype=float)
    if points.shape != (25, 3):
        raise ValueError('MANUS points must have shape (25, 3)')
    return _extract(points, [4, 9, 14, 19, 24], [3, 8, 13, 18, 23],
                    timestamp_s, basis, wrist, scale, source_kind, include_directions,
                    target_mode, robot_rest_chains_m,
                    [[1,2,3,4],[6,7,8,9],[11,12,13,14],[16,17,18,19],[21,22,23,24]])


def from_mediapipe(world, timestamp_s, *, hand_side, basis, wrist, scale,
                   palm_x_sign=1.0, include_directions=False,
                   target_mode='wrist_scaled', robot_rest_chains_m=None):
    """Canonicalize native 21-point observations, then extract endpoints.

    Palm axes: z wrist-to-middle MCP; y little-to-index MCP, orthogonalized
    against z; x = y cross z. Left-hand x reflection precedes the explicit
    calibrated palm_x_sign. Neither reflection is a display mirroring option.
    Degenerate/missing palm geometry invalidates the whole observation.
    """
    basis, wrist, scale = _transform(basis, wrist, scale)
    if hand_side not in ('Left', 'Right'):
        raise ValueError('hand_side must be Left or Right')
    if palm_x_sign not in (-1., 1.):
        raise ValueError('palm_x_sign must be -1 or 1')
    if target_mode not in ('wrist_scaled', 'bone_scaled'):
        raise ValueError('target_mode must be wrist_scaled or bone_scaled')
    if target_mode == 'bone_scaled':
        _reference_chains(robot_rest_chains_m)
    if world is None:
        return _missing(timestamp_s, 'mediapipe', include_directions)
    points = np.asarray(world, dtype=float)
    if points.shape != (21, 3):
        raise ValueError('MediaPipe world landmarks must have shape (21, 3)')
    palm = points[[0, 5, 9, 17]]
    if not np.isfinite(palm).all():
        return _missing(timestamp_s, 'mediapipe', include_directions)
    z = points[9] - points[0]
    z_length = np.linalg.norm(z)
    if not np.isfinite(z_length) or z_length <= 1e-8:
        return _missing(timestamp_s, 'mediapipe', include_directions)
    z = z / z_length
    lateral = points[5] - points[17]
    y = lateral - (lateral @ z) * z
    y_length = np.linalg.norm(y)
    if not np.isfinite(y_length) or y_length <= 1e-8:
        return _missing(timestamp_s, 'mediapipe', include_directions)
    y = y / y_length
    x = np.cross(y, z) * (1. if hand_side == 'Right' else -1.) * palm_x_sign
    local = (points - points[0]) @ np.stack([x, y, z], axis=1)
    return _extract(local, [4, 8, 12, 16, 20], [3, 7, 11, 15, 19],
                    timestamp_s, basis, wrist, scale, 'mediapipe', include_directions,
                    target_mode, robot_rest_chains_m, np.arange(1,21).reshape(5,4))


def iter_mediapipe_jsonl(path, *, hand_side='Right', input_field='raw'):
    """Yield (source timestamp, native world points or None, untouched row).

    raw selects exactly one matching detection with score >= .7. mapper
    replays the saved mapper input, including injected noise and dropouts.
    Missing/ambiguous detections preserve the frame and its timestamp.
    """
    if hand_side not in ('Left', 'Right'):
        raise ValueError('hand_side must be Left or Right')
    if input_field not in ('raw', 'mapper'):
        raise ValueError('input_field must be raw or mapper')
    previous = None
    with Path(path).open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            timestamp = float(row['timestamp_s'])
            if not np.isfinite(timestamp) or (previous is not None and timestamp <= previous):
                raise ValueError(f'{path}:{line_number}: timestamps must be finite and strictly increasing')
            previous = timestamp
            if input_field == 'mapper':
                if row.get('selected_side') != hand_side:
                    raise ValueError(f'{path}:{line_number}: selected_side does not match {hand_side}')
                world = row['mapper_world_landmarks_m']
            else:
                candidates = [d for d in row.get('detections', [])
                              if d.get('side') == hand_side
                              and float(d.get('handedness_score', 0.)) >= .7]
                world = candidates[0]['world_landmarks_m'] if len(candidates) == 1 else None
            if world is not None:
                world = np.array(world, dtype=float, copy=True)
                if world.shape != (21, 3):
                    raise ValueError(f'{path}:{line_number}: world landmarks must have shape (21, 3)')
            yield timestamp, world, row
