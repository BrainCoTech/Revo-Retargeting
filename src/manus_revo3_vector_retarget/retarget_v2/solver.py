"""Whole-hand direction + grasp-vector optimization, authored independently."""
from dataclasses import dataclass, asdict
import time
import numpy as np
from scipy.optimize import least_squares

from .input import FINGERS


@dataclass
class SolverConfig:
    direction_weight: float = 1.0
    reach_weight: float = .4
    pair_weight: float = .45
    pinch_weight: float = 5.0
    pinch_on_m: float = .014
    pinch_off_m: float = .035
    pinch_gap_m: float = .008
    smooth_weight: float = .045
    posture_weight: float = .008
    clearance_weight: float = .7
    max_evaluations: int = 18
    velocity_limit: float = 2.0
    acceleration_limit: float = 8.0

    def __post_init__(self):
        values = np.array(list(asdict(self).values()), dtype=float)
        if not np.isfinite(values).all() or np.any(values <= 0):
            raise ValueError("Solver parameters must be finite positive values")
        if self.pinch_on_m >= self.pinch_off_m or self.pinch_weight < self.pair_weight:
            raise ValueError("Invalid pinch blend interval/weights")
        if not isinstance(self.max_evaluations, int):
            raise ValueError("max_evaluations must be an integer")


class Retargeter:
    def __init__(self, robot, calibration, config=None, joint_limits=None):
        hand_side = getattr(robot, "hand_side", "right")
        if getattr(calibration, "hand_side", hand_side) != hand_side:
            raise ValueError("Calibration hand side does not match the robot model")
        self.robot, self.calibration = robot, calibration
        self.config = config or SolverConfig()
        self.lower, self.upper = robot.lower.copy(), robot.upper.copy()
        if joint_limits is not None:
            if len(joint_limits) != 2:
                raise ValueError("joint_limits must contain lower and upper model-order arrays")
            lower, upper = (np.asarray(value, dtype=float) for value in joint_limits)
            if (lower.shape != self.lower.shape or upper.shape != self.upper.shape
                    or not np.isfinite(lower).all() or not np.isfinite(upper).all()
                    or np.any(lower < self.lower - 1e-12) or np.any(upper > self.upper + 1e-12)
                    or np.any(upper - lower <= 2e-6)):
                raise ValueError("joint_limits must be finite nonempty intervals inside the model URDF bounds")
            self.lower, self.upper = lower.copy(), upper.copy()
        self.q = np.clip(np.full(len(robot.joint_names), .08), self.lower+1e-6, self.upper-1e-6)
        index = {n: i for i, n in enumerate(robot.point_names)}
        self.chain_ids = np.array([[index[f"{f}_{p}"] for p in ("root", "pip", "dip", "tip")] for f in FINGERS])
        self.lengths = np.asarray(robot.finger_lengths)
        self.scale = float(np.median(self.lengths.sum(axis=1)/calibration.lengths.sum(axis=1)))
        rest = robot.points(np.zeros(len(robot.joint_names)))[self.chain_ids]
        fallback = rest[0, -1]-rest[1:, -1]
        self.pair_fallback = fallback/np.linalg.norm(fallback, axis=1)[:, None]
        self.last = None
        self.count = 0
        # Non-adjacent finger centreline proxies, kept separate from true mesh collision.
        self.clearance_pairs = np.array([(a, b) for a in range(15) for b in range(a+1, 15) if a//3 != b//3])

    def targets(self, message):
        features = self.calibration.features(message)
        features["directions_world"] = features["directions"] @ self.robot.palm_basis.T
        vectors = features["tip_vectors"] @ self.robot.palm_basis.T
        d = features["tip_distances"]
        c = self.config
        u = np.clip((c.pinch_off_m-d)/(c.pinch_off_m-c.pinch_on_m), 0., 1.)
        blend = u*u*(3-2*u)
        target_d = (1-blend)*d*self.scale + blend*c.pinch_gap_m
        direction = vectors / np.maximum(d[:, None], 1e-8)
        # Contact direction is undefined at coincidence. Preserve the last valid
        # direction, falling back to the model's neutral separation on startup.
        missing = d < 1e-6
        direction[missing] = self.pair_fallback[missing]
        self.pair_fallback[~missing] = direction[~missing]
        features["pair_target"] = direction * target_d[:, None]
        features["pair_weights"] = c.pair_weight + (c.pinch_weight-c.pair_weight)*blend
        features["pinch_blend"] = blend
        features["reach_target"] = np.sum(features["directions_world"]*self.lengths[:, :, None], axis=1)
        return features

    def evaluate(self, q, target, previous):
        points, jac = self.robot.points(q, jacobian=True)
        chain, cj = points[self.chain_ids], jac[self.chain_ids]
        segments, sj = np.diff(chain, axis=1), np.diff(cj, axis=1)
        # Each segment length is a model constant. These residuals compare shape,
        # keeping the human and robot lengths independent.
        dirs, dj = segments/self.lengths[:, :, None], sj/self.lengths[:, :, None, None]
        c = self.config
        direction_w = np.sqrt(c.direction_weight*np.array([1., .8, .55]))[None, :, None]
        residuals = [((dirs-target["directions_world"])*direction_w).ravel()]
        jacobians = [(dj*direction_w[..., None]).reshape(-1, len(q))]
        total_length = self.lengths.sum(axis=1)
        reach = chain[:, -1]-chain[:, 0]
        rj = cj[:, -1]-cj[:, 0]
        reach_w = np.sqrt(c.reach_weight)/total_length[:, None]
        residuals.append(((reach-target["reach_target"])*reach_w).ravel())
        jacobians.append((rj*reach_w[..., None]).reshape(-1, len(q)))
        pair = chain[0, -1]-chain[1:, -1]
        pj = cj[0, -1]-cj[1:, -1]
        pair_w = np.sqrt(target["pair_weights"][:, None])/.08
        residuals.append(((pair-target["pair_target"])*pair_w).ravel())
        jacobians.append((pj*pair_w[..., None]).reshape(-1, len(q)))
        # Soft centreline clearance discourages gross crossings; this is a proxy,
        # not a hardware collision certificate.
        samples = np.stack(((chain[:, 1]+chain[:, 2])*.5, (chain[:, 2]+chain[:, 3])*.5, chain[:, 3]), axis=1).reshape(-1, 3)
        sample_j = np.stack(((cj[:, 1]+cj[:, 2])*.5, (cj[:, 2]+cj[:, 3])*.5, cj[:, 3]), axis=1).reshape(-1, 3, len(q))
        a, b = self.clearance_pairs.T
        delta, delta_j = samples[a]-samples[b], sample_j[a]-sample_j[b]
        distance = np.linalg.norm(delta, axis=1)
        # 6 mm around distal cap sites; 12 mm for interior centreline samples.
        minimum = np.where((a % 3 == 2) & (b % 3 == 2), .006, .012)
        active = distance < minimum
        cw = np.sqrt(c.clearance_weight)/.02
        residuals.append(np.maximum(0., minimum-distance)*cw)
        djc = -np.einsum('ij,ijk->ik', delta/np.maximum(distance[:, None], 1e-8), delta_j)*cw
        jacobians.append(djc*active[:, None])
        residuals.extend([np.sqrt(c.smooth_weight)*(q-previous), np.sqrt(c.posture_weight)*q])
        jacobians.extend([np.eye(len(q))*np.sqrt(c.smooth_weight), np.eye(len(q))*np.sqrt(c.posture_weight)])
        return np.concatenate(residuals), np.concatenate(jacobians)

    def solve(self, message):
        target = self.targets(message)
        previous = self.q.copy()
        started = time.perf_counter()
        cache_q, cache_result = None, None
        def evaluate(q):
            nonlocal cache_q, cache_result
            if cache_q is None or not np.array_equal(q, cache_q):
                cache_result = self.evaluate(q, target, previous)
                cache_q = q.copy()
            return cache_result
        initial = evaluate(previous)[0]
        result = least_squares(lambda q: evaluate(q)[0], previous, jac=lambda q: evaluate(q)[1],
                               bounds=(self.lower+1e-7, self.upper-1e-7),
                               max_nfev=80 if self.count == 0 else self.config.max_evaluations,
                               ftol=2e-4, xtol=2e-4, gtol=2e-4, x_scale='jac')
        accepted = (np.isfinite(result.x).all() and np.dot(result.fun, result.fun) <= np.dot(initial, initial)+1e-8)
        if accepted:
            self.q = result.x.copy()
        self.count += 1
        self.last = target
        return self.q.copy(), dict(accepted=bool(accepted), converged=bool(result.success), nfev=int(result.nfev),
                                  solve_ms=(time.perf_counter()-started)*1000,
                                  objective=float(np.dot(result.fun, result.fun)))

    def metrics(self, q, target):
        points = self.robot.points(q)
        chain = points[self.chain_ids]
        directions = np.diff(chain, axis=1)/self.lengths[:, :, None]
        cos = np.clip(np.sum(directions*target["directions_world"], axis=2), -1, 1)
        pair = chain[0, -1]-chain[1:, -1]
        reach = chain[:, -1]-chain[:, 0]
        return dict(direction_deg=np.rad2deg(np.arccos(cos)),
                    pair_error_mm=np.linalg.norm(pair-target["pair_target"], axis=1)*1000,
                    robot_tip_distance_mm=np.linalg.norm(pair, axis=1)*1000,
                    human_tip_distance_mm=target["tip_distances"]*1000,
                    pinch_blend=target["pinch_blend"],
                    reach_error_mm=np.linalg.norm(reach-target["reach_target"], axis=1)*1000)
