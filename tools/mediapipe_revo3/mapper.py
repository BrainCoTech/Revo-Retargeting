"""21-point MediaPipe → official Revo3 kinematic vector IK (local baseline)."""
import mujoco
import numpy as np
from scipy.optimize import least_squares

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little']
CHAINS = [[0, 1, 2, 3, 4], [0, 5, 6, 7, 8], [0, 9, 10, 11, 12],
          [0, 13, 14, 15, 16], [0, 17, 18, 19, 20]]


def unit(v):
    norm = np.linalg.norm(v, axis=-1, keepdims=True)
    if np.any(norm < 1e-6):
        raise ValueError('degenerate_bone_or_palm')
    return v / norm


def canonical(points, side='Right'):
    """Palm axes: x palmar, y little→index, z wrist→middle MCP.

    Right-hand x = y cross z; left x reflected to map the left hand to the
    right robot. Display mirroring is separate from inference coordinates.
    """
    p = np.asarray(points, dtype=float)
    if p.shape != (21, 3) or not np.isfinite(p).all():
        raise ValueError('invalid_world_landmarks')
    width = np.linalg.norm(p[5] - p[17])
    if not .025 < width < .14:
        raise ValueError('implausible_palm_width')
    z = unit(p[9] - p[0])
    lateral = p[5] - p[17]
    y = unit(lateral - (lateral @ z) * z)
    if np.linalg.norm(np.cross(unit(lateral), z)) < .25:
        raise ValueError('degenerate_palm')
    x = np.cross(y, z) * (1 if side == 'Right' else -1)
    local = (p - p[0]) @ np.stack([x, y, z], axis=1)
    for chain in CHAINS:
        lengths = np.linalg.norm(np.diff(p[chain], axis=0), axis=1)
        if np.any(lengths < .004) or np.any(lengths > .15):
            raise ValueError('implausible_bone_length')
    return local, width


class Mapper:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        if self.model.nq != 21 or self.model.nv != 21:
            raise ValueError('Expected official 21-axis Revo3 model')
        self.names = [self.model.joint(i).name for i in range(21)]
        self.lo, self.hi = self.model.jnt_range.T.copy()
        self.neutral = np.clip(np.zeros(21), self.lo, self.hi)
        self.q = self.neutral.copy()
        self.ids = []
        for finger in FINGERS:
            joints = ['CMP', 'MCP', 'PIP'] if finger == 'thumb' else ['MCP', 'PIP', 'DIP']
            self.ids.append([self.model.body(f'right_{finger}_{j}_Link').id for j in joints]
                            + [self.model.body(f'right_{finger}_tip_Link').id])
        self.ids = np.asarray(self.ids)
        raw = self.fk(self.q)
        wrist = self.data.body('right_hand_base_link').xpos.copy()
        z = unit(raw[2, 0] - wrist)
        y = raw[1, 0] - raw[4, 0]
        y = unit(y - (y @ z) * z)
        self.basis = np.stack([np.cross(y, z), y, z], axis=1)
        self.wrist = wrist
        self.rest = (raw - wrist) @ self.basis
        self.width = np.linalg.norm(raw[1, 0] - raw[4, 0])
        self.last_valid = None
        self.last_local = None
        self.last_time = None
        self.speed = 3.0

    def fk(self, q, jac=False):
        self.data.qpos[:] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        points = self.data.xpos[self.ids].copy()
        if not jac:
            return points
        j = np.zeros((5, 4, 3, 21))
        jr = np.zeros((3, 21))
        for f in range(5):
            for k in range(4):
                mujoco.mj_jacBody(self.model, self.data, j[f, k], jr, int(self.ids[f, k]))
        return points, j

    def update(self, world, time_s, side='Right'):
        if not np.isfinite(time_s):
            raise ValueError('timestamps must be finite')
        if self.last_time is not None and time_s <= self.last_time:
            raise ValueError('timestamps must increase')
        dt = 1 / 30 if self.last_time is None else min(time_s - self.last_time, .1)
        self.last_time = time_s
        status, fit = 'tracking', None
        local = None
        target_q = self.q.copy()
        if world is not None:
            try:
                local, width = canonical(world, side)
                # Reject isolated severe landmark jumps; permit re-acquisition
                # after 0.3 seconds rather than permanently latching stale data.
                if self.last_local is not None and time_s - self.last_valid < .3:
                    if np.max(np.linalg.norm(local - self.last_local, axis=1)) > .08:
                        raise ValueError('landmark_jump')
                human = local[np.array([c[1:] for c in CHAINS])]
                # Anchor each chain at its official robot base. A global wrist
                # target would force flexion simply to compensate for differing
                # human/robot palm lengths. Match observed bone directions to
                # robot segment lengths; never modify the stored observations.
                directions = unit(np.diff(human, axis=1))
                lengths = np.linalg.norm(np.diff(self.rest, axis=1), axis=2, keepdims=True)
                target = np.concatenate([self.rest[:, :1],
                    self.rest[:, :1] + np.cumsum(directions * lengths, axis=1)], axis=1)
                target_world = target @ self.basis.T + self.wrist
                previous = self.q.copy()
                cache = {}

                def evaluate(q):
                    if 'q' in cache and np.array_equal(q, cache['q']):
                        return cache['r'], cache['j']
                    p, j = self.fk(q, jac=True)
                    # Position targets plus thumb-relative tip vectors. All
                    # intermediate points constrain flexion distribution.
                    r = np.concatenate([((p - target_world) / .05).ravel(),
                        (((p[1:, -1] - p[0, -1]) -
                          (target_world[1:, -1] - target_world[0, -1])) / .05).ravel(),
                        .15 * (q - previous)])
                    jj = np.vstack([j.reshape(-1, 21) / .05,
                                    (j[1:, -1] - j[0, -1]).reshape(-1, 21) / .05,
                                    .15 * np.eye(21)])
                    cache.update(q=q.copy(), r=r, j=jj)
                    return r, jj

                result = least_squares(lambda q: evaluate(q)[0],
                    np.clip(previous, self.lo + 1e-6, self.hi - 1e-6),
                    jac=lambda q: evaluate(q)[1], bounds=(self.lo, self.hi),
                    max_nfev=15, loss='soft_l1', f_scale=.2,
                    ftol=1e-4, xtol=1e-4, gtol=1e-4)
                if not np.isfinite(result.x).all():
                    raise ValueError('invalid_solver_output')
                target_q = result.x
                fit = {'converged': bool(result.success), 'nfev': int(result.nfev),
                       'target_rmse_m': float(np.sqrt(np.mean((self.fk(target_q) - target_world)**2)))}
                self.last_valid, self.last_local = time_s, local.copy()
            except ValueError as error:
                status = str(error)
        else:
            status = 'missing_hand'
        if status != 'tracking':
            age = float('inf') if self.last_valid is None else time_s - self.last_valid
            if age > .3:
                target_q = self.neutral
                status += ':return_neutral'
            else:
                status += ':hold'
        alpha = 1 - np.exp(-dt / .06)
        self.q = np.clip(self.q + np.clip(alpha * (target_q - self.q),
                                        -self.speed * dt, self.speed * dt), self.lo, self.hi)
        robot = (self.fk(self.q) - self.wrist) @ self.basis
        return {'q_command_rad': self.q.tolist(), 'status': status, 'fit': fit,
                'palm_landmarks_m': None if local is None else local.tolist(),
                'robot_landmarks_m': robot.tolist(), 'limiter_dt_s': dt}
