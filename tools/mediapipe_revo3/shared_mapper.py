"""Local frontend for the shared endpoint solver and official MuJoCo dynamics."""
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[2]
CORE = REPO / 'experiments/revo3_lab/scripts'
sys.path.insert(0, str(CORE))
from endpoint_input import from_mediapipe
from revo3_model import Hand, FINGERS
from vector_solver import Solver


class SharedMapper:
    def __init__(self, model_path, *, scale=1., palm_x_sign=1., solver_config=None):
        if not np.isfinite(scale) or scale <= 0 or palm_x_sign not in (-1, 1):
            raise ValueError('Expected positive finite scale and palm_x_sign +/-1')
        profile_path = Path(solver_config) if solver_config else CORE.parent / 'configs/endpoint_baseline.json'
        self.profile_path = profile_path.resolve()
        profile = json.loads(profile_path.read_text())
        self.config = profile['solver'].copy()
        if (self.config.get('kind') != 'vector' or self.config.get('pad_pair_weight', 0) != 0
                or self.config.get('observation_mode', 'legacy') != 'legacy'):
            raise ValueError('Shared endpoint profile requires vector solver without skeleton losses')
        self.max_gap_s = float(profile.get('max_gap_s', 1.))
        if not np.isfinite(self.max_gap_s) or self.max_gap_s < 1/30:
            raise ValueError('max_gap_s must be finite and at least 1/30')
        self.hand = Hand(model_path)
        self.model = self.hand.model
        self.names, self.lo, self.hi = self.hand.names, self.hand.lo, self.hand.hi
        self.solver = Solver(self.hand, self.config, 1/30)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = self.hand.q0
        self.data.ctrl[:] = self.hand.q0
        mujoco.mj_forward(self.model, self.data)
        self.scale, self.palm_x_sign = scale, palm_x_sign
        self.last_time = self.origin = None
        self.ids = np.array([[self.model.body(f'right_{finger}_{joint}_Link').id
                              for joint in (['CMP', 'MCP', 'PIP', 'tip'] if finger == 'thumb'
                                            else ['MCP', 'PIP', 'DIP', 'tip'])] for finger in FINGERS])

    def update(self, world, time_s, side='Right'):
        if not np.isfinite(time_s) or (self.last_time is not None and not 0 < time_s-self.last_time <= self.max_gap_s):
            raise ValueError(f'Invalid source timestamp/gap; maximum gap is {self.max_gap_s}s')
        target = from_mediapipe(world, time_s, hand_side=side, basis=self.hand.basis,
                               wrist=self.hand.wrist, scale=self.scale, palm_x_sign=self.palm_x_sign,
                               include_directions=bool(self.config.get('distal_direction_weight', 0)))
        if self.origin is None:
            self.origin = time_s
        self.last_time = time_s
        while self.data.time < time_s-self.origin-1e-10:
            mujoco.mj_step(self.model, self.data)
            if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                    or np.max(abs(self.data.qvel)) > 1e4):
                raise RuntimeError('Unstable dynamics')
        mujoco.mj_forward(self.model, self.data)
        command, detail = self.solver.solve_endpoints(target)
        if not np.isfinite(command).all():
            raise RuntimeError('Invalid endpoint command')
        self.data.ctrl[:] = command
        local = None
        if world is not None and target.valid.any():
            # Display native 21-point topology; same palm convention as adapter.
            points = np.asarray(world, dtype=float)
            z = points[9]-points[0]; z /= np.linalg.norm(z)
            y = points[5]-points[17]; y -= (y @ z)*z; y /= np.linalg.norm(y)
            x = np.cross(y, z)*(1 if side == 'Right' else -1)*self.palm_x_sign
            local = (points-points[0]) @ np.stack([x, y, z], axis=1)
            local = [[float(v) if np.isfinite(v) else None for v in row] for row in local]
        robot = (self.data.xpos[self.ids]-self.hand.wrist) @ self.hand.basis
        return {'q_command_rad': command.tolist(), 'q_actual_rad': self.data.qpos.tolist(),
                'qvel_actual_rad_s': self.data.qvel.tolist(), 'simulation_time_s': float(self.data.time),
                'source_time_origin_s': self.origin, 'status': detail['status'], 'fit': detail,
                'palm_landmarks_m': local, 'robot_landmarks_m': robot.tolist(),
                'target_tips_m': target.positions_m.tolist(), 'target_valid': target.valid.tolist(),
                'actual_tips_m': self.data.site_xpos[self.hand.tip_ids].tolist(),
                'simulation_warning_counts': [int(w.number) for w in self.data.warning],
                'limiter_dt_s': detail['limiter_dt_s']}
