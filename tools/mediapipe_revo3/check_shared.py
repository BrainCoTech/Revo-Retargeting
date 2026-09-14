"""Local shared-endpoint regression checks using the existing repository environment."""
import unittest
import numpy as np
import mujoco
from check import fixture
from local_model import prepare_local_model
from shared_mapper import SharedMapper
from endpoint_input import EndpointTargets


class SharedChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = prepare_local_model()

    def setUp(self):
        self.mapper = SharedMapper(self.path)

    def test_causal_actual_state(self):
        m = self.mapper
        a = m.update(fixture(), 10.)
        np.testing.assert_array_equal(a['q_actual_rad'], m.hand.q0)
        data = mujoco.MjData(m.model)
        data.qpos[:] = m.hand.q0; data.ctrl[:] = a['q_command_rad']
        while data.time < .1-1e-10:
            mujoco.mj_step(m.model, data)
        b = m.update(fixture()*[1.,1.,.9], 10.1)
        np.testing.assert_allclose(b['q_actual_rad'], data.qpos, atol=1e-12)
        self.assertAlmostEqual(b['simulation_time_s'], data.time)

    def test_missing_hold_neutral_and_recovery(self):
        m = self.mapper
        a = m.update(fixture(), 0.)
        b = m.update(None, .1)
        self.assertEqual(b['status'], 'hold')
        np.testing.assert_array_equal(a['q_command_rad'], b['q_command_rad'])
        c = m.update(None, .5)
        self.assertEqual(c['status'], 'return_neutral')
        self.assertLess(np.linalg.norm(np.array(c['q_command_rad'])-m.hand.q0),
                        np.linalg.norm(np.array(b['q_command_rad'])-m.hand.q0))
        d = m.update(fixture(), .53)
        self.assertEqual(d['status'], 'tracking')
        self.assertLessEqual(np.max(abs(np.array(d['q_command_rad'])-c['q_command_rad'])), .03*4+1e-10)

    def test_input_transform_and_time_validation(self):
        m = self.mapper
        p = fixture()
        first = m.update(p, 0.)
        other = SharedMapper(self.path)
        angle=.7
        rotation=np.array([[np.cos(angle),-np.sin(angle),0],
                           [np.sin(angle),np.cos(angle),0],[0,0,1.]])
        second=other.update(p@rotation.T+[.2,-.4,.3],0.)
        np.testing.assert_allclose(first['target_tips_m'],second['target_tips_m'],atol=1e-12)
        for stamp in (0., float('nan'), 2.):
            with self.assertRaises(ValueError): m.update(p, stamp)
        with self.assertRaises(ValueError): SharedMapper(self.path,scale=0.)

    def test_direction_jacobian_and_endpoint_loss(self):
        m = self.mapper; h=m.hand
        q=h.lo+.3*(h.hi-h.lo)
        h.fk(q)
        axes,jac=h.distal_directions()
        for i in (0,2,4,5,8,12,16,20):
            step=np.zeros(21); step[i]=1e-6
            h.fk(q+step); plus,_=h.distal_directions()
            h.fk(q-step); minus,_=h.distal_directions()
            np.testing.assert_allclose(jac[:,:,i],(plus-minus)/2e-6,atol=1e-8)
        from vector_solver import Solver
        solver=Solver(h,{**m.config,'distal_direction_weight':.3},1/30)
        target=EndpointTargets(0.,h.fk(q)[0],np.ones(5,bool),axes,np.ones(5,bool))
        result,detail=solver.solve_endpoints(target)
        self.assertTrue(np.isfinite(result).all())
        self.assertTrue(np.isfinite(detail['ik_solution']).all())

    def test_bone_scaled_size_invariance_and_invalid_bone(self):
        from endpoint_input import from_manus
        h = self.mapper.hand
        indices = np.array([[1,2,3,4],[6,7,8,9],[11,12,13,14],
                            [16,17,18,19],[21,22,23,24]])
        points = np.zeros((25,3))
        points[indices] = h.rest_chains
        options = dict(basis=h.basis, wrist=h.wrist, scale=1.,
                       target_mode='bone_scaled', robot_rest_chains_m=h.rest_chains,
                       include_directions=True)
        target = from_manus(points, 0., **options)
        np.testing.assert_allclose(target.positions_m, h.rest_tips @ h.basis.T + h.wrist, atol=1e-12)
        changed = points.copy()
        for finger, chain in enumerate(indices):
            vectors = np.diff(points[chain], axis=0) * np.array([.6,1.4,.8])[:,None]
            changed[chain] = points[chain[0]] + [0,.01*finger,-.03] + np.vstack([np.zeros(3),np.cumsum(vectors,axis=0)])
        resized = from_manus(changed, .1, **options)
        np.testing.assert_allclose(resized.positions_m, target.positions_m, atol=1e-12)
        changed[24] = changed[23]
        invalid = from_manus(changed, .2, **options)
        np.testing.assert_array_equal(invalid.valid, [True,True,True,True,False])
        self.assertFalse(invalid.direction_valid[-1])

    def test_mediapipe_size_invariance_and_legacy_profile(self):
        from endpoint_input import from_mediapipe
        from shared_mapper import CORE
        h = self.mapper.hand
        options = dict(hand_side='Right', basis=h.basis, wrist=h.wrist, scale=1.,
                       target_mode='bone_scaled', robot_rest_chains_m=h.rest_chains)
        first = from_mediapipe(fixture(), 0., **options)
        second = from_mediapipe(fixture()*.6, .1, **options)
        np.testing.assert_allclose(first.positions_m, second.positions_m, atol=1e-12)
        old = SharedMapper(self.path, solver_config=CORE.parent/'configs/endpoint_baseline.json')
        output = old.update(fixture(), 0.)
        self.assertEqual(old.target_mode, 'wrist_scaled')
        direct = from_mediapipe(fixture(), 0., hand_side='Right', basis=h.basis,
                               wrist=h.wrist, scale=1.)
        np.testing.assert_array_equal(output['target_tips_m'], direct.positions_m)

    def test_open_endpoints_recover_from_folded_little_finger(self):
        h = self.mapper.hand
        solver = self.mapper.solver
        solver.previous[-2:] = np.deg2rad(85.)
        h.fk(h.q0)
        directions, _ = h.distal_directions()
        for i in range(30):
            target = EndpointTargets(i*.1, h.rest_tips @ h.basis.T + h.wrist, np.ones(5,bool), directions)
            command, _ = solver.solve_endpoints(target)
        self.assertLess(np.max(np.abs(command[-2:])), np.deg2rad(5.))

    def test_full_official_dynamics(self):
        m=self.mapper.model
        self.assertEqual((m.nq,m.nv,m.nu,m.nexclude),(21,21,21,5))
        self.assertGreater(m.nmesh,0)
        self.assertGreater(np.count_nonzero(m.geom_contype),0)
        np.testing.assert_array_equal(m.opt.gravity,0.)
        self.assertAlmostEqual(m.opt.timestep,.002)
        np.testing.assert_allclose(m.actuator_gainprm[:,0],3.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
