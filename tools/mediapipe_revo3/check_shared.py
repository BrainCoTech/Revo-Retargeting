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
