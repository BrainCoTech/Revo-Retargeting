"""Numerical regression checks for the new offline software path, without hardware tests."""
import json
from pathlib import Path
import unittest
from lab_common import configure, checked, run, write_json
configure()
import numpy as np
from ingest_manus import normalize, NAMES
from revo3_model import Hand, prepare_model
from vector_solver import Solver
from evaluate_vector import resample, dynamic_eval


class VectorChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hand = Hand(prepare_model())
        cls.data = np.load(checked('data/normalized/manus_official_ufbx_v1/finger_agility.npz'),allow_pickle=False)
        cls.config = json.loads((Path(__file__).resolve().parents[1]/'configs/vector_batch_01.json').read_text())

    def test_landmark_jacobian_matches_finite_difference(self):
        h = self.hand
        q = h.lo + .37*(h.hi-h.lo)
        _,_,jac = h.fk(q,jac=True)
        numeric=np.zeros_like(jac)
        for i in range(21):
            delta=np.zeros(21);delta[i]=1e-6
            numeric[:,:,i]=(h.fk(q+delta)[0]-h.fk(q-delta)[0])/(2e-6)
        np.testing.assert_allclose(jac,numeric,rtol=1e-5,atol=1e-8)

    def test_palm_frame_is_rigid_transform_invariant(self):
        world=self.data['world_positions_m'][::100]
        a=.73
        rot=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
        before,basis=normalize(world,NAMES)
        after,_=normalize(world@rot.T+np.array([.5,-.2,.9]),NAMES)
        np.testing.assert_allclose(before,after,atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(basis),1,atol=1e-12)

    def test_fixed_base_filter_changes_no_kinematics_or_limits(self):
        previous = self.hand
        revised = Hand(prepare_model('right_official_pd_v2'))
        np.testing.assert_array_equal(previous.model.jnt_range,revised.model.jnt_range)
        np.testing.assert_array_equal(previous.model.actuator_forcerange,revised.model.actuator_forcerange)
        q = previous.lo + .4*(previous.hi-previous.lo)
        np.testing.assert_allclose(previous.fk(q)[0],revised.fk(q)[0],atol=1e-12)
        self.assertEqual(revised.model.nexclude,5)

    def test_dynamic_landmarks_match_post_step_joint_state(self):
        h = self.hand
        qs = np.stack([np.clip(h.q0+i*.01,h.lo,h.hi) for i in range(4)])
        targets = np.stack([h.fk(q)[0] for q in qs])
        actual,tips,_,_ = dynamic_eval(h,qs,targets,np.arange(4)/30)
        expected = np.stack([h.fk(q)[0] for q in actual])
        np.testing.assert_allclose(tips,expected,atol=1e-12)

    def test_contact_separation_jacobian(self):
        h = self.hand
        rng = np.random.default_rng(7)
        checked_rows = 0
        for _ in range(12):
            q = rng.uniform(h.lo,h.hi)
            h.fk(q)
            residual,jac = h.collision_penalty()
            active = np.flatnonzero(residual > 0.02)
            if not len(active):
                continue
            numeric = np.zeros_like(jac)
            for j in range(21):
                delta = np.zeros(21); delta[j] = 1e-7
                h.fk(q+delta); plus,_ = h.collision_penalty()
                h.fk(q-delta); minus,_ = h.collision_penalty()
                numeric[:,j] = (plus-minus)/(2e-7)
            np.testing.assert_allclose(jac[active],numeric[active],atol=.08,rtol=.02)
            checked_rows += len(active)
            if checked_rows >= 3:
                break
        self.assertGreaterEqual(checked_rows,3)

    def test_sequence_time_and_bones_are_valid(self):
        d=self.data
        self.assertTrue(np.all(np.diff(d['timestamps'])>0))
        self.assertGreater(d['timestamps'][-1],30)
        for i,parent in enumerate(d['parent']):
            if parent>=0:
                length=np.linalg.norm(d['world_positions_m'][:,i]-d['world_positions_m'][:,parent],axis=1)
                self.assertLess(length.std(),1e-5)

    def test_solver_bounds_and_common_rate_limit(self):
        h=self.hand
        ts,points=resample(self.data,30,4)
        for candidate in self.config['candidates']:
            solver=Solver(h,{**self.config['common'],**candidate},1/30)
            prev=h.q0.copy()
            for p in points:
                target=h.rest_tips@h.basis.T+h.wrist+np.array([.01,0,0])
                q,_=solver.solve(p,target)
                self.assertTrue(np.isfinite(q).all())
                self.assertTrue(np.all(q>=h.lo) and np.all(q<=h.hi))
                self.assertLessEqual(np.max(np.abs(q-prev)),4/30+1e-10)
                prev=q


if __name__=='__main__':
    with run('check_vector') as (out,manifest):
        with (out/'checks.log').open('w') as f:
            result=unittest.TextTestRunner(stream=f,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(VectorChecks))
        write_json(out/'checks.json',{'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors)})
        print((out/'checks.log').read_text())
        if not result.wasSuccessful():
            raise RuntimeError('Numerical regression failed')
