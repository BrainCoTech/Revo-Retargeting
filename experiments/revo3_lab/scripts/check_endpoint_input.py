"""Endpoint adapters and shared solver regressions; registered DSW only."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from lab_common import checked, configure, run, write_json
configure()
import numpy as np
from endpoint_input import EndpointTargets, from_manus, from_mediapipe, iter_mediapipe_jsonl
from ingest_manus import NAMES, normalize
from revo3_model import Hand, prepare_model
from vector_solver import Solver


def fixture():
    """Synthetic geometry only: no claim of recorded motion or tracking quality."""
    p = np.zeros((25, 3))
    p[1:5] = [[.005,.035,.018],[.012,.04,.03],[.02,.042,.045],[.03,.04,.055]]
    for start, lateral, height in [(5,.028,.065),(10,0.,.075),(15,-.017,.07),(20,-.03,.055)]:
        p[start:start+5] = [[0,lateral,height-.025], [0,lateral,height],
                            [.01,lateral,height+.025], [.02,lateral,height+.04],
                            [.03,lateral,height+.05]]
    return p


MP_INDICES = [0,1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19,21,22,23,24]


class AdapterChecks(unittest.TestCase):
    def test_manus_mediapipe_same_geometry_and_reflection(self):
        world = fixture()
        canonical, _ = normalize(world[None], NAMES)
        for side in ['Right', 'Left']:
            for sign in [-1., 1.]:
                p = canonical[0].copy()
                p[:, 0] *= sign * (1 if side == 'Right' else -1)
                kw = dict(basis=np.eye(3), wrist=np.array([.1,.2,.3]), scale=1.2,
                          include_directions=True)
                manus = from_manus(p, 1., **kw)
                mp = from_mediapipe(world[MP_INDICES], 1., hand_side=side,
                                    palm_x_sign=sign, **kw)
                np.testing.assert_allclose(manus.positions_m, mp.positions_m, atol=1e-12)
                np.testing.assert_allclose(manus.directions, mp.directions, atol=1e-12)

    def test_mediapipe_rigid_motion_invariance(self):
        world = fixture()[MP_INDICES]
        kw = dict(hand_side='Right', basis=np.eye(3), wrist=np.zeros(3), scale=.9,
                  include_directions=True)
        reference = from_mediapipe(world, 0., **kw)
        for angle in [-1., .3, 1.7]:
            c,s = np.cos(angle),np.sin(angle)
            r = np.array([[c,0,s],[0,1,0],[-s,0,c]])
            moved = from_mediapipe(world @ r.T + [.6,-.4,.8], 0., **kw)
            np.testing.assert_allclose(moved.positions_m, reference.positions_m, atol=1e-12)
            np.testing.assert_allclose(moved.directions, reference.directions, atol=1e-12)

    def test_masks_copies_and_direction_normalization(self):
        p = np.ones((5,3)); p[1] = np.nan
        d = np.tile([0.,0.,7.], (5,1)); d[1] = np.inf
        mask = np.array([True,False,True,True,True])
        frame = EndpointTargets(0.,p,mask,d,mask)
        p[0] = 100.; d[0] = 100.; mask[:] = False
        np.testing.assert_array_equal(frame.positions_m[1], 0.)
        np.testing.assert_array_equal(frame.directions[1], 0.)
        np.testing.assert_allclose(frame.directions[frame.direction_valid], [[0,0,1]]*4)
        np.testing.assert_array_equal(frame.positions_m[0], 1.)
        for bad in [float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                EndpointTargets(bad,np.zeros((5,3)),np.ones(5,bool))
        with self.assertRaises(ValueError):
            EndpointTargets(0.,np.zeros((4,3)),np.ones(5,bool))
        with self.assertRaises(ValueError):
            EndpointTargets(0.,np.zeros((5,3)),np.ones(5,bool),np.zeros((5,3)))

    def test_missing_and_degenerate_geometry(self):
        kw = dict(hand_side='Right',basis=np.eye(3),wrist=np.zeros(3),scale=1.)
        for world in [None,np.zeros((21,3)),np.full((21,3),np.nan)]:
            frame = from_mediapipe(world,0.,include_directions=True,**kw)
            self.assertFalse(frame.valid.any())
            self.assertTrue(np.isfinite(frame.positions_m).all())
        with self.assertRaises(ValueError):
            from_mediapipe(np.zeros((25,3)),0.,**kw)
        with self.assertRaises(ValueError):
            from_mediapipe(None,0.,**{**kw,'scale':0.})

    def test_jsonl_missing_ambiguity_timestamps_and_mapper(self):
        detection = dict(side='Right',handedness_score=.95,
                         world_landmarks_m=fixture()[MP_INDICES].tolist())
        rows = [dict(timestamp_s=i/30,detections=ds,selected_side='Right',
                     mapper_world_landmarks_m=None)
                for i,ds in enumerate([[detection],[],[detection,detection]])]
        with tempfile.TemporaryDirectory(dir=checked('tmp')) as folder:
            path = Path(folder)/'frames.jsonl'
            def save(values):
                path.write_text(''.join(json.dumps(row)+'\n' for row in values))
            save(rows)
            result = list(iter_mediapipe_jsonl(path))
            self.assertEqual(len(result),3)
            self.assertIsNotNone(result[0][1])
            self.assertIsNone(result[1][1]); self.assertIsNone(result[2][1])
            self.assertEqual(result[0][2],rows[0])
            self.assertTrue(all(world is None for _,world,_ in iter_mediapipe_jsonl(path,input_field='mapper')))
            with self.assertRaises(ValueError):
                list(iter_mediapipe_jsonl(path,input_field='mapper',hand_side='Left'))
            save([rows[0],rows[0]])
            with self.assertRaisesRegex(ValueError,'strictly increasing'):
                list(iter_mediapipe_jsonl(path))


class SolverChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        profile = json.loads((Path(__file__).resolve().parents[1]/'configs/endpoint_baseline.json').read_text())
        cls.config = profile['solver']
        cls.model = prepare_model('right_official_pd_v2')

    def setUp(self):
        self.hand = Hand(self.model)
        self.solver = Solver(self.hand,dict(self.config),1/30)
        self.goal_q = np.clip(self.hand.q0 + .22*(self.hand.hi-self.hand.lo),self.hand.lo,self.hand.hi)
        self.goal = self.hand.fk(self.goal_q)[0]

    def frame(self, stamp, valid=None, positions=None):
        return EndpointTargets(stamp,self.goal if positions is None else positions,
                               np.ones(5,bool) if valid is None else valid)

    def assert_bounded(self, q, previous, dt):
        self.assertTrue(np.isfinite(q).all())
        self.assertTrue(np.all(q >= self.hand.lo-1e-10))
        self.assertTrue(np.all(q <= self.hand.hi+1e-10))
        self.assertLessEqual(float(np.max(abs(q-previous))),
                             self.config['max_command_speed_rad_s']*dt+1e-10)

    def test_reachable_targets_improve_with_limits_and_rate(self):
        initial = np.linalg.norm(self.hand.fk(self.hand.q0)[0]-self.goal)
        previous = self.hand.q0.copy()
        for i in range(25):
            q,detail = self.solver.solve_endpoints(self.frame(i/30))
            self.assert_bounded(q,previous,detail['limiter_dt_s'])
            previous = q
        final = np.linalg.norm(self.hand.fk(q)[0]-self.goal)
        self.assertLess(final,initial*.8)
        print(json.dumps(dict(check='reachable_fk',initial_error_m=float(initial),final_error_m=float(final))))

    def test_partial_mask_ignores_invalid_coordinates(self):
        a = Solver(self.hand,dict(self.config),1/30)
        b = Solver(self.hand,dict(self.config),1/30)
        mask = np.array([True,True,False,True,False])
        p = self.goal.copy(); p[~mask] = np.nan
        other = self.goal.copy(); other[~mask] = 1e6
        initial = np.linalg.norm(self.hand.fk(self.hand.q0)[0][mask]-self.goal[mask])
        for i in range(12):
            qa,detail = a.solve_endpoints(self.frame(i/30,mask,p))
            qb,_ = b.solve_endpoints(self.frame(i/30,mask,other))
            np.testing.assert_allclose(qa,qb,atol=1e-12)
            self.assertEqual(detail['status'],'partial_tracking')
        self.assertLess(np.linalg.norm(self.hand.fk(qa)[0][mask]-self.goal[mask]),initial)

    def test_no_intermediate_joint_dependency(self):
        p = fixture()
        changed = p.copy(); changed[[5,10,15,20]] = np.nan
        kw = dict(basis=self.hand.basis,wrist=self.hand.wrist,scale=1.)
        a = from_manus(p,0.,**kw); b = from_manus(changed,0.,**kw)
        np.testing.assert_array_equal(a.positions_m,b.positions_m)
        with patch('vector_solver.finger_angles',side_effect=AssertionError('skeleton prior called')):
            q,_ = self.solver.solve_endpoints(a)
        self.assertTrue(np.isfinite(q).all())

    def test_missing_hold_return_and_reacquisition_dt(self):
        q,_ = self.solver.solve_endpoints(self.frame(0.))
        hold = self.config.get('hold_s',.3)
        missing = np.zeros(5,bool)
        held,detail = self.solver.solve_endpoints(self.frame(hold/2,missing))
        self.assertEqual(detail['status'],'hold')
        np.testing.assert_array_equal(held,q)
        returned,detail = self.solver.solve_endpoints(self.frame(hold+.1,missing))
        self.assertEqual(detail['status'],'return_neutral')
        self.assertLess(np.linalg.norm(returned-self.hand.q0),np.linalg.norm(q-self.hand.q0))
        self.assert_bounded(returned,held,detail['limiter_dt_s'])
        recovered,detail = self.solver.solve_endpoints(self.frame(hold+.11))
        self.assertAlmostEqual(detail['limiter_dt_s'],.01)
        self.assert_bounded(recovered,returned,.01)
        with self.assertRaises(ValueError):
            self.solver.solve_endpoints(self.frame(hold+.11))

    def test_optional_direction_loss_finite(self):
        from pinch_geometry import AllPadGeometry
        geometry = AllPadGeometry(self.hand)
        self.hand.fk(self.goal_q)
        directions = geometry.evaluate()[2].copy()
        solver = Solver(self.hand,{**self.config,'distal_direction_weight':.3},1/30)
        frame = EndpointTargets(0.,self.goal,np.ones(5,bool),directions,np.ones(5,bool))
        q,detail = solver.solve_endpoints(frame)
        self.assertTrue(np.isfinite(q).all())
        self.assertTrue(np.isfinite(detail['ik_solution']).all())


def main():
    with run('check_endpoint_input') as (out,manifest):
        suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
        result = unittest.TextTestRunner(stream=sys.stdout,verbosity=2).run(suite)
        write_json(out/'checks.json',dict(tests=result.testsRun,failures=len(result.failures),
                                        errors=len(result.errors),success=result.wasSuccessful()))
        if not result.wasSuccessful():
            raise SystemExit(1)


if __name__ == '__main__':
    main()
