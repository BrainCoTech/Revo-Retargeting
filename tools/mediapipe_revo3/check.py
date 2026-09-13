"""Local adapter regression checks. Synthetic fixtures test invariants, not realism."""
import unittest
import numpy as np
from assets import ROOT
from mapper import Mapper, canonical


def fixture():
    p = np.zeros((21, 3))
    p[1:5] = [[0, .03, .035], [.008, .052, .06], [.02, .065, .077], [.03, .071, .095]]
    for start, y, z in [(5, .03, .08), (9, 0, .09), (13, -.025, .084), (17, -.045, .073)]:
        p[start:start+4] = [[0, y, z], [.005, y, z+.035], [.018, y, z+.058], [.033, y, z+.072]]
    return p


class Checks(unittest.TestCase):
    def setUp(self):
        self.mapper = Mapper(ROOT / 'assets/kinematics.xml')

    def test_rigid_transform_and_scale(self):
        p = fixture()
        a = .73
        r = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        local, width = canonical(p)
        transformed, width2 = canonical(1.2 * p @ r.T + [2, 3, -1])
        np.testing.assert_allclose(local / width, transformed / width2, atol=1e-12)

    def test_left_right_reflection(self):
        p = fixture()
        right, _ = canonical(p, 'Right')
        left, _ = canonical(p * [1, -1, 1], 'Left')
        np.testing.assert_allclose(left, right, atol=1e-12)

    def test_missing_invalid_and_recovery(self):
        m = self.mapper
        initial = m.update(fixture(), 0)
        self.assertEqual(initial['status'], 'tracking')
        q = m.q.copy()
        held = m.update(None, .1)
        self.assertIn(':hold', held['status'])
        np.testing.assert_array_equal(q, m.q)
        released = m.update(np.full((21, 3), np.nan), .5)
        self.assertIn(':return_neutral', released['status'])
        self.assertLessEqual(np.linalg.norm(m.q), np.linalg.norm(q))
        self.assertEqual(m.update(fixture(), .55)['status'], 'tracking')

    def test_speed_and_limits_irregular_time(self):
        m = self.mapper
        for t in [0, .002, .02, .3, .31, .32, .4]:
            old = m.q.copy()
            result = m.update(fixture(), t)
            self.assertTrue(np.all(m.q >= m.lo) and np.all(m.q <= m.hi))
            self.assertLessEqual(np.max(np.abs(m.q-old)), m.speed * result['limiter_dt_s'] + 1e-12)

    def test_nonmonotonic_rejected(self):
        self.mapper.update(None, 1)
        with self.assertRaises(ValueError):
            self.mapper.update(None, 1)

    def test_official_fk_jacobian(self):
        m = self.mapper
        q = (m.lo + m.hi) * .5
        _, analytic = m.fk(q, jac=True)
        for i in [0, 2, 4, 5, 6, 10, 20]:
            step = np.zeros(21)
            step[i] = 1e-6
            numeric = (m.fk(q + step) - m.fk(q - step)) / 2e-6
            np.testing.assert_allclose(analytic[..., i], numeric, atol=1e-8)

    def test_robot_rest_does_not_create_artificial_flexion(self):
        m = self.mapper
        points = np.vstack([np.zeros(3), m.rest.reshape(20, 3)])
        for i in range(15):
            result = m.update(points, i / 30)
            self.assertEqual(result['status'], 'tracking')
        self.assertLess(np.max(np.abs(m.q - m.neutral)), .01)


if __name__ == '__main__':
    unittest.main(verbosity=2)
