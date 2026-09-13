"""Physics/Jacobian regressions for rubbing; run in the registered DSW only."""
import unittest
from lab_common import run, write_json
from check_pinch import PinchChecks
from pinch_geometry import pad_tangent
from run_rubbing import relative_velocity, surface_slip
import mujoco
import numpy as np


class RubbingChecks(PinchChecks):
    def test_articulated_input_preserves_bones_and_moves_distal_hinges(self):
        from rubbing_input import articulated_input
        from run_pinch import source_data
        from synthesize_partner_input import angle
        d,points,_=source_data(self.config)
        base=points[np.argmin(abs(d['timestamps']-self.config['source_pose_s']))]
        ps,qs,details=articulated_input(base,d['parent'],np.linspace(0,2,61),2.,15.)
        self.assertLess(details['max_bone_length_error_m'],1e-10)
        np.testing.assert_allclose(ps[0],ps[-1],atol=1e-12)
        for a,b,c in [(1,2,3),(2,3,4),(6,7,8),(7,8,9)]:
            measured=np.array([angle(p[b]-p[a],p[c]-p[b]) for p in ps])
            self.assertGreater(np.degrees(np.ptp(measured)),15.)
        keep=[0,1,5,6]+list(range(10,25))
        np.testing.assert_allclose(ps[:,keep],np.broadcast_to(base[keep],ps[:,keep].shape),atol=1e-12)
        with self.assertRaises(ValueError):
            articulated_input(base,d['parent'],np.linspace(0,2,61),2.,120.)

    def test_counter_curl_calibration_matches_measured_input_angle_changes(self):
        from rubbing_input import articulated_input,posture_targets,default_neutral
        from run_pinch import source_data
        from synthesize_partner_input import angle
        d,points,_=source_data(self.config)
        base=points[np.argmin(abs(d['timestamps']-self.config['source_pose_s']))]
        ps,qs,_=articulated_input(base,d['parent'],np.linspace(0,2,61),2.,15.,'counter')
        target=posture_targets(self.hand,self.hand.q0,qs,default_neutral('counter'))
        for child,ancestor in enumerate(d['parent']):
            if ancestor>=0:
                lengths=np.linalg.norm(ps[:,child]-ps[:,ancestor],axis=1)
                np.testing.assert_allclose(lengths,np.linalg.norm(base[child]-base[ancestor]),atol=1e-12)
        keep=[0,1,5,6]+list(range(10,25))
        np.testing.assert_allclose(ps[:,keep],np.broadcast_to(base[keep],ps[:,keep].shape),atol=1e-12)
        for dof,(a,b,c) in zip([3,4,7,8],[(1,2,3),(2,3,4),(6,7,8),(7,8,9)]):
            measured=np.array([angle(p[b]-p[a],p[c]-p[b]) for p in ps])
            np.testing.assert_allclose(target[:,dof]-target[0,dof],measured-measured[0],atol=1e-10)
        with self.assertRaises(ValueError):
            posture_targets(self.hand,self.hand.q0,qs,[15.,0.,40.,12.])

    def test_sliding_residual_jacobian(self):
        h = self.hand; q = h.lo+.37*(h.hi-h.lo)
        offset = .004; gap = -.00035
        def residual(q, jacobian=False):
            p,n,a,jp,jn,ja = h.pad_fk(q, True)
            tangent,jt = pad_tangent(a[1], n[1], ja[1], jn[1])
            return (p[0]-p[1]-gap*n[1]-offset*tangent,
                    jp[0]-jp[1]-gap*jn[1]-offset*jt)
        _, analytic = residual(q)
        finite = np.zeros_like(analytic)
        for j in range(21):
            delta = np.zeros(21); delta[j] = 1e-6
            finite[:,j] = (residual(q+delta)[0]-residual(q-delta)[0])/(2e-6)
        np.testing.assert_allclose(analytic, finite, atol=1e-8, rtol=1e-5)

    def test_rigid_common_motion_is_not_slip(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
        <body name="root"><freejoint/><geom size=".01" mass="1"/>
        <body name="a" pos="0.1 0 0"><geom size=".01" mass="1"/></body>
        <body name="b" pos="0 0.1 0"><geom size=".01" mass="1"/></body>
        </body></worldbody></mujoco>''')
        data = mujoco.MjData(model); data.qvel[:] = [.2,-.3,.4,1.,2.,3.]
        mujoco.mj_forward(model, data)
        v = relative_velocity(model, data, [model.body('a').id, model.body('b').id], np.array([.1,.1,.1]))
        np.testing.assert_allclose(v, 0., atol=1e-12)

    def test_known_relative_translation(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
        <body name="a"><joint type="slide" axis="0 1 0"/><geom size=".01" mass="1"/></body>
        <body name="b" pos="0.1 0 0"><geom size=".01"/></body>
        </worldbody></mujoco>''')
        data = mujoco.MjData(model); data.qvel[0] = .012
        mujoco.mj_forward(model, data)
        v = relative_velocity(model, data, [model.body('a').id, model.body('b').id], np.array([.05,0,0]))
        np.testing.assert_allclose(v, [0,.012,0], atol=1e-12)

    def test_uncontacted_motion_has_no_measured_slip(self):
        h = self.hand; data = mujoco.MjData(h.model)
        data.qpos[:] = h.q0; data.qvel[:] = .2
        mujoco.mj_forward(h.model, data)
        self.assertEqual(surface_slip(h, data, self.config), (0.,0.,0.))


if __name__=='__main__':
    with run('check_rubbing') as (out, mf):
        result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RubbingChecks))
        write_json(out/'checks.json', {'tests':result.testsRun, 'failures':len(result.failures), 'errors':len(result.errors)})
        if not result.wasSuccessful():
            raise RuntimeError('Rubbing regression failed')
