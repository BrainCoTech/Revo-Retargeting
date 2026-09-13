"""Independent finite-difference geometry checks and contact-state regressions."""
import json
import unittest
from pathlib import Path
from lab_common import configure,checked,run,write_json
configure()
import mujoco
import numpy as np
from pinch_geometry import PadHand
from run_pinch import contact_state
from command_limiter import CommandLimiter


class PinchChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((Path(__file__).resolve().parents[1]/'configs/pinch_task_v1.json').read_text())
        cls.hand=PadHand(checked('models/revo3/right_official_pd_v2/scene.xml'),cls.config)

    def test_pad_position_normal_axis_jacobians(self):
        h=self.hand;q=h.lo+.37*(h.hi-h.lo)
        *_,jp,jn,ja=h.pad_fk(q,True)
        numeric=[np.zeros_like(jp) for _ in range(3)]
        for j in range(21):
            delta=np.zeros(21);delta[j]=1e-6
            plus=h.pad_fk(q+delta);minus=h.pad_fk(q-delta)
            for i in range(3):numeric[i][:,:,j]=(plus[i]-minus[i])/(2e-6)
        for analytic,finite in zip([jp,jn,ja],numeric):np.testing.assert_allclose(analytic,finite,atol=1e-8,rtol=1e-5)

    def test_patch_rejects_back_side_and_proximal_points(self):
        h=self.hand;h.pad_fk(h.lo+.42*(h.hi-h.lo))
        for i,finger in enumerate(['thumb','index']):
            cfg=self.config['pads'][finger];b=h.pad_body_ids[i]
            def world(local):return h.data.xpos[b]+h.data.xmat[b].reshape(3,3)@local
            p=h.pad_positions[i].copy();self.assertTrue(h.point_in_pad(finger,world(p),h.data))
            for axis,value in [(cfg['normal_axis'],-.004),(cfg['width_axis'],.012),(cfg['long_axis'],0.)]:
                bad=p.copy();bad[axis]=value
                self.assertFalse(h.point_in_pad(finger,world(bad),h.data))

    def test_saved_dynamic_contact_and_geometry_preserved(self):
        h=self.hand;m=h.model;d=mujoco.MjData(m)
        original=mujoco.MjModel.from_xml_path(str(checked('models/revo3/right_official_pd_v2/scene.xml')))
        for attr in ['geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity','jnt_range','actuator_forcerange','mesh_vert','exclude_signature']:
            np.testing.assert_array_equal(getattr(m,attr),getattr(original,attr))
        root=checked('runs/20260913T090155Z_pinch_hold_f567ad65')
        for label,expected in [('old_vector',False),('pad_vector',True)]:
            trajectory=np.load(root/label/'trajectory.npz',allow_pickle=False)
            d.qpos[:]=trajectory['q_actual'][-1];d.qvel[:]=trajectory['qvel_actual'][-1];d.ctrl[:]=trajectory['q_target'][-1]
            mujoco.mj_forward(m,d);result=contact_state(h,d,self.config)
            self.assertEqual(result['valid_pad_contact'],expected)
            if expected:
                self.assertLess(result['max_penetration_m'],.001)
                # Independently transform every real target-contact point into both link frames.
                points=[]
                for c in d.contact:
                    if {int(c.geom1),int(c.geom2)}!=set(h.pad_geom_ids):continue
                    local=[d.xmat[b].reshape(3,3).T@(c.pos-d.xpos[b]) for b in h.pad_body_ids]
                    self.assertTrue(.011<local[0][1]<.029 and abs(local[0][0])<.007 and local[0][2]>.003)
                    self.assertTrue(.010<local[1][2]<.026 and abs(local[1][1])<.007 and local[1][0]>.003)
                    points.append(local)
                self.assertGreater(len(points),0)

    def test_open_pose_never_reports_contact(self):
        h=self.hand;d=mujoco.MjData(h.model);d.qpos[:]=h.q0;d.ctrl[:]=h.q0
        mujoco.mj_forward(h.model,d)
        self.assertFalse(contact_state(h,d,self.config)['valid_pad_contact'])

    def test_limiter_brakes_at_bounds_and_reverses_without_acceleration_spikes(self):
        h=self.hand;limiter=CommandLimiter(h.q0,h.lo,h.hi);commands=[h.q0.copy()]
        for target in [h.hi,h.lo,h.hi,h.q0]:
            for _ in range(80):commands.append(limiter.update(target))
        commands=np.array(commands)
        self.assertTrue(np.all(commands>=h.lo-1e-8) and np.all(commands<=h.hi+1e-8))
        velocity=np.diff(commands,axis=0)*30
        self.assertLessEqual(float(abs(velocity).max()),4+1e-8)
        self.assertLessEqual(float(abs(np.diff(velocity,axis=0)*30).max()),20+1e-8)
        np.testing.assert_allclose(commands[-1],h.q0,atol=1e-8)


if __name__=='__main__':
    with run('check_pinch') as (out,manifest):
        with (out/'checks.log').open('w') as log:
            result=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(PinchChecks))
        write_json(out/'checks.json',{'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors)})
        print((out/'checks.log').read_text())
        if not result.wasSuccessful():raise RuntimeError('Pad geometry/contact checks failed')
