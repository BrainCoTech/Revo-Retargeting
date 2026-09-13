"""Generalized opposition regression including the accepted thumb-index path."""
import json
from pathlib import Path
import unittest
from lab_common import checked,configure,run,write_json
configure()
import numpy as np
import mujoco
from check_pinch import PinchChecks
from pinch_geometry import PadHand,solve_pads,source_target
from run_pinch import source_data,contact_state
from command_limiter import CommandLimiter


class PartnerChecks(PinchChecks):
    def test_static_target_braking_does_not_overshoot(self):
        h=self.hand;target=h.q0+.37*(h.hi-h.q0)
        limiter=CommandLimiter(h.q0,h.lo,h.hi,brake_at_target=True);commands=[h.q0.copy()]
        for _ in range(100):commands.append(limiter.update(target))
        commands=np.array(commands)
        self.assertTrue(np.all(commands>=h.q0-1e-9) and np.all(commands<=target+1e-9))
        np.testing.assert_allclose(commands[-1],target,atol=1e-8)
        self.assertLessEqual(float(abs(np.diff(commands,n=2,axis=0)*900).max()),20+1e-8)

    def test_partner_kinematic_columns_and_jacobians(self):
        for finger,start,tip in [('middle',9,14),('ring',13,19),('little',17,24)]:
            cfg=json.loads((Path(__file__).resolve().parents[1]/f'configs/pinch_{finger}_v1.json').read_text())
            h=PadHand(checked('models/revo3/right_official_pd_v2/scene.xml'),cfg)
            np.testing.assert_array_equal(h.active_dofs,np.r_[np.arange(5),np.arange(start,start+4)])
            self.assertEqual(h.source_tip_index,tip)
            q=h.lo+.37*(h.hi-h.lo);*_,jp,jn,ja=h.pad_fk(q,True)
            for array in [jp,jn,ja]:np.testing.assert_allclose(array[:,:,h.inactive_dofs],0,atol=1e-12)
            for j in h.active_dofs:
                delta=np.zeros(21);delta[j]=1e-6;plus=h.pad_fk(q+delta);minus=h.pad_fk(q-delta)
                for k,analytic in enumerate([jp,jn,ja]):np.testing.assert_allclose(analytic[:,:,j],(plus[k]-minus[k])/(2e-6),atol=1e-8,rtol=1e-5)

    def test_actual_index_contact_cannot_count_for_other_partners(self):
        trajectory=np.load(checked('runs/20260913T090155Z_pinch_hold_f567ad65/pad_vector/trajectory.npz'),allow_pickle=False)
        for finger in ['middle','ring','little']:
            cfg=json.loads((Path(__file__).resolve().parents[1]/f'configs/pinch_{finger}_v1.json').read_text())
            h=PadHand(checked('models/revo3/right_official_pd_v2/scene.xml'),cfg);d=mujoco.MjData(h.model)
            d.qpos[:]=trajectory['q_actual'][-1];d.qvel[:]=trajectory['qvel_actual'][-1];d.ctrl[:]=trajectory['q_target'][-1]
            mujoco.mj_forward(h.model,d);result=contact_state(h,d,cfg)
            self.assertFalse(result['valid_pad_contact']);self.assertGreater(result['non_target_penetration_m'],0)

    def test_source_pair_and_nonparticipating_joints(self):
        for finger in ['middle','ring','little']:
            cfg=json.loads((Path(__file__).resolve().parents[1]/f'configs/pinch_{finger}_v1.json').read_text())
            h=PadHand(checked('models/revo3/right_official_pd_v2/scene.xml'),cfg)
            data,ps,_=source_data(cfg);p=ps[np.argmin(abs(data['timestamps']-cfg['source_pose_s']))]
            target=source_target(h,p)
            self.assertAlmostEqual(target['source_gap_m'],float(np.linalg.norm(p[4]-p[h.source_tip_index])),12)
            q,_=solve_pads(h,p,h.q0,cfg)
            np.testing.assert_array_equal(q[h.inactive_dofs],0)
            self.assertTrue(np.all(q>=h.lo) and np.all(q<=h.hi))


if __name__=='__main__':
    with run('check_partners') as (out,manifest):
        with (out/'checks.log').open('w') as log:
            result=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(PartnerChecks))
        write_json(out/'checks.json',{'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors)})
        print((out/'checks.log').read_text())
        if not result.wasSuccessful():raise RuntimeError('Partner regression failed')
