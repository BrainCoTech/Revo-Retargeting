"""Regression of the public Solver and shared geometric residuals; DSW only."""
import importlib.util
import json
import unittest
from pathlib import Path
from lab_common import checked,run,write_json
from check_vector import VectorChecks
from vector_solver import Solver,pair_observation,pad_pair_terms
from pinch_geometry import AllPadGeometry
from run_pinch import source_data
import numpy as np


class SharedChecks(VectorChecks):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.shared=json.loads((Path(__file__).resolve().parents[1]/'configs/shared_batch_01.json').read_text())
        cfg=json.loads((Path(__file__).resolve().parents[1]/'configs/pinch_task_v1.json').read_text())
        d,p,_=source_data(cfg);cls.points=p[np.argmin(abs(d['timestamps']-21.3))]

    def targets(self,points):
        h=self.hand;scale=h.robot_width/np.linalg.norm(points[6]-points[21])
        return points[[4,9,14,19,24]]*scale@h.basis.T+h.wrist

    def test_anisotropic_pad_jacobian(self):
        h=self.hand;geom=AllPadGeometry(h)
        cfg={**self.shared['common'],**self.shared['candidates'][-1]}
        coordinates=np.array([[.003,-.002,-.0003]]*4);activation=np.array([.9,.4,.2,.1])
        q=h.lo+.38*(h.hi-h.lo)
        def residual(q):
            h.fk(q);return pad_pair_terms(geom.evaluate(),coordinates,activation,cfg)
        _,analytic=residual(q);finite=np.zeros_like(analytic)
        for k in range(21):
            d=np.zeros(21);d[k]=1e-6
            finite[:,k]=(residual(q+d)[0]-residual(q-d)[0])/2e-6
        np.testing.assert_allclose(analytic,finite,atol=1e-6,rtol=1e-5)

    def test_hybrid_rejects_pad_losses(self):
        cfg={**self.shared['common'],**self.shared['candidates'][-1],'kind':'hybrid'}
        with self.assertRaisesRegex(ValueError,'21-axis'):
            Solver(self.hand,cfg,1/30)

    def test_backside_does_not_activate_pad_contact(self):
        cfg={**self.shared['common'],**self.shared['candidates'][-1]}
        p=self.points.copy();p[4]=p[9]+[.001,0,0]
        target=self.targets(p);_,frames,_,_=pair_observation(p,target,self.hand)
        for sign in [-1,1]:
            target[0]=target[1]+sign*.005*frames[0,:,2]
            _,detail=Solver(self.hand,cfg,1/30).solve(p,target)
            self.assertAlmostEqual(detail['input_pair_coordinates_m'][0][2],sign*.005)
            if sign<0:self.assertEqual(detail['pair_activation'][0],0.)
            else:self.assertGreater(detail['pair_activation'][0],.9)

    def test_input_tangent_normal_components_are_not_interchanged(self):
        p=self.points;target=self.targets(p)
        c,frames,_,_=pair_observation(p,target,self.hand)
        # Perturb only the measured relative vector. The measuring frame must
        # not rotate towards the thumb and hide a normal input as tangential.
        for axis in range(3):
            shifted=target.copy();shifted[0]+=.004*frames[0,:,axis]
            changed,_,_,_=pair_observation(p,shifted,self.hand)
            expected=np.zeros(3);expected[axis]=.004
            np.testing.assert_allclose(changed[0]-c[0],expected,atol=1e-12)

    def test_common_rigid_motion_does_not_create_pair_signal(self):
        p=self.points;c,_,_,_=pair_observation(p,self.targets(p),self.hand)
        for angle in np.linspace(-1,1,7):
            r=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
            moved=p@r.T+np.array([.3*angle,.1,.5])
            new,frames,_,_=pair_observation(moved,self.targets(moved),self.hand)
            np.testing.assert_allclose(new,c,atol=1e-12)
            for f in frames:
                np.testing.assert_allclose(f.T@f,np.eye(3),atol=1e-12)
                self.assertAlmostEqual(np.linalg.det(f),1.)

    def test_legacy_configuration_preserves_frozen_solver(self):
        path=checked('runs/20260913T084011Z_vector_c5ab0b5e/code/scripts/vector_solver.py')
        spec=importlib.util.spec_from_file_location('frozen_vector_reference',path)
        old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
        cfg={**self.shared['common'],**self.shared['candidates'][0]}
        baseline=old.Solver(self.hand,cfg,1/30);current=Solver(self.hand,cfg,1/30)
        for _ in range(5):
            target=self.targets(self.points)
            a,_=baseline.solve(self.points,target);b,_=current.solve(self.points,target)
            np.testing.assert_allclose(a,b,atol=1e-7,rtol=1e-6)

    def test_side_observation_branch_inside_public_solver(self):
        cfg={**self.shared['common'],**self.shared['candidates'][2]}
        for flex in [0.,45.,70.,110.,120.]:
            for side in [-12.,0.,12.]:
                p=self.points.copy();a,b=np.radians([side,flex])
                v=np.array([np.sin(b),-np.sin(a)*np.cos(b),np.cos(a)*np.cos(b)])
                for k in range(4):
                    j=5+5*k;p[j]=p[j+1]-[0,0,.03]
                    for n in range(1,4):p[j+n+1]=p[j+n]+.025*v
                _,d=Solver(self.hand,cfg,1/30).solve(p,self.targets(p))
                np.testing.assert_allclose(d['side_reference_rad'],a,atol=1e-12)


if __name__=='__main__':
    with run('check_shared_solver') as (out,mf):
        with (out/'checks.log').open('w') as log:
            result=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SharedChecks))
        write_json(out/'checks.json',{'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors)})
        print((out/'checks.log').read_text(),flush=True)
        if not result.wasSuccessful():raise RuntimeError('Shared solver regression failed')
