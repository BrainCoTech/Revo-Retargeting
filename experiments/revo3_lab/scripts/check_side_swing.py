"""Independent per-finger identity, invariance and missing-observation checks."""
import unittest
import numpy as np
from lab_common import configure,run,write_json
configure()
from side_swing import SideMapper,lateral_observation


def fixture():
    p=np.zeros((25,3));sides=np.radians([4.,-7.,11.,-2.]);flexes=np.radians([0.,45.,120.,60.])
    for k,(s,f) in enumerate(zip(sides,flexes)):
        j=5+5*k;p[j+1]=[0.,0.,.03]
        p[j+2]=p[j+1]+.04*np.array([np.sin(f),-np.sin(s)*np.cos(f),np.cos(s)*np.cos(f)])
    return p,sides


class Checks(unittest.TestCase):
    def test_distinct_finger_signs_and_branch(self):
        p,expected=fixture();_,angles,confidence=lateral_observation(p)
        np.testing.assert_allclose(angles,expected,atol=1e-12)
        self.assertTrue(np.all(confidence>=.49))

    def test_scale_and_translation_invariance_without_mutation(self):
        p,expected=fixture();before=p.copy()
        for scale in [.1,1.,10.]:
            _,angles,_=lateral_observation(p*scale+np.array([.4,-.2,.7]))
            np.testing.assert_allclose(angles,expected,atol=1e-12)
        np.testing.assert_array_equal(p,before)

    def test_missing_finger_holds_then_releases_without_affecting_others(self):
        p,expected=fixture();mapper=SideMapper(np.full(4,-.2618),np.full(4,.2618))
        mapper.update(p);missing=p.copy();missing[7]=np.nan
        for _ in range(2):q,diag=mapper.update(missing)
        np.testing.assert_allclose(q,expected,atol=1e-12)
        for _ in range(7):q,diag=mapper.update(missing)
        self.assertEqual(q[0],0.)
        np.testing.assert_allclose(q[1:],expected[1:],atol=1e-12)
        q,_=mapper.update(p);np.testing.assert_allclose(q,expected,atol=1e-12)

    def test_per_finger_robot_limits(self):
        p,expected=fixture();low=np.radians([-3,-5,-8,-1]);high=-low
        mapper=SideMapper(low,high);q,_=mapper.update(p)
        np.testing.assert_allclose(q,np.clip(expected,low,high),atol=1e-12)


if __name__=='__main__':
    with run('check_side_swing') as (out,mf):
        with (out/'checks.log').open('w') as log:
            result=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Checks))
        write_json(out/'checks.json',{'tests':result.testsRun,'passed':result.wasSuccessful(),'failures':len(result.failures),'errors':len(result.errors)})
        print((out/'checks.log').read_text())
        if not result.wasSuccessful():raise RuntimeError('Side-swing checks failed')
