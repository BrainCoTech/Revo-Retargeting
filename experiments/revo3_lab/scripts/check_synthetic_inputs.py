"""Validate generated input geometry and provenance independently of robot success."""
import json
import unittest
from lab_common import checked,configure,run,write_json,digest
configure()
import numpy as np
from run_pinch import source_data


class SyntheticChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sources=[]
        for folder in checked('runs').iterdir():
            mf=folder/'manifest.json'
            if not mf.exists():continue
            meta=json.loads(mf.read_text())
            if meta['kind']=='synthetic_partner_input' and meta['status']=='success':cls.sources.append((folder,meta))
        if len(cls.sources)<2:raise RuntimeError('Expected ring and little IK fixtures')

    def test_every_bone_and_joint_bound_is_preserved(self):
        for folder,_ in self.sources:
            d=np.load(folder/'source.npz',allow_pickle=False);p=d['palm_positions_m'];s=json.loads((folder/'synthesis.json').read_text())
            self.assertTrue(np.all(np.diff(d['timestamps'])>0))
            for child,parent in enumerate(d['parent']):
                if parent>=0:
                    length=np.linalg.norm(p[:,child]-p[:,parent],axis=1)
                    np.testing.assert_allclose(length,length[0],atol=1e-12)
            low,high=np.array(s['joint_delta_bounds_rad']);q=d['synthetic_joint_deltas']
            self.assertTrue(np.all(q>=low-1e-8) and np.all(q<=high+1e-8))
            np.testing.assert_allclose(p[0],p[-1],atol=1e-12)

    def test_ik_closes_target_and_open_fixture_releases(self):
        for folder,_ in self.sources:
            cfg=json.loads((folder/'config.json').read_text());d,points,_=source_data(cfg)
            target={'middle':14,'ring':19,'little':24}[cfg['target_finger']]
            gap=np.linalg.norm(points[:,4]-points[:,target],axis=1)
            held=(d['timestamps']>=.75)&(d['timestamps']<=2.75)
            self.assertTrue(np.all(gap[held]<.005))
            s=json.loads((folder/'synthesis.json').read_text())
            if s.get('open_ik'):
                self.assertGreater(gap[0],.028);self.assertGreater(gap[-1],.028)

    def test_provenance_and_nonparticipating_points(self):
        for folder,meta in self.sources:
            cfg=json.loads((folder/'config.json').read_text());self.assertEqual(cfg['source_kind'],'synthetic_ik')
            source=checked(meta['template']['path']);self.assertEqual(digest(source),meta['template']['sha256'])
            original=np.load(source,allow_pickle=False)['palm_positions_m'][meta['template']['frame']]
            d=np.load(folder/'source.npz',allow_pickle=False);points=d['palm_positions_m']
            tip={'middle':14,'ring':19,'little':24}[cfg['target_finger']]
            changed=[2,3,4,tip-2,tip-1,tip]
            keep=np.setdiff1d(np.arange(25),changed)
            np.testing.assert_allclose(points[:,keep],np.broadcast_to(original[keep],points[:,keep].shape),atol=1e-12)


if __name__=='__main__':
    with run('check_synthetic_inputs') as (out,mf):
        with (out/'checks.log').open('w') as log:
            result=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SyntheticChecks))
        write_json(out/'checks.json',{'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors)})
        print((out/'checks.log').read_text())
        if not result.wasSuccessful():raise RuntimeError('Synthetic fixture validation failed')
