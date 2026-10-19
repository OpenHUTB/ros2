import unittest
import numpy as np
from payload_mass_estimation.data import features,Estimator,WINDOW

class Contracts(unittest.TestCase):
    def rows(self):
        t=np.arange(60)*.05;return np.c_[t,3+t*.1,t*.2,np.full(60,1.15*(9.81+.2))]
    def test_model_empty_mass(self):
        import mujoco
        from pathlib import Path
        from payload_mass_estimation.data import EMPTY_MASS
        model=mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'models/community_quadrotor.xml'))
        self.assertAlmostEqual(model.body_mass.sum(),EMPTY_MASS)
    def test_split_integrity(self):
        import json
        from pathlib import Path
        items=json.loads((Path(__file__).resolve().parents[1]/'data/manifest.json').read_text())
        ids=[x['seed'] for x in items];self.assertEqual(len(ids),len(set(ids)))
        self.assertEqual({s:sum(x['split']==s for x in items) for s in ['train','validation','test','shift']},{'train':90,'validation':30,'test':30,'shift':30})
    def test_physics_known_mass(self):
        _,m=features(self.rows()[:WINDOW]);self.assertAlmostEqual(m,.2,places=6)
    def test_stream_offline(self):
        est=Estimator(lambda x:x.mean())
        for k,row in enumerate(self.rows()):
            result=est.step(row)
            if k<WINDOW-1:self.assertIsNone(result)
            else:
                x,b=features(self.rows()[k-WINDOW+1:k+1]);self.assertAlmostEqual(result['mass_kg'],x.mean());self.assertAlmostEqual(result['physics_kg'],b)
    def test_invalid_resets(self):
        est=Estimator(lambda x:0)
        for row in self.rows()[:WINDOW]:est.step(row)
        self.assertIsNone(est.step([np.nan,0,0,0]));self.assertEqual(est.rows,[])
    def test_gap_resets(self):
        est=Estimator(lambda x:0)
        for row in self.rows()[:WINDOW]:est.step(row)
        self.assertIsNone(est.step([7,0,0,0]));self.assertEqual(len(est.rows),1)
    def test_no_height_or_label_input(self):
        a=self.rows()[:WINDOW];x,b=features(a);a[:,1]=999
        np.testing.assert_array_equal(x,features(a)[0])
    def test_causal_force_alignment(self):
        a=self.rows()[:WINDOW];x,b=features(a);a[-1,3]=999
        np.testing.assert_array_equal(x,features(a)[0])

if __name__=='__main__':unittest.main()
