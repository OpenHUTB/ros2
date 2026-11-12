import json
from pathlib import Path
import unittest
import numpy as np
from landing_assessment.data import observable,prepare,WINDOW
from landing_assessment.inference import Estimator
from landing_assessment.learning import load,predict
from landing_assessment.simulation import rotation

class ContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest=json.loads(Path('data/raw/manifest.json').read_text())
        cls.raw=np.load(Path('data/raw')/cls.manifest[-1]['file'])['raw']
    def test_contact_and_labels_not_features(self):
        altered=self.raw.copy();altered[:,14:]=99999
        np.testing.assert_array_equal(observable(self.raw),observable(altered))
    def test_causality(self):
        altered=self.raw.copy();altered[20:,4:7]+=123
        np.testing.assert_array_equal(observable(self.raw)[:20],observable(altered)[:20])
    def test_episode_split(self):
        splits=prepare('data/raw');ids=[set(v['qepisode']) for v in splits.values()]
        self.assertTrue(all(not a&b for i,a in enumerate(ids) for b in ids[i+1:]))
    def test_online_offline_equivalence_and_precontact_gate(self):
        net,meta=load('models/trained/mlp_42');features=observable(self.raw);estimator=Estimator('models/trained/mlp_42');impacts=[]
        for i,row in enumerate(self.raw):
            result=estimator.step(row[0],row[1:4],row[4:7],row[7:11],row[11:14])
            if i>=WINDOW-1:
                prob,quality=predict(net,meta,features[i-WINDOW+1:i+1][None]);self.assertEqual(result['phase_index'],int(prob[0].argmax()))
                self.assertAlmostEqual(result['confidence'],float(prob.max()),places=5)
            if 'predicted_peak_load_factor' in result:
                impacts.append(result);self.assertLess(row[0],self.manifest[-1]['contact_time_s'])
        self.assertEqual(len(impacts),1)
    def test_bad_payload_resets(self):
        estimator=Estimator('models/trained/mlp_42');row=self.raw[0]
        result=estimator.step(0,[float('nan'),0,1],[0,0,0],[1,0,0,0],[0,0,0])
        self.assertFalse(result['ready']);self.assertEqual(result['reason'],'invalid_odometry')
    def test_timestamp_rollback_resets(self):
        estimator=Estimator('models/trained/mlp_42')
        for row in self.raw[:12]:estimator.step(row[0],row[1:4],row[4:7],row[7:11],row[11:14])
        row=self.raw[0];self.assertFalse(estimator.step(row[0],row[1:4],row[4:7],row[7:11],row[11:14])['ready'])
    def test_ros_body_world_roundtrip(self):
        for row in self.raw:
            R=rotation(row[7:11]);np.testing.assert_allclose(R@(R.T@row[4:7]),row[4:7],atol=1e-8)

if __name__=='__main__':unittest.main()
