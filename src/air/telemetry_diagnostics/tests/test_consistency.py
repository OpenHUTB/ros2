import csv
import unittest
import numpy as np
import torch
from consistency import ConsistencyDetector, extract, transform


class ConsistencyTests(unittest.TestCase):
    def test_stream_scores_equal_batch_with_bad_packet_recovery(self):
        with open('data/prepared/test/episode_026_position_drift/observations.csv') as f:
            rows=list(csv.DictReader(f))
        d=ConsistencyDetector('models/consistency_42')
        x,idx,_=extract(rows)
        with torch.no_grad():
            expected=d.model(torch.from_numpy(transform(x,np.array(d.config['scale'])))).numpy()
        actual=[d.step(row) for row in rows]
        np.testing.assert_allclose([actual[i]['score'] for i in idx],expected,rtol=1e-4,atol=1e-5)
        self.assertTrue(d.step({'bad':1})['alarm'])
        self.assertFalse(d.step(dict(rows[-1],source_ns=int(rows[-1]['source_ns'])+50000000))['model_ready'])


if __name__=='__main__':unittest.main()
