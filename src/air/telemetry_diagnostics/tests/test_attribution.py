import csv
import unittest
from pathlib import Path

import numpy as np

from attribution import CHANNELS, attribute
from consistency import ConsistencyDetector
from stream_detector import Detector


class AttributionTests(unittest.TestCase):
    def test_dominant_channel_follows_residual(self):
        residual = np.zeros(6)
        residual[5] = 1.0
        result = attribute(residual, np.ones(6))
        self.assertEqual(result['dominant'], ['vz'])
        self.assertAlmostEqual(result['channels']['vz'], 1.0)
        self.assertAlmostEqual(sum(result['channels'].values()), 1.0)

    def test_two_strong_channels_are_both_reported(self):
        residual = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 1.0])
        result = attribute(residual, np.ones(6))
        self.assertEqual(sorted(result['dominant']), ['vx', 'vz'])

    def test_scale_matters(self):
        # 同样的残差，通道尺度小的一侧占比更大
        result = attribute(np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
                           np.array([1.0, 4.0, 1.0, 1.0, 1.0, 1.0]))
        self.assertEqual(result['dominant'][0], 'px')
        self.assertGreater(result['channels']['px'], result['channels']['py'])

    def test_zero_residual_gives_empty_attribution(self):
        result = attribute(np.zeros(6), np.ones(6))
        self.assertEqual(result['dominant'], [])
        self.assertEqual(sorted(result['channels']), sorted(CHANNELS))
        self.assertEqual(sum(result['channels'].values()), 0.0)

    def _stream(self, detector):
        path = Path('sample/observations.csv')
        if not path.exists():
            self.skipTest('缺少 sample/observations.csv')
        with path.open() as f:
            rows = list(csv.DictReader(f))[:60]
        return [detector.step(row) for row in rows]

    def test_gru_detector_reports_channels(self):
        results = self._stream(Detector('models/gru_42'))
        ready = [r for r in results if r['score'] is not None]
        self.assertTrue(ready, '预热后应至少有一个可用评分')
        self.assertTrue(all('channels' in r for r in ready))
        self.assertAlmostEqual(sum(ready[0]['channels'].values()), 1.0, places=2)

    def test_consistency_detector_reports_channels(self):
        results = self._stream(ConsistencyDetector('models/consistency_42'))
        ready = [r for r in results if r['score'] is not None]
        self.assertTrue(ready, '预热后应至少有一个可用评分')
        self.assertTrue(all('channels' in r for r in ready))
        self.assertAlmostEqual(sum(ready[0]['channels'].values()), 1.0, places=2)


if __name__ == '__main__':
    unittest.main()
