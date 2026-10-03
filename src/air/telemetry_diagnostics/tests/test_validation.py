import unittest
from stream_detector import Detector
from train_evaluate import KEYS


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.detector = Detector('models/gru_42')
        self.row = dict(arrival_ns=1000000000, source_ns=1000000000, delivered=1,
                        **{k: 0.0 for k in KEYS})

    def test_nonfinite_is_not_healthy(self):
        for value in [float('nan'), float('inf'), 'broken']:
            row = dict(self.row, px=value)
            result = self.detector.step(row)
            self.assertTrue(result['alarm'])
            self.assertEqual(result['reason'], 'invalid_payload')

    def test_missing_fields_and_nonobject_do_not_crash(self):
        for row in [{}, [], None, {'delivered': 'bad'}]:
            self.assertEqual(self.detector.step(row)['reason'], 'invalid_payload')

    def test_backward_clock_flagged_and_history_reset(self):
        self.detector.step(self.row)
        result = self.detector.step(dict(self.row, source_ns=900000000))
        self.assertEqual(result['reason'], 'stale_timestamp')
        self.assertFalse(result['data_valid'])
        self.assertEqual(self.detector.features, [])

    def test_warmup_not_reported_as_model_ready(self):
        result = self.detector.step(self.row)
        self.assertFalse(result['model_ready'])
        self.assertTrue(result['data_valid'])


if __name__ == '__main__': unittest.main()
