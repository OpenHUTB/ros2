import unittest
import numpy as np
from pattern_data import geometry, sample_window, KEYS
from pattern_stream import Recognizer


class FeaturesTest(unittest.TestCase):
    def setUp(self):
        self.t = 1_700_000_000_000_000_000 + np.arange(161, dtype=np.int64) * 50_000_000
        s = np.arange(161) * .05
        self.z = np.column_stack([np.cos(s), np.sin(s), s * .2, -np.sin(s),
                                  np.cos(s), s * 0 + .2, -np.cos(s), -np.sin(s), s * 0])

    def test_translation_and_yaw_invariance(self):
        theta = .73
        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        changed = self.z.copy()
        for k in [0, 3, 6]:
            changed[:, k:k+2] = changed[:, k:k+2] @ rotation.T
        changed[:, :3] += [100, -30, 5]
        np.testing.assert_allclose(geometry(self.z), geometry(changed), atol=2e-6)

    def test_future_samples_cannot_change_window(self):
        expected = sample_window(self.t[:141], self.z[:141], int(self.t[140]))
        changed = self.z.copy()
        changed[141:] = np.nan
        np.testing.assert_array_equal(expected, sample_window(self.t, changed, int(self.t[140])))

    def test_gap_is_rejected(self):
        keep = np.ones(len(self.t), bool)
        keep[80:90] = False
        self.assertIsNone(sample_window(self.t[keep], self.z[keep], int(self.t[140])))

    def test_insufficient_history_is_rejected(self):
        self.assertIsNone(sample_window(self.t[:100], self.z[:100], int(self.t[99])))


class StreamTest(unittest.TestCase):
    def setUp(self):
        self.config = dict(mean=[0]*8, std=[1]*8, warmup_s=7,
                           inference_period_s=.5, confidence_threshold=.8, stable_predictions=3)
        self.stream = Recognizer(predict=lambda x: np.array([.96, .01, .01, .01, .01]), config=self.config)

    def message(self, i, segment='a'):
        return dict(timestamp_ns=1_700_000_000_000_000_000 + i*50_000_000,
                    segment_id=segment, **dict(zip(KEYS, [0, 0, -8, 1, 0, 0, 0, 0, 0])))

    def feed(self, start, stop, segment='a'):
        return [out for i in range(start, stop) if (out := self.stream.update(self.message(i, segment))) is not None]

    def test_warmup_and_debounce(self):
        self.assertEqual(self.feed(0, 140), [])
        out = self.feed(140, 181)
        self.assertIsNone(out[0]['event'])
        events = [r['event'] for r in out if r['event']]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['label'], 'line')

    def test_segment_reset_prevents_cross_flight_windows(self):
        self.feed(0, 170)
        self.assertEqual(self.feed(170, 200, 'b'), [])
        self.assertEqual(self.stream.stable, 'unknown')

    def test_gap_and_clock_reset_clear_history(self):
        self.feed(0, 170)
        self.assertIsNone(self.stream.update(self.message(190)))
        self.assertEqual(len(self.stream.samples), 1)
        self.assertIsNone(self.stream.update(self.message(100)))
        self.assertEqual(len(self.stream.samples), 1)

    def test_bad_telemetry_clears_history(self):
        self.feed(0, 170)
        message = self.message(170)
        message['vx'] = float('nan')
        with self.assertRaises(ValueError):
            self.stream.update(message)
        self.assertEqual(len(self.stream.samples), 0)

    def test_uncertain_is_logged_without_claiming_class(self):
        self.stream.predict = lambda x: np.array([.2]*5)
        out = self.feed(0, 170)
        self.assertEqual(out[-1]['stable_label'], 'uncertain')
        self.assertEqual(out[-1]['label'], 'uncertain')


if __name__ == '__main__':
    unittest.main()
