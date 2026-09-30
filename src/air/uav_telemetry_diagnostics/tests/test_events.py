import unittest
import numpy as np
from holdout_evaluate import score_events, scenarios
from telemetry_data import CHANNELS


class EventTests(unittest.TestCase):
    def test_preexisting_alarm_not_counted_as_detection(self):
        labels = np.array([False, False, True, True, False])
        r = score_events(np.ones(5, dtype=bool), labels, np.arange(5.))
        self.assertFalse(r['detected'])
        self.assertEqual(r['false_alarm_events'], 1)

    def test_delay_and_recovery_false_alarm(self):
        labels = np.array([False, True, True, False, False])
        r = score_events(np.array([False, False, True, False, True]), labels, np.arange(5.) * .05)
        self.assertAlmostEqual(r['delay_s'], .05)
        self.assertEqual(r['false_alarm_events'], 1)

    def test_clean_labels_and_smooth_recovery(self):
        rows = [dict(timestamp_ns=10**18 + i*50_000_000,
                     **{k: '0' for k in CHANNELS}) for i in range(361)]
        items = list(scenarios(rows))
        self.assertEqual(len(items), 9)
        self.assertFalse(items[0][3].any())
        for kind, severity, obs, _ in items:
            if kind == 'smooth_position_drift':
                p = np.array([r['px'] for r in obs])
                self.assertLessEqual(np.abs(np.diff(p)).max(), severity * .05 + 1e-8)
                self.assertAlmostEqual(p[-1], 0)


if __name__ == '__main__': unittest.main()
