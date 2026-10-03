import unittest
from telemetry_data import CHANNELS, inject


class InjectionTests(unittest.TestCase):
    def setUp(self):
        self.rows = [dict(timestamp_ns=str(10**18 + i * 50_000_000),
                          **{k: str(i / 100) for k in CHANNELS}) for i in range(100)]

    def test_no_labels_or_reference_in_observations(self):
        observations, _ = inject(self.rows, 'position_drift', 1, .3)
        self.assertEqual(set(observations[0]), set(CHANNELS + ['arrival_ns', 'source_ns', 'delivered']))

    def test_dropout_is_missing_not_zero_measurement(self):
        obs, labels = inject(self.rows, 'dropout', 1, .3)
        self.assertTrue(any(not r['delivered'] for r in obs))
        for row, label in zip(obs, labels):
            self.assertEqual(row['delivered'], 1 - label['anomaly'])
            if label['anomaly']:
                self.assertEqual(row['px'], '')

    def test_freeze_retains_stale_source_time(self):
        obs, labels = inject(self.rows, 'freeze', 1, .3)
        frozen = [r for r, label in zip(obs, labels) if label['anomaly']]
        self.assertEqual(len({r['source_ns'] for r in frozen}), 1)
        self.assertEqual(len({r['arrival_ns'] for r in frozen}), len(frozen))

    def test_reproducible_and_original_unchanged(self):
        before = [r.copy() for r in self.rows]
        self.assertEqual(inject(self.rows, 'velocity_spike', 5, .3),
                         inject(self.rows, 'velocity_spike', 5, .3))
        self.assertEqual(before, self.rows)


if __name__ == '__main__':
    unittest.main()
