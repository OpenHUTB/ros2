import unittest

from uav_perception_confidence_guard.core import (
    PerceptionConfidenceGuard,
    PerceptionGuardConfig,
    PerceptionSample,
)


class PerceptionConfidenceTests(unittest.TestCase):
    def sample(self, t=0.0, count=2, confidence=0.9, entropy=0.2, latency=40, age=50, lux=400, expected=True):
        return PerceptionSample(t, 1, expected, count, confidence, entropy, latency, age, lux)

    def test_nominal_perception(self):
        self.assertEqual(
            PerceptionConfidenceGuard().evaluate(self.sample()).state, "NOMINAL"
        )

    def test_missing_expected_target_is_blind(self):
        result = PerceptionConfidenceGuard().evaluate(self.sample(count=0))
        self.assertEqual(result.state, "BLIND")

    def test_missing_expected_target_risk_matches_formula(self):
        result = PerceptionConfidenceGuard().evaluate(self.sample(count=0))
        self.assertAlmostEqual(result.risk, 0.0275, places=6)

    def test_missing_target_not_expected_is_not_immediate_blind(self):
        result = PerceptionConfidenceGuard().evaluate(
            self.sample(count=0, expected=False)
        )
        self.assertNotEqual(result.state, "BLIND")

    def test_high_latency_is_blind(self):
        result = PerceptionConfidenceGuard().evaluate(self.sample(latency=520))
        self.assertEqual(result.state, "BLIND")

    def test_low_confidence_raises_risk(self):
        result = PerceptionConfidenceGuard().evaluate(
            self.sample(confidence=0.25, entropy=1.2, latency=280, age=900)
        )
        self.assertGreater(result.risk, 0.35)

    def test_invalid_confidence(self):
        with self.assertRaises(ValueError):
            PerceptionConfidenceGuard().evaluate(self.sample(confidence=1.2))

    def test_non_increasing_time(self):
        guard = PerceptionConfidenceGuard()
        guard.evaluate(self.sample(1.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0))

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            PerceptionGuardConfig(degraded_risk=0.8, unreliable_risk=0.6)

    def test_risk_is_bounded(self):
        result = PerceptionConfidenceGuard().evaluate(
            self.sample(confidence=0, entropy=10, latency=1000, age=2000, lux=0)
        )
        self.assertLessEqual(result.risk, 1.0)


if __name__ == "__main__":
    unittest.main()
