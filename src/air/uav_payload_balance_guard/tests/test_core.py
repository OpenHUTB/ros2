import unittest

from uav_payload_balance_guard.core import (
    PayloadBalanceConfig,
    PayloadBalanceGuard,
    PayloadSample,
)


class PayloadBalanceTests(unittest.TestCase):
    def sample(self, t=0.0, left=22.0, right=21.5, roll=1.0, sway=0.05, mass=4.0):
        return PayloadSample(t, left, right, roll, sway, mass, 8.0)

    def test_balanced_payload(self):
        self.assertEqual(
            PayloadBalanceGuard().evaluate(self.sample()).state, "BALANCED"
        )

    def test_unstable_payload_immediately_flags(self):
        result = PayloadBalanceGuard().evaluate(
            self.sample(left=35.0, right=4.0, roll=19.0, sway=1.5, mass=7.5)
        )
        self.assertEqual(result.state, "UNSTABLE")

    def test_offset_sign(self):
        result = PayloadBalanceGuard().evaluate(
            self.sample(left=18.0, right=28.0, roll=-8.0)
        )
        self.assertLess(result.estimated_cg_offset_m, 0.0)

    def test_risk_rose_with_payload(self):
        guard = PayloadBalanceGuard()
        low = guard.evaluate(self.sample(mass=2.0))
        guard2 = PayloadBalanceGuard()
        high = guard2.evaluate(self.sample(mass=7.8))
        self.assertGreaterEqual(high.risk, low.risk)

    def test_invalid_total_thrust(self):
        with self.assertRaises(ValueError):
            PayloadBalanceGuard().evaluate(self.sample(left=0.0, right=0.0))

    def test_non_increasing_time(self):
        guard = PayloadBalanceGuard()
        guard.evaluate(self.sample(1.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0))

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            PayloadBalanceConfig(adjust_risk=0.7, warning_risk=0.6)

    def test_recommendation_for_unstable(self):
        result = PayloadBalanceGuard().evaluate(
            self.sample(left=35.0, right=4.0, roll=19.0, sway=1.5)
        )
        self.assertEqual(result.recommendation, "ABORT_MANEUVER_AND_LAND")

    def test_ratios_are_bounded(self):
        result = PayloadBalanceGuard().evaluate(self.sample(mass=20.0))
        self.assertLessEqual(result.payload_ratio, 1.0)
        self.assertLessEqual(result.thrust_imbalance_ratio, 1.0)


if __name__ == "__main__":
    unittest.main()
