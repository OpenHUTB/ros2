import unittest

from uav_link_outage_guard.core import (
    LinkGuardConfig,
    LinkQualityGuard,
    LinkSample,
)


class LinkOutageGuardTests(unittest.TestCase):
    def sample(self, t, rssi=-58.0, latency=40.0, loss=0.01, age=40.0):
        return LinkSample(t, rssi, latency, loss, age)

    def test_healthy_link(self):
        result = LinkQualityGuard().evaluate(self.sample(0.0))
        self.assertEqual(result.state, "HEALTHY")
        self.assertEqual(result.recommendation, "CONTINUE")

    def test_degraded_link_requires_hysteresis(self):
        guard = LinkQualityGuard()
        results = [guard.evaluate(self.sample(i * 0.1, -80.0, 190.0, 0.25, 260.0)) for i in range(4)]
        self.assertEqual(results[0].state, "HEALTHY")
        self.assertEqual(results[-1].state, "DEGRADED")

    def test_lost_link_is_immediate(self):
        result = LinkQualityGuard().evaluate(self.sample(0.0, -95.0, 700.0, 0.9, 2000.0))
        self.assertEqual(result.state, "LINK_LOST")
        self.assertEqual(result.recommendation, "EXECUTE_FAILSAFE")

    def test_return_state(self):
        guard = LinkQualityGuard()
        result = None
        for index in range(5):
            result = guard.evaluate(self.sample(index * 0.1, -88.0, 360.0, 0.58, 650.0))
        self.assertIn(result.state, {"RETURN", "LINK_LOST"})

    def test_negative_rssi_trend_estimates_horizon(self):
        guard = LinkQualityGuard()
        result = None
        for index in range(8):
            result = guard.evaluate(self.sample(index, -70.0 - 2.0 * index, 90.0, 0.1, 100.0))
        self.assertLess(result.trend_db_per_s, 0.0)
        self.assertIsNotNone(result.time_to_horizon_s)

    def test_invalid_packet_loss(self):
        with self.assertRaises(ValueError):
            LinkQualityGuard().evaluate(self.sample(0.0, loss=1.2))

    def test_non_increasing_timestamp(self):
        guard = LinkQualityGuard()
        guard.evaluate(self.sample(2.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0))

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            LinkGuardConfig(degraded_risk=0.8, hold_risk=0.6, return_risk=0.9)


if __name__ == "__main__":
    unittest.main()
