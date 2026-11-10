import unittest

from uav_compass_interference_guard.core import (
    CompassGuardConfig,
    CompassInterferenceGuard,
    CompassSample,
    _angle_diff_deg,
)


class CompassInterferenceGuardTests(unittest.TestCase):
    def sample(self, t, heading=30.0, gps=30.0, vio=30.0,
               field=50.0, current=12.0, speed=8.0, alt=30.0):
        return CompassSample(t, heading, gps, vio, field, current, speed, alt)

    def test_angle_diff_wraps(self):
        self.assertAlmostEqual(_angle_diff_deg(10.0, 350.0), 20.0)
        self.assertAlmostEqual(_angle_diff_deg(350.0, 10.0), -20.0)
        self.assertAlmostEqual(_angle_diff_deg(185.0, -175.0), 0.0, places=3)

    def test_calm_when_healthy(self):
        result = CompassInterferenceGuard().evaluate(self.sample(0.0))
        self.assertEqual(result.state, "CALM")
        self.assertEqual(result.recommendation, "CONTINUE")
        self.assertLess(result.risk, 0.1)

    def test_suspected_requires_hysteresis(self):
        guard = CompassInterferenceGuard()
        results = [
            guard.evaluate(self.sample(i * 0.1, field=78.0, alt=5.0))
            for i in range(4)
        ]
        self.assertEqual(results[0].state, "CALM")
        self.assertEqual(results[-1].state, "SUSPECTED")

    def test_gps_heading_only_counts_at_speed(self):
        # Large magnetic-vs-GPS mismatch but standing still -> GPS not trusted.
        result = CompassInterferenceGuard().evaluate(
            self.sample(0.0, heading=90.0, gps=30.0, vio=30.0, speed=0.5)
        )
        self.assertEqual(result.gps_heading_diff_deg, 0.0)
        # At speed the same mismatch is detected.
        result2 = CompassInterferenceGuard().evaluate(
            self.sample(0.0, heading=90.0, gps=30.0, vio=30.0, speed=8.0)
        )
        self.assertAlmostEqual(result2.gps_heading_diff_deg, 60.0, places=3)

    def test_biased_state_on_persistent_heading_mismatch(self):
        guard = CompassInterferenceGuard()
        result = None
        for i in range(6):
            result = guard.evaluate(
                self.sample(i * 0.1, heading=60.0, gps=30.0, vio=31.0, field=75.0)
            )
        self.assertEqual(result.state, "BIASED")
        self.assertEqual(result.recommendation, "FUSE_GPS_VIO_HEADING")

    def test_hard_field_limit_immediately_lost(self):
        result = CompassInterferenceGuard().evaluate(
            self.sample(0.0, heading=30.0, gps=30.0, vio=30.0, field=130.0)
        )
        self.assertEqual(result.state, "COMPASS_LOST")
        self.assertEqual(result.recommendation, "DISREGARD_COMPASS")

    def test_hard_heading_diff_immediately_lost(self):
        result = CompassInterferenceGuard().evaluate(
            self.sample(0.0, heading=120.0, gps=30.0, vio=30.0, speed=8.0)
        )
        self.assertEqual(result.state, "COMPASS_LOST")

    def test_low_altitude_flag(self):
        result = CompassInterferenceGuard().evaluate(self.sample(0.0, alt=5.0))
        self.assertTrue(result.low_altitude_flag)
        result2 = CompassInterferenceGuard().evaluate(self.sample(0.0, alt=30.0))
        self.assertFalse(result2.low_altitude_flag)

    def test_invalid_mag_field(self):
        with self.assertRaises(ValueError):
            CompassInterferenceGuard().evaluate(self.sample(0.0, field=-5.0))

    def test_non_increasing_timestamp(self):
        guard = CompassInterferenceGuard()
        guard.evaluate(self.sample(2.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0))

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            CompassGuardConfig(suspect_risk=0.8, biased_risk=0.6, lost_risk=0.9)


if __name__ == "__main__":
    unittest.main()
