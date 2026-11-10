import unittest

from uav_gps_integrity_guard.core import GpsIntegrityGuard, GpsSample


class GpsIntegrityTests(unittest.TestCase):
    def sample(self, t, x, reported=3.0, accuracy=0.9, imu=3.0):
        return GpsSample(t, x, 0.0, 25.0, reported, accuracy, imu)

    def test_clean_track_is_valid(self):
        guard = GpsIntegrityGuard()
        result = guard.evaluate(self.sample(0.0, 0.0))
        result = guard.evaluate(self.sample(0.2, 0.6))
        self.assertEqual(result.state, "VALID")

    def test_position_jump_is_rejected(self):
        guard = GpsIntegrityGuard()
        guard.evaluate(self.sample(0.0, 0.0))
        result = guard.evaluate(self.sample(0.2, 8.0, reported=3.0))
        self.assertEqual(result.state, "REJECTED")

    def test_imu_residual_raises_risk(self):
        guard = GpsIntegrityGuard()
        guard.evaluate(self.sample(0.0, 0.0))
        result = guard.evaluate(self.sample(0.2, 0.6, reported=1.0, imu=6.0))
        self.assertGreater(result.risk, 0.0)

    def test_poor_accuracy_is_rejected(self):
        result = GpsIntegrityGuard().evaluate(self.sample(0.0, 0.0, accuracy=7.0))
        self.assertEqual(result.state, "REJECTED")

    def test_invalid_speed_rejected(self):
        with self.assertRaises(ValueError):
            GpsIntegrityGuard().evaluate(self.sample(0.0, 0.0, reported=-1.0))

    def test_non_increasing_time_rejected(self):
        guard = GpsIntegrityGuard()
        guard.evaluate(self.sample(1.0, 3.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0, 3.0))

    def test_recommendation_for_spoof(self):
        guard = GpsIntegrityGuard()
        result = guard.evaluate(self.sample(0.0, 0.0, accuracy=7.0))
        self.assertEqual(result.recommendation, "REJECT_GPS_USE_DEADRECKONING")

    def test_risk_is_bounded(self):
        result = GpsIntegrityGuard().evaluate(
            self.sample(0.0, 0.0, reported=100.0, accuracy=100.0, imu=0.0)
        )
        self.assertGreaterEqual(result.risk, 0.0)
        self.assertLessEqual(result.risk, 1.0)


if __name__ == "__main__":
    unittest.main()
