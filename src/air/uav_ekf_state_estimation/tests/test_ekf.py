import unittest

import numpy as np

from uav_ekf_state_estimation.attitude import ComplementaryAttitude, euler_to_rotation
from uav_ekf_state_estimation.ekf import NavigationEKF


class EkfTest(unittest.TestCase):
    def test_gps_update_reduces_covariance(self):
        ekf = NavigationEKF(np.zeros(3))
        before = float(np.trace(ekf.P))
        ekf.update_position(np.array([1.0, -2.0]), sigma_m=1.0)
        self.assertLess(float(np.trace(ekf.P)), before)
        self.assertTrue(np.all(np.isfinite(ekf.x)))

    def test_barometer_update_changes_altitude(self):
        ekf = NavigationEKF(np.array([0.0, 0.0, -20.0]))
        ekf.update_altitude(-10.0, sigma_m=0.5)
        self.assertGreater(ekf.position[2], -20.0)
        self.assertTrue(np.all(np.isfinite(ekf.x)))

    def test_hover_prediction_is_finite(self):
        ekf = NavigationEKF(np.array([0.0, 0.0, -20.0]))
        rotation = euler_to_rotation(0.0, 0.0, 0.0)
        for _ in range(100):
            ekf.predict(np.array([0.0, 0.0, -9.80665]), rotation, 0.01)
        self.assertTrue(np.all(np.isfinite(ekf.x)))
        self.assertLess(float(np.linalg.norm(ekf.velocity)), 1e-6)

    def test_attitude_filter_tracks_small_angles(self):
        attitude = ComplementaryAttitude()
        for _ in range(200):
            attitude.update(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, 9.80665]),
                np.array([1.0, 0.0, 0.0]),
                0.01,
            )
        self.assertAlmostEqual(attitude.roll, 0.0, places=2)
        self.assertAlmostEqual(attitude.pitch, 0.0, places=2)


if __name__ == "__main__":
    unittest.main()
