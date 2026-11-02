"""四分之一车模型的静力学、参数与积分行为。"""

import unittest

import numpy as np

from speed_bump_ride_comfort.bump import SpeedBump
from speed_bump_ride_comfort.vehicle import GRAVITY, QuarterCar


class QuarterCarParameterTest(unittest.TestCase):
    def test_default_parameters_are_physically_plausible(self):
        vehicle = QuarterCar()
        self.assertAlmostEqual(vehicle.body_natural_frequency_hz, 1.474, delta=0.005)
        self.assertAlmostEqual(vehicle.wheel_natural_frequency_hz, 12.554, delta=0.02)
        self.assertAlmostEqual(vehicle.damping_ratio, 0.278, delta=0.002)
        # 单角静载应在 3–5 kN 量级（约 1.5 t 车的四分之一）
        self.assertGreater(vehicle.static_wheel_load_n, 3000.0)
        self.assertLess(vehicle.static_wheel_load_n, 5000.0)
        self.assertAlmostEqual(
            vehicle.static_wheel_load_n, (350.0 + 45.0) * GRAVITY, places=6
        )

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            QuarterCar(sprung_mass_kg=0.0)
        with self.assertRaises(ValueError):
            QuarterCar(unsprung_mass_kg=-1.0)
        with self.assertRaises(ValueError):
            QuarterCar(suspension_stiffness_n_per_m=0.0)
        with self.assertRaises(ValueError):
            QuarterCar(tire_stiffness_n_per_m=-5.0)
        with self.assertRaises(ValueError):
            QuarterCar(suspension_damping_n_s_per_m=-1.0)


class QuarterCarDynamicsTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = QuarterCar()

    def test_static_equilibrium_has_zero_derivative(self):
        derivative = self.vehicle.derivatives(np.zeros(4), 0.0)
        self.assertTrue(np.allclose(derivative, 0.0))

    def test_compression_accelerates_body_upwards_and_wheel_downwards(self):
        # 车身相对车轮被压缩 10 mm：悬架弹簧把车身推上去，把非悬挂质量压下去。
        state = np.array([0.010, 0.0, 0.0, 0.0])
        derivative = self.vehicle.derivatives(state, 0.0)
        self.assertLess(derivative[1], 0.0)
        self.assertGreater(derivative[3], 0.0)
        self.assertAlmostEqual(derivative[1], -0.010 * 30000.0 / 350.0, places=9)
        self.assertAlmostEqual(derivative[3], 0.010 * 30000.0 / 45.0, places=9)

    def test_tire_load_ratio_starts_at_static_load(self):
        passage = self.vehicle.simulate(SpeedBump(), 3.0, dt=2.0e-3)
        self.assertAlmostEqual(float(passage.tire_load_ratio[0]), 1.0, places=9)
        self.assertLess(float(passage.time_s[0]), 0.0)  # 有接近段

    def test_response_decays_after_the_bump(self):
        passage = self.vehicle.simulate(SpeedBump(), 5.0, dt=2.0e-3, settle_s=2.0)
        early = np.abs(passage.sprung_accel_mps2[passage.time_s <= 0.6])
        late = np.abs(passage.sprung_accel_mps2[passage.time_s >= 1.6])
        self.assertLess(float(late.max()), 0.2 * float(early.max()))

    def test_simulation_is_deterministic(self):
        first = self.vehicle.simulate(SpeedBump(), 4.0, dt=2.0e-3)
        second = self.vehicle.simulate(SpeedBump(), 4.0, dt=2.0e-3)
        for name in ("time_s", "road_height_m", "sprung_position_m",
                     "sprung_accel_mps2", "travel_m", "tire_load_ratio"):
            self.assertTrue(np.array_equal(getattr(first, name), getattr(second, name)))

    def test_invalid_simulation_arguments_are_rejected(self):
        bump = SpeedBump()
        with self.assertRaises(ValueError):
            self.vehicle.simulate(bump, 0.0)
        with self.assertRaises(ValueError):
            self.vehicle.simulate(bump, 3.0, dt=0.0)
        with self.assertRaises(ValueError):
            self.vehicle.simulate(bump, 3.0, settle_s=-1.0)

    def test_higher_speed_gives_larger_body_acceleration(self):
        peaks = []
        for speed in (1.5, 3.0, 6.0):
            passage = self.vehicle.simulate(SpeedBump(), speed, dt=2.0e-3)
            peaks.append(float(np.max(np.abs(passage.sprung_accel_mps2))))
        self.assertLess(peaks[0], peaks[1])
        self.assertLess(peaks[1], peaks[2])


if __name__ == "__main__":
    unittest.main()
