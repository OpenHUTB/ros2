"""两个工况的仿真结果与汇总口径。"""

import unittest

import numpy as np

from brake_thermal_fade.disc import BrakeDisc
from brake_thermal_fade.scenario import (
    build_demo,
    simulate_descent,
    simulate_stop_cycles,
    summary,
)
from brake_thermal_fade.vehicle import BrakingSpec, stopping_distance_m


class DescentTest(unittest.TestCase):
    def setUp(self):
        self.disc = BrakeDisc()
        self.braking = BrakingSpec()

    def test_starts_at_ambient_and_heats_monotonically(self):
        run = simulate_descent(self.disc, self.braking, 60.0, duration_s=300.0, dt=1.0)
        self.assertAlmostEqual(float(run.temperature_c[0]), self.disc.spec.ambient_c)
        self.assertTrue(np.all(np.diff(run.temperature_c) >= -1e-9))
        self.assertEqual(run.time_s.size, run.temperature_c.size)

    def test_final_temperature_approaches_equilibrium(self):
        run = simulate_descent(self.disc, self.braking, 80.0, duration_s=3600.0, dt=2.0)
        self.assertIsNotNone(run.equilibrium_c)
        self.assertAlmostEqual(float(run.temperature_c[-1]), run.equilibrium_c, delta=3.0)

    def test_higher_speed_enters_fade_earlier(self):
        slow = simulate_descent(self.disc, self.braking, 40.0, duration_s=600.0, dt=1.0)
        middle = simulate_descent(self.disc, self.braking, 60.0, duration_s=600.0, dt=1.0)
        fast = simulate_descent(self.disc, self.braking, 80.0, duration_s=600.0, dt=1.0)
        self.assertIsNone(slow.fade_onset_s)  # 40 km/h 平衡温度低于衰退起点
        self.assertIsNotNone(middle.fade_onset_s)
        self.assertIsNotNone(fast.fade_onset_s)
        self.assertLess(fast.fade_onset_s, middle.fade_onset_s)

    def test_stop_distance_consistent_with_temperature(self):
        run = simulate_descent(self.disc, self.braking, 80.0, duration_s=600.0, dt=2.0)
        expected = stopping_distance_m(
            100.0 / 3.6, float(run.temperature_c[-1]), self.braking, self.disc.curve
        )
        self.assertAlmostEqual(run.stop_distance_m, expected, places=6)
        self.assertGreater(run.stop_distance_m, run.stop_distance_cold_m)

    def test_invalid_duration_is_rejected(self):
        with self.assertRaises(ValueError):
            simulate_descent(self.disc, self.braking, 60.0, duration_s=0.0)


class StopCyclesTest(unittest.TestCase):
    def setUp(self):
        self.disc = BrakeDisc()
        self.braking = BrakingSpec()

    def test_temperature_accumulates_between_stops(self):
        run = simulate_stop_cycles(self.disc, self.braking, cycles=8, cooldown_s=10.0)
        self.assertTrue(np.all(np.diff(run.temperature_before_c) > 0.0))
        self.assertTrue(np.all(run.temperature_after_c > run.temperature_before_c))

    def test_first_distance_equals_cold_distance(self):
        run = simulate_stop_cycles(self.disc, self.braking, cycles=6, cooldown_s=10.0)
        self.assertAlmostEqual(run.distance_m[0], run.distance_cold_m, places=9)
        # 冷态 100→0 km/h 的停车距离应在 40–60 m 量级
        self.assertGreater(run.distance_m[0], 40.0)
        self.assertLess(run.distance_m[0], 60.0)

    def test_distance_grows_with_cycles(self):
        run = simulate_stop_cycles(self.disc, self.braking, cycles=16, cooldown_s=10.0)
        self.assertTrue(np.all(np.diff(run.distance_m) >= -1e-9))
        self.assertGreater(run.distance_m[-1], run.distance_m[0] * 1.15)

    def test_more_cooldown_means_lower_temperature(self):
        quick = simulate_stop_cycles(self.disc, self.braking, cycles=10, cooldown_s=5.0)
        slow = simulate_stop_cycles(self.disc, self.braking, cycles=10, cooldown_s=60.0)
        self.assertGreater(
            float(quick.temperature_after_c[-1]), float(slow.temperature_after_c[-1])
        )

    def test_invalid_arguments_are_rejected(self):
        with self.assertRaises(ValueError):
            simulate_stop_cycles(self.disc, self.braking, cycles=0)
        with self.assertRaises(ValueError):
            simulate_stop_cycles(self.disc, self.braking, cycles=2, cooldown_s=-1.0)


class DemoBundleTest(unittest.TestCase):
    def test_bundle_has_three_speeds_and_one_cycle_run(self):
        bundle = build_demo(cycles=8, cooldown_s=10.0)
        self.assertEqual(len(bundle.descents), 3)
        self.assertIsNotNone(bundle.cycles)
        self.assertEqual(bundle.cycles.cycle.size, 8)

    def test_summary_is_self_consistent(self):
        payload = summary(build_demo(cycles=8, cooldown_s=10.0))
        self.assertEqual(len(payload["descents"]), 3)
        self.assertEqual(payload["disc"]["heat_capacity_j_per_k"], 3500.0)
        self.assertEqual(payload["friction"]["mu_cold"], 0.40)
        cycles = payload["stop_cycles"]
        self.assertEqual(cycles["cycles"], 8)
        self.assertGreater(cycles["distance_last_m"], cycles["distance_first_m"])
        self.assertEqual(len(cycles["per_cycle"]), 8)
        for descent in payload["descents"]:
            self.assertGreaterEqual(descent["stop_distance_m"], 0.0)

    def test_summary_is_deterministic(self):
        first = summary(build_demo(cycles=6, cooldown_s=10.0))
        second = summary(build_demo(cycles=6, cooldown_s=10.0))
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
