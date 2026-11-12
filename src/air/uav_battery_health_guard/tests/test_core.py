import unittest

from uav_battery_health_guard.core import (
    BatteryHealthConfig,
    BatteryHealthGuard,
    BatterySample,
)


class BatteryHealthTests(unittest.TestCase):
    def sample(self, t=0.0, cells=(3.9, 3.89, 3.9, 3.89), current=20, temp=30, resistance=14):
        return BatterySample(t, cells, current, temp, resistance)

    def test_balanced_battery_is_healthy(self):
        self.assertEqual(BatteryHealthGuard().evaluate(self.sample()).state, "HEALTHY")

    def test_critical_imbalance_triggers_landing(self):
        result = BatteryHealthGuard().evaluate(
            self.sample(cells=(4.0, 3.70, 3.95, 3.92))
        )
        self.assertEqual(result.state, "LAND_NOW")

    def test_moderate_imbalance_degrades(self):
        guard = BatteryHealthGuard()
        result = None
        for index in range(3):
            result = guard.evaluate(
                self.sample(index * 0.2, cells=(3.92, 3.80, 3.90, 3.88))
            )
        self.assertIn(result.state, {"DEGRADED", "LIMIT_LOAD"})

    def test_high_temperature_triggers_landing(self):
        result = BatteryHealthGuard().evaluate(self.sample(temp=68.0))
        self.assertEqual(result.state, "LAND_NOW")

    def test_requires_two_cells(self):
        with self.assertRaises(ValueError):
            BatteryHealthGuard().evaluate(self.sample(cells=(3.9,)))

    def test_invalid_voltage(self):
        with self.assertRaises(ValueError):
            BatteryHealthGuard().evaluate(self.sample(cells=(3.9, -1.0)))

    def test_non_increasing_time(self):
        guard = BatteryHealthGuard()
        guard.evaluate(self.sample(1.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0))

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            BatteryHealthConfig(
                degraded_imbalance_v=0.2,
                limit_imbalance_v=0.1,
                critical_imbalance_v=0.3,
            )

    def test_risk_is_bounded(self):
        result = BatteryHealthGuard().evaluate(
            self.sample(cells=(3.0, 4.2), temp=80, resistance=100)
        )
        self.assertLessEqual(result.risk, 1.0)
        self.assertGreaterEqual(result.risk, 0.0)


if __name__ == "__main__":
    unittest.main()
