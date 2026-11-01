#!/usr/bin/env python3
"""模拟传感器数学计算测试."""

import math
import unittest

from ros1_message_time_sync.sensor_math import periodic_sensor_value


class TestPeriodicSensorValue(unittest.TestCase):
    """检查周期函数的关键采样点."""

    def test_quarter_periods(self):
        """信号应在预期时刻到达基础值、峰值和谷值."""
        self.assertAlmostEqual(
            periodic_sensor_value(0.0, 20.0, 2.0, 8.0),
            20.0,
        )
        self.assertAlmostEqual(
            periodic_sensor_value(2.0, 20.0, 2.0, 8.0),
            22.0,
        )
        self.assertAlmostEqual(
            periodic_sensor_value(4.0, 20.0, 2.0, 8.0),
            20.0,
        )
        self.assertAlmostEqual(
            periodic_sensor_value(6.0, 20.0, 2.0, 8.0),
            18.0,
        )

    def test_invalid_period_is_rejected(self):
        """非正周期应被拒绝."""
        with self.assertRaises(ValueError):
            periodic_sensor_value(0.0, 20.0, 2.0, 0.0)

    def test_value_stays_in_expected_range(self):
        """多个采样点的值应保持在基础值加减幅值范围内."""
        for sample_index in range(101):
            value = periodic_sensor_value(
                sample_index * 0.1,
                45.0,
                8.0,
                30.0,
            )
            self.assertGreaterEqual(value, 37.0 - 1e-12)
            self.assertLessEqual(value, 53.0 + 1e-12)
            self.assertTrue(math.isfinite(value))


if __name__ == '__main__':
    unittest.main()
