#!/usr/bin/env python3
"""正弦信号计算函数的单元测试."""

import math
import unittest

from ros1_parameter_sine_wave.math_utils import compute_sine


class TestComputeSine(unittest.TestCase):
    """检查正弦信号的关键采样点."""

    def test_quarter_periods(self):
        """1 Hz、幅值为 2 的信号应在四分之一周期处到达峰值."""
        self.assertAlmostEqual(compute_sine(2.0, 1.0, 0.0), 0.0)
        self.assertAlmostEqual(compute_sine(2.0, 1.0, 0.25), 2.0)
        self.assertAlmostEqual(compute_sine(2.0, 1.0, 0.5), 0.0)
        self.assertAlmostEqual(compute_sine(2.0, 1.0, 0.75), -2.0)

    def test_phase_shift(self):
        """初相位应为 pi/2 时，零时刻的值为幅值."""
        value = compute_sine(1.0, 1.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(value, 1.0)


if __name__ == '__main__':
    unittest.main()
