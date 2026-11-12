# -*- coding: utf-8 -*-
"""三层洋流模型单测：可复现性、剖面、方向。

    python3 -m unittest discover -s test -p "test_*.py" -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from rov_handling_eval.current_model import CurrentField, CurrentParams  # noqa: E402


def quiet(**kw):
    """关掉噪声，只留下平均流与剖面，方便断言。"""
    base = dict(mean_speed=0.0, sigma_ratio=0.0, turb_sigma=0.0)
    base.update(kw)
    return CurrentParams(**base)


class TestCurrent(unittest.TestCase):
    def test_zero_current(self):
        f = CurrentField(quiet())
        v = f.step(0.02, -5.0)
        np.testing.assert_allclose(v, 0.0)

    def test_mean_speed_and_direction(self):
        f = CurrentField(quiet(mean_speed=0.3, direction_deg=0.0))
        v = f.step(0.02, 0.0)          # 深度 0 → 剪切系数 1
        self.assertAlmostEqual(np.hypot(v[0], v[1]), 0.3, places=6)
        self.assertGreater(v[0], 0.29)  # 0° 指向 +x

    def test_direction_90(self):
        f = CurrentField(quiet(mean_speed=0.3, direction_deg=90.0))
        v = f.step(0.02, 0.0)
        self.assertGreater(v[1], 0.29)
        self.assertAlmostEqual(v[0], 0.0, places=6)

    def test_shear_decreases_with_depth(self):
        shallow = CurrentField(quiet(mean_speed=0.4)).step(0.02, 0.0)
        deep = CurrentField(quiet(mean_speed=0.4)).step(0.02, -10.0)
        self.assertGreater(np.linalg.norm(shallow[:2]), np.linalg.norm(deep[:2]))

    def test_no_vertical_component(self):
        f = CurrentField(quiet(mean_speed=0.3))
        self.assertAlmostEqual(f.step(0.02, -3.0)[2], 0.0)

    def test_reproducible_with_seed(self):
        """同 seed、同步长 → 完全一样；这是"实验可比较"的前提。"""
        a = CurrentField(quiet(mean_speed=0.3, sigma_ratio=0.2, seed=7))
        b = CurrentField(quiet(mean_speed=0.3, sigma_ratio=0.2, seed=7))
        va = [a.step(0.02, -2.0) for _ in range(50)]
        vb = [b.step(0.02, -2.0) for _ in range(50)]
        np.testing.assert_allclose(va, vb)

    def test_different_seed_differs(self):
        a = CurrentField(quiet(mean_speed=0.3, sigma_ratio=0.2, seed=1))
        b = CurrentField(quiet(mean_speed=0.3, sigma_ratio=0.2, seed=2))
        va = a.step(0.02, -2.0)
        vb = b.step(0.02, -2.0)
        self.assertGreater(np.linalg.norm(va - vb), 1e-6)

    def test_reset(self):
        f = CurrentField(quiet(mean_speed=0.3, sigma_ratio=0.2, seed=3))
        v1 = [f.step(0.02, -2.0) for _ in range(10)]
        f.reset()
        v2 = [f.step(0.02, -2.0) for _ in range(10)]
        np.testing.assert_allclose(v1, v2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
