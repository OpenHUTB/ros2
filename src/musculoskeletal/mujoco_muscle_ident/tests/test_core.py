"""核心测试：正模型有界性与 LM 收敛。"""

import unittest

import numpy as np

from mujoco_muscle_ident.core_mj import (
    PARAM_NAMES, THETA_LIMIT, TRUE, lm_fit, make_measurement, simulate,
)


class TestForward(unittest.TestCase):
    def test_angle_within_limit(self):
        theta = simulate(dict(TRUE))
        self.assertLessEqual(theta.max(), THETA_LIMIT + 0.15)
        self.assertGreaterEqual(theta.min(), -THETA_LIMIT - 0.15)

    def test_deterministic(self):
        self.assertTrue(np.allclose(simulate(dict(TRUE)), simulate(dict(TRUE))))


class TestFit(unittest.TestCase):
    def test_recovers_truth(self):
        meas, _ = make_measurement()
        p0 = np.array([300.0, 0.26, 0.012, 0.035, 0.08])
        fit, _, _ = lm_fit(p0, meas)
        for k in PARAM_NAMES:
            self.assertLess(abs(fit[k] - TRUE[k]) / TRUE[k], 0.10)


if __name__ == "__main__":
    unittest.main()
