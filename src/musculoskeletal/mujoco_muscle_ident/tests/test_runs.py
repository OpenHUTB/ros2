"""实验层测试：噪声单调、相关性对称、D-最优与误差一致。"""

import unittest

import numpy as np

from mujoco_muscle_ident import runs


class TestNoise(unittest.TestCase):
    def test_error_grows_with_noise(self):
        res = runs.run_i2(noises=(0.0, 5.0), seeds=2)
        clean, noisy = res["levels"]
        self.assertGreater(noisy["mean_err_pct"], clean["mean_err_pct"])


class TestCorrelation(unittest.TestCase):
    def test_symmetric_unit_diagonal(self):
        m = np.asarray(runs.run_i3()["matrix"])
        self.assertTrue(np.allclose(m, m.T, atol=1e-6))
        self.assertTrue(np.allclose(np.diag(m), 1.0, atol=1e-6))


class TestExcitation(unittest.TestCase):
    def test_d_optimal_ordering(self):
        lv = runs.run_i5()["levels"]           # 已按 D 降序
        d = [x["d_optimal"] for x in lv]
        self.assertEqual(d, sorted(d, reverse=True))
        # D 越大误差越小
        self.assertLessEqual(lv[0]["mean_err_pct"], lv[-1]["mean_err_pct"] + 1e-9)


if __name__ == "__main__":
    unittest.main()
