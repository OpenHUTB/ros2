"""卡尔曼滤波的单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import kalman as kf


class TestKalman(unittest.TestCase):
    def setUp(self):
        self.r = kf.simulate(duration=10.0, dt=0.05)

    def test_filter_beats_raw_measurement(self):
        meas = kf.rmse(self.r["z"], self.r["p_true"])
        est = kf.rmse(self.r["p_hat"], self.r["p_true"])
        self.assertLess(est, meas * 0.6)

    def test_velocity_is_estimated_without_measurement(self):
        idx = [i for i, t in enumerate(self.r["ts"]) if t >= 3.0]
        v_rmse = kf.rmse([self.r["v_hat"][i] for i in idx],
                         [self.r["v_true"][i] for i in idx])
        self.assertLess(v_rmse, 0.15)
        self.assertAlmostEqual(self.r["v_hat"][-1], self.r["v_true"][-1], delta=0.15)

    def test_covariance_shrinks_and_is_psd(self):
        f = kf.KalmanCV(0.05)
        p0 = f.P[0][0]
        for k in range(50):
            f.predict()
            f.update(1.0 + 0.1 * (k % 3))
            P = f.P
            self.assertAlmostEqual(P[0][1], P[1][0], places=12)   # 对称
            self.assertGreaterEqual(P[0][0] * P[1][1] - P[0][1] ** 2, -1e-12)
        self.assertLess(f.P[0][0], p0)                            # 方差下降

    def test_gain_balances_by_noise_ratio(self):
        # R 大（测量不可信）-> 位置增益应更小，即一步更新对状态的改动更小
        low_r = kf.KalmanCV(0.05, q_accel=0.05, r_meas=0.01)
        high_r = kf.KalmanCV(0.05, q_accel=0.05, r_meas=100.0)
        deltas = []
        for f in (low_r, high_r):
            f.predict()
            before = f.pos
            f.update(0.0)                     # 测量把它往 0 拉
            deltas.append(abs(f.pos - before))
        self.assertGreater(deltas[0], deltas[1] * 10)   # 低 R 增益远大于高 R
        self.assertGreater(low_r.P[0][0], 0.0)

    def test_matrix_helpers(self):
        A = [[1.0, 2.0], [3.0, 4.0]]
        I = kf.mat_mul(A, kf.inv2(A))
        self.assertAlmostEqual(I[0][0], 1.0, places=10)
        self.assertAlmostEqual(I[0][1], 0.0, places=10)
        self.assertAlmostEqual(I[1][1], 1.0, places=10)
        self.assertEqual(kf.transpose([[1.0, 2.0]]), [[1.0], [2.0]])

    def test_rng_reproducible(self):
        a = [kf.Rng(7).normal() for _ in range(1)]
        r1, r2 = kf.Rng(7), kf.Rng(7)
        s1 = [r1.normal() for _ in range(5)]
        s2 = [r2.normal() for _ in range(5)]
        self.assertEqual(s1, s2)
        self.assertAlmostEqual(a[0], s1[0])
        # 均值近似 0、方差近似 1
        rng = kf.Rng(99)
        xs = [rng.normal() for _ in range(4000)]
        mean = sum(xs) / len(xs)
        var = sum((x - mean) ** 2 for x in xs) / len(xs)
        self.assertLess(abs(mean), 0.06)
        self.assertAlmostEqual(var, 1.0, delta=0.1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
