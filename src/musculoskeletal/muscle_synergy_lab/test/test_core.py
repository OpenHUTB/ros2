# -*- coding: utf-8 -*-
"""肌肉协同分解的单元测试（纯 numpy，不需要 ROS）。

    python -m unittest discover -s test -p "test_*.py" -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from muscle_synergy_lab.core import (k_for_vaf, nmf, recovery_check,  # noqa: E402
                                     scan_k, subspace_angle_deg, synth_data,
                                     vaf)


class TestSynthData(unittest.TestCase):
    def test_shape_and_nonneg(self):
        d = synth_data(n_muscles=8, n_synergies=3, n_samples=120, seed=1)
        self.assertEqual(d.E.shape, (8, 120))
        self.assertEqual(d.W_true.shape, (8, 3))
        self.assertTrue(np.all(d.E >= 0))
        self.assertTrue(np.all(d.W_true >= 0))

    def test_reproducible(self):
        a = synth_data(seed=7).E
        b = synth_data(seed=7).E
        np.testing.assert_allclose(a, b)


class TestNMF(unittest.TestCase):
    def setUp(self):
        self.d = synth_data(noise=0.03, seed=0)

    def test_vaf_monotone(self):
        """k 越大重建误差不应变大（VAF 单调不减）。"""
        rows = scan_k(self.d.E, k_max=6)
        v = [r["vaf"] for r in rows]
        for a, b in zip(v, v[1:]):
            self.assertGreaterEqual(b, a - 1e-3)

    def test_vaf_bounds(self):
        rows = scan_k(self.d.E, k_max=8)
        for r in rows:
            self.assertGreaterEqual(r["vaf"], -1e-9)
            self.assertLessEqual(r["vaf"], 1.0 + 1e-9)

    def test_full_rank_perfect(self):
        """k = 肌肉数时必然完美重建。"""
        W, H = nmf(self.d.E, self.d.E.shape[0])
        self.assertGreater(vaf(self.d.E, W, H), 0.999)

    def test_factors_nonneg(self):
        W, H = nmf(self.d.E, 3)
        self.assertTrue(np.all(W >= 0))
        self.assertTrue(np.all(H >= 0))

    def test_deterministic(self):
        W1, _ = nmf(self.d.E, 3, seed=5)
        W2, _ = nmf(self.d.E, 3, seed=5)
        np.testing.assert_allclose(W1, W2)

    def test_bad_k(self):
        with self.assertRaises(ValueError):
            nmf(self.d.E, 0)
        with self.assertRaises(ValueError):
            nmf(self.d.E, self.d.E.shape[0] + 1)


class TestRecovery(unittest.TestCase):
    def test_noiseless_recovers(self):
        """无噪声、k 取真值时，VAF 接近 1 且协同方向基本一致（夹角 < 2°）。"""
        d = synth_data(noise=0.0, seed=0)
        r = recovery_check(d.E, d.W_true, 3)
        self.assertGreater(r["vaf"], 0.99)
        self.assertLess(r["angle_deg"], 2.0)

    def test_underfit_vaf_low_angle_small(self):
        """k 小于真值：VAF 明显偏低，但子空间夹角仍然很小。

        夹角度量的是"子空间包含关系"——k=2 的因子落在真值 3 维空间内部，
        所以夹角 ≈0 并不能说明"选对了协同数"。这正是 review 指出的问题。
        """
        d = synth_data(noise=0.0, seed=0)
        r = recovery_check(d.E, d.W_true, 2)
        self.assertLess(r["vaf"], 0.90)
        self.assertLess(r["angle_deg"], 2.0)

    def test_angle_permutation_invariant(self):
        """回归测试：把真值列重排，夹角必须不变（旧实现会读出 37.6/48.2/84.8）。"""
        d = synth_data(noise=0.03, seed=0)
        W, _H = nmf(d.E, 2, seed=0)
        base = subspace_angle_deg(W, d.W_true)
        for perm in ([1, 2, 0], [2, 0, 1], [2, 1, 0]):
            self.assertAlmostEqual(subspace_angle_deg(W, d.W_true[:, perm]), base, places=6)

    def test_angle_can_tell_different_structure(self):
        """夹角对"完全不同的协同结构"是有区分力的（这里应该很大）。"""
        d = synth_data(noise=0.03, seed=0)
        rng = np.random.default_rng(3)
        other = rng.random(d.W_true.shape)
        self.assertGreater(subspace_angle_deg(d.W_true, other), 45.0)

    def test_angle_nested_across_k(self):
        """1..5 维的解与真值 3 维空间的夹角都很小（嵌套/包含两种情况）。"""
        d = synth_data(noise=0.03, seed=0)
        for k in (1, 2, 3, 4, 5):
            self.assertLess(recovery_check(d.E, d.W_true, k)["angle_deg"], 3.0)

    def test_overfit_subspace_contains(self):
        """k 大于真值时，子空间包含真值，夹角应回落到 < 3°。"""
        d = synth_data(noise=0.03, seed=0)
        r = recovery_check(d.E, d.W_true, 4)
        self.assertLess(r["angle_deg"], 3.0)

    def test_k_for_vaf(self):
        d = synth_data(noise=0.03, seed=0)
        rows = scan_k(d.E, k_max=6)
        self.assertEqual(k_for_vaf(rows, 0.95), 3)
        self.assertIsNone(k_for_vaf(rows, 1.5))

    def test_angle_basics(self):
        W = np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
        self.assertLess(subspace_angle_deg(W, W), 1e-3)
        # 正交子空间：e1 张成的直线 vs e2 张成的直线
        a = np.array([[1.0], [0.0], [0.0]])
        b = np.array([[0.0], [1.0], [0.0]])
        self.assertAlmostEqual(subspace_angle_deg(a, b), 90.0, delta=1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
