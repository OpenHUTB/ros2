# -*- coding: utf-8 -*-
"""Bezier 曲线与 B 样条算法离线单元测试。

不依赖 pygame，纯算法层验证。
"""

import math
import unittest

from bezier_trajectory.bezier import (
    binomial,
    bezier_point_de_casteljau,
    bezier_point_bernstein,
    bezier_derivative,
    curvature,
    arc_length,
    sample_curve,
    bspline_basis,
    bspline_point,
    sample_bspline,
)


class TestBinomial(unittest.TestCase):
    """组合数 C(n, k)。"""

    def test_basic_values(self):
        self.assertEqual(binomial(0, 0), 1)
        self.assertEqual(binomial(5, 0), 1)
        self.assertEqual(binomial(5, 5), 1)
        self.assertEqual(binomial(5, 2), 10)
        self.assertEqual(binomial(6, 3), 20)
        self.assertEqual(binomial(10, 4), 210)

    def test_symmetry(self):
        for n in range(10):
            for k in range(n + 1):
                self.assertEqual(binomial(n, k), binomial(n, n - k))

    def test_out_of_range(self):
        self.assertEqual(binomial(5, -1), 0)
        self.assertEqual(binomial(5, 6), 0)


class TestBezierEndpoints(unittest.TestCase):
    """Bezier 曲线端点插值性质：B(0)=P0, B(1)=Pn。"""

    def test_start_equals_first_control_point(self):
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        result = bezier_point_de_casteljau(pts, 0.0)
        self.assertAlmostEqual(result[0], pts[0][0])
        self.assertAlmostEqual(result[1], pts[0][1])

    def test_end_equals_last_control_point(self):
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        result = bezier_point_de_casteljau(pts, 1.0)
        self.assertAlmostEqual(result[0], pts[-1][0])
        self.assertAlmostEqual(result[1], pts[-1][1])


class TestDeCasteljauVsBernstein(unittest.TestCase):
    """de Casteljau 与 Bernstein 两种求值方式结果应一致。"""

    def test_two_methods_match(self):
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        for i in range(11):
            t = i / 10
            p1 = bezier_point_de_casteljau(pts, t)
            p2 = bezier_point_bernstein(pts, t)
            self.assertAlmostEqual(p1[0], p2[0], places=8)
            self.assertAlmostEqual(p1[1], p2[1], places=8)

    def test_linear_bezier_is_line(self):
        """2 个控制点时 Bezier 退化为直线段。"""
        pts = [(0, 0), (100, 200)]
        for i in range(11):
            t = i / 10
            x, y = bezier_point_de_casteljau(pts, t)
            self.assertAlmostEqual(x, 100 * t)
            self.assertAlmostEqual(y, 200 * t)


class TestDerivative(unittest.TestCase):
    """导数与切向量。"""

    def test_derivative_at_endpoints(self):
        """端点处一阶导数 = 相邻控制点差 * n。"""
        pts = [(0, 0), (100, 0), (100, 100), (200, 100)]
        n = len(pts) - 1  # 3
        # B'(0) = n * (P1 - P0)
        dx, dy = bezier_derivative(pts, 0.0, 1)
        self.assertAlmostEqual(dx, n * (pts[1][0] - pts[0][0]))
        self.assertAlmostEqual(dy, n * (pts[1][1] - pts[0][1]))
        # B'(1) = n * (Pn - Pn-1)
        dx, dy = bezier_derivative(pts, 1.0, 1)
        self.assertAlmostEqual(dx, n * (pts[-1][0] - pts[-2][0]))
        self.assertAlmostEqual(dy, n * (pts[-1][1] - pts[-2][1]))

    def test_derivative_zero_order_equals_point(self):
        pts = [(10, 20), (30, 40), (50, 60)]
        p = bezier_derivative(pts, 0.5, 0)
        p0 = bezier_point_de_casteljau(pts, 0.5)
        self.assertAlmostEqual(p[0], p0[0])
        self.assertAlmostEqual(p[1], p0[1])


class TestCurvature(unittest.TestCase):
    """曲率。"""

    def test_curvature_of_line_is_zero(self):
        """直线的曲率应为 0。"""
        pts = [(0, 0), (100, 0)]
        for i in range(5):
            t = i / 4
            self.assertAlmostEqual(curvature(pts, t), 0.0)

    def test_curvature_nonnegative(self):
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        for i in range(11):
            t = i / 10
            self.assertGreaterEqual(curvature(pts, t), 0.0)

    def test_curvature_nonzero_for_curved(self):
        """弯曲曲线应有非零曲率。"""
        pts = [(0, 0), (0, 100), (100, 100), (100, 0)]
        self.assertGreater(curvature(pts, 0.5), 0.0)


class TestArcLength(unittest.TestCase):
    """弧长。"""

    def test_line_segment_length(self):
        pts = [(0, 0), (300, 400)]
        length = arc_length(pts, samples=100)
        self.assertAlmostEqual(length, 500.0, places=1)

    def test_length_positive_for_curve(self):
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        length = arc_length(pts)
        self.assertGreater(length, 0.0)
        # 弧长应大于等于直线距离
        direct = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        self.assertGreaterEqual(length, direct - 1)

    def test_empty_points(self):
        self.assertEqual(arc_length([]), 0.0)
        self.assertEqual(arc_length([(1, 2)]), 0.0)


class TestSampling(unittest.TestCase):
    """采样函数。"""

    def test_sample_count(self):
        pts = [(0, 0), (50, 100), (100, 0)]
        samples = sample_curve(pts, n=50)
        self.assertEqual(len(samples), 50)

    def test_sample_endpoints(self):
        pts = [(10, 20), (50, 100), (100, 0)]
        samples = sample_curve(pts, n=10)
        self.assertAlmostEqual(samples[0][0], pts[0][0])
        self.assertAlmostEqual(samples[-1][0], pts[-1][0])


class TestBSpline(unittest.TestCase):
    """B 样条曲线。"""

    def test_bspline_basis_nonneg(self):
        """基函数值非负（B 样条性质）。"""
        knots = [0, 0, 0, 1, 2, 2, 2]
        for u_val in [0.0, 0.5, 1.0, 1.5]:
            for i in range(4):
                self.assertGreaterEqual(bspline_basis(i, 2, u_val, knots), 0.0)

    def test_bspline_point_in_convex_hull(self):
        """B 样条曲线点应在控制点坐标范围内。"""
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        samples = sample_bspline(pts, degree=3, n=50)
        xs = [p[0] for p in samples]
        ys = [p[1] for p in samples]
        self.assertGreaterEqual(min(xs), -10)   # B 样条不一定通过控制点但应在附近
        self.assertLessEqual(max(xs), 310)
        self.assertGreaterEqual(min(ys), -10)
        self.assertLessEqual(max(ys), 310)

    def test_bspline_sample_count(self):
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        samples = sample_bspline(pts, degree=3, n=30)
        self.assertEqual(len(samples), 30)

    def test_bspline_endpoint_behavior(self):
        """clamped B 样条通过端点（与 Bezier 类似的性质）。"""
        pts = [(0, 0), (100, 200), (200, 50), (300, 300)]
        samples = sample_bspline(pts, degree=3, n=20)
        # clamped B 样条起点应接近第一个控制点
        self.assertAlmostEqual(samples[0][0], pts[0][0], places=0)


if __name__ == "__main__":
    unittest.main()
