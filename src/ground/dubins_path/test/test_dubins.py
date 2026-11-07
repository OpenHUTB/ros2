# -*- coding: utf-8 -*-
"""Dubins 曲线算法离线单元测试。

不依赖 pygame，纯算法层验证。
"""

import math
import unittest

from dubins_path.dubins import (
    mod2pi,
    shortest_path,
    all_paths,
    sample_path,
    _advance,
    _compute_path,
    _endpoint,
    DUBINS_TYPES,
)


def yaw_close(a: float, b: float, tol: float = 1e-6) -> bool:
    """比较两个 yaw（考虑 2π 周期）。"""
    d = mod2pi(a - b + math.pi) - math.pi
    return abs(d) <= tol


class TestMod2pi(unittest.TestCase):
    """角度归一化到 [0, 2π)。"""

    def test_zero(self):
        self.assertAlmostEqual(mod2pi(0.0), 0.0)

    def test_two_pi(self):
        self.assertAlmostEqual(mod2pi(2.0 * math.pi), 0.0)

    def test_pi(self):
        self.assertAlmostEqual(mod2pi(math.pi), math.pi)

    def test_negative_pi(self):
        self.assertAlmostEqual(mod2pi(-math.pi), math.pi)

    def test_negative_two_pi(self):
        self.assertAlmostEqual(mod2pi(-2.0 * math.pi), 0.0)

    def test_three_pi(self):
        self.assertAlmostEqual(mod2pi(3.0 * math.pi), math.pi)

    def test_large_value(self):
        self.assertAlmostEqual(mod2pi(10.0 * math.pi), 0.0)


class TestSixCombinations(unittest.TestCase):
    """6 种基本组合公式。"""

    def test_all_combinations_callable(self):
        """所有 6 种类型都应可调用且不抛异常。"""
        start = (0.0, 0.0, 0.0)
        end = (10.0, 5.0, math.pi / 2.0)
        R = 2.0
        for pt in DUBINS_TYPES:
            res = _compute_path(start, end, R, pt)
            # 允许返回 None（数值退化）或非负三元组
            if res is not None:
                t, p, q = res
                self.assertGreaterEqual(t, 0.0)
                self.assertGreaterEqual(p, 0.0)
                self.assertGreaterEqual(q, 0.0)


class TestShortestPath(unittest.TestCase):
    """最短路径选取。"""

    def setUp(self):
        self.start = (0.3, 0.4, 1.5)
        self.end = (4.0, 3.0, 2.5)
        self.radius = 1.5

    def test_returns_valid_type(self):
        best_type, _, _ = shortest_path(self.start, self.end, self.radius)
        self.assertIn(best_type, DUBINS_TYPES)

    def test_length_positive(self):
        _, _, length = shortest_path(self.start, self.end, self.radius)
        self.assertGreater(length, 0.0)

    def test_shortest_le_candidates(self):
        """最短路径长度应 ≤ 任一候选长度。"""
        best_type, _, best_len = shortest_path(self.start, self.end, self.radius)
        all_res = all_paths(self.start, self.end, self.radius)
        for name, _, length in all_res:
            self.assertLessEqual(best_len, length + 1e-9,
                                 f"shortest > {name}")

    def test_radius_doubles_length(self):
        """位姿与 R 同时缩放 2x，路径长度应翻倍。"""
        _, _, len1 = shortest_path(self.start, self.end, self.radius)
        s2 = (2.0 * self.start[0], 2.0 * self.start[1], self.start[1] - self.start[1] + self.start[0] - self.start[0] + self.start[2])
        e2 = (2.0 * self.end[0], 2.0 * self.end[1], self.end[2])
        _, _, len2 = shortest_path(s2, e2, 2.0 * self.radius)
        self.assertAlmostEqual(len2, 2.0 * len1, places=3)

    def test_invalid_radius_raises(self):
        with self.assertRaises(ValueError):
            shortest_path(self.start, self.end, 0.0)
        with self.assertRaises(ValueError):
            shortest_path(self.start, self.end, -1.0)


class TestEndpointAlignment(unittest.TestCase):
    """路径起末位姿对齐：sample_path 终点应接近 end。"""

    def setUp(self):
        self.start = (0.3, 0.4, 1.5)
        self.end = (4.0, 3.0, 2.5)
        self.radius = 1.5

    def test_start_alignment(self):
        best_type, tpq, _ = shortest_path(self.start, self.end, self.radius)
        samples = sample_path(self.start, self.radius, tpq, best_type, n=50)
        self.assertAlmostEqual(samples[0][0], self.start[0], places=4)
        self.assertAlmostEqual(samples[0][1], self.start[1], places=4)
        self.assertTrue(yaw_close(samples[0][2], self.start[2]))

    def test_end_alignment(self):
        """路径终点位姿应等于 end（在 1e-2 容差内）。"""
        best_type, tpq, _ = shortest_path(self.start, self.end, self.radius)
        samples = sample_path(self.start, self.radius, tpq, best_type, n=50)
        self.assertAlmostEqual(samples[-1][0], self.end[0], places=2)
        self.assertAlmostEqual(samples[-1][1], self.end[1], places=2)
        self.assertTrue(yaw_close(samples[-1][2], self.end[2], tol=1e-3))


class TestSymmetry(unittest.TestCase):
    """起末路径端点对齐性（镜像时间反演对称的间接验证）。"""

    def test_endpoints_align_both_directions(self):
        """start->end 与 end->start 的最短路径端点都应对齐。"""
        start = (0.0, 0.0, 0.6)
        end = (5.0, 5.0, math.pi + 0.3)
        radius = 1.5
        # start -> end
        b1, t1, _ = shortest_path(start, end, radius)
        ep1 = _endpoint(start, radius, t1, b1)
        self.assertAlmostEqual(ep1[0], end[0], places=2)
        self.assertAlmostEqual(ep1[1], end[1], places=2)
        # end -> start
        b2, t2, _ = shortest_path(end, start, radius)
        ep2 = _endpoint(end, radius, t2, b2)
        self.assertAlmostEqual(ep2[0], start[0], places=2)
        self.assertAlmostEqual(ep2[1], start[1], places=2)


class TestSampling(unittest.TestCase):
    """采样函数。"""

    def test_sample_count(self):
        start = (0.0, 0.0, 0.0)
        end = (8.0, 4.0, 1.0)
        radius = 1.5
        best_type, tpq, _ = shortest_path(start, end, radius)
        samples = sample_path(start, radius, tpq, best_type, n=30)
        self.assertEqual(len(samples), 30)

    def test_sample_single_point(self):
        start = (0.0, 0.0, 0.0)
        end = (8.0, 4.0, 1.0)
        radius = 1.5
        best_type, tpq, _ = shortest_path(start, end, radius)
        samples = sample_path(start, radius, tpq, best_type, n=1)
        self.assertEqual(len(samples), 1)


class TestAdvance(unittest.TestCase):
    """_advance：沿段前进 length 距离后的位姿。"""

    def test_straight(self):
        nx, ny, new_yaw = _advance(0.0, 0.0, 0.0, radius=10.0, seg="S", length=5.0)
        self.assertAlmostEqual(nx, 5.0)
        self.assertAlmostEqual(ny, 0.0)
        self.assertTrue(yaw_close(new_yaw, 0.0))

    def test_straight_with_yaw(self):
        nx, ny, new_yaw = _advance(0.0, 0.0, math.pi / 2.0,
                                      radius=10.0, seg="S", length=5.0)
        self.assertAlmostEqual(nx, 0.0, places=5)
        self.assertAlmostEqual(ny, 5.0, places=2)
        self.assertTrue(yaw_close(new_yaw, math.pi / 2.0))

    def test_left_turn_yaw_increases(self):
        """左转：yaw 应增加 length/R。"""
        R, L = 10.0, 5.0
        _, _, new_yaw = _advance(0.0, 0.0, 0.0, radius=R, seg="L", length=L)
        self.assertTrue(yaw_close(new_yaw, L / R, tol=1e-5))

    def test_right_turn_yaw_decreases(self):
        """右转：yaw 应减少 length/R（mod2pi 后为 2π - L/R）。"""
        R, L = 10.0, 5.0
        _, _, new_yaw = _advance(0.0, 0.0, 0.0, radius=R, seg="R", length=L)
        self.assertTrue(yaw_close(new_yaw, 0.0 - L / R, tol=1e-5))

    def test_zero_length_unchanged(self):
        nx, ny, new_yaw = _advance(3.0, 4.0, 1.0,
                                   radius=10.0, seg="L", length=0.0)
        self.assertEqual((nx, ny, new_yaw), (3.0, 4.0, 1.0))

    def test_left_turn_full_circle(self):
        """左转走完整圆（2πR）：位置回到起点，yaw 回到原值。"""
        R = 3.0
        yaw = 0.7
        nx, ny, new_yaw = _advance(5.0, 5.0, yaw,
                                   radius=R, seg="L", length=2.0 * math.pi * R)
        self.assertAlmostEqual(nx, 5.0, places=2)
        self.assertAlmostEqual(ny, 5.0, places=2)
        self.assertTrue(yaw_close(new_yaw, yaw))


if __name__ == "__main__":
    unittest.main()