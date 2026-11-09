"""Dubins 路径规划单元测试。

覆盖：端点闭环精度、退化情形、左右镜像对称、曲率约束、半径缩放律、
六情形枚举与排序、非法输入。全部不依赖 ROS / numpy / matplotlib。
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

import dubins as db


def mirror(pose):
    """关于 x 轴镜像：左转变右转，路径长度应不变。"""
    return (pose[0], -pose[1], -pose[2])


def swap_lr(case):
    return case.translate(str.maketrans('LR', 'RL'))


class TestDegenerate(unittest.TestCase):
    def test_identical_poses_give_zero_length(self):
        p = db.dubins_path((1.0, 2.0, 0.7), (1.0, 2.0, 0.7), 1.0)
        self.assertAlmostEqual(p.length, 0.0, places=9)

    def test_straight_line_when_same_heading(self):
        p = db.dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 1.0)
        self.assertEqual(p.case, 'LSL')
        self.assertAlmostEqual(p.length, 10.0, places=9)
        self.assertAlmostEqual(p.segments[0][1], 0.0, places=9)
        self.assertAlmostEqual(p.segments[1][1], 10.0, places=9)

    def test_straight_line_along_any_heading(self):
        # 沿 30° 方向走 8 m，长度必须是 8，与航向无关
        th = math.radians(30.0)
        goal = (8.0 * math.cos(th), 8.0 * math.sin(th), th)
        p = db.dubins_path((0.0, 0.0, th), goal, 1.0)
        self.assertAlmostEqual(p.length, 8.0, places=9)


class TestEndpoint(unittest.TestCase):
    """核心断言：把三段弧长积分回去，必须精确落在目标位姿上。"""

    CASES = [
        ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (10.0, 0.0, math.pi / 2)),
        ((0.0, 0.0, 0.0), (10.0, 0.0, math.pi)),
        ((0.0, 0.0, 0.0), (6.0, 4.0, -math.pi / 2)),
        ((0.0, 0.0, math.pi / 2), (-3.0, 4.0, -math.pi / 3)),
        ((1.0, 2.0, 0.7), (6.5, -1.2, 2.4)),
        ((0.0, 0.0, 0.0), (0.5, 0.2, math.pi)),
        ((-4.0, 3.0, 2.9), (-4.2, 3.1, -2.9)),
    ]

    def test_endpoint_exact(self):
        for start, goal in self.CASES:
            p = db.dubins_path(start, goal, 1.0)
            dp, dth = p.endpoint_error(goal)
            self.assertLess(dp, 1e-9, 'endpoint pos err on %s -> %s' % (start, goal))
            self.assertLess(dth, 1e-9, 'endpoint yaw err on %s -> %s' % (start, goal))

    def test_pose_at_length_equals_goal(self):
        for start, goal in self.CASES:
            p = db.dubins_path(start, goal, 1.5)
            x, y, th, _ = p.pose_at(p.length)
            self.assertAlmostEqual(x, goal[0], places=8)
            self.assertAlmostEqual(y, goal[1], places=8)
            self.assertAlmostEqual(db.wrap_pi(th - goal[2]), 0.0, places=8)
            x0, y0, th0, _ = p.pose_at(0.0)
            self.assertAlmostEqual(x0, start[0], places=9)
            self.assertAlmostEqual(y0, start[1], places=9)

    def test_segment_lengths_sum_to_total(self):
        for start, goal in self.CASES:
            p = db.dubins_path(start, goal, 2.0)
            self.assertAlmostEqual(sum(s[1] for s in p.segments), p.length, places=12)
            for kind, seg_len in p.segments:
                self.assertGreaterEqual(seg_len, 0.0)


class TestSymmetry(unittest.TestCase):
    def test_mirror_preserves_length_and_swaps_case(self):
        start, goal = (0.0, 0.0, 0.0), (7.0, 1.5, 1.1)
        p = db.dubins_path(start, goal, 1.0)
        pm = db.dubins_path(mirror(start), mirror(goal), 1.0)
        self.assertAlmostEqual(p.length, pm.length, places=9)
        self.assertEqual(swap_lr(p.case), pm.case)

    def test_u_turn_lsr_equals_rsl(self):
        p = db.dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, math.pi), 1.0)
        q = db.dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, -math.pi), 1.0)
        self.assertAlmostEqual(p.length, q.length, places=9)
        self.assertIn(p.case, ('LSR', 'RSL'))


class TestCurvature(unittest.TestCase):
    def test_curvature_never_exceeds_limit(self):
        for radius in (0.5, 1.0, 3.0):
            p = db.dubins_path((0.0, 0.0, 0.0), (9.0, 2.0, math.pi), radius)
            limit = 1.0 / radius
            for _x, _y, _th, kappa in p.sample(step=0.02):
                self.assertLessEqual(abs(kappa), limit + 1e-12)

    def test_curvature_is_either_zero_or_boundary(self):
        # Dubins 路径只在「满舵」或「直线」两种曲率上取值，没有中间量
        p = db.dubins_path((0.0, 0.0, 0.0), (9.0, 2.0, math.pi), 2.0)
        values = set()
        for _x, _y, _th, kappa in p.sample(step=0.05):
            values.add(round(abs(kappa), 9))
        self.assertTrue(values.issubset({0.0, round(1.0 / 2.0, 9)}))

    def test_heading_change_matches_arc_length(self):
        # 圆弧段的航向变化量必须等于 kappa * 弧长
        p = db.dubins_path((0.0, 0.0, 0.0), (0.0, 4.0, math.pi), 1.0)
        pts = p.sample(step=0.005)
        total = 0.0
        for i in range(1, len(pts)):
            total += db.wrap_pi(pts[i][2] - pts[i - 1][2])
        self.assertAlmostEqual(abs(total), abs(db.wrap_pi(math.pi)), delta=1e-6)


class TestScaling(unittest.TestCase):
    def test_length_scales_with_radius(self):
        base = db.dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, math.pi), 1.0)
        for k in (0.5, 2.0, 4.0):
            p = db.dubins_path((0.0, 0.0, 0.0), (10.0 * k, 0.0, math.pi), k)
            self.assertAlmostEqual(p.length, base.length * k, places=8)

    def test_case_choice_is_scale_invariant(self):
        base = db.dubins_path((0.0, 0.0, 0.0), (10.0, 0.0, math.pi), 1.0)
        for k in (0.3, 2.5):
            p = db.dubins_path((0.0, 0.0, 0.0), (10.0 * k, 0.0, math.pi), k)
            self.assertEqual(p.case, base.case)


class TestEnumerate(unittest.TestCase):
    def test_paths_sorted_and_nonempty(self):
        paths = db.all_paths((0.0, 0.0, 0.0), (5.0, 5.0, math.pi / 2), 1.0)
        self.assertGreater(len(paths), 0)
        lengths = [p.length for p in paths]
        self.assertEqual(lengths, sorted(lengths))

    def test_shortest_is_first(self):
        paths = db.all_paths((0.0, 0.0, 0.0), (5.0, 5.0, math.pi / 2), 1.0)
        best = db.dubins_path((0.0, 0.0, 0.0), (5.0, 5.0, math.pi / 2), 1.0)
        self.assertAlmostEqual(best.length, paths[0].length, places=12)

    def test_all_cases_are_valid_names(self):
        for p in db.all_paths((0.0, 0.0, 0.0), (5.0, 5.0, math.pi / 2), 1.0):
            self.assertIn(p.case, ('LSL', 'LSR', 'RSL', 'RSR', 'RLR', 'LRL'))

    def test_short_detour_uses_three_arcs(self):
        # 位置几乎不动、只把车头掉过来，最优解必须是弧-弧-弧
        p = db.dubins_path((0.0, 0.0, 0.0), (0.3, 0.2, math.pi), 1.0)
        self.assertIn(p.case, ('RLR', 'LRL'))
        self.assertAlmostEqual(p.segments[1][0], p.case[1])


class TestInvalid(unittest.TestCase):
    def test_zero_or_negative_radius_rejected(self):
        for r in (0.0, -1.0):
            with self.assertRaises(ValueError):
                db.dubins_path((0, 0, 0), (1, 1, 0), r)

    def test_angle_helpers(self):
        self.assertAlmostEqual(db.mod2pi(-0.5), 2.0 * math.pi - 0.5, places=12)
        self.assertAlmostEqual(db.wrap_pi(3.0 * math.pi), math.pi, places=12)
        self.assertAlmostEqual(db.wrap_pi(-3.0 * math.pi), math.pi, places=12)
        for a in (-10.0, -math.pi, 0.0, math.pi, 7.0):
            w = db.wrap_pi(a)
            self.assertGreater(w, -math.pi - 1e-12)
            self.assertLessEqual(w, math.pi + 1e-12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
