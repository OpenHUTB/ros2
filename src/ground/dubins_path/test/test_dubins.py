# -*- coding: utf-8 -*-
"""Dubins 曲线算法离线单元测试。

不依赖 pygame，纯算法层验证。
"""

import math
import os
import sys
import unittest

# 直接 `python test/test_dubins.py` 时也能 import 到上级包
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

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
        """位姿坐标与 R 同时缩放 2x，路径长度应翻倍（yaw 不缩放）。"""
        _, _, len1 = shortest_path(self.start, self.end, self.radius)
        s2 = (2.0 * self.start[0], 2.0 * self.start[1], self.start[2])
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


class TestPackaging(unittest.TestCase):
    """打包配置自检：setup.py 引用的文件必须真实存在。

    回归用例：setup.py 的 data_files 曾引用不存在的 requirements.txt，
    colcon build 会直接报错；而 share/dubins_path 又写了三条。
    """

    def test_data_files_referenced_exist(self):
        pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        expected = [
            "package.xml", "README.md", "requirements.txt",
            "main.py", "main.sh", "main.bat",
            os.path.join("resource", "dubins_path"),
            os.path.join("config", "params.yaml"),
            os.path.join("launch", "main.launch.py"),
            os.path.join("dubins_path", "main.py"),
        ]
        for rel in expected:
            self.assertTrue(os.path.exists(os.path.join(pkg, rel)), rel)

    def test_setup_py_share_dir_listed_once(self):
        pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        src = open(os.path.join(pkg, "setup.py"), encoding="utf-8").read()
        self.assertEqual(src.count('("share/" + package_name,'), 1,
                         "share/<pkg> 目录应只出现一次")


class TestParamsLoading(unittest.TestCase):
    """config/params.yaml 必须真的被读取（而不是配了不生效）。"""

    def test_shipped_params_keys_effective(self):
        from dubins_path import config as C

        path = os.path.join(os.path.dirname(__file__), "..", "config", "params.yaml")
        self.assertTrue(os.path.isfile(path), path)

        old = (C.WIDTH, C.HEIGHT, C.FPS, C.DEFAULT_START, C.DEFAULT_END,
               C.DEFAULT_RADIUS, C.CURVE_SAMPLES, C.ANIM_STEPS)
        try:
            used = C.load_params(path)
            self.assertEqual(used, path)
            self.assertEqual(C.WIDTH, 900)
            self.assertEqual(C.HEIGHT, 600)
            self.assertEqual(C.FPS, 60)
            self.assertEqual(len(C.DEFAULT_START), 3)
            self.assertEqual(len(C.DEFAULT_END), 3)
            self.assertAlmostEqual(C.DEFAULT_RADIUS, 80.0)
            self.assertEqual(C.CURVE_SAMPLES, 200)
            self.assertEqual(C.ANIM_STEPS, 120)
        finally:
            (C.WIDTH, C.HEIGHT, C.FPS, C.DEFAULT_START, C.DEFAULT_END,
             C.DEFAULT_RADIUS, C.CURVE_SAMPLES, C.ANIM_STEPS) = old


# ---------------------------------------------------------------------------
# 算法正确性对照（评审意见：几何实现必须与独立方法对齐）
# ---------------------------------------------------------------------------
def _mod2pi(t):
    return t - 2.0 * math.pi * math.floor(t / (2.0 * math.pi))


def _closed_form(start, end, R):
    """独立的闭式实现（Shkel/Lumelsky 归一化公式），仅用于交叉校验。

    说明：闭式的 LRL/RLR 只给出「中间弧 >= π」那一支；几何实现会把两个
    中间圆都算出来取较短者，所以 CCC 只需满足「不长于闭式那一支」。
    返回 {类型: (t, p, q)}，长度已乘 R。
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    D = math.hypot(dx, dy)
    if D < 1e-12 and abs(_mod2pi(end[2] - start[2] + math.pi) - math.pi) < 1e-12:
        return {k: (0.0, 0.0, 0.0) for k in ("LSL", "RSR", "LSR", "RSL", "LRL", "RLR")}

    d = D / R
    theta = math.atan2(dy, dx) if D > 1e-12 else 0.0
    alpha = _mod2pi(start[2] - theta)
    beta = _mod2pi(end[2] - theta)
    ca, sa, cb, sb = math.cos(alpha), math.sin(alpha), math.cos(beta), math.sin(beta)
    cab = math.cos(alpha - beta)
    out = {}

    psq = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if psq >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out["LSL"] = (_mod2pi(-alpha + tmp), math.sqrt(psq), _mod2pi(beta - tmp))

    psq = 2 + d * d - 2 * cab + 2 * d * (-sa + sb)
    if psq >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out["RSR"] = (_mod2pi(alpha - tmp), math.sqrt(psq), _mod2pi(-beta + tmp))

    psq = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if psq >= 0:
        p = math.sqrt(psq)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out["LSR"] = (_mod2pi(-alpha + tmp), p, _mod2pi(-_mod2pi(beta) + tmp))

    psq = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
    if psq >= 0:
        p = math.sqrt(psq)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out["RSL"] = (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))

    tmp = (6.0 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0:
        phi = math.atan2(ca - cb, d + sa - sb)
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha - phi + p / 2.0)
        out["LRL"] = (t, p, _mod2pi(_mod2pi(beta) - alpha - t + p))

    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        phi = math.atan2(ca - cb, d - sa + sb)
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - phi + p / 2.0)
        out["RLR"] = (t, p, _mod2pi(alpha - beta - t + p))

    return {k: (t * R, p * R, q * R) for k, (t, p, q) in out.items()}


def _pose_gap(start, tpq, ptype, R, end):
    ep = _endpoint(start, R, tpq, ptype)
    return (math.hypot(ep[0] - end[0], ep[1] - end[1]),
            abs(_mod2pi(ep[2] - end[2] + math.pi) - math.pi))


class TestRslRegression(unittest.TestCase):
    """RSL 曾经因为内公切线方向取错而完全缺失。"""

    def test_rsl_present_and_shortest(self):
        start, end, R = (0.0, 0.0, 0.0), (6.0, 0.0, math.pi / 2), 2.0
        names = [t for t, _, _ in all_paths(start, end, R)]
        self.assertIn("RSL", names)
        best_type, tpq, length = shortest_path(start, end, R)
        self.assertEqual(best_type, "RSL")
        # 与闭式结果一致：t=1.2870, p=2.0000, q=4.4286, 总长 7.7156
        self.assertAlmostEqual(tpq[0], 1.2870, places=4)
        self.assertAlmostEqual(tpq[1], 2.0000, places=4)
        self.assertAlmostEqual(tpq[2], 4.4286, places=4)
        self.assertAlmostEqual(length, 7.7156, places=4)

    def test_all_six_types_can_appear(self):
        """随机扫一遍，六种类型都必须至少出现一次（防 RSL/CCC 再次丢失）。"""
        import random
        rng = random.Random(11)
        seen = set()
        for _ in range(400):
            start = (rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(0, 6.28))
            end = (rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(0, 6.28))
            R = rng.uniform(2.0, 8.0)
            for t, _, _ in all_paths(start, end, R):
                seen.add(t)
            if len(seen) == 6:
                break
        self.assertEqual(seen, set(DUBINS_TYPES), f"缺少: {set(DUBINS_TYPES) - seen}")


class TestCccBranch(unittest.TestCase):
    """CCC 的两个中间圆都有效，必须取较短者（原来直接返回第一个）。"""

    def test_picks_shorter_rlr(self):
        # 评审给出的用例：合法 RLR 总长 11.5556，原本返回 26.1435 并让 LRL 胜出
        start, end, R = (0.0, 0.0, 0.0), (2.0, 2.0, math.pi), 2.0
        rlr = _compute_path(start, end, R, "RLR")
        lrl = _compute_path(start, end, R, "LRL")
        self.assertAlmostEqual(sum(rlr), 11.5556, places=4)
        self.assertAlmostEqual(rlr[1], 8.9194, places=4)
        self.assertGreater(sum(lrl), sum(rlr))
        best_type, best_tpq, best_len = shortest_path(start, end, R)
        self.assertEqual(best_type, "RLR")
        self.assertAlmostEqual(best_len, 11.5556, places=4)

    def test_ccc_endpoint_aligns(self):
        for start, end, R in (((0.0, 0.0, 0.0), (6.0, 0.0, math.pi / 2), 2.0),
                              ((0.0, 0.0, 0.0), (2.0, 2.0, math.pi), 2.0),
                              ((3.0, -4.0, 1.2), (-5.0, 6.0, 4.4), 3.0)):
            for ptype in ("LRL", "RLR"):
                tpq = _compute_path(start, end, R, ptype)
                if tpq is None:
                    continue
                dp, dyaw = _pose_gap(start, tpq, ptype, R, end)
                self.assertLess(dp, 1e-6, f"{ptype} 位置不闭合")
                self.assertLess(dyaw, 1e-6, f"{ptype} 朝向不闭合")


class TestZeroLength(unittest.TestCase):
    """起末位姿完全相同时应返回零长度路径，而不是抛 ValueError。"""

    def test_same_pose_returns_zero(self):
        best_type, tpq, length = shortest_path((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0)
        self.assertIn(best_type, DUBINS_TYPES)
        self.assertEqual(tpq, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(length, 0.0)

    def test_same_pose_with_same_yaw_shifted_2pi(self):
        """朝向相差 2π 应视为同一位姿。"""
        _, _, length = shortest_path((1.0, 2.0, 0.0), (1.0, 2.0, 2 * math.pi), 3.0)
        self.assertAlmostEqual(length, 0.0)

    def test_short_sample_of_zero_path(self):
        samples = sample_path((0.0, 0.0, 0.0), 1.0, (0.0, 0.0, 0.0), "LSL", n=5)
        self.assertEqual(samples, [(0.0, 0.0, 0.0)])


class TestClosedFormReference(unittest.TestCase):
    """与独立闭式实现对照：CSC 逐类型相等、shortest 不劣于闭式最优、端点闭合。"""

    def setUp(self):
        import random
        self.rng = random.Random(2026)

    def _cases(self, n):
        for _ in range(n):
            yield ((self.rng.uniform(-30, 30), self.rng.uniform(-30, 30),
                    self.rng.uniform(0, 2 * math.pi)),
                   (self.rng.uniform(-30, 30), self.rng.uniform(-30, 30),
                    self.rng.uniform(0, 2 * math.pi)),
                   self.rng.uniform(2.0, 10.0))

    def test_csc_matches_closed_form(self):
        checked = 0
        for start, end, R in self._cases(120):
            ref = _closed_form(start, end, R)
            got = {t: tpq for t, tpq, _ in all_paths(start, end, R)}
            for ptype in ("LSL", "RSR", "LSR", "RSL"):
                if ptype in ref and ptype in got:
                    for a, b in zip(got[ptype], ref[ptype]):
                        self.assertAlmostEqual(a, b, places=6,
                                               msg=f"{ptype} {start}->{end} R={R}")
                    checked += 1
        self.assertGreater(checked, 200, "有效对照样本太少")

    def test_shortest_is_optimal_and_closed(self):
        for start, end, R in self._cases(150):
            ref = _closed_form(start, end, R)
            ref_best = min(sum(v) for v in ref.values())
            best_type, tpq, length = shortest_path(start, end, R)
            cand_min = min(sum(v) for _, v, _ in all_paths(start, end, R))

            # 与六种候选中的最小一致，且不劣于闭式最优，也不短于直线距离
            self.assertAlmostEqual(length, cand_min, places=9)
            self.assertLessEqual(length, ref_best + 1e-6)
            straight = math.hypot(end[0] - start[0], end[1] - start[1])
            self.assertGreaterEqual(length, straight - 1e-9)

            # 路径必须真的落在目标位姿上
            dp, dyaw = _pose_gap(start, tpq, best_type, R, end)
            self.assertLess(dp, 1e-6)
            self.assertLess(dyaw, 1e-6)


if __name__ == "__main__":
    unittest.main()