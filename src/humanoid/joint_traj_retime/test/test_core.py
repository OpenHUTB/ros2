# -*- coding: utf-8 -*-
"""关节轨迹重定时的单元测试（纯 numpy，不需要 ROS）。

    python -m unittest discover -s test -p "test_*.py" -v
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from joint_traj_retime.core import (Limits, build_trajectory, max_time_scale,  # noqa: E402
                                    peak_jerk, retime, split_duration, utilization)
from joint_traj_retime.examples import example  # noqa: E402


class TestSpline(unittest.TestCase):
    def setUp(self):
        self.tk = [0.0, 0.5, 1.0, 1.5, 2.0]
        self.qk = np.array([[0.0, 0.4, -0.2, 0.3, 0.0],
                            [0.1, 0.0, 0.5, 0.0, 0.1]])

    def test_passes_through_knots(self):
        """插值必须精确穿过关键帧。"""
        tr = build_trajectory(self.tk, self.qk, dt=0.001)
        for i, t in enumerate(self.tk):
            k = int(round((t - self.tk[0]) / 0.001))
            np.testing.assert_allclose(tr.q[:, k], self.qk[:, i], atol=1e-9)

    def test_second_derivative_continuous(self):
        """C² 连续：相邻采样点的 ddq 不应出现跳变（相对峰值 < 1%）。"""
        tr = build_trajectory(self.tk, self.qk, dt=0.001)
        jump = np.abs(np.diff(tr.ddq, axis=1)).max()
        self.assertLess(jump, 0.01 * max(np.abs(tr.ddq).max(), 1e-9))

    def test_velocity_matches_numeric_derivative(self):
        """解析一阶导与位置数值差分一致（相对误差 < 2%）。"""
        tr = build_trajectory(self.tk, self.qk, dt=0.001)
        num = np.gradient(tr.q, tr.t, axis=1)
        mid = slice(50, -50)
        err = np.abs(tr.dq[:, mid] - num[:, mid]).max()
        self.assertLess(err, 0.02 * max(np.abs(tr.dq).max(), 1e-9))

    def test_rejects_bad_shapes(self):
        with self.assertRaises(ValueError):
            build_trajectory([0.0, 1.0], np.zeros(5))
        with self.assertRaises(ValueError):
            build_trajectory([0.0, 1.0], np.zeros((2, 3)))


class TestTimestampValidation(unittest.TestCase):
    """review 指出：时间戳不严格递增时不能静默算出有限的 α。"""

    def test_non_monotonic_raises(self):
        tk = [0.0, 1.0, 0.5]
        qk = np.zeros((2, 3))
        with self.assertRaises(ValueError) as ctx:
            build_trajectory(tk, qk)
        self.assertIn("严格递增", str(ctx.exception))

    def test_duplicate_raises(self):
        tk = [0.0, 0.0, 1.0]
        qk = np.zeros((2, 3))
        with self.assertRaises(ValueError) as ctx:
            build_trajectory(tk, qk)
        self.assertIn("严格递增", str(ctx.exception))

    def test_message_names_the_bad_pair(self):
        with self.assertRaises(ValueError) as ctx:
            build_trajectory([0.0, 0.5, 0.4], np.zeros((1, 3)))
        self.assertIn("1→2", str(ctx.exception))


class TestDurationSplit(unittest.TestCase):
    """review 指出：nanosec 可能被舍入到 1e9，必须靠整数纳秒拆分。"""

    def test_round_up_to_whole_second(self):
        self.assertEqual(split_duration(3.9999999996), (4, 0))

    def test_half_second(self):
        self.assertEqual(split_duration(1.5), (1, 500000000))

    def test_zero(self):
        self.assertEqual(split_duration(0.0), (0, 0))

    def test_nanosec_always_in_range(self):
        for sec in (0.0, 0.9999999999, 1.9999999999, 2.0000000001, 59.9999999999):
            s, ns = split_duration(sec)
            self.assertGreaterEqual(ns, 0)
            self.assertLess(ns, 1_000_000_000)

    def test_negative_raises(self):
        with self.assertRaises(ValueError):
            split_duration(-0.1)

class TestScaling(unittest.TestCase):
    def setUp(self):
        self.tk, self.qk, self.v, self.a = example()
        self.traj = build_trajectory(self.tk, self.qk)
        self.limits = Limits(self.v, self.a)

    def test_respects_limits_exactly(self):
        """重定时后所有关节的占用率必须 ≤ 1。"""
        r = max_time_scale(self.traj, self.limits)
        fast = retime(self.traj, r["alpha"])
        u = utilization(fast, self.limits)
        self.assertLessEqual(max(u["speed"]), 1.0 + 1e-9)
        self.assertLessEqual(max(u["accel"]), 1.0 + 1e-9)

    def test_alpha_is_maximal(self):
        """再放 0.5% 就会超限，说明 α 是上确界而不是保守估计。"""
        r = max_time_scale(self.traj, self.limits)
        over = retime(self.traj, r["alpha"] * 1.005)
        u = utilization(over, self.limits)
        self.assertGreater(max(max(u["speed"]), max(u["accel"])), 1.0)

    def test_single_joint_analytic(self):
        """单关节正弦：v 约束下 α = v_max/(Aω)，a 约束下 α = sqrt(a_max/(Aω²))。"""
        A, T = 0.5, 2.0
        w = 2 * np.pi / T
        tk = [0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0]
        qk = np.array([[A * np.sin(w * t) for t in tk]])
        tr = build_trajectory(tk, qk)
        lim = Limits([1.0], [100.0])          # 只让速度约束起作用
        r = max_time_scale(tr, lim)
        expect = 1.0 / (A * w)
        self.assertAlmostEqual(r["alpha"], expect, delta=0.02 * expect)

        lim2 = Limits([100.0], [4.0])         # 只让加速度约束起作用
        r2 = max_time_scale(tr, lim2)
        expect2 = np.sqrt(4.0 / (A * w * w))
        self.assertAlmostEqual(r2["alpha"], expect2, delta=0.03 * expect2)

    def test_monotonic_in_limits(self):
        """上限放大一倍，α 应随之变大。"""
        base = max_time_scale(self.traj, self.limits)["alpha"]
        loose = Limits([2 * x for x in self.v], [2 * x for x in self.a])
        self.assertGreater(max_time_scale(self.traj, loose)["alpha"], base)

    def test_retime_preserves_shape(self):
        """位置曲线不变：q_α(t/α) == q(t)。"""
        alpha = 1.7
        fast = retime(self.traj, alpha)
        idx = np.arange(0, len(self.traj.t), 10)
        np.testing.assert_allclose(fast.q[:, ::10][:, :len(idx)], self.traj.q[:, idx],
                                   atol=1e-9)

    def test_duration_shrinks(self):
        r = max_time_scale(self.traj, self.limits)
        self.assertLess(r["duration_after"], r["duration_before"])
        self.assertAlmostEqual(r["duration_after"] * r["alpha"],
                               r["duration_before"], places=6)

    def test_reports_limiting_joint(self):
        r = max_time_scale(self.traj, self.limits)
        self.assertIn(r["limited_by"], ("speed", "accel"))
        self.assertGreaterEqual(r["joint"], 0)
        self.assertLess(r["joint"], len(self.v))

    def test_jerk_grows_after_retime(self):
        """压时间会放大抖动：jerk 应随 α³ 增长（这里只检查单调）。"""
        j0 = peak_jerk(self.traj)
        r = max_time_scale(self.traj, self.limits)
        j1 = peak_jerk(retime(self.traj, r["alpha"]))
        self.assertGreaterEqual(j1, j0 * 0.99)

    def test_static_trajectory_raises(self):
        tr = build_trajectory([0.0, 1.0], np.zeros((2, 2)))
        with self.assertRaises(ValueError):
            max_time_scale(tr, Limits([1.0, 1.0], [1.0, 1.0]))

    def test_mismatched_limits_raise(self):
        with self.assertRaises(ValueError):
            max_time_scale(self.traj, Limits([1.0], [1.0]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
