# -*- coding: utf-8 -*-
"""标定算法的纯逻辑单元测试（不需要 roscore / rospy）。

运行：
    python3 -m pytest src/ground/diff_drive_odom_calib/test -q
    # 或（无 pytest 时）
    python3 -m unittest discover -s src/ground/diff_drive_odom_calib/test -v
"""

import math
import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from diff_drive_odom_calib import calibration as calib          # noqa: E402
from diff_drive_odom_calib.model import (DiffDrive, OdometryIntegrator,   # noqa: E402
                                         pose_difference, wrap_angle)

DT = 0.02
NOMINAL = dict(c_s=1.0, c_theta=1.0, k_head=0.0)


def make_robot(slip=0.0, seed=0):
    return DiffDrive(r_left=0.0505, r_right=0.0495, b_true=0.306,
                     slip_std=slip, seed=seed)


class TestKinematics(unittest.TestCase):

    def test_nominal_robot_has_no_drift(self):
        """理想机器人（r 相等、b 与标称一致）：里程计应与真值完全重合。"""
        robot = DiffDrive(r_left=0.05, r_right=0.05, b_true=0.30, slip_std=0.0)
        cmd = calib.square_cmd(side=2.0, v=0.2, w=1.0, turns=1)
        r = calib.evaluate(robot, NOMINAL, DT, cmd)
        self.assertLess(r["err_pos"], 1e-9)
        self.assertLess(abs(r["err_theta"]), 1e-9)

    def test_unequal_wheels_cause_heading_drift(self):
        """左轮比右轮大 → 直行时向右偏（航向单调减小）。"""
        robot = make_robot()
        log, true_traj, _ = calib.straight_log(robot, DT, distance=2.0, v=0.2)
        dth = sum(o["dth_true"] for o in log)
        self.assertLess(dth, 0.0, "左轮大应向右偏，实测 Δθ=%.4f" % dth)
        # 理论值：Δθ = (r_r - r_l) / (r_nom · b_true) · L
        expect = (0.0495 - 0.0505) / (0.05 * 0.306) * 2.0
        self.assertAlmostEqual(dth, expect, places=3)

    def test_wrong_track_scales_rotation(self):
        """轮距报大 → 里程计以为转得少，实际转得多（比例 = b_nom / b_true）。"""
        robot = DiffDrive(r_left=0.05, r_right=0.05, b_true=0.33, slip_std=0.0)
        log, _, _ = calib.rotation_log(robot, DT, turns=1.0, w=1.0)
        th_true = sum(o["dth_true"] for o in log)
        th_odom = sum(0.05 * (o["dphi_right"] - o["dphi_left"]) / 0.30 for o in log)
        self.assertAlmostEqual(th_true / th_odom, 0.30 / 0.33, places=6)


class TestIdentify(unittest.TestCase):

    def test_recovers_parameters_without_slip(self):
        robot = make_robot(slip=0.0)
        prm = calib.identify(calib.straight_log(robot, DT, 2.0, 0.2),
                             calib.rotation_log(robot, DT, 3.0, 1.0))
        self.assertAlmostEqual(prm["c_s"], 1.0, places=4)
        self.assertAlmostEqual(prm["c_theta"], 0.30 / 0.306, places=4)
        self.assertAlmostEqual(prm["k_head"], (0.0495 - 0.0505) / (0.05 * 0.306), places=4)

    def test_identify_degrades_gracefully_with_slip(self):
        """有打滑噪声时参数仍有 1% 以内的精度（打滑是非系统误差，进入随机项）。"""
        robot = make_robot(slip=0.01, seed=7)
        prm = calib.identify(calib.straight_log(robot, DT, 2.0, 0.2),
                             calib.rotation_log(robot, DT, 3.0, 1.0))
        self.assertLess(abs(prm["c_theta"] - 0.30 / 0.306), 0.02)
        self.assertLess(abs(prm["c_s"] - 1.0), 0.02)

    def test_calibration_reduces_square_drift(self):
        """核心断言：方形路径上的终点误差必须显著下降。"""
        robot = make_robot(slip=0.004, seed=1)
        prm = calib.identify(calib.straight_log(robot, DT, 2.0, 0.2),
                             calib.rotation_log(robot, DT, 3.0, 1.0))
        sq = calib.square_cmd(side=4.0, v=0.2, w=1.0, turns=4)
        before = calib.evaluate(robot, NOMINAL, DT, sq)
        after = calib.evaluate(robot, prm, DT, sq)
        self.assertGreater(before["err_pos"], 1.0, "标定前应当有米级漂移")
        self.assertLess(after["err_pos"], 0.10, "标定后应降到 10 cm 以内")
        self.assertLess(abs(after["err_theta"]), math.radians(2.0))


class TestRepeatability(unittest.TestCase):

    def test_nonsystematic_error_remains(self):
        """标定消不掉打滑造成的散布：重复实验的标准差应大于 0。"""
        robot = make_robot(slip=0.01, seed=0)
        prm = calib.identify(calib.straight_log(robot, DT, 2.0, 0.2),
                             calib.rotation_log(robot, DT, 3.0, 1.0))
        sq = calib.square_cmd(side=2.0, v=0.2, w=1.0, turns=2)
        rep = calib.repeat_test(robot, prm, DT, sq, n=6)
        self.assertGreater(rep["std_pos"], 0.0)
        self.assertLess(rep["mean_pos"], 0.15)


class TestUtils(unittest.TestCase):

    def test_wrap_angle(self):
        self.assertAlmostEqual(abs(wrap_angle(3 * math.pi)), math.pi, places=9)
        self.assertAlmostEqual(wrap_angle(0.5), 0.5, places=12)

    def test_pose_difference_is_rotation_invariant(self):
        """两个位姿整体旋转后，误差在各自航向系下的分量应不变。"""
        est, true = (1.0, 2.0, 0.3), (0.5, 1.5, 0.1)
        d1 = pose_difference(est, true)
        th = 0.7
        c, s = math.cos(th), math.sin(th)
        rot = lambda p: (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2] + th)
        d2 = pose_difference(rot(est), rot(true))
        for a, b in zip(d1, d2):
            self.assertAlmostEqual(a, b, places=9)


if __name__ == "__main__":
    unittest.main()
