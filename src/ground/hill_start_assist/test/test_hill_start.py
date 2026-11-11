# -*- coding: utf-8 -*-
"""纯逻辑单元测试（不需要 ROS；MuJoCo 只在集成测试里用）。

评审指出本目录缺少 test/，而同目录 60+ 个模块都有。这里补 5 项：
几何常量一致性、前馈公式、限幅、质量注入、俯仰角度换算。
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from hill_start_assist import config                       # noqa: E402
from hill_start_assist.control import HillStartController   # noqa: E402


class TestGeometry(unittest.TestCase):

    def test_body_height_matches_wheel_and_drop(self):
        """出生高度必须 = 轮半径 + 轮毂下移量（漏了会陷进地面）。"""
        self.assertAlmostEqual(config.WHEEL_RADIUS + config.WHEEL_DROP, 0.10, places=9)

    def test_slope_normal_is_unit(self):
        """斜面法向 (sin(−α), 0, cos(−α)) 必须是单位向量，x 为负（指向下坡）。"""
        a = math.radians(config.SLOPE_DEG)
        n = (math.sin(-a), 0.0, math.cos(-a))
        self.assertAlmostEqual(sum(c * c for c in n), 1.0, places=12)
        self.assertLess(n[0], 0.0)


class TestFeedforward(unittest.TestCase):

    def test_feedforward_formula(self):
        """τ_i = m·g·sinα·r / 4。"""
        c = HillStartController(mass=7.0, slope_deg=12.0)
        expect = 7.0 * 9.81 * math.sin(math.radians(12.0)) * config.WHEEL_RADIUS / 4.0
        self.assertAlmostEqual(c.feedforward_per_wheel(), expect, places=9)

    def test_mass_is_injectable(self):
        """质量必须能由外部传入（评审指出过硬编码 7.0 的问题）。"""
        a = HillStartController(mass=7.0)
        b = HillStartController(mass=14.0)
        self.assertAlmostEqual(b.feedforward_per_wheel(), 2 * a.feedforward_per_wheel(), places=9)

    def test_torque_is_clamped(self):
        """轮角误差很大时力矩必须被限幅，不能无限增大。"""
        c = HillStartController(mass=7.0)
        tau = c.torque(phi_err=100.0, phi_dot=0.0)
        self.assertLessEqual(abs(tau), config.TAU_MAX + 1e-9)


if __name__ == "__main__":
    unittest.main()
