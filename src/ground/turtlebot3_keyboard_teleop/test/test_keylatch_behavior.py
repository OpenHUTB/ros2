#!/usr/bin/env python3
"""KeyLatch 状态机测试：超时归零、按键恢复、空格停车。"""
import os, sys, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from turtlebot3_keyboard_teleop.keyboard_logic import KeyLatch, Limits


class TestKeyLatch(unittest.TestCase):
    def test_timeout_continuous_zero(self):
        # 超时后持续发零速度，不会突然又动
        l = KeyLatch(limits=Limits(timeout_sec=0.5))
        l.press("w", now=100.0)
        # 超时前
        self.assertAlmostEqual(l.get_cmd(now=100.4)[0], 0.5)
        # 超时后连续多次都是零
        for t in (100.6, 101.0, 102.0, 105.0):
            vx, wz = l.get_cmd(now=t)
            self.assertAlmostEqual(vx, 0.0, msg=f"t={t} 应已归零")
            self.assertAlmostEqual(wz, 0.0)

    def test_press_after_timeout(self):
        # 超时后再按键，速度恢复
        l = KeyLatch(limits=Limits(timeout_sec=0.5))
        l.press("w", now=100.0)
        l.get_cmd(now=101.0)  # 已超时
        l.press("a", now=102.0)
        self.assertAlmostEqual(l.get_cmd(now=102.1)[1], 1.0)

    def test_space_immediate_stop(self):
        # 空格立即停车，不用等超时
        l = KeyLatch()
        l.press("w", now=100.0)
        l.press(" ", now=100.1)
        vx, wz = l.get_cmd(now=100.2)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(wz, 0.0)


if __name__ == "__main__":
    unittest.main()
