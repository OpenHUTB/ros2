#!/usr/bin/env python3
import os, sys, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from turtlebot3_keyboard_teleop.keyboard_logic import key_to_cmd, Limits, KeyLatch


class TestKeyboardLogic(unittest.TestCase):
    def test_w(self): self.assertAlmostEqual(key_to_cmd("w")[0], 0.5)
    def test_s(self): self.assertAlmostEqual(key_to_cmd("s")[0], -0.5)
    def test_a(self): self.assertAlmostEqual(key_to_cmd("a")[1], 1.0)
    def test_d(self): self.assertAlmostEqual(key_to_cmd("d")[1], -1.0)
    def test_space(self): self.assertEqual(key_to_cmd(" "), (0.0, 0.0))
    def test_unknown(self): self.assertEqual(key_to_cmd("x"), (0.0, 0.0))
    def test_case(self): self.assertAlmostEqual(key_to_cmd("W")[0], 0.5)
    def test_custom_limits(self):
        lim = Limits(max_vx=1.0)
        self.assertAlmostEqual(key_to_cmd("w", lim)[0], 1.0)
    def test_empty(self): self.assertEqual(key_to_cmd(""), (0.0, 0.0))


class TestKeyLatch(unittest.TestCase):
    def test_press(self):
        l = KeyLatch(); l.press("w", now=100.0)
        self.assertAlmostEqual(l.get_cmd(now=100.1)[0], 0.5)

    def test_timeout(self):
        l = KeyLatch(limits=Limits(timeout_sec=0.5))
        l.press("w", now=100.0)
        self.assertAlmostEqual(l.get_cmd(now=100.3)[0], 0.5)
        self.assertAlmostEqual(l.get_cmd(now=100.6)[0], 0.0)

    def test_repress(self):
        l = KeyLatch(limits=Limits(timeout_sec=0.5))
        l.press("w", now=100.0)
        l.press("a", now=100.4)
        self.assertAlmostEqual(l.get_cmd(now=100.6)[1], 1.0)
        self.assertAlmostEqual(l.get_cmd(now=101.0)[1], 0.0)

    def test_init_zero(self):
        l = KeyLatch()
        self.assertEqual(l.get_cmd(now=0.0), (0.0, 0.0))


    def test_press_zero_time(self):
        # 回归：last_time=0.0 哨兵曾导致超时永久失效
        l = KeyLatch()
        l.press("w", now=0.0)
        self.assertAlmostEqual(l.get_cmd(now=0.1)[0], 0.5)
        self.assertAlmostEqual(l.get_cmd(now=1.0)[0], 0.0)

    def test_sentinel_none(self):
        # last_time=None 时不应误判超时
        l = KeyLatch()
        self.assertIsNone(l.last_time)
        self.assertEqual(l.get_cmd(now=12345.0), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
