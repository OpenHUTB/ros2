"""uav_keyboard_control 纯逻辑本地测试（不依赖 ROS / 终端）。

跑法：python3 test/test_keyboard_local.py
"""
from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from uav_keyboard_control import logic as L


class TestKeyToCmd(unittest.TestCase):
    def setUp(self):
        self.s = L.Speeds(forward=2.0, side=0.5, vert=1.0, yaw=0.4)

    def test_forward(self):
        self.assertEqual(L.key_to_cmd("w", self.s), (2.0, 0.0, 0.0, 0.0))
        self.assertEqual(L.key_to_cmd("s", self.s), (-2.0, 0.0, 0.0, 0.0))

    def test_side(self):
        self.assertEqual(L.key_to_cmd("a", self.s), (0.0, 0.5, 0.0, 0.0))
        self.assertEqual(L.key_to_cmd("d", self.s), (0.0, -0.5, 0.0, 0.0))

    def test_vert(self):
        self.assertEqual(L.key_to_cmd("r", self.s), (0.0, 0.0, 1.0, 0.0))
        self.assertEqual(L.key_to_cmd("f", self.s), (0.0, 0.0, -1.0, 0.0))

    def test_yaw(self):
        self.assertEqual(L.key_to_cmd("q", self.s), (0.0, 0.0, 0.0, 0.4))
        self.assertEqual(L.key_to_cmd("e", self.s), (0.0, 0.0, 0.0, -0.4))

    def test_case_insensitive(self):
        self.assertEqual(L.key_to_cmd("W", self.s), L.key_to_cmd("w", self.s))

    def test_none_key_is_zero(self):
        self.assertEqual(L.key_to_cmd(None, self.s), (0.0, 0.0, 0.0, 0.0))

    def test_unknown_key_is_none(self):
        self.assertIsNone(L.key_to_cmd("x", self.s))
        self.assertIsNone(L.key_to_cmd("\x1b", self.s))  # ESC 转义序列
        self.assertIsNone(L.key_to_cmd("\x1b[A", self.s))  # 方向键


class TestKeyLatch(unittest.TestCase):
    def setUp(self):
        self.s = L.Speeds(forward=2.0, side=0.5, vert=1.0, yaw=0.4)
        self.l = L.KeyLatch(speeds=self.s, hold_timeout=0.3)

    def test_idle_is_hover(self):
        self.assertEqual(self.l.current(0.0), (0.0, 0.0, 0.0, 0.0))

    def test_hold_within_timeout(self):
        self.l.on_key("w", 0.0)
        self.assertEqual(self.l.current(0.1), (2.0, 0.0, 0.0, 0.0))
        self.assertEqual(self.l.current(0.3), (2.0, 0.0, 0.0, 0.0))  # 边界
        self.assertEqual(self.l.current(0.301), (0.0, 0.0, 0.0, 0.0))  # 超时悬停

    def test_repeated_key_refreshes_hold(self):
        self.l.on_key("w", 0.0)
        self.l.on_key("w", 0.25)          # 还在按住，刷新时间
        self.assertEqual(self.l.current(0.5), (2.0, 0.0, 0.0, 0.0))  # 0.5-0.25=0.25<0.3

    def test_new_key_overrides(self):
        self.l.on_key("w", 0.0)
        self.l.on_key("a", 0.1)
        self.assertEqual(self.l.current(0.15), (0.0, 0.5, 0.0, 0.0))

    def test_none_does_not_refresh(self):
        self.l.on_key("w", 0.0)
        self.l.on_key(None, 0.2)   # 没读到键，不刷新
        self.assertEqual(self.l.current(0.4), (0.0, 0.0, 0.0, 0.0))  # 0.4-0.0=0.4>0.3

    def test_unknown_key_does_not_refresh(self):
        self.l.on_key("w", 0.0)
        self.l.on_key("\x1b[A", 0.2)   # 方向键转义序列
        self.assertEqual(self.l.current(0.4), (0.0, 0.0, 0.0, 0.0))

    def test_exit_keys_do_not_refresh(self):
        self.l.on_key("w", 0.0)
        self.l.on_key("\x1b", 0.1)
        self.l.on_key("\x03", 0.2)
        self.assertEqual(self.l.current(0.5), (0.0, 0.0, 0.0, 0.0))


class TestPackageLayout(unittest.TestCase):
    """静态断言：包结构是 ament_python，不是 catkin。"""

    def _read(self, rel):
        root = os.path.join(os.path.dirname(__file__), "..")
        with open(os.path.join(root, rel), encoding="utf-8") as f:
            return f.read()

    def test_package_xml_is_ament_python(self):
        xml = self._read("package.xml")
        self.assertIn("ament_python", xml)
        self.assertNotIn("catkin", xml)
        self.assertNotIn("rospy", xml)

    def test_no_cmakelists(self):
        root = os.path.join(os.path.dirname(__file__), "..")
        self.assertFalse(os.path.exists(os.path.join(root, "CMakeLists.txt")))

    def test_launch_is_python(self):
        root = os.path.join(os.path.dirname(__file__), "..")
        files = os.listdir(os.path.join(root, "launch"))
        self.assertTrue(any(f.endswith(".launch.py") for f in files))
        self.assertFalse(any(f.endswith(".launch") for f in files))

    def test_node_uses_rclpy_not_rospy(self):
        src = self._read("uav_keyboard_control/keyboard_node.py")
        self.assertIn("import rclpy", src)
        self.assertNotIn("rospy", src)

    def test_yaml_param_names_match_node(self):
        yaml = self._read("config/keyboard.yaml")
        node = self._read("uav_keyboard_control/keyboard_node.py")
        for name in ("forward_speed", "side_speed", "vert_speed",
                     "yaw_rate", "hold_timeout", "publish_hz", "cmd_vel_topic"):
            self.assertIn(name, yaml, f"yaml missing {name}")
            self.assertIn(name, node, f"node missing {name}")
        # 旧的错误参数名不允许再出现
        self.assertNotIn("horiz_speed", yaml)
        self.assertNotIn("horiz_speed", node)


if __name__ == "__main__":
    unittest.main(verbosity=2)
