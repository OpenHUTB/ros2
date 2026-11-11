"""Offline tests for raw terminal exit keys; ROS and terminal are mocked.

覆盖：
  * Ctrl-C / ESC 退出时不再下发运动指令，并补发一条零速（避免无人机保持最后速度）
  * 方向键（ESC [ A 等转义序列）不再被当成 ESC，不会误退出
  * 正常按键仍然只发布一条指令
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/keyboard_control.py'


class _Vec3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist(object):
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


class ExitKeyTests(unittest.TestCase):
    def setUp(self):
        ros = types.ModuleType('rospy')
        ros.is_shutdown = Mock(return_value=False)
        ros.logwarn = Mock()
        ros.Subscriber = lambda *a, **k: None
        geometry = types.ModuleType('geometry_msgs')
        messages = types.ModuleType('geometry_msgs.msg')
        messages.Twist = _Twist
        nav = types.ModuleType('nav_msgs')
        nav_msg = types.ModuleType('nav_msgs.msg')
        nav_msg.Odometry = type('Odometry', (object,), {})
        with patch.dict(sys.modules, {'rospy': ros, 'geometry_msgs': geometry,
                                      'geometry_msgs.msg': messages,
                                      'nav_msgs': nav, 'nav_msgs.msg': nav_msg}):
            spec = importlib.util.spec_from_file_location('keyboard_under_test', SCRIPT)
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)

        self.published = []
        self.node = self.module.KeyboardController.__new__(self.module.KeyboardController)
        self.node.pub = Mock()
        self.node.pub.publish = Mock(side_effect=self.published.append)
        self.node.rate = Mock()
        self.node.tracker = self.module.KeyTracker()
        self.node.smoother = self.module.VelocitySmoother()
        self.node.skills = self.module.MacroRunner()
        self.node.pose = None
        self.node.pose_time = 0.0
        self.node.step_once = Mock(return_value='command')
        self.context = None

    # ------------------------------------------------------------------ 工具
    def spin_keys(self, keys):
        context = Mock()
        context.__enter__ = Mock(return_value=context)
        context.__exit__ = Mock(return_value=False)
        context.get_key = Mock(side_effect=list(keys) + [AssertionError('Loop did not exit')])
        self.context = context
        with patch.object(self.module, 'RawKeyReader', return_value=context), patch('builtins.print'):
            self.node.spin()
        context.__exit__.assert_called_once()
        return context

    def assert_zero_cmd(self, msg, label='退出时应补发零速'):
        zeros = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        self.assertEqual(zeros, (0.0, 0.0, 0.0, 0.0), label)

    # ------------------------------------------------------------------ 用例
    def test_ctrl_c_exits_without_motion(self):
        reader = self.spin_keys(['\x03'])
        self.assertEqual(reader.get_key.call_count, 1)
        self.node.step_once.assert_not_called()
        self.node.rate.sleep.assert_not_called()
        self.assertEqual(len(self.published), 1, '退出时只应补发一条零速')
        self.assert_zero_cmd(self.published[0])

    def test_escape_still_exits(self):
        self.spin_keys(['\x1b', None])
        self.node.step_once.assert_not_called()
        self.assertEqual(len(self.published), 1)
        self.assert_zero_cmd(self.published[0])

    def test_arrow_key_sequence_is_not_exit(self):
        reader = self.spin_keys(['\x1b', '[', 'A', '\x03'])
        self.assertEqual(reader.get_key.call_count, 4, '转义序列应被完整吃掉')
        self.node.step_once.assert_not_called()
        self.assertEqual(len(self.published), 1)
        self.assert_zero_cmd(self.published[0])

    def test_movement_then_ctrl_c(self):
        self.spin_keys(['w', '\x03'])
        self.node.step_once.assert_called_once_with('w', now=unittest.mock.ANY)
        self.node.rate.sleep.assert_called_once()
        self.assertEqual(len(self.published), 2, '一条运动指令 + 一条退出零速')
        self.assertEqual(self.published[0], 'command')
        self.assert_zero_cmd(self.published[1])

    def test_timeout_then_ctrl_c(self):
        self.spin_keys([None, '\x03'])
        self.node.step_once.assert_called_once_with(None, now=unittest.mock.ANY)
        self.assertEqual(self.published[0], 'command')
        self.assert_zero_cmd(self.published[1])


if __name__ == '__main__':
    unittest.main()
