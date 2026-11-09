"""Offline tests for raw terminal exit keys; ROS and terminal are mocked."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/keyboard_control.py'

class ExitKeyTests(unittest.TestCase):
    def setUp(self):
        ros = types.ModuleType('rospy')
        ros.is_shutdown = Mock(return_value=False)
        geometry = types.ModuleType('geometry_msgs')
        messages = types.ModuleType('geometry_msgs.msg')
        messages.Twist = Mock()
        with patch.dict(sys.modules, {'rospy': ros, 'geometry_msgs': geometry,
                                      'geometry_msgs.msg': messages}):
            spec = importlib.util.spec_from_file_location('keyboard_under_test', SCRIPT)
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.node = self.module.KeyboardController.__new__(self.module.KeyboardController)
        self.node.pub = Mock()
        self.node.rate = Mock()
        self.node.step_once = Mock(return_value='command')

    def spin_keys(self, keys):
        context = Mock()
        context.__enter__ = Mock(return_value=context)
        context.__exit__ = Mock(return_value=False)
        context.get_key = Mock(side_effect=list(keys) + [AssertionError('Loop did not exit')])
        with patch.object(self.module, 'RawKeyReader', return_value=context), patch('builtins.print'):
            self.node.spin()
        context.__exit__.assert_called_once()
        return context

    def test_ctrl_c_exits_without_publishing(self):
        reader = self.spin_keys(['\x03'])
        self.assertEqual(reader.get_key.call_count, 1)
        self.node.pub.publish.assert_not_called()
        self.node.rate.sleep.assert_not_called()

    def test_escape_still_exits(self):
        self.spin_keys(['\x1b'])
        self.node.pub.publish.assert_not_called()

    def test_movement_then_ctrl_c(self):
        self.spin_keys(['w', '\x03'])
        self.node.step_once.assert_called_once_with('w')
        self.node.pub.publish.assert_called_once_with('command')
        self.node.rate.sleep.assert_called_once()

    def test_timeout_then_ctrl_c(self):
        self.spin_keys([None, '\x03'])
        self.node.step_once.assert_called_once_with(None)
        self.node.pub.publish.assert_called_once_with('command')

if __name__ == '__main__':
    unittest.main()
