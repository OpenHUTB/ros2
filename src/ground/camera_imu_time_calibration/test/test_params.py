#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""参数读取辅助的单元测试（不需要 ROS / rclpy）。

用一个假节点模拟 rclpy 的 ``declare_parameter`` / ``get_parameter`` 语义：
不传覆盖值时声明出来的是 NOT_SET（取出 None），传了覆盖值就按覆盖值的类型存。

    cd src/ground/camera_imu_time_calibration
    python3 -m unittest discover -s test -v
"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera_imu_time_calibration import ros_params  # noqa: E402


class FakeNode:
    """最小可用的 rclpy Node 替身，只实现被用到的两个方法。"""

    def __init__(self, overrides=None):
        self.overrides = dict(overrides or {})
        self.declared = {}

    def declare_parameter(self, name, value=None, descriptor=None):
        # 与 rclpy 一致：有覆盖值就用覆盖值（类型由覆盖值决定），否则用默认值
        self.declared[name] = self.overrides.get(name, value)
        return SimpleNamespace(name=name, value=self.declared[name])

    def get_parameter(self, name):
        return SimpleNamespace(name=name, value=self.declared[name])


class TestNumberParam(unittest.TestCase):
    def test_integer_override_becomes_float(self):
        """命令行写 -p offset_ms:=120（int）不应报类型错误，而是变成 120.0。"""
        node = FakeNode({'offset_ms': 120})
        self.assertEqual(ros_params.number_param(node, 'offset_ms', 50.0), 120.0)
        self.assertIsInstance(
            ros_params.number_param(node, 'offset_ms', 50.0), float)

    def test_float_override(self):
        node = FakeNode({'cam_rate': 15.5})
        self.assertAlmostEqual(ros_params.number_param(node, 'cam_rate', 30.0), 15.5)

    def test_missing_uses_default(self):
        node = FakeNode()
        self.assertEqual(ros_params.number_param(node, 'window_sec', 8.0), 8.0)

    def test_declared_without_default_value(self):
        """必须以"无默认值"声明，否则类型会被定死（评审 @Sousa-cc 指出的坑）。"""
        node = FakeNode()
        ros_params.number_param(node, 'window_sec', 8.0)
        self.assertIsNone(node.declared['window_sec'])


class TestIntegerParam(unittest.TestCase):
    def test_float_override_truncates_to_int(self):
        node = FakeNode({'min_samples': 40.0})
        value = ros_params.integer_param(node, 'min_samples', 10)
        self.assertEqual(value, 40)
        self.assertIsInstance(value, int)

    def test_missing_uses_default(self):
        node = FakeNode()
        self.assertEqual(ros_params.integer_param(node, 'seed', 7), 7)


class TestStringParam(unittest.TestCase):
    def test_override(self):
        node = FakeNode({'imu_topic': '/camera/imu'})
        self.assertEqual(ros_params.string_param(node, 'imu_topic', '/imu'),
                         '/camera/imu')

    def test_missing_uses_default(self):
        node = FakeNode()
        self.assertEqual(
            ros_params.string_param(node, 'cam_topic', '/camera/angular_velocity'),
            '/camera/angular_velocity')

    def test_default_is_declared(self):
        """字符串没有类型歧义，默认值直接声明进去，便于 ros2 param describe 查看。"""
        node = FakeNode()
        ros_params.string_param(node, 'imu_topic', '/imu')
        self.assertEqual(node.declared['imu_topic'], '/imu')


if __name__ == '__main__':
    unittest.main()
