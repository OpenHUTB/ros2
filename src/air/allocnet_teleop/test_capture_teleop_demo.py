#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""capture_teleop_demo.py 的离线回归测试。

不依赖 ROS：用 mock 的 rospy / 消息模块导入脚本，覆盖：
1. --frames / --max-mb 的默认值；
2. 显式传入时参数能正确解析；
3. main() 会按 --frames 创建目录（旧版直接使用未定义的 FRAMES，启动即 NameError）。

运行：
    python3 test_capture_teleop_demo.py
    python3 -m pytest test_capture_teleop_demo.py -q
"""

import importlib.util
import os
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "capture_teleop_demo.py")


def _load_module():
    rospy = types.ModuleType("rospy")
    rospy.ROSInterruptException = type("ROSInterruptException", (Exception,), {})
    rospy.init_node = lambda *args, **kwargs: None
    rospy.Publisher = lambda *args, **kwargs: None
    rospy.Subscriber = lambda *args, **kwargs: None

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})

    visualization_msgs = types.ModuleType("visualization_msgs")
    visualization_msgs_msg = types.ModuleType("visualization_msgs.msg")
    visualization_msgs_msg.Marker = type("Marker", (), {})

    modules = {
        "rospy": rospy,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "visualization_msgs": visualization_msgs,
        "visualization_msgs.msg": visualization_msgs_msg,
    }
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(
            "capture_teleop_demo_under_test", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class CaptureTeleopArgsTest(unittest.TestCase):
    def setUp(self):
        self.module = _load_module()

    def test_default_frames_and_max_mb(self):
        args = self.module.parse_args([])
        self.assertEqual(args.frames, "/tmp/shots")
        self.assertEqual(args.max_mb, 2000)

    def test_custom_frames_and_max_mb(self):
        args = self.module.parse_args(["--frames", "~/shots", "--max-mb", "12"])
        self.assertEqual(args.frames, "~/shots")
        self.assertEqual(args.max_mb, 12)

    def test_main_creates_frames_directory(self):
        class StopAfterInit(Exception):
            pass

        def stop(*args, **kwargs):
            raise StopAfterInit()

        self.module.rospy.init_node = stop
        with tempfile.TemporaryDirectory() as tmp:
            target = os.path.join(tmp, "shots")
            with self.assertRaises(StopAfterInit):
                self.module.main(["--frames", target])
            self.assertTrue(os.path.isdir(target))


if __name__ == "__main__":
    unittest.main(verbosity=2)
