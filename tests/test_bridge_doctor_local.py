#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge_doctor 本地测试（不需要 ROS/仿真器）."""
from __future__ import annotations

import os
import sys
import types

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
sys.path.insert(0, os.path.join(PKG, "scripts"))

rospy = types.ModuleType("rospy")
rospy.AnyMsg = type("AnyMsg", (), {})
rospy.get_param = lambda key, default=None: default
sys.modules["rospy"] = rospy
tf2 = types.ModuleType("tf2_ros")
tf2.Buffer = type("Buffer", (), {})
tf2.TransformListener = type("TransformListener", (), {})
sys.modules["tf2_ros"] = tf2

import bridge_doctor  # noqa: E402


def test_rate_status():
    print("== A. 频率判定 ==")
    check(bridge_doctor.rate_status(10.0, 5.0) == "PASS", "10Hz >= 5Hz -> PASS")
    check(bridge_doctor.rate_status(2.0, 5.0) == "LOW", "2Hz < 5Hz -> LOW")
    check(bridge_doctor.rate_status(0.0, 5.0) == "NO_DATA", "0Hz -> NO_DATA")


def test_table():
    print("== B. 表格 ==")
    text = bridge_doctor.format_table([
        ("/uav/odom", 5.0, 10.0, "PASS"),
        ("/camera/depth", 1.0, 0.0, "NO_DATA"),
    ])
    check("TOPIC" in text and "MIN_HZ" in text and "ACTUAL_HZ" in text, "表头存在")
    check("/uav/odom" in text and "/camera/depth" in text, "话题存在")
    check("PASS" in text and "NO_DATA" in text, "状态存在")
    check(text.count("\n") >= 3, "至少 4 行")


def test_project_files():
    print("== C. 工程文件 ==")
    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    readme = open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check(os.path.isfile(os.path.join(PKG, "scripts", "bridge_doctor.py")),
          "bridge_doctor.py 存在")
    check("bridge_doctor.py" in cmake, "CMakeLists 安装 bridge_doctor.py")
    check("bridge_doctor" in readme, "README 记录 bridge_doctor")


if __name__ == "__main__":
    test_rate_status()
    test_table()
    test_project_files()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
