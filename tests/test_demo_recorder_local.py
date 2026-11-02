#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo_recorder 本地测试（不需要 ROS/仿真器）."""
from __future__ import annotations

import os
import sys
import types

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
sys.path.insert(0, os.path.join(PKG, "scripts"))

rospy = types.ModuleType("rospy")
rospy.Time = types.SimpleNamespace(now=lambda: 0.0)
rospy.Duration = lambda _v: None
sys.modules["rospy"] = rospy
cv2 = types.ModuleType("cv2")
sys.modules["cv2"] = cv2
cv_bridge = types.ModuleType("cv_bridge")
cv_bridge.CvBridge = type("CvBridge", (), {})
sys.modules["cv_bridge"] = cv_bridge
nav = types.ModuleType("nav_msgs")
navm = types.ModuleType("nav_msgs.msg")
navm.Odometry = type("Odometry", (), {})
nav.msg = navm
sys.modules["nav_msgs"] = nav
sys.modules["nav_msgs.msg"] = navm
sensor = types.ModuleType("sensor_msgs")
sensorm = types.ModuleType("sensor_msgs.msg")
sensorm.Image = type("Image", (), {})
sensor.msg = sensorm
sys.modules["sensor_msgs"] = sensor
sys.modules["sensor_msgs.msg"] = sensorm

import demo_recorder  # noqa: E402


def test_frame_name():
    print("== A. 文件名 ==")
    check(demo_recorder.frame_name(1) == "000001", "1 -> 000001")
    check(demo_recorder.frame_name(42) == "000042", "42 -> 000042")


def test_depth():
    print("== B. 深度转换 ==")
    u16 = np.array([[0, 1000, 2500]], dtype=np.uint16)
    out = demo_recorder.depth_to_meters(u16, "16UC1")
    check(np.allclose(out, [[0.0, 1.0, 2.5]]), "16UC1 mm -> m")
    f32 = np.array([[np.nan, 1.5]], dtype=np.float32)
    out = demo_recorder.depth_to_meters(f32, "32FC1")
    check(out[0, 0] == 0.0 and out[0, 1] == 1.5, "NaN -> 0")


def test_fresh():
    print("== C. 同步判定 ==")
    check(demo_recorder.is_fresh(10.0, (9.9, 10.0, 9.8), 0.3), "0.2s 内 -> 新")
    check(not demo_recorder.is_fresh(10.0, (9.0, 10.0), 0.3), "1s 前 -> 过期")
    check(not demo_recorder.is_fresh(10.0, (), 0.3), "空时间戳 -> 过期")


def test_meta():
    print("== D. meta 行 ==")
    pose = {"x": 1, "y": 2, "z": 3, "qx": 0, "qy": 0, "qz": 0, "qw": 1}
    row = demo_recorder.make_meta(7, 1.0, 1.1, 1.2, pose, 0.3)
    check(row["frame"] == "000007", "frame 编号")
    check(row["rgb_stamp"] == "1.000000", "时间戳格式")
    check(row["x"] == "1.000000" and row["qw"] == "1.000000", "位姿字段")


def test_project_files():
    print("== E. 工程文件 ==")
    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    readme = open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check(os.path.isfile(os.path.join(PKG, "scripts", "demo_recorder.py")),
          "demo_recorder.py 存在")
    check("demo_recorder.py" in cmake, "CMakeLists 安装 demo_recorder.py")
    check("demo_recorder" in readme, "README 记录 demo_recorder")


if __name__ == "__main__":
    test_frame_name()
    test_depth()
    test_fresh()
    test_meta()
    test_project_files()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
