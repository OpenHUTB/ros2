#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度避障本地测试（不需要 ROS / 仿真器 / GPU）.

A. 走廊筛选
B. 最近距离
C. 减速曲线
D. 工程文件一致性
"""
from __future__ import annotations

import os
import sys

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
sys.path.insert(0, os.path.join(PKG, "scripts"))

# 桩掉 ROS 依赖，纯数学测试不需要 roscore
import types  # noqa: E402

sys.modules["rospy"] = types.ModuleType("rospy")
gm = types.ModuleType("geometry_msgs")
gmm = types.ModuleType("geometry_msgs.msg")
gmm.Twist = type("Twist", (), {})
gm.msg = gmm
sys.modules["geometry_msgs"] = gm
sys.modules["geometry_msgs.msg"] = gmm
sm = types.ModuleType("sensor_msgs")
smm = types.ModuleType("sensor_msgs.msg")
smm.PointCloud2 = type("PointCloud2", (), {})
sm.msg = smm
sm.point_cloud2 = types.ModuleType("sensor_msgs.point_cloud2")
sys.modules["sensor_msgs"] = sm
sys.modules["sensor_msgs.msg"] = smm
sys.modules["sensor_msgs.point_cloud2"] = sm.point_cloud2

import depth_avoidance  # noqa: E402


def test_corridor():
    print("== A. 走廊筛选 ==")
    pts = np.array([
        [0.0, 0.0, 1.0],
        [2.0, 0.0, 1.0],
        [0.0, 2.0, 1.0],
        [0.0, 0.0, 20.0],
        [0.0, 0.0, -1.0],
    ], dtype=np.float32)
    out = depth_avoidance.corridor_points(pts, 1.0, 1.0, 12.0)
    check(out.shape == (1, 3), "只保留走廊内 1 点: %r" % (out.shape,))
    check(np.allclose(out[0], [0.0, 0.0, 1.0]), "保留点坐标正确")


def test_distance():
    print("== B. 最近距离 ==")
    pts = np.array([[0.0, 0.0, 5.0], [0.1, 0.0, 3.0]], dtype=np.float32)
    check(abs(depth_avoidance.nearest_distance(pts, 1.0, 1.0, 12.0) - 3.0) < 1e-6,
          "最近距离=3.0")
    empty = np.empty((0, 3), dtype=np.float32)
    check(depth_avoidance.nearest_distance(empty, 1.0, 1.0, 12.0) == float("inf"),
          "无点返回 inf")


def test_speed():
    print("== C. 减速曲线 ==")
    f = depth_avoidance.compute_speed
    check(f(1.0, 2.0, 4.0, 1.5) == 0.0, "d<=stop 停车")
    check(abs(f(3.0, 2.0, 4.0, 1.5) - 0.75) < 1e-6, "d=3 半速 0.75")
    check(f(5.0, 2.0, 4.0, 1.5) == 1.5, "d>=slow 全速")
    check(f(float("inf"), 2.0, 4.0, 1.5) == 1.5, "inf 也全速")


def test_project_files():
    print("== D. 工程文件 ==")
    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    launch = open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read()
    cfg = open(os.path.join(PKG, "config", "bridge.yaml"), encoding="utf-8").read()
    readme = open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check("depth_avoidance.py" in cmake, "CMakeLists 安装 depth_avoidance.py")
    check('enable_depth_avoidance' in launch, "main.launch 暴露开关")
    check('type="depth_avoidance.py"' in launch, "main.launch 启动 depth_avoidance.py")
    for key in ("forward_speed", "slow_distance", "stop_distance", "lateral_limit"):
        check(key in cfg, "bridge.yaml 含 %s" % key)
    check("深度避障" in readme, "README 记录深度避障")


if __name__ == "__main__":
    test_corridor()
    test_distance()
    test_speed()
    test_project_files()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
