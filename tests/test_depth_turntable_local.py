#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""depth_turntable 本地测试（不需要 ROS / 仿真器）."""
import os
import sys
import tempfile

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
sys.path.insert(0, os.path.join(PKG, "scripts"))

import depth_turntable  # noqa: E402


def test_angles():
    print("== A. 旋转角 ==")
    a = depth_turntable.turntable_angles(8)
    check(len(a) == 8, "8 帧")
    check(abs(a[0]) < 1e-9, "首帧 0 度")
    check(a[-1] < 2 * np.pi, "末帧小于 360 度")


def test_points():
    print("== B. 点云读取/降采样 ==")
    with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as f:
        np.save(f.name, np.array([[0, 0, 1], [1, 2, 3], [np.nan, 0, 0]],
                                 dtype=np.float32))
        path = f.name
    try:
        pts = depth_turntable.load_points(path)
        check(pts.shape == (2, 3), "过滤 NaN 后剩 2 点")
    finally:
        os.unlink(path)
    pts = np.arange(30, dtype=np.float32).reshape(10, 3)
    check(len(depth_turntable.subsample(pts, 5)) == 5, "降采样到 5 点")


def test_render():
    print("== C. 渲染 ==")
    pts = np.array([[0, 0, 1], [0.5, -0.2, 3], [-0.5, 0.1, 5]], dtype=np.float32)
    img = depth_turntable.render_frame(pts, 0.3, (320, 240))
    check(img.size == (320, 240), "图像尺寸 320x240")


if __name__ == "__main__":
    test_angles()
    test_points()
    test_render()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
