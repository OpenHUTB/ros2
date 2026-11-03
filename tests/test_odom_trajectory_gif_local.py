#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_trajectory_gif 本地测试（不需要 ROS / 仿真器）."""
import os
import sys
import tempfile

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
sys.path.insert(0, os.path.join(PKG, "scripts"))

import odom_trajectory_gif  # noqa: E402


def test_trajectory():
    print("== A. 示例轨迹 ==")
    pts = odom_trajectory_gif.sample_trajectory(120)
    check(len(pts) == 120, "120 点")
    check(all(len(p) == 2 for p in pts), "每点 (x,y)")


def test_csv():
    print("== B. CSV 读取 ==")
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False,
                                     encoding="utf-8") as f:
        f.write("x,y\n1,2\n3,4\nbad,row\n")
        path = f.name
    try:
        pts = odom_trajectory_gif.read_csv(path)
        check(pts == [(1.0, 2.0), (3.0, 4.0)], "跳过非法行")
    finally:
        os.unlink(path)


def test_render():
    print("== C. 降采样/渲染 ==")
    pts = odom_trajectory_gif.sample_trajectory(200)
    check(len(odom_trajectory_gif.downsample(pts, 50)) <= 50, "降采样到 50 内")
    box = odom_trajectory_gif.bounds(pts)
    check(box[0] < box[1] and box[2] < box[3], "包围盒有效")
    img = odom_trajectory_gif.render_frame(pts, 0.5, box, (320, 240))
    check(img.size == (320, 240), "图像尺寸 320x240")


if __name__ == "__main__":
    test_trajectory()
    test_csv()
    test_render()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
