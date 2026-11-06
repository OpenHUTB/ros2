#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure ground-plane segmentation tests (no ROS)."""
import os
import sys

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import ground_filter  # noqa: E402


def test_fit():
    print("== A. RANSAC 平面 ==")
    rng = np.random.default_rng(1)
    ground = np.c_[rng.uniform(-10, 10, 400), rng.uniform(-10, 10, 400),
                   rng.normal(0, 0.02, 400)]
    pts = np.vstack([ground, [[2, 2, 2], [3, 3, 3], [-2, 1, 2.5]]])
    normal, d = ground_filter.fit_plane(pts)
    check(abs(abs(normal[2]) - 1.0) < 0.02, "法向量接近 z 轴")
    check(abs(d) < 0.05, "平面距离接近 0")


def test_split():
    print("== B. 地面/障碍分割 ==")
    pts = np.array([[0, 0, 0.02], [1, 1, -0.01], [2, 2, 2.0]], dtype=float)
    normal, d = np.array([0.0, 0.0, 1.0]), 0.0
    ground, obstacle = ground_filter.split_cloud(pts, normal, d, 0.2)
    check(ground.tolist() == [True, True, False], "地面掩码正确")
    check(obstacle.tolist() == [False, False, True], "障碍掩码取反")
    m = ground_filter.plane_metrics(pts, ground)
    check(m["ground"] == 2 and m["obstacles"] == 1, "统计数量正确")
    check(abs(m["ground_ratio"] - 2 / 3.0) < 1e-9, "地面比例正确")


def test_empty():
    print("== C. 边界 ==")
    try:
        ground_filter.fit_plane(np.zeros((2, 3)))
        check(False, "少于 3 点应报错")
    except ValueError:
        check(True, "少于 3 点报错")


if __name__ == "__main__":
    test_fit()
    test_split()
    test_empty()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
