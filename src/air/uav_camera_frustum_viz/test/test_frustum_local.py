#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure geometry tests for camera frustum (no ROS)."""
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

import frustum  # noqa: E402


def test_corners():
    print("== A. 视锥角点 ==")
    c = frustum.frustum_corners(640, 480, 320, 320, 320, 240, 1.0, 5.0)
    check(c.shape == (8, 3), "8 个三维角点")
    check(np.allclose(c[:4, 2], 1.0), "近平面 z=1")
    check(np.allclose(c[4:, 2], 5.0), "远平面 z=5")


def test_axes():
    print("== B. 光学系 -> 机体系 ==")
    optical = np.array([[1.0, 2.0, 3.0]])
    body = frustum.optical_to_body(optical)
    check(np.allclose(body, [[3.0, -1.0, -2.0]]), "x右/y下/z前 -> 前/左/上")


def test_world():
    print("== C. 世界系 ==")
    c = frustum.world_corners([0, 0, 0], [0, 0, 0, 1], 640, 480,
                              320, 320, 320, 240, 1.0, 5.0)
    check(c.shape == (8, 3), "世界系 8 点")
    check(np.allclose(c[0], [1.0, 1.0, 0.75]), "近平面左上角映射正确")
    edges = frustum.edge_points(c)
    check(edges.shape == (24, 3), "12 条边 -> 24 个端点")


if __name__ == "__main__":
    test_corners()
    test_axes()
    test_world()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
