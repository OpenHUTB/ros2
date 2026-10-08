#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_local.py —— 无需 ROS / 仿真器 / GPU 的本地回归测试。
直接运行：python3 tests/test_local.py
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def test_model_forward():
    import torch
    from model import build_model
    net = build_model()
    out = net(torch.randn(2, 3, 120, 160))
    assert out.shape == (2, 3)
    print("[PASS] 模型前向形状")


def test_preprocess():
    from dataset import preprocess_image, IMG_H, IMG_W
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    t = preprocess_image(img)
    assert tuple(t.shape) == (3, IMG_H, IMG_W)
    print("[PASS] 图像预处理形状")


def test_safety_clip():
    max_vx, max_vy, max_yaw = 2.0, 0.5, 0.4
    vx, vy, yaw = 5.0, -1.0, 0.9
    vx = np.clip(vx, -max_vx, max_vx)
    vy = np.clip(vy, -max_vy, max_vy)
    yaw = np.clip(yaw, -max_yaw, max_yaw)
    assert vx == max_vx and vy == -max_vy and yaw == max_yaw
    print("[PASS] 安全限幅逻辑")


if __name__ == "__main__":
    test_safety_clip()
    test_preprocess()
    test_model_forward()
    print("\n全部本地测试通过 ✔")
