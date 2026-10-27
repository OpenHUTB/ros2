#!/usr/bin/env python3
"""Offline smoke tests that do not require ROS or Gazebo.

Run:
    python3 scripts/test_offline.py
"""
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import (  # noqa: E402
    FEATURE_NAMES,
    INPUT_DIM,
    MLPController,
    NeuralController,
    PIExpert,
    package_path,
)


def check_model():
    weights = package_path("models/mlp_policy.pt")
    norm = package_path("models/norm_stats.npz")
    assert os.path.isfile(weights), "缺少 MLP 权重: {}".format(weights)
    assert os.path.isfile(norm), "缺少归一化文件: {}".format(norm)
    controller = NeuralController(weights, norm, 0.9, 1.6)
    features = np.zeros(INPUT_DIM, dtype=np.float32)
    v_cmd, w_cmd = controller.command(features)
    assert np.isfinite(v_cmd) and np.isfinite(w_cmd)
    assert abs(v_cmd) <= 0.9 + 1e-6
    assert abs(w_cmd) <= 1.6 + 1e-6
    print("[test] MLP 推理正常: v={:.3f}, w={:.3f}".format(v_cmd, w_cmd))


def check_pi():
    expert = PIExpert(1.1, 1.6, 1.8, 3.2, 0.8, 1.0, 0.9, 1.6)
    v, w = expert.command(0.5, 0.0, 0.0, 0.0, 0.05)
    assert v >= 0.5
    assert abs(w) < 1e-6
    for _ in range(100):
        v, w = expert.command(0.5, 0.2, 0.0, 0.0, 0.05)
    assert v <= 0.9 + 1e-6 and w <= 1.6 + 1e-6
    print("[test] PI 专家输出正常: v={:.3f}, w={:.3f}".format(v, w))


def check_shapes():
    model = MLPController()
    out = model(torch.zeros(3, INPUT_DIM))
    assert tuple(out.shape) == (3, 2)
    assert len(FEATURE_NAMES) == INPUT_DIM
    print("[test] 张量维度正常:", tuple(out.shape))


def main():
    check_shapes()
    check_pi()
    check_model()
    print("[test] 全部离线测试通过")


if __name__ == "__main__":
    main()
