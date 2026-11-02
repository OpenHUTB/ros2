#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_local.py —— 无需 ROS / 仿真器 / GPU 的本地回归测试。
直接运行：python3 tests/test_local.py

覆盖：
- 模型前向形状、图像预处理、安全限幅（原有）；
- 权重缺失时部署侧必须抛错安全停止，禁止随机模型起飞（新增）；
- 随机权重仅限离线自检显式开启（新增）；
- NaN / inf / 长度异常等非法输出被拦截，不下发速度指令（新增）。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def test_model_forward():
    import torch
    from model import build_model
    net = build_model()
    out = net(torch.randn(2, 3, 120, 160))
    assert out.shape == (2, 3)
    print("[PASS] 模型前向形状")


def test_preprocess():
    import numpy as np
    from dataset import preprocess_image, IMG_H, IMG_W
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    t = preprocess_image(img)
    assert tuple(t.shape) == (3, IMG_H, IMG_W)
    print("[PASS] 图像预处理形状")


def test_safety_clip():
    import numpy as np
    max_vx, max_vy, max_yaw = 2.0, 0.5, 0.4
    vx, vy, yaw = 5.0, -1.0, 0.9
    vx = np.clip(vx, -max_vx, max_vx)
    vy = np.clip(vy, -max_vy, max_vy)
    yaw = np.clip(yaw, -max_yaw, max_yaw)
    assert vx == max_vx and vy == -max_vy and yaw == max_yaw
    print("[PASS] 安全限幅逻辑")


def test_missing_weights_rejects_deploy():
    """回归：权重缺失时部署侧（allow_random=False）必须抛错安全停止，
    不允许随机初始化模型驱动无人机。本用例不依赖 torch。"""
    from safety import load_trained_model, WeightMissingError
    missing = os.path.join(os.path.dirname(__file__), "nonexistent_weights.pth")
    try:
        load_trained_model(missing, None, lambda: None, allow_random=False)
    except WeightMissingError:
        print("[PASS] 缺失权重：部署侧抛 WeightMissingError 并安全停止")
        return
    raise AssertionError("缺失权重时部署侧应当抛出 WeightMissingError")


def test_allow_random_only_for_selfcheck():
    """回归：随机权重只允许在离线自检（allow_random=True）显式开启时使用。"""
    import torch
    from model import build_model
    from safety import load_trained_model
    missing = os.path.join(os.path.dirname(__file__), "nonexistent_weights.pth")
    model = load_trained_model(
        missing, torch.device("cpu"), build_model, allow_random=True)
    assert model is not None
    print("[PASS] 随机权重仅限离线自检（allow_random=True）显式开启")


def test_invalid_action_rejected():
    """回归：NaN / inf / 长度异常 / 空输出被拦截，不下发速度指令。"""
    from safety import sanitize_action
    limits = (2.0, 0.5, 0.4)
    assert sanitize_action([float("nan"), 0.1, 0.1], *limits) is None
    assert sanitize_action([1.0, float("inf"), 0.1], *limits) is None
    assert sanitize_action([1.0, 0.1, float("-inf")], *limits) is None
    assert sanitize_action([0.1, 0.1], *limits) is None      # 长度不足
    assert sanitize_action([0.1, 0.1, 0.1, 0.1], *limits) is None  # 长度超出
    assert sanitize_action(None, *limits) is None            # 空输出
    print("[PASS] 非法输出（NaN/inf/长度异常）被拦截，不下发指令")


def test_valid_action_clipped():
    """回归：合法但超界的输出被正确限幅到安全范围，正常范围内不被改变。"""
    from safety import sanitize_action
    vx, vy, yaw = sanitize_action([5.0, -1.0, 0.9], 2.0, 0.5, 0.4)
    assert vx == 2.0 and vy == -0.5 and yaw == 0.4
    vx, vy, yaw = sanitize_action([0.5, -0.2, 0.1], 2.0, 0.5, 0.4)
    assert vx == 0.5 and vy == -0.2 and yaw == 0.1
    print("[PASS] 合法输出被正确安全限幅")


if __name__ == "__main__":
    # 先跑不依赖 torch / numpy 的安全回归测试，再跑模型与预处理测试。
    test_missing_weights_rejects_deploy()
    test_invalid_action_rejected()
    test_valid_action_clipped()
    test_safety_clip()
    test_preprocess()
    test_model_forward()
    test_allow_random_only_for_selfcheck()
    print("\n全部本地测试通过 ✔")
