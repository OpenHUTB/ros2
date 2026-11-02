#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_neural_local.py —— 视觉导航安全层本地回归测试（无需 ROS / torch / 仿真器）。

直接运行：python3 tests/test_neural_local.py

覆盖：
- 权重缺失时部署侧抛错安全停止，禁止随机模型起飞；
- NaN / inf / 长度异常等非法输出被拦截，不下发速度指令；
- 合法但超界的输出被正确限幅。
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "src", "air", "uav_neural_perception", "scripts"))


def test_missing_weights_rejects_deploy():
    """回归：权重缺失时部署侧（allow_random=False）必须抛错安全停止。"""
    from safety import load_trained_model, WeightMissingError
    missing = os.path.join(os.path.dirname(__file__), "nonexistent_weights.pth")
    try:
        load_trained_model(missing, None, lambda: None, allow_random=False)
    except WeightMissingError:
        print("[PASS] 缺失权重：部署侧抛 WeightMissingError 并安全停止")
        return
    raise AssertionError("缺失权重时部署侧应当抛出 WeightMissingError")


def test_invalid_action_rejected():
    """回归：NaN / inf / 长度异常 / 空输出被拦截，不下发速度指令。"""
    from safety import sanitize_action
    limits = (2.0, 0.5, 0.4)
    assert sanitize_action([float("nan"), 0.1, 0.1], *limits) is None
    assert sanitize_action([1.0, float("inf"), 0.1], *limits) is None
    assert sanitize_action([1.0, 0.1, float("-inf")], *limits) is None
    assert sanitize_action([0.1, 0.1], *limits) is None       # 长度不足
    assert sanitize_action([0.1, 0.1, 0.1, 0.1], *limits) is None  # 长度超出
    assert sanitize_action(None, *limits) is None             # 空输出
    assert sanitize_action("bad", *limits) is None            # 不可转 float
    print("[PASS] 非法输出（NaN/inf/长度异常）被拦截，不下发指令")


def test_valid_action_clipped():
    """回归：合法但超界的输出被限幅到安全范围，正常范围内不被改变。"""
    from safety import sanitize_action
    vx, vy, yaw = sanitize_action([5.0, -1.0, 0.9], 2.0, 0.5, 0.4)
    assert vx == 2.0 and vy == -0.5 and yaw == 0.4
    vx, vy, yaw = sanitize_action([0.5, -0.2, 0.1], 2.0, 0.5, 0.4)
    assert vx == 0.5 and vy == -0.2 and yaw == 0.1
    print("[PASS] 合法输出被正确安全限幅")


if __name__ == "__main__":
    test_missing_weights_rejects_deploy()
    test_invalid_action_rejected()
    test_valid_action_clipped()
    print("\n安全层本地测试全部通过")
