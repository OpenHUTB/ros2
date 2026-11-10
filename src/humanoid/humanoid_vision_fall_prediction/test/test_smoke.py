"""test_smoke.py —— 冒烟测试：不依赖 mujoco 的部分。

覆盖四件事：
  1. 前向输出是合法概率分布；
  2. 反向传播与有限差分梯度一致（真正的正确性校验）；
  3. 权重存取往返一致；
  4. 控制器在阈值以下不施加修正、以上按类别施加正确方向的修正。
"""

import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from humanoid_vision_fall_prediction.nn import FallPredictorMLP
from humanoid_vision_fall_prediction.controller import BalanceController
from humanoid_vision_fall_prediction.sim import DEFAULT_STAND_TARGET


def test_mlp_forward():
    model = FallPredictorMLP(256, 64, 32, 5)
    x = np.random.randn(8, 256)
    probs = model.forward(x)
    assert probs.shape == (8, 5)
    assert np.all(probs >= 0.0)
    assert np.allclose(probs.sum(axis=1), 1.0, atol=1e-5)


def test_mlp_train_step():
    model = FallPredictorMLP(256, 64, 32, 5)
    x = np.random.randn(16, 256)
    y = np.random.randint(0, 5, 16)
    loss1 = model.train_step(x, y, lr=0.01)
    loss2 = model.train_step(x, y, lr=0.01)
    assert loss1 > 0
    assert np.isfinite(loss2)


def _ce_loss(model, X, y):
    probs = model.forward(X)
    return float(-np.log(probs[np.arange(len(y)), y] + 1e-12).mean())


def test_mlp_gradient_check():
    """反向传播梯度必须等于有限差分梯度。

    旧测试只检查“loss 有限”，即使反向传播写错也照样通过。
    """
    model = FallPredictorMLP(6, 5, 4, 3, seed=0)
    rng = np.random.default_rng(1)
    X = rng.normal(size=(4, 6))
    y = np.array([0, 1, 2, 1])

    params0 = {k: getattr(model, k).copy()
               for k in ("W1", "b1", "W2", "b2", "W3", "b3")}
    for k in ("_vW1", "_vb1", "_vW2", "_vb2", "_vW3", "_vb3"):
        getattr(model, k)[:] = 0.0  # 初速度为 0：v = -lr * dW

    lr = 0.1
    model.train_step(X, y, lr=lr)
    dW1_bp = -model._vW1 / lr          # 反推反向传播算出的梯度
    for k, v in params0.items():
        setattr(model, k, v.copy())

    eps = 1e-6
    num = np.zeros_like(model.W1)
    for idx in np.ndindex(model.W1.shape):
        model.W1[idx] += eps
        lp = _ce_loss(model, X, y)
        model.W1[idx] -= 2 * eps
        lm = _ce_loss(model, X, y)
        model.W1[idx] += eps
        num[idx] = (lp - lm) / (2 * eps)

    denom = max(1e-8, float(np.abs(num).max()))
    rel = float(np.abs(num - dW1_bp).max()) / denom
    assert rel < 1e-3, f"反向传播梯度与数值梯度不一致 (rel={rel:.3e})"


def test_mlp_save_load():
    """存取往返后预测必须完全一致（防止权重静默加载失败）。"""
    model = FallPredictorMLP(256, 64, 32, 5, seed=7)
    x = np.random.default_rng(2).normal(size=(5, 256))
    before, _ = model.predict(x)

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "w.npz")
        model.save(path)
        fresh = FallPredictorMLP(256, 64, 32, 5, seed=99)
        fresh.load(path)
    after, conf = fresh.predict(x)
    assert np.array_equal(before, after), "加载后的预测与保存前不一致"
    assert np.all(conf >= 0.0)


def test_controller():
    ctrl = BalanceController()
    for cls in range(5):
        target = ctrl.target_for(cls, 0.9)
        assert target.shape == (15,)


def test_controller_threshold():
    """置信度低于阈值时不得施加任何修正。"""
    ctrl = BalanceController(gain=0.6, confidence_threshold=0.70)
    target = ctrl.target_for(1, 0.5)   # forward 但置信度不足
    assert np.allclose(target, DEFAULT_STAND_TARGET), "低置信度仍然施加了修正"


def test_controller_direction():
    """forward 应增大踝背屈与髋前倾，backward 应与之相反。"""
    ctrl = BalanceController(gain=1.0, confidence_threshold=0.1)
    fwd = ctrl.target_for(1, 1.0)
    bwd = ctrl.target_for(2, 1.0)
    assert fwd[3] > DEFAULT_STAND_TARGET[3], "forward 未增大 ankle_pitch"
    assert fwd[1] > DEFAULT_STAND_TARGET[1], "forward 未增大 hip_pitch"
    assert bwd[3] < DEFAULT_STAND_TARGET[3], "backward 未减小 ankle_pitch"
    assert np.allclose(fwd - DEFAULT_STAND_TARGET, -(bwd - DEFAULT_STAND_TARGET)), \
        "forward/backward 修正不互为反向"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"[ok] {name}")
    print("all smoke tests passed")
