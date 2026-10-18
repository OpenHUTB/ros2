"""test_smoke.py —— 冒烟测试：不依赖 mujoco 的部分。"""

import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from humanoid_vision_fall_prediction.nn import FallPredictorMLP
from humanoid_vision_fall_prediction.controller import BalanceController


def test_mlp_forward():
    model = FallPredictorMLP(256, 64, 32, 5)
    x = np.random.randn(8, 256)
    probs = model.forward(x)
    assert probs.shape == (8, 5)
    assert np.allclose(probs.sum(axis=1), 1.0, atol=1e-5)


def test_mlp_train_step():
    model = FallPredictorMLP(256, 64, 32, 5)
    x = np.random.randn(16, 256)
    y = np.random.randint(0, 5, 16)
    loss1 = model.train_step(x, y, lr=0.01)
    loss2 = model.train_step(x, y, lr=0.01)
    assert loss1 > 0
    assert np.isfinite(loss2)


def test_controller():
    ctrl = BalanceController()
    for cls in range(5):
        target = ctrl.target_for(cls, 0.9)
        assert target.shape == (15,)


if __name__ == "__main__":
    test_mlp_forward()
    test_mlp_train_step()
    test_controller()
    print("all smoke tests passed")
