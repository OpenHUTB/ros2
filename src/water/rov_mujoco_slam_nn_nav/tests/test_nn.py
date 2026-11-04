"""神经网络单元测试：数值梯度检查 + 可学习性冒烟测试。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rov_mujoco_slam_nn_nav import nn  # noqa: E402


def grad_check_mlp():
    rng = np.random.default_rng(0)
    params = {k: v.astype(np.float64) for k, v in nn.init_mlp([4, 8, 3], rng).items()}
    x = rng.normal(size=(6, 4)).astype(np.float64)
    y = rng.normal(size=(6, 3)).astype(np.float64)
    pred, acts = nn.mlp_forward(params, x, cache=True)
    loss, g = nn.mse_loss(pred, y)
    grads = nn.mlp_backward(params, acts, g)
    worst = 0.0
    for k in ("W0", "b0", "W1", "b1"):
        idx = tuple(rng.integers(0, s) for s in params[k].shape)
        eps = 1e-4
        orig = params[k][idx]
        params[k][idx] = orig + eps
        lp = nn.mse_loss(nn.mlp_forward(params, x), y)[0]
        params[k][idx] = orig - eps
        lm = nn.mse_loss(nn.mlp_forward(params, x), y)[0]
        params[k][idx] = orig
        num = (lp - lm) / (2 * eps)
        worst = max(worst, abs(num - grads[k][idx]) / max(1e-6, abs(num) + abs(grads[k][idx])))
    print(f"MLP grad check worst rel err = {worst:.2e}")
    assert worst < 1e-3, worst


def grad_check_cnn():
    rng = np.random.default_rng(1)
    params = {k: (v.astype(np.float64) if v.dtype != np.int32 else v) for k, v in nn.init_cnn(12, 16, n_filters=4, k=3, stride=2, hidden=8, out_dim=2, rng=rng).items()}
    x = rng.normal(size=(5, 12, 16)).astype(np.float64)
    y = rng.normal(size=(5, 2)).astype(np.float64)
    out, cache = nn.cnn_forward(params, x, cache=True)
    loss, g = nn.mse_loss(out, y)
    grads = nn.cnn_backward(params, cache, g)
    worst = 0.0
    for k in ("K", "cb", "W1", "b1", "W2", "b2"):
        for _ in range(3):
            idx = tuple(rng.integers(0, s) for s in params[k].shape)
            eps = 1e-4
            orig = params[k][idx]
            params[k][idx] = orig + eps
            lp = nn.mse_loss(nn.cnn_forward(params, x), y)[0]
            params[k][idx] = orig - eps
            lm = nn.mse_loss(nn.cnn_forward(params, x), y)[0]
            params[k][idx] = orig
            num = (lp - lm) / (2 * eps)
            worst = max(worst, abs(num - grads[k][idx]) / max(1e-6, abs(num) + abs(grads[k][idx])))
    print(f"CNN grad check worst rel err = {worst:.2e}")
    assert worst < 1e-2, worst


def learn_mlp():
    rng = np.random.default_rng(2)
    X = rng.normal(size=(600, 3)).astype(np.float32)
    Y = (np.sin(X[:, :1]) + 0.5 * X[:, 1:2] ** 2).astype(np.float32)
    params = nn.init_mlp([3, 32, 32, 1], rng)
    hist = nn.train_mlp(params, X, Y, epochs=60, lr=3e-3, seed=1)
    print(f"MLP fit loss {hist[0]['loss']:.4f} -> {hist[-1]['loss']:.4f}")
    assert hist[-1]["loss"] < 0.05


def learn_cnn():
    rng = np.random.default_rng(3)
    X = rng.normal(size=(300, 12, 16)).astype(np.float32)
    Y = np.stack([X.mean(axis=(1, 2)), X.max(axis=(1, 2))], axis=1).astype(np.float32)
    params = nn.init_cnn(12, 16, n_filters=6, k=3, stride=2, hidden=32, out_dim=2, rng=rng)
    hist = nn.train_cnn(params, X, Y, epochs=30, lr=2e-3, seed=2)
    print(f"CNN fit loss {hist[0]['loss']:.4f} -> {hist[-1]['loss']:.4f}")
    assert hist[-1]["loss"] < hist[0]["loss"]


if __name__ == "__main__":
    grad_check_mlp()
    grad_check_cnn()
    learn_mlp()
    learn_cnn()
    print("nn tests passed")


