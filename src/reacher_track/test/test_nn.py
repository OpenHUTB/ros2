# -*- coding: utf-8 -*-
"""test_nn.py —— 纯 NumPy 的 MLP / Adam 自检，不需要 mujoco 与 ROS。

覆盖三件在训练里真的会错的事：
  1. 前向的批量与逐条是否一致（写错 reshape 时单条推理会在部署时偏掉）；
  2. 反向梯度是否等于数值梯度（手写反向最容易在这里错，且训练不会报错）；
  3. 存档/读档是否无损（`save` 用 `np.savez`，`load` 按 dims 重建）。
"""
import numpy as np

from reacher_track.nn import MLP, Adam


def test_forward_shape_and_batch_consistency():
    net = MLP([6, 16, 16, 2], seed=0)
    x = np.random.RandomState(1).randn(5, 6)

    out = net.forward(x)
    assert out.shape == (5, 2)

    rows = np.vstack([net.forward(x[i:i + 1]) for i in range(5)])
    assert np.allclose(out, rows, atol=1e-12), '批量前向与逐条前向不一致'


def test_backward_matches_numeric_gradient():
    """中心差分校验手写反向。损失取 L = 0.5 * mean_n sum_out (out - y)^2，
    与 `mse_grad` + `backward` 里 `/n`、`mean` 的口径一致。"""
    rng = np.random.RandomState(0)
    net = MLP([6, 8, 3], seed=0)
    X = rng.randn(7, 6)
    Y = rng.randn(7, 3)

    out = net.forward(X)
    gw, gb = net.backward(net.mse_grad(Y))

    def loss():
        o = net.forward(X)
        return 0.5 * np.mean(np.sum((o - Y) ** 2, axis=1))

    eps = 1e-6
    for i in range(net.n_layer):
        # 每层抽查 6 个权重、全部偏置，够抓符号/缩放错误又不至于跑太久
        idx = rng.choice(net.W[i].size, size=min(6, net.W[i].size), replace=False)
        for k in idx:
            flat = net.W[i].reshape(-1)
            old = flat[k]
            flat[k] = old + eps
            lp = loss()
            flat[k] = old - eps
            lm = loss()
            flat[k] = old
            num = (lp - lm) / (2 * eps)
            assert abs(num - gw[i].reshape(-1)[k]) < 1e-6, \
                'W%d[%d] 数值梯度 %.8f 与解析梯度 %.8f 不符' % (
                    i, k, num, gw[i].reshape(-1)[k])
        for k in range(net.b[i].size):
            old = net.b[i][k]
            net.b[i][k] = old + eps
            lp = loss()
            net.b[i][k] = old - eps
            lm = loss()
            net.b[i][k] = old
            num = (lp - lm) / (2 * eps)
            assert abs(num - gb[i][k]) < 1e-6, \
                'b%d[%d] 数值梯度 %.8f 与解析梯度 %.8f 不符' % (i, k, num, gb[i][k])


def test_save_load_roundtrip(tmp_path):
    net = MLP([6, 12, 2], seed=3)
    x = np.random.RandomState(4).randn(4, 6)
    before = net.forward(x)

    path = str(tmp_path / 'net.npz')
    net.save(path)

    other = MLP([6, 1, 1])            # 故意给错结构，验证 load 会按存档重建
    other.load(path)
    assert other.dims == [6, 12, 2]
    assert np.allclose(other.forward(x), before, atol=1e-12)


def test_adam_reduces_loss():
    """Adam 在监督回归上必须真的下降——反过来若步长/动量写错，这里会红。"""
    rng = np.random.RandomState(5)
    X = rng.randn(200, 3)
    Y = X @ np.array([[1.0], [-2.0], [0.5]]) + 0.3

    net = MLP([3, 16, 1], seed=0)
    opt = Adam(lr=1e-2)
    first = last = None
    for _ in range(200):
        out = net.forward(X)
        gw, gb = net.backward(net.mse_grad(Y))
        opt.step(*net.params(), gw, gb)
        last = float(np.mean((out - Y) ** 2))
        if first is None:
            first = last
    assert last < first * 0.1, 'Adam 200 步后损失只从 %.5f 降到 %.5f' % (first, last)
