import os

import numpy as np

from uav_neural_safety_filter.core.network import NetConfig, PolicyNet, SafetyNet


def _fd(net, X, Y, eps=1e-6):
    y, cache = net.forward(X, cache=True)
    net.optimizer(1e-3).zero_grad()
    net.backward(2.0 * (y - Y) / y.size, cache)
    g = net.layers[0].grads[0][0, 0]
    old = net.layers[0].W[0, 0]
    net.layers[0].W[0, 0] = old + eps
    lp = float(np.mean((net.forward(X) - Y) ** 2))
    net.layers[0].W[0, 0] = old - eps
    lm = float(np.mean((net.forward(X) - Y) ** 2))
    net.layers[0].W[0, 0] = old
    num = (lp - lm) / (2 * eps)
    return abs(num - g) / max(abs(num) + abs(g), 1e-9)


def test_policy_grad():
    rng = np.random.default_rng(0)
    net = PolicyNet(hidden=8)
    X = rng.normal(size=(6, 119))
    Y = rng.normal(size=(6, 3))
    assert _fd(net, X, Y) < 1e-5


def test_safety_h_grad():
    rng = np.random.default_rng(1)
    net = SafetyNet(NetConfig(hidden=8))
    X = rng.normal(size=(6, 116))
    A = rng.normal(size=(6, 5))
    h, q, cache = net.forward(X, A)
    net.optimizer(1e-3).zero_grad()
    net.backward(2 * h / len(h), np.zeros_like(q), cache)
    g = net.l1.grads[0][0, 0]
    old = net.l1.W[0, 0]
    net.l1.W[0, 0] = old + 1e-6
    h2, _, _ = net.forward(X, A)
    lp = float(np.mean(h2 ** 2))
    net.l1.W[0, 0] = old - 1e-6
    h3, _, _ = net.forward(X, A)
    lm = float(np.mean(h3 ** 2))
    net.l1.W[0, 0] = old
    num = (lp - lm) / 2e-6
    assert abs(num - g) / max(abs(num) + abs(g), 1e-9) < 1e-4


def test_policy_save_load(tmp_path):
    net = PolicyNet(hidden=8)
    X = np.random.default_rng(2).normal(size=(3, 119))
    p = os.path.join(tmp_path, "p.npz")
    net.save(p)
    net2 = PolicyNet.load(p)
    assert np.allclose(net.forward(X), net2.forward(X))


def test_safety_save_load(tmp_path):
    net = SafetyNet(NetConfig(hidden=8))
    X = np.random.default_rng(3).normal(size=(3, 116))
    A = np.random.default_rng(4).normal(size=(3, 5))
    p = os.path.join(tmp_path, "s.npz")
    net.save(p)
    net2 = SafetyNet.load(p)
    assert np.allclose(net.forward(X, A)[0], net2.forward(X, A)[0])
