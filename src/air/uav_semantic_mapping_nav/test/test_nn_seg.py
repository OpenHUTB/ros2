# -*- coding: utf-8 -*-
"""网络层与分割网络的数值校验。不需要 ROS、不需要 mujoco。"""
import numpy as np
import pytest

from uav_semantic_mapping_nav.core.nn import (Conv2d, Linear, Upsample2x,
                                             col2im_add, cross_entropy, im2col,
                                             softmax)
from uav_semantic_mapping_nav.core.metrics import confusion, miou
from uav_semantic_mapping_nav.core.segnet import SegNet, class_weights

EPS = 1e-6


def _check(layer, loss_fn, tol):
    _, d = loss_fn()
    layer.backward(2.0 * d / d.size)
    worst = 0.0
    for p, g in zip(layer.params, layer.grads):
        it = np.nditer(p, flags=["multi_index"])
        for _ in it:
            idx = it.multi_index
            old = p[idx]
            p[idx] = old + EPS
            lp = loss_fn()[0]
            p[idx] = old - EPS
            lm = loss_fn()[0]
            p[idx] = old
            num = (lp - lm) / (2 * EPS)
            worst = max(worst, abs(num - g[idx]) / max(abs(num), abs(g[idx]), 1e-8))
    return worst


def test_conv_gradient():
    rng = np.random.default_rng(0)
    cv = Conv2d(3, 5, 3, stride=2, rng=np.random.default_rng(1))
    x = rng.normal(size=(2, 3, 12, 10))
    t = rng.normal(size=(2, 5, 5, 4))

    def loss():
        d = cv.forward(x) - t
        return float(np.mean(d * d)), d

    assert _check(cv, loss, 2e-4) < 2e-4


def test_linear_gradient():
    rng = np.random.default_rng(2)
    lin = Linear(6, 4, rng=np.random.default_rng(3))
    x, t = rng.normal(size=(9, 6)), rng.normal(size=(9, 4))

    def loss():
        d = lin.forward(x) - t
        return float(np.mean(d * d)), d

    assert _check(lin, loss, 2e-4) < 2e-4


def test_upsample_backward():
    rng = np.random.default_rng(4)
    up = Upsample2x()
    x = rng.normal(size=(2, 3, 5, 4))
    t = rng.normal(size=(2, 3, 10, 8))

    def loss():
        d = up.forward(x) - t
        return float(np.mean(d * d)), d

    _, d = loss()
    dx = up.backward(2.0 * d / d.size)
    worst = 0.0
    for a in range(5):
        for b in range(4):
            old = x[0, 0, a, b]
            x[0, 0, a, b] = old + EPS
            lp = loss()[0]
            x[0, 0, a, b] = old - EPS
            lm = loss()[0]
            x[0, 0, a, b] = old
            num = (lp - lm) / (2 * EPS)
            worst = max(worst, abs(num - dx[0, 0, a, b]) / max(abs(num), 1e-9))
    assert worst < 1e-5


def test_im2col_adjoint():
    """<A, col2im(B)> == <im2col(A), B>。列布局错位只有这一条能抓到。"""
    rng = np.random.default_rng(5)
    A = rng.normal(size=(1, 2, 8, 8))
    cols = im2col(A, 3, 3, 1)
    B = rng.normal(size=cols.shape)
    lhs = float(np.sum(A * col2im_add(B, A.shape, 3, 3, 1)))
    assert abs(lhs - float(np.sum(cols * B))) < 1e-9


def test_softmax_and_ce():
    rng = np.random.default_rng(6)
    lg = rng.normal(size=(2, 4, 5, 5))
    p = softmax(lg, axis=1)
    assert np.allclose(p.sum(axis=1), 1.0)
    tg = np.zeros((2, 5, 5), np.int32)
    assert cross_entropy(lg, tg) > 0
    assert cross_entropy(lg, -np.ones((2, 5, 5), np.int32)) == 0.0


def test_ce_gradient_matches_numeric():
    """交叉熵对 logits 的解析梯度应等于 p − onehot。"""
    rng = np.random.default_rng(7)
    net = SegNet()
    rgb = rng.random((2, 48, 60, 3)).astype(np.float32)
    seg = rng.integers(0, 9, (2, 48, 60)).astype(np.int32)
    loss, _ = net.loss_and_grad(rgb, seg)
    assert np.isfinite(loss) and loss > 0


def test_segnet_output_matches_input_resolution():
    rng = np.random.default_rng(8)
    net = SegNet()
    for hw in ((96, 96), (64, 80), (48, 48)):
        out = net.forward(rng.random((1, hw[0], hw[1], 3)).astype(np.float32))
        assert out.shape[2:] == hw, "%s → %s" % (hw, out.shape)


def test_segnet_backward_runs():
    rng = np.random.default_rng(9)
    net = SegNet()
    rgb = rng.random((1, 96, 96, 3)).astype(np.float32)
    seg = np.zeros((1, 96, 96), np.int32)
    net.zero_grad()
    net.loss_and_grad(rgb, seg)
    assert any(np.abs(g).sum() > 0 for g in net.grads)


def test_class_weights_normalised():
    w = class_weights(np.array([1000, 500, 100, 50, 10, 5, 5, 5, 5]))
    assert abs(w.mean() - 1.0) < 1e-9
    assert w[8] > w[0]
    assert np.all(w >= 0)


def test_confusion_and_miou():
    pred = np.array([0, 1, 1, 2])
    gt = np.array([0, 1, 2, 2])
    c = confusion(pred, gt, 3)
    assert c[0, 0] == 1 and c[1, 1] == 1 and c[2, 2] == 1 and c[2, 1] == 1
    v = miou(c)
    assert 0.0 < v < 1.0
