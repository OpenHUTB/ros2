# -*- coding: utf-8 -*-
"""手写反向传播的数值梯度校验。不需要 ROS、不需要 mujoco。"""
import numpy as np
import pytest

from uav_neural_vio_slam.core.nn import GRUCell, Linear, col2im_add, im2col
from uav_neural_vio_slam.core.network import LS_CLIP, VioNet, gaussian_nll

EPS = 1e-6


def _numeric(loss_fn, p, idx):
    old = p[idx]
    p[idx] = old + EPS
    lp = loss_fn()[0]
    p[idx] = old - EPS
    lm = loss_fn()[0]
    p[idx] = old
    return (lp - lm) / (2 * EPS)


def _check(layer, loss_fn, tol=2e-4):
    _, d = loss_fn()
    layer.backward(2.0 * d / d.size)
    worst = 0.0
    for p, g in zip(layer.params, layer.grads):
        it = np.nditer(p, flags=["multi_index"])
        for _ in it:
            idx = it.multi_index
            num = _numeric(loss_fn, p, idx)
            worst = max(worst, abs(num - g[idx]) / max(abs(num), abs(g[idx]), 1e-8))
    return worst


def test_linear_gradient():
    rng = np.random.default_rng(0)
    lin = Linear(4, 3, rng=np.random.default_rng(1))
    x, t = rng.normal(size=(7, 4)), rng.normal(size=(7, 3))

    def loss():
        d = lin.forward(x) - t
        return float(np.mean(d * d)), d

    assert _check(lin, loss) < 2e-5


def test_gru_gradient():
    rng = np.random.default_rng(2)
    gru = GRUCell(6, 5, rng=np.random.default_rng(3))
    x, h = rng.normal(size=(4, 6)), rng.normal(size=(4, 5)) * 0.3
    t = rng.normal(size=(4, 5))

    def loss():
        d = gru.forward(x, h) - t
        return float(np.mean(d * d)), d

    assert _check(gru, loss) < 2e-4


def test_gru_gradient_wrt_h():
    rng = np.random.default_rng(4)
    gru = GRUCell(6, 5, rng=np.random.default_rng(5))
    x, h = rng.normal(size=(3, 6)), rng.normal(size=(3, 5)) * 0.3
    t = rng.normal(size=(3, 5))

    def loss():
        d = gru.forward(x, h) - t
        return float(np.mean(d * d)), d

    _, d = loss()
    _, dh = gru.backward(2.0 * d / d.size)
    worst = 0.0
    for j in range(h.shape[1]):
        num = _numeric(loss, h, (0, j))
        worst = max(worst, abs(num - dh[0, j]) / max(abs(num), 1e-8))
    assert worst < 2e-4


def test_im2col_adjoint():
    """<A, col2im(B)> == <im2col(A), B>。

    数值梯度检查抓不到列布局错位：前向反向同时转置时互为转置仍然成立，
    梯度照样对得上。只有这个伴随恒等式能验证布局本身。
    """
    rng = np.random.default_rng(6)
    A = rng.normal(size=(1, 2, 6, 6))
    cols = im2col(A, 3, 3, 1)
    B = rng.normal(size=cols.shape)
    lhs = float(np.sum(A * col2im_add(B, A.shape, 3, 3, 1)))
    rhs = float(np.sum(cols * B))
    assert abs(lhs - rhs) < 1e-9


def test_vio_net_shapes_and_small_init():
    rng = np.random.default_rng(7)
    net = VioNet()
    dp, dr, ls, h = net.forward_step(rng.normal(size=(2, 2, 48, 64)),
                                     rng.normal(size=(2, 10, 6)),
                                     net.init_hidden(2))
    assert dp.shape == (2, 3) and dr.shape == (2, 3) and ls.shape == (2, 6)
    assert h.shape == (2, net.cfg.hidden)
    # 输出头小增益初始化：训练前的增量预测应接近 0（但仍非零，不能当成恒等）
    assert float(np.abs(dp).max()) < 0.05
    assert float(np.abs(dr).max()) < 0.05


def test_gaussian_nll_is_finite():
    nll = gaussian_nll(np.zeros((4, 3)), np.zeros((4, 3)))
    assert np.isfinite(nll)
    # 残差为零、方差为 1 时，NLL 应为 0（常数项已略去）
    assert abs(nll) < 1e-9


def test_log_sigma_is_clipped():
    """log σ² 必须限幅在 LS_CLIP 内，否则 exp 会溢出。"""
    rng = np.random.default_rng(8)
    net = VioNet()
    for layer in net.layers:
        if hasattr(layer, "b"):
            layer.b += 50.0
    _, _, ls, _ = net.forward_step(rng.normal(size=(1, 2, 48, 64)),
                                   rng.normal(size=(1, 10, 6)),
                                   net.init_hidden(1))
    assert np.all(ls >= LS_CLIP[0]) and np.all(ls <= LS_CLIP[1])
    sigma = np.exp(0.5 * ls)
    assert np.all(np.isfinite(sigma))
