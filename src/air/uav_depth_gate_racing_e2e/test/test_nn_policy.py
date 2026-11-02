# -*- coding: utf-8 -*-
"""卷积/GRU/注意力与策略网络的正反向。不需要 ROS、不需要 mujoco。"""
import math

import numpy as np
import pytest

from uav_depth_gate_racing_e2e.core.nn import (Adam, Conv2d, GRUCell, LayerNorm,
                                              Linear, SelfAttention, col2im_add,
                                              im2col)
from uav_depth_gate_racing_e2e.core.policy import ARCHS, Policy, PolicyConfig


def _num_grad(f, p: np.ndarray, idx, eps=1e-6):
    """中心差分某个标量元素。``idx`` 必须是完整的索引元组——少给一维会取到
    一整块子数组（``W[0, 1]`` 是 3×3，不是标量），断言就成了数组比较。"""
    old = p[idx]
    p[idx] = old + eps
    fp = f()
    p[idx] = old - eps
    fm = f()
    p[idx] = old
    return (fp - fm) / (2 * eps)


def test_im2col_adjoint_identity():
    """<A, col2im(B)> == <im2col(A), B>。列布局错位时数值梯度照样过，只有它能发现。"""
    rng = np.random.default_rng(0)
    x = rng.normal(size=(2, 3, 9, 11))
    b = rng.normal(size=(2, 3 * 3 * 3, 4 * 5))
    lhs = float(np.sum(x * col2im_add(b, x.shape, 3, 3, 2)))
    rhs = float(np.sum(im2col(x, 3, 3, 2) * b))
    assert abs(lhs - rhs) < 1e-9 * max(abs(rhs), 1.0)


def test_conv_grad():
    rng = np.random.default_rng(1)
    c = Conv2d(2, 3, 3, stride=2, rng=rng)
    x = rng.normal(size=(2, 2, 9, 9))
    w = rng.normal(size=(2, 3, 4, 4))
    c.forward(x)
    c.backward(w)
    for idx in ((0, 1, 0, 2), (1, 0, 2, 1), (2, 1, 1, 0)):
        num = _num_grad(lambda: float(np.sum(c.forward(x) * w)), c.W, idx)
        assert abs(num - c.grads[0][idx]) < 1e-5 * max(abs(num), 1.0)


def test_gru_grad():
    rng = np.random.default_rng(2)
    g = GRUCell(5, 4, rng=rng)
    x = rng.normal(size=(3, 5))
    h = rng.normal(size=(3, 4))
    w = rng.normal(size=(3, 4))
    g.forward(x, h)
    g.backward(w)
    for idx in ((0, 0), (2, 11), (4, 5)):
        num = _num_grad(lambda: float(np.sum(g.forward(x, h) * w)), g.W, idx)
        assert abs(num - g.grads[0][idx]) < 1e-5 * max(abs(num), 1.0)


def test_attention_grad():
    rng = np.random.default_rng(3)
    a = SelfAttention(6, rng=rng)
    x = rng.normal(size=(2, 4, 6))
    w = rng.normal(size=(2, 4, 6))
    a.forward(x)
    a.backward(w)
    for idx in ((0, 0), (3, 4)):
        num = _num_grad(lambda: float(np.sum(a.forward(x) * w)), a.Wq, idx)
        assert abs(num - a.grads[0][idx]) < 1e-5 * max(abs(num), 1.0)


def test_layernorm_zero_mean_unit_var():
    x = np.random.default_rng(4).normal(size=(3, 7)) * 3.0 + 5.0
    y = LayerNorm().forward(x)
    assert np.allclose(y.mean(axis=-1), 0.0, atol=1e-6)
    assert np.allclose(y.var(axis=-1), 1.0, atol=1e-3)


# --------------------------------------------------------------------------- #
def _obs(rng, hw=(24, 32)):
    return {"depth": (rng.random(hw) * 255).astype(np.uint8),
            "vel": rng.normal(size=3).astype(np.float32)}


@pytest.mark.parametrize("arch", ARCHS)
def test_policy_shapes(arch):
    p = Policy(PolicyConfig(arch=arch, depth_hw=(24, 32), hidden=16))
    act, logp, val, h, raw = p.act(_obs(np.random.default_rng(5)), p.init_hidden(1))
    assert act.shape == (1, 3) and raw.shape == (1, 3)
    assert np.isfinite(logp[0]) and np.isfinite(val[0])
    assert np.all(np.abs(act) <= 1.0)


@pytest.mark.parametrize("arch", ARCHS)
def test_policy_head_grad(arch):
    """策略头的解析梯度 vs 数值梯度。

    只在这一层逐元素比对是可靠的：策略头是 Linear，直接接在特征上，中间没有
    非线性折点。往下走到主干就不行了——见 ``test_trunk_grad_descends``。
    """
    rng = np.random.default_rng(6)
    p = Policy(PolicyConfig(arch=arch, depth_hw=(24, 32), hidden=16))
    obs = _obs(rng)
    w = rng.normal(size=(1, 3))
    p.forward(obs)
    p.backward(w, np.zeros(1), np.zeros(3))
    head = p.head_mean
    for idx in ((0, 0), (3, 2)):
        num = _num_grad(lambda: float(np.sum(p.forward(obs)[0] * w)), head.W, idx)
        assert abs(num - head.grads[0][idx]) < 2e-5 * max(abs(num), 1.0)


@pytest.mark.parametrize("arch", ARCHS)
def test_trunk_grad_descends(arch):
    """主干第一层的解析梯度必须是下降方向，且下降量对得上一步的长度。

    **整网逐元素比数值梯度在这里没有意义**：扰动 1e-5 就会让几千个 ReLU 里的
    某几个跨过折点，每次翻转对数值梯度的贡献与真实梯度同量级。实测
    ``corr(g, n) = 0.72``、逐元素比值落在 −136 到 +8 之间——真梯度与噪声混在
    一起，怎么调容差都验不出问题。逐元素的可信校验在层一级（Conv2d / GRU /
    自注意力三个测试）；整网用"下降量"验，噪声在求和里被平均掉。

    取一批观测而不是单帧：折点噪声在不同样本上独立，信号随批大小线性增长、
    噪声只按 √N 增长，单帧只有 0.3 倍的下降量、8 帧能到 0.9 倍。

    这一步能抓住符号写反与量级写错（例如 PPO 里出现过的那次）。
    """
    rng = np.random.default_rng(6)
    n = 8
    p = Policy(PolicyConfig(arch=arch, depth_hw=(24, 32), hidden=16))
    obs = {"depth": (rng.random((n, 24, 32)) * 255).astype(np.uint8),
           "vel": rng.normal(size=(n, 3)).astype(np.float32)}
    w = rng.normal(size=(1, 3))

    def loss() -> float:
        return float(np.sum(p.forward(obs, p.init_hidden(n))[0] * w))

    p.forward(obs, p.init_hidden(n))
    p.backward(np.repeat(w, n, axis=0), np.zeros(n), np.zeros(3))
    trunk = p.enc[0]
    g = trunk.grads[0].copy()
    gnorm2 = float((g ** 2).sum())
    assert gnorm2 > 0.0
    eta = 1e-3
    l0 = loss()
    trunk.W -= eta * g
    l1 = loss()
    trunk.W += eta * g
    drop, pred = l0 - l1, eta * gnorm2
    # 带宽很松：折点噪声实测吃掉七成下降量。这一步只判"方向对不对、量级没写错
    # 十倍以上"，符号反了 drop 会直接变负。
    assert drop > 0.05 * pred, (drop, pred)
    assert drop < 5.0 * pred, (drop, pred)


class _ZeroRng:
    """零噪声采样器：让 raw == mean，两侧都能精确算出来。"""

    def normal(self, size=None):
        return np.zeros(size)


def test_tanh_jacobian_in_logp():
    """logp 必须含 tanh 的雅可比项 Σ log(1−a²)；漏掉时动作会长期贴边界。

    把策略头的偏置推离 0，让 tanh 落在非线性区——均值≈0 时该修正项只有
    1e-6 量级（``1−tanh²≈1``），漏掉也看不出来，测试就没有区分度。
    """
    p = Policy(PolicyConfig(arch="mlp", depth_hw=(24, 32), hidden=16))
    p.head_mean.b[:] = 2.0
    obs = _obs(np.random.default_rng(7))
    act, logp, _, _, raw = p.act(obs, rng=_ZeroRng())
    mean, _, _ = p.forward(obs)
    assert np.allclose(raw, mean)                    # 零噪声 → raw 就是均值
    assert abs(float(act[0, 0])) > 0.5               # 确实在非线性区
    # a = tanh(raw) ⇒ p(a) = p(raw)/(1−a²)，所以 logp = base − Σlog(1−a²)
    corr = float(np.sum(np.log(1.0 - np.tanh(raw) ** 2 + 1e-6)))
    assert corr < -0.5
    assert abs(logp[0] - (p._logp(raw, mean)[0] - corr)) < 1e-6


def test_std_state_independent():
    p = Policy(PolicyConfig(arch="cnn", depth_hw=(24, 32), hidden=16))
    assert np.allclose(p.std, math.exp(p.cfg.log_std_init))
    p.log_std[:] = 10.0
    assert np.all(p.std <= math.exp(0.5) + 1e-9)      # LOG_STD_MAX 截断


@pytest.mark.parametrize("arch", ["mlp", "cnn", "attn"])
def test_save_load_roundtrip(arch, tmp_path):
    p = Policy(PolicyConfig(arch=arch, depth_hw=(24, 32), hidden=16, seed=3))
    f = str(tmp_path / ("p_%s.npz" % arch))
    p.save(f, meta={"stage": "test"})
    q = Policy.load(f)
    assert q.n_params() == p.n_params()
    rng = np.random.default_rng(8)
    obs = _obs(rng)
    a1 = p.act(obs, deterministic=True)[0]
    a2 = q.act(obs, deterministic=True)[0]
    assert np.allclose(a1, a2, atol=1e-6)


def test_gru_batch_matches_stepwise():
    """同一组 (帧, 隐状态) 对，攒成一批过与逐帧过结果必须一致。

    注意不能拿"批量前向 + 零隐状态"和"逐帧递推"比：逐帧时第 k 帧的隐状态
    是前 k−1 帧累积出来的，批量版本每一行都是零。要比的是**同一组配对的
    (帧, 进入时的隐状态)**——这正是 PPO 回放一段轨迹时喂给网络的东西。
    """
    p = Policy(PolicyConfig(arch="gru", depth_hw=(24, 32), hidden=16))
    rng = np.random.default_rng(9)
    frames = [_obs(rng) for _ in range(3)]
    seq_mean, seq_h = [], []
    h = p.init_hidden(1)
    for f in frames:
        seq_h.append(h)
        mean_s, _, h = p.forward(f, h)
        seq_mean.append(mean_s[0])
    batch = {"depth": np.stack([f["depth"] for f in frames]),
             "vel": np.stack([f["vel"] for f in frames])}
    mean_b, _, _ = p.forward(batch, np.concatenate(seq_h, axis=0))
    for k in range(3):
        assert np.allclose(mean_b[k], seq_mean[k], atol=1e-12), k


def test_adam_reduces_loss():
    rng = np.random.default_rng(10)
    x = rng.normal(size=(32, 4))
    y = x @ np.array([1.0, -2.0, 0.5, 0.0]) + 0.3
    lin = Linear(4, 1, rng=rng)
    opt = Adam([lin], lr=0.05)
    first = None
    for _ in range(60):
        pred = lin.forward(x)[:, 0]
        loss = float(np.mean((pred - y) ** 2))
        first = first if first is not None else loss
        lin.grads[0][...] = (2.0 / len(x)) * x.T @ (pred - y)[:, None]
        lin.grads[1][...] = (2.0 / len(x)) * np.sum(pred - y)
        opt.step()
    assert loss < 0.1 * first
