#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 numpy 神经网络层：卷积、全连接、GRU、单头自注意力、Adam。

反向传播全部手写并用数值梯度校验（``--selftest``）。层接口统一：

    y = layer.forward(x)        # 前向，缓存反向所需张量
    dx = layer.backward(dy)     # 反向，梯度写入 layer.grads
    layer.params / layer.grads  # 同序列表，交给 Adam
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

__all__ = ["im2col", "col2im_add", "out_size", "Layer", "Linear", "Conv2d",
           "GRUCell", "SelfAttention", "LayerNorm", "Adam", "relu", "tanh",
           "sigmoid", "log_softmax", "softmax"]


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def tanh(x: np.ndarray) -> np.ndarray:
    return np.tanh(x)


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))


def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    z = x - x.max(axis=axis, keepdims=True)
    e = np.exp(z)
    return e / e.sum(axis=axis, keepdims=True)


def log_softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    z = x - x.max(axis=axis, keepdims=True)
    return z - np.log(np.exp(z).sum(axis=axis, keepdims=True))


def out_size(n: int, k: int, stride: int) -> int:
    return (n - k) // stride + 1


# --------------------------------------------------------------------------- #
def im2col(x: np.ndarray, kh: int, kw: int, stride: int) -> np.ndarray:
    """(N,C,H,W) → (N, C·kh·kw, OH·OW)，valid 卷积。

    ``as_strided`` 的维度顺序与 reshape 目标一致。顺序不一致时 reshape 按内存序
    重新分组；此时前向与反向仍互为转置、数值梯度校验通过，只有伴随恒等式
    ``<A, col2im(B)> == <im2col(A), B>`` 能发现（见 selftest）。
    """
    N, C, H, W = x.shape
    OH, OW = out_size(H, kh, stride), out_size(W, kw, stride)
    s = x.strides
    v = np.lib.stride_tricks.as_strided(
        x, shape=(N, C, kh, kw, OH, OW),
        strides=(s[0], s[1], s[2], s[3], s[2] * stride, s[3] * stride))
    return v.reshape(N, C * kh * kw, OH * OW)


def col2im_add(cols: np.ndarray, shape, kh: int, kw: int, stride: int) -> np.ndarray:
    N, C, H, W = shape
    OH, OW = out_size(H, kh, stride), out_size(W, kw, stride)
    dx = np.zeros((N, C, H, W))
    c = cols.reshape(N, C, kh, kw, OH, OW)
    for i in range(kh):
        for j in range(kw):
            dx[:, :, i:i + OH * stride:stride, j:j + OW * stride:stride] += \
                c[:, :, i, j]
    return dx


# --------------------------------------------------------------------------- #
class Layer:
    params: List[np.ndarray] = []
    grads: List[np.ndarray] = []

    def forward(self, x: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def backward(self, dy: np.ndarray) -> np.ndarray:
        raise NotImplementedError


class Linear(Layer):
    def __init__(self, n_in: int, n_out: int, rng=None, gain: float = 1.0):
        rng = rng or np.random.default_rng(0)
        lim = math.sqrt(6.0 / (n_in + n_out)) * gain
        self.W = rng.uniform(-lim, lim, (n_in, n_out))
        self.b = np.zeros(n_out)
        self.params = [self.W, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.b)]
        self._x = None

    def forward(self, x):
        self._x = x
        return x @ self.W + self.b

    def backward(self, dy):
        d = dy.reshape(-1, dy.shape[-1])
        x = self._x.reshape(-1, self._x.shape[-1])
        self.grads[0][...] = x.T @ d
        self.grads[1][...] = d.sum(axis=0)
        return dy @ self.W.T


class Conv2d(Layer):
    def __init__(self, c_in: int, c_out: int, k: int, stride: int = 1,
                 rng=None, gain: float = 1.0):
        rng = rng or np.random.default_rng(0)
        lim = math.sqrt(6.0 / (c_in * k * k + c_out * k * k)) * gain
        self.W = rng.uniform(-lim, lim, (c_out, c_in, k, k))
        self.b = np.zeros(c_out)
        self.k, self.stride = int(k), int(stride)
        self.params = [self.W, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.b)]
        self._cache = None

    def forward(self, x):
        N, C, H, W = x.shape
        cols = im2col(x, self.k, self.k, self.stride)
        OH = out_size(H, self.k, self.stride)
        OW = out_size(W, self.k, self.stride)
        self._cache = (cols, x.shape, N, OH, OW)
        out = self.W.reshape(self.W.shape[0], -1) @ cols + self.b[:, None]
        return out.reshape(N, -1, OH, OW)

    def backward(self, dy):
        cols, x_shape, N, OH, OW = self._cache
        d = dy.reshape(N, -1, OH * OW)
        self.grads[0][...] = (d @ cols.transpose(0, 2, 1)).sum(axis=0) \
            .reshape(self.W.shape)
        self.grads[1][...] = d.sum(axis=(0, 2))
        dcols = self.W.reshape(self.W.shape[0], -1).T @ d
        return col2im_add(dcols, x_shape, self.k, self.k, self.stride)


class GRUCell(Layer):
    """单步 GRU（截断 BPTT：梯度不回传到上一步）。"""

    def __init__(self, n_in: int, n_hid: int, rng=None):
        rng = rng or np.random.default_rng(0)
        self.n_in, self.n_hid = int(n_in), int(n_hid)
        lim = math.sqrt(6.0 / (n_in + n_hid))
        self.W = rng.uniform(-lim, lim, (n_in, 3 * n_hid))
        self.U = rng.uniform(-lim, lim, (n_hid, 3 * n_hid))
        self.b = np.zeros(3 * n_hid)
        self.W[:, 2 * n_hid:] *= 0.5
        self.params = [self.W, self.U, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.U),
                      np.zeros_like(self.b)]
        self._c = None

    def forward(self, x, h):
        H = self.n_hid
        g = x @ self.W + h @ self.U + self.b
        z = sigmoid(g[:, :H])
        r = sigmoid(g[:, H:2 * H])
        n = tanh(x @ self.W[:, 2 * H:] + r * (h @ self.U[:, 2 * H:])
                 + self.b[2 * H:])
        self._c = (x, h, z, r, n)
        return (1.0 - z) * n + z * h

    def backward(self, dh_new):
        H = self.n_hid
        x, h, z, r, n = self._c
        dz = dh_new * (h - n)
        dn = dh_new * (1.0 - z)
        da = dn * (1.0 - n ** 2)
        dr = da * (h @ self.U[:, 2 * H:])
        dh_rhs = (da * r) @ self.U[:, 2 * H:].T
        dg_z, dg_r = dz * z * (1 - z), dr * r * (1 - r)
        self.grads[0][...] = 0.0
        self.grads[1][...] = 0.0
        self.grads[2][...] = 0.0
        self.grads[0][:, :H] = x.T @ dg_z
        self.grads[0][:, H:2 * H] = x.T @ dg_r
        self.grads[0][:, 2 * H:] = x.T @ da
        self.grads[1][:, :H] = h.T @ dg_z
        self.grads[1][:, H:2 * H] = h.T @ dg_r
        self.grads[1][:, 2 * H:] = h.T @ (da * r)
        self.grads[2][:H] = dg_z.sum(axis=0)
        self.grads[2][H:2 * H] = dg_r.sum(axis=0)
        self.grads[2][2 * H:] = da.sum(axis=0)
        dx = dg_z @ self.W[:, :H].T + dg_r @ self.W[:, H:2 * H].T \
            + da @ self.W[:, 2 * H:].T
        dh = dg_z @ self.U[:, :H].T + dg_r @ self.U[:, H:2 * H].T + dh_rhs \
            + dh_new * z
        return dx, dh


class LayerNorm(Layer):
    """对最后一维做归一化。无参数，只做 (x−μ)/σ。"""

    params: List[np.ndarray] = []
    grads: List[np.ndarray] = []

    def __init__(self, eps: float = 1e-5):
        self.eps = eps
        self._c = None

    def forward(self, x):
        mu = x.mean(axis=-1, keepdims=True)
        var = x.var(axis=-1, keepdims=True)
        inv = 1.0 / np.sqrt(var + self.eps)
        self._c = (x, mu, inv)
        return (x - mu) * inv

    def backward(self, dy):
        x, mu, inv = self._c
        d = x.shape[-1]
        xc = (x - mu) * inv
        return inv * (dy - dy.mean(axis=-1, keepdims=True)
                      - xc * (dy * xc).mean(axis=-1, keepdims=True))


class SelfAttention(Layer):
    """单头自注意力（无位置编码；token 顺序由上游卷积的空间结构决定）。"""

    def __init__(self, dim: int, rng=None, gain: float = 0.5):
        rng = rng or np.random.default_rng(0)
        lim = math.sqrt(6.0 / (dim + dim)) * gain
        self.Wq = rng.uniform(-lim, lim, (dim, dim))
        self.Wk = rng.uniform(-lim, lim, (dim, dim))
        self.Wv = rng.uniform(-lim, lim, (dim, dim))
        self.Wo = rng.uniform(-lim, lim, (dim, dim))
        self.params = [self.Wq, self.Wk, self.Wv, self.Wo]
        self.grads = [np.zeros_like(p) for p in self.params]
        self._c = None

    def forward(self, x: np.ndarray) -> np.ndarray:
        """``x`` 为 (B, T, D)。"""
        q, k, v = x @ self.Wq, x @ self.Wk, x @ self.Wv
        s = q @ k.transpose(0, 2, 1) / math.sqrt(x.shape[-1])
        a = softmax(s, axis=-1)
        o = a @ v
        self._c = (x, q, k, v, a)
        return o @ self.Wo

    def backward(self, dy):
        x, q, k, v, a = self._c
        d = x.shape[-1]
        self.grads[3][...] = np.einsum("btd,bte->de", a @ v, dy)
        do = dy @ self.Wo.T
        da = do @ v.transpose(0, 2, 1)
        dv = a.transpose(0, 2, 1) @ do
        ds = a * (da - (da * a).sum(axis=-1, keepdims=True))
        dq = ds @ k / math.sqrt(d)
        dk = ds.transpose(0, 2, 1) @ q / math.sqrt(d)
        self.grads[0][...] = np.einsum("btd,bte->de", x, dq)
        self.grads[1][...] = np.einsum("btd,bte->de", x, dk)
        self.grads[2][...] = np.einsum("btd,bte->de", x, dv)
        return dq @ self.Wq.T + dk @ self.Wk.T + dv @ self.Wv.T


# --------------------------------------------------------------------------- #
class Adam:
    def __init__(self, layers: Sequence[Layer], lr: float = 3e-4,
                 b1: float = 0.9, b2: float = 0.999, eps: float = 1e-8,
                 weight_decay: float = 0.0, clip: float = 0.0):
        self.layers = list(layers)
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.wd, self.clip = weight_decay, clip
        flat = [p for l in self.layers for p in l.params]
        self.m = [np.zeros_like(p) for p in flat]
        self.v = [np.zeros_like(p) for p in flat]
        self.t = 0

    def zero_grad(self) -> None:
        for l in self.layers:
            for g in l.grads:
                g[...] = 0.0

    def step(self) -> float:
        self.t += 1
        gn = 0.0
        k = 0
        for l in self.layers:
            for i in range(len(l.params)):
                p, g = l.params[i], l.grads[i]
                if self.wd:
                    g = g + self.wd * p
                gn += float(np.sum(g * g))
                self.m[k] = self.b1 * self.m[k] + (1 - self.b1) * g
                self.v[k] = self.b2 * self.v[k] + (1 - self.b2) * (g * g)
                mh = self.m[k] / (1 - self.b1 ** self.t)
                vh = self.v[k] / (1 - self.b2 ** self.t)
                upd = self.lr * mh / (np.sqrt(vh) + self.eps)
                if self.clip > 0.0:
                    np.clip(upd, -self.clip, self.clip, out=upd)
                p -= upd
                k += 1
        return math.sqrt(gn)
