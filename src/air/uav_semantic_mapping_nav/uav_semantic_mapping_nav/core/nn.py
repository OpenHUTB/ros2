#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 numpy 的神经网络层：卷积、最近邻上采样、全连接、Adam、交叉熵。

反向传播全部手写，用数值梯度逐点校验（``--selftest``）。层接口约定：

    y = layer.forward(x)        # 前向，同时缓存反向所需张量
    dx = layer.backward(dy)     # 反向，梯度存入 layer.grads
    layer.params / layer.grads  # 同序列表，交给 Adam
"""
from __future__ import annotations

import math
from typing import List, Sequence

import numpy as np

__all__ = ["im2col", "col2im_add", "Layer", "Linear", "Conv2d", "Upsample2x",
           "Adam", "sigmoid", "softmax", "cross_entropy", "out_size"]


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))


def softmax(logits: np.ndarray, axis: int = 1) -> np.ndarray:
    z = logits - logits.max(axis=axis, keepdims=True)
    e = np.exp(z)
    return e / e.sum(axis=axis, keepdims=True)


def cross_entropy(logits: np.ndarray, target: np.ndarray,
                  ignore_index: int = -1) -> float:
    """逐像素交叉熵。``logits`` 为 (N,C,H,W)，``target`` 为 (N,H,W) 类别索引。

    ``ignore_index`` 的像素（未标注 / 无效）不参与损失。
    """
    n, c, h, w = logits.shape
    lg = logits.transpose(0, 2, 3, 1).reshape(-1, c)
    tg = np.asarray(target).reshape(-1)
    keep = tg != ignore_index
    if not np.any(keep):
        return 0.0
    lg, tg = lg[keep], tg[keep]
    p = softmax(lg, axis=1)
    return float(-np.mean(np.log(np.maximum(p[np.arange(len(tg)), tg], 1e-12))))


def out_size(n: int, k: int, stride: int) -> int:
    return (n - k) // stride + 1


# --------------------------------------------------------------------------- #
def im2col(x: np.ndarray, kh: int, kw: int, stride: int) -> np.ndarray:
    """(N,C,H,W) → (N, C·kh·kw, OH·OW)，valid 卷积，不补零。

    用 ``as_strided`` 零拷贝取窗。``as_strided`` 的维度顺序与 reshape 的目标
    一致。顺序不一致时 reshape 按内存序重新分组，布局被打乱；此时前向与反向
    仍互为转置、数值梯度校验通过，只有伴随恒等式
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
    """im2col 的伴随算子：列梯度累加回输入形状（重叠处相加）。"""
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
    """层接口。``params`` / ``grads`` 顺序一一对应。"""

    params: List[np.ndarray] = []
    grads: List[np.ndarray] = []

    def forward(self, x: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def backward(self, dy: np.ndarray) -> np.ndarray:
        raise NotImplementedError


class Linear(Layer):
    def __init__(self, n_in: int, n_out: int, rng: np.random.Generator = None,
                 gain: float = 1.0):
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
        self.grads[0][...] = self._x.T @ dy
        self.grads[1][...] = dy.sum(axis=0)
        return dy @ self.W.T


class Conv2d(Layer):
    """valid 卷积，stride 可调。"""

    def __init__(self, c_in: int, c_out: int, k: int, stride: int = 1,
                 rng: np.random.Generator = None, gain: float = 1.0):
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
        OH, OW = out_size(H, self.k, self.stride), out_size(W, self.k, self.stride)
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


class Upsample2x(Layer):
    """最近邻 2× 上采样。反向时把梯度按 2×2 块求和回拼。

    用最近邻而不是转置卷积：分割的输出是类别图，像素之间不需要可学习的
    插值；最近邻没有参数、反向只有加法，同精度下比转置卷积快一倍以上。
    """

    params: List[np.ndarray] = []
    grads: List[np.ndarray] = []

    def forward(self, x: np.ndarray) -> np.ndarray:
        return np.repeat(np.repeat(x, 2, axis=2), 2, axis=3)

    def backward(self, dy: np.ndarray) -> np.ndarray:
        n, c, h2, w2 = dy.shape
        h, w = h2 // 2, w2 // 2
        d = dy[:, :, :h * 2, :w * 2]
        return d.reshape(n, c, h, 2, w, 2).sum(axis=(3, 5))


class Adam:
    """对一组层做 Adam 更新。"""

    def __init__(self, layers: Sequence[Layer], lr: float = 2e-3,
                 b1: float = 0.9, b2: float = 0.999, eps: float = 1e-8,
                 weight_decay: float = 0.0, clip: float = 0.0):
        self.layers = list(layers)
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.wd, self.clip = weight_decay, clip
        self.m = [np.zeros_like(p) for l in self.layers for p in l.params]
        self.v = [np.zeros_like(p) for l in self.layers for p in l.params]
        self.t = 0

    def zero_grad(self) -> None:
        for l in self.layers:
            for g in l.grads:
                g[...] = 0.0

    def step(self) -> float:
        self.t += 1
        gn = 0.0
        flat = [(l.params[i], l.grads[i]) for l in self.layers
                for i in range(len(l.params))]
        for k, (p, g) in enumerate(flat):
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
        return math.sqrt(gn)
