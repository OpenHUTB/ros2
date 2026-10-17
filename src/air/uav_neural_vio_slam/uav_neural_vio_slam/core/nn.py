#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 numpy 的神经网络层：卷积、GRU、全连接、Adam。

不引 PyTorch 的原因与前一模块相同：ROS Humble 侧是 Python 3.8，
仿真侧是 3.12，纯 numpy 两边都能 import，也不需要往虚拟机里塞 2 GB 依赖。

反向传播全部手写并用数值梯度逐点校验（``--selftest``）。
所有层遵循同一套约定：

    y = layer.forward(x)             # 前向，同时缓存反向所需张量
    dx = layer.backward(dy)          # 反向，梯度存入 layer.grads
    layer.params / layer.grads       # 同序列表，交给 Adam
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

__all__ = ["im2col", "col2im_add", "Layer", "Linear", "Conv2d", "GRUCell",
           "Adam", "sigmoid", "rnn_pack", "rnn_unpack"]


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))


# --------------------------------------------------------------------------- #
# 卷积的 im2col / col2im
# --------------------------------------------------------------------------- #
def _out_size(n: int, k: int, stride: int) -> int:
    return (n - k) // stride + 1


def im2col(x: np.ndarray, kh: int, kw: int, stride: int) -> np.ndarray:
    """(N,C,H,W) → (N, C·kh·kw, OH·OW)，只做 valid 卷积（不补零）。

    用 ``as_strided`` 零拷贝取窗；补零会引入大量无效乘加，
    本模块的输入本来就够小（64×48），valid 足够用。

    ⚠️ ``as_strided`` 的**维度顺序必须和 reshape 的目标一致**，否则
    ``reshape`` 会按内存序重新分组，得到的不是 im2col 矩阵，而是一个
    被搅乱的布局。更阴的是：这时前向和反向仍然互为转置，**数值梯度校验
    照样通过**（层变成了"核权重被置换过的卷积"），只有伴随恒等式
    ``<A, col2im(B)> == <im2col(A), B>`` 才能把它揪出来。
    """
    N, C, H, W = x.shape
    OH, OW = _out_size(H, kh, stride), _out_size(W, kw, stride)
    s = x.strides
    v = np.lib.stride_tricks.as_strided(
        x, shape=(N, C, kh, kw, OH, OW),
        strides=(s[0], s[1], s[2], s[3], s[2] * stride, s[3] * stride))
    return v.reshape(N, C * kh * kw, OH * OW)


def col2im_add(cols: np.ndarray, shape, kh: int, kw: int, stride: int) -> np.ndarray:
    """im2col 的伴随算子：把列梯度累加回输入形状（重叠处相加）。"""
    N, C, H, W = shape
    OH, OW = _out_size(H, kh, stride), _out_size(W, kw, stride)
    dx = np.zeros((N, C, H, W))
    c = cols.reshape(N, C, kh, kw, OH, OW)
    for i in range(kh):
        for j in range(kw):
            dx[:, :, i:i + OH * stride:stride, j:j + OW * stride:stride] += c[:, :, i, j]
    return dx


# --------------------------------------------------------------------------- #
# 层
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
    """valid 卷积，stride 可调，本模块不用补零。"""

    def __init__(self, c_in: int, c_out: int, k: int, stride: int = 1,
                 rng: np.random.Generator = None):
        rng = rng or np.random.default_rng(0)
        lim = math.sqrt(6.0 / (c_in * k * k + c_out * k * k))
        self.W = rng.uniform(-lim, lim, (c_out, c_in, k, k))
        self.b = np.zeros(c_out)
        self.k, self.stride = int(k), int(stride)
        self.params = [self.W, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.b)]
        self._cache = None

    def forward(self, x):
        N, C, H, W = x.shape
        cols = im2col(x, self.k, self.k, self.stride)
        OH, OW = _out_size(H, self.k, self.stride), _out_size(W, self.k, self.stride)
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
    """单步 GRU。

    只做**单步**前向/反向（隐藏状态在训练时按序列推进，但梯度不回传到上一步），
    也就是窗口为 1 的截断 BPTT。VIO 的序列长度动辄上千帧，完整 BPTT 既慢又
    没有额外收益——真正携带长期信息的是隐状态本身，不是它的梯度。
    """

    def __init__(self, n_in: int, n_hid: int, rng: np.random.Generator = None):
        rng = rng or np.random.default_rng(0)
        self.n_in, self.n_hid = int(n_in), int(n_hid)
        lim = math.sqrt(6.0 / (n_in + n_hid))
        # [Wz Wr Wh, Uz Ur Uh, bz br bh]
        self.W = rng.uniform(-lim, lim, (n_in, 3 * n_hid))
        self.U = rng.uniform(-lim, lim, (n_hid, 3 * n_hid))
        self.b = np.zeros(3 * n_hid)
        self.W[:, 2 * n_hid:] *= 0.5          # 候选态初始偏小，起步更稳
        self.params = [self.W, self.U, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.U),
                      np.zeros_like(self.b)]
        self._c = None

    def forward(self, x: np.ndarray, h: np.ndarray) -> np.ndarray:
        H = self.n_hid
        g = x @ self.W + h @ self.U + self.b
        z = sigmoid(g[:, :H])
        r = sigmoid(g[:, H:2 * H])
        n = np.tanh(x @ self.W[:, 2 * H:] + r * (h @ self.U[:, 2 * H:])
                    + self.b[2 * H:])
        h_new = (1.0 - z) * n + z * h
        self._c = (x, h, z, r, n)
        return h_new

    def backward(self, dh_new):
        H = self.n_hid
        x, h, z, r, n = self._c
        dz = dh_new * (h - n)
        dn = dh_new * (1.0 - z)
        dh_direct = dh_new * z
        # n = tanh(a), a = x·Wh + r*(h·Uh) + bh
        da = dn * (1.0 - n ** 2)
        uh = h @ self.U[:, 2 * H:]
        dr = (da * uh)
        dh_rhs = (da * r) @ self.U[:, 2 * H:].T
        zz = z * (1.0 - z)
        rr = r * (1.0 - r)
        dg_z, dg_r = dz * zz, dr * rr

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
            + dh_direct
        return dx, dh


# --------------------------------------------------------------------------- #
# 优化器
# --------------------------------------------------------------------------- #
class Adam:
    """对一组层做 Adam 更新；把全部参数展平成一个大向量再更新。"""

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


def rnn_pack(*arrays: np.ndarray) -> List[np.ndarray]:
    """把一组数组按 (T, B, …) 打包成列表。"""
    return [np.asarray(a) for a in arrays]


def rnn_unpack(packed, index: int):
    return packed[index]
