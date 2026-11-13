"""Elementary NumPy layers with explicit forward / backward passes."""

from __future__ import annotations

from typing import Optional

import numpy as np


def _make_rng(seed: int = 0) -> np.random.Generator:
    return np.random.default_rng(int(seed) & 0xFFFFFFFF)


class Dense:
    """Fully connected layer: ``y = x @ W + b``."""

    def __init__(self, in_dim: int, out_dim: int, init: str = "he",
                 seed: int = 0):
        self.in_dim = int(in_dim)
        self.out_dim = int(out_dim)
        self.init = str(init)
        rng = _make_rng(seed)
        if init == "he":
            scale = np.sqrt(2.0 / max(1, self.in_dim))
        elif init == "xavier":
            scale = np.sqrt(2.0 / max(1, self.in_dim + self.out_dim))
        elif init == "lecun":
            scale = np.sqrt(1.0 / max(1, self.in_dim))
        else:
            raise ValueError("unknown init %r" % (init,))
        self.W = (rng.standard_normal((self.in_dim, self.out_dim))
                  * scale).astype(np.float64)
        self.b = np.zeros(self.out_dim, dtype=np.float64)
        self.dW = np.zeros_like(self.W)
        self.db = np.zeros_like(self.b)
        self.x: Optional[np.ndarray] = None

    def forward(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if x.ndim == 1:
            x = x[None, :]
        self.x = x
        return x @ self.W + self.b

    def backward(self, dY: np.ndarray) -> np.ndarray:
        dY = np.asarray(dY, dtype=np.float64)
        self.dW = self.x.T @ dY
        self.db = dY.sum(axis=0)
        return dY @ self.W.T

    # -- parameter plumbing used by MLP / optimiser -------------------------
    def params(self):
        return [self.W, self.b]

    def grads(self):
        return [self.dW, self.db]

    def copy_params_from(self, other: "Dense") -> None:
        self.W[...] = other.W
        self.b[...] = other.b


class ReLU:
    def __init__(self) -> None:
        self.mask: Optional[np.ndarray] = None

    def forward(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        self.mask = x > 0.0
        return x * self.mask

    def backward(self, dY: np.ndarray) -> np.ndarray:
        return np.asarray(dY, dtype=np.float64) * self.mask


class Tanh:
    def __init__(self) -> None:
        self.y: Optional[np.ndarray] = None

    def forward(self, x: np.ndarray) -> np.ndarray:
        self.y = np.tanh(np.asarray(x, dtype=np.float64))
        return self.y

    def backward(self, dY: np.ndarray) -> np.ndarray:
        return np.asarray(dY, dtype=np.float64) * (1.0 - self.y * self.y)


class Sigmoid:
    def __init__(self) -> None:
        self.y: Optional[np.ndarray] = None

    def forward(self, x: np.ndarray) -> np.ndarray:
        x = np.clip(np.asarray(x, dtype=np.float64), -40.0, 40.0)
        self.y = 1.0 / (1.0 + np.exp(-x))
        return self.y

    def backward(self, dY: np.ndarray) -> np.ndarray:
        return np.asarray(dY, dtype=np.float64) * self.y * (1.0 - self.y)


def activation(name: str = "relu"):
    name = str(name).lower()
    if name == "relu":
        return ReLU()
    if name == "tanh":
        return Tanh()
    if name == "sigmoid":
        return Sigmoid()
    raise ValueError("unknown activation %r" % (name,))


class Conv2D:
    """2-D convolution, NCHW layout, same padding by default.

    Implemented with im2col + matrix multiplication so the heavy work goes
    through NumPy's BLAS instead of many small ``einsum`` calls (much faster
    on the CPU-only training VM).  The column cache also makes the backward
    pass straightforward.
    """

    def __init__(self, in_ch: int, out_ch: int, k: int = 3,
                 stride: int = 1, pad: int = 1, seed: int = 0):
        self.in_ch = int(in_ch)
        self.out_ch = int(out_ch)
        self.k = int(k)
        self.stride = int(stride)
        self.pad = int(pad)
        rng = _make_rng(seed)
        fan_in = self.in_ch * self.k * self.k
        scale = np.sqrt(2.0 / max(1, fan_in))
        self.W = (rng.standard_normal((self.out_ch, self.in_ch,
                                       self.k, self.k)) * scale).astype(np.float64)
        self.b = np.zeros(self.out_ch, dtype=np.float64)
        self.dW = np.zeros_like(self.W)
        self.db = np.zeros_like(self.b)
        self.x: Optional[np.ndarray] = None
        self.xp: Optional[np.ndarray] = None
        self.cols: Optional[np.ndarray] = None
        self.out_shape = None

    def _im2col(self, xp: np.ndarray, ho: int, wo: int) -> np.ndarray:
        n, c = xp.shape[0], xp.shape[1]
        p, k, s = self.pad, self.k, self.stride
        if s == 1:
            win = np.lib.stride_tricks.sliding_window_view(xp, (k, k),
                                                           axis=(2, 3))
        else:
            win = np.lib.stride_tricks.sliding_window_view(xp, (k, k),
                                                           axis=(2, 3))
            win = win[:, :, ::s, ::s]
        walk = win[:, :, :ho, :wo].transpose(0, 2, 3, 1, 4, 5)
        return np.ascontiguousarray(walk).reshape(n * ho * wo, c * k * k)

    def forward(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        n, c, h, w = x.shape
        if c != self.in_ch:
            raise ValueError("Conv2D expected %d channels, got %d"
                             % (self.in_ch, c))
        self.x = x
        p, k, s = self.pad, self.k, self.stride
        self.xp = np.pad(x, ((0, 0), (0, 0), (p, p), (p, p))) if p > 0 else x
        ho = (h + 2 * p - k) // s + 1
        wo = (w + 2 * p - k) // s + 1
        self.out_shape = (n, self.out_ch, ho, wo)
        cols = self._im2col(self.xp, ho, wo)
        self.cols = cols
        wmat = self.W.reshape(self.out_ch, -1).T
        out = cols @ wmat + self.b
        return out.reshape(n, ho, wo, self.out_ch).transpose(0, 3, 1, 2)

    def backward(self, dY: np.ndarray) -> np.ndarray:
        dY = np.asarray(dY, dtype=np.float64)
        n, oc, ho, wo = dY.shape
        if (n, oc, ho, wo) != self.out_shape:
            raise ValueError("Conv2D backward shape mismatch")
        p, k, s = self.pad, self.k, self.stride
        h, w = self.x.shape[2], self.x.shape[3]
        dout = dY.transpose(0, 2, 3, 1).reshape(n * ho * wo, oc)
        self.dW = (dout.T @ self.cols).reshape(self.out_ch, self.in_ch, k, k)
        self.db = dout.sum(axis=0)
        dxp = np.zeros_like(self.xp)
        dout_hw = dout.reshape(n, ho, wo, oc)
        for i in range(k):
            for j in range(k):
                contrib = dout @ self.W[:, :, i, j]  # (M, C)
                contrib = contrib.reshape(n, ho, wo, self.in_ch)
                dxp[:, :, i:i + s * ho:s, j:j + s * wo:s] += \
                    contrib.transpose(0, 3, 1, 2)
        if p > 0:
            return dxp[:, :, p:p + h, p:p + w]
        return dxp

    def params(self):
        return [self.W, self.b]

    def grads(self):
        return [self.dW, self.db]

    def copy_params_from(self, other: "Conv2D") -> None:
        self.W[...] = other.W
        self.b[...] = other.b
