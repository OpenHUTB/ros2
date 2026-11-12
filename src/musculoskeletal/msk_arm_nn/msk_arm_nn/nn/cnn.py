"""Small convolutional network for 48x64 RGB camera images (NCHW)."""

from __future__ import annotations

import json
import os

import numpy as np

from .layers import Conv2D, Dense, ReLU
from .optim import Adam


def maxpool2x2(x: np.ndarray):
    """2x2 max pooling; returns (pooled, reshape cache for backward)."""
    n, c, h, w = x.shape
    h2, w2 = h // 2, w // 2
    xr = x[:, :, :h2 * 2, :w2 * 2].reshape(n, c, h2, 2, w2, 2)
    out = xr.max(axis=(3, 5))
    return out, xr


def maxpool2x2_backward(dout: np.ndarray, xr: np.ndarray) -> np.ndarray:
    """Backward pass for :func:`maxpool2x2`.

    ``xr`` is the reshape cache produced by the forward pass.  The mask is
    built against the pooled maximum (not against the upstream gradient!);
    ties (e.g. ReLU patches that are all zero) split the gradient evenly.
    """
    pmax = xr.max(axis=(3, 5))
    mask = xr == pmax[:, :, :, None, :, None]
    norm = mask.sum(axis=(3, 5), keepdims=True).astype(np.float64)
    dxr = mask / norm * dout[:, :, :, None, :, None]
    n, c, h2, w2 = dout.shape
    return dxr.reshape(n, c, h2 * 2, w2 * 2)


class CNN:
    """conv(3->8) -> relu -> pool -> conv(8->16) -> relu -> pool -> fc -> out."""

    def __init__(self, in_ch: int, in_h: int, in_w: int,
                 conv_channels=(8, 16), fc: int = 64, out_dim: int = 2,
                 seed: int = 2026, lr: float = 1e-3, k: int = 3,
                 stride1: int = 2):
        self.in_ch = int(in_ch)
        self.in_h = int(in_h)
        self.in_w = int(in_w)
        self.conv_channels = (int(conv_channels[0]), int(conv_channels[1]))
        self.fc_units = int(fc)
        self.out_dim = int(out_dim)
        self.k = int(k)
        self.stride1 = int(stride1)
        self.lr = float(lr)
        rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        c1, c2 = self.conv_channels
        self.conv1 = Conv2D(self.in_ch, c1, k=k, stride=self.stride1, pad=1,
                            seed=int(rng.integers(0, 2 ** 31 - 1)))
        h1 = (self.in_h + 2 - self.k) // self.stride1 + 1
        w1 = (self.in_w + 2 - self.k) // self.stride1 + 1
        self.conv2 = Conv2D(c1, c2, k=k, stride=1, pad=1,
                            seed=int(rng.integers(0, 2 ** 31 - 1)))
        h2 = (h1 // 2) // 2
        w2 = (w1 // 2) // 2
        self.flat_dim = c2 * h2 * w2
        self.fc1 = Dense(self.flat_dim, self.fc_units,
                         seed=int(rng.integers(0, 2 ** 31 - 1)))
        self.out = Dense(self.fc_units, self.out_dim,
                         seed=int(rng.integers(0, 2 ** 31 - 1)))
        self.relu1 = ReLU()
        self.relu2 = ReLU()
        self.relu3 = ReLU()
        self.opt = Adam(self.params(), lr=lr)
        self._cache = {}

    # ------------------------------------------------------------------ core
    def params(self):
        return (self.conv1.params() + self.conv2.params()
                + self.fc1.params() + self.out.params())

    def grads(self):
        return (self.conv1.grads() + self.conv2.grads()
                + self.fc1.grads() + self.out.grads())

    def forward(self, X: np.ndarray) -> np.ndarray:
        X = np.asarray(X, dtype=np.float64)
        if X.ndim == 3:
            X = X[None, ...]
        h = self.relu1.forward(self.conv1.forward(X))
        h, xr1 = maxpool2x2(h)
        h = self.relu2.forward(self.conv2.forward(h))
        h, xr2 = maxpool2x2(h)
        self._cache["xr1"] = xr1
        self._cache["xr2"] = xr2
        self._cache["pool2_shape"] = h.shape
        flat = h.reshape(h.shape[0], -1)
        self._cache["flat"] = flat
        f = self.relu3.forward(self.fc1.forward(flat))
        return self.out.forward(f)

    def backward(self, dY: np.ndarray) -> np.ndarray:
        g = self.out.backward(np.asarray(dY, dtype=np.float64))
        g = self.fc1.backward(self.relu3.backward(g))
        g = g.reshape(self._cache["pool2_shape"])
        g = maxpool2x2_backward(g, self._cache["xr2"])
        g = self.conv2.backward(self.relu2.backward(g))
        g = maxpool2x2_backward(g, self._cache["xr1"])
        return self.conv1.backward(self.relu1.backward(g))

    def predict(self, X: np.ndarray) -> np.ndarray:
        X = np.asarray(X, dtype=np.float64)
        single = X.ndim == 3
        y = self.forward(X[None, ...] if single else X)
        return y[0] if single else y

    def step(self) -> None:
        self.opt.step(self.grads())

    def set_lr(self, lr: float) -> None:
        self.opt.lr = float(lr)

    def train_step(self, X: np.ndarray, Y: np.ndarray) -> float:
        pred = self.forward(X)
        diff = pred - np.asarray(Y, dtype=np.float64)
        value = float(np.mean(diff * diff))
        self.backward(2.0 * diff / diff.size)
        self.step()
        return value

    # ------------------------------------------------------------------- io
    def save(self, path: str, extra: dict = None) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        np.savez(
            path,
            conv1_W=self.conv1.W, conv1_b=self.conv1.b,
            conv2_W=self.conv2.W, conv2_b=self.conv2.b,
            fc_W=self.fc1.W, fc_b=self.fc1.b,
            out_W=self.out.W, out_b=self.out.b,
            in_shape=np.asarray([self.in_ch, self.in_h, self.in_w],
                                dtype=np.int64),
            conv_channels=np.asarray(self.conv_channels, dtype=np.int64),
            fc=np.asarray(self.fc_units, dtype=np.int64),
            out_dim=np.asarray(self.out_dim, dtype=np.int64),
            stride1=np.asarray(self.stride1, dtype=np.int64),
            lr=np.asarray(float(self.opt.lr)),
        )
        if extra is not None:
            with open(os.path.splitext(path)[0] + ".json", "w",
                      encoding="utf-8") as fh:
                json.dump(extra, fh, indent=2, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> "CNN":
        with np.load(path, allow_pickle=False) as data:
            in_ch, in_h, in_w = [int(v) for v in data["in_shape"]]
            channels = tuple(int(v) for v in data["conv_channels"])
            fc = int(data["fc"])
            out_dim = int(data["out_dim"])
            stride1 = int(data["stride1"]) if "stride1" in data.files else 2
            lr = float(data["lr"])
            net = cls(in_ch, in_h, in_w, conv_channels=channels, fc=fc,
                      out_dim=out_dim, seed=0, lr=lr, stride1=stride1)
            net.conv1.W[...] = data["conv1_W"]
            net.conv1.b[...] = data["conv1_b"]
            net.conv2.W[...] = data["conv2_W"]
            net.conv2.b[...] = data["conv2_b"]
            net.fc1.W[...] = data["fc_W"]
            net.fc1.b[...] = data["fc_b"]
            net.out.W[...] = data["out_W"]
            net.out.b[...] = data["out_b"]
        return net
