"""Multi-layer perceptron with an internal Adam optimiser."""

from __future__ import annotations

import json
import os
from typing import List, Sequence

import numpy as np

from .layers import Dense, activation as _activation
from .losses import softmax_cross_entropy
from .optim import Adam


class MLP:
    """A plain feed-forward network built from :class:`~.layers.Dense` blocks.

    ``sizes`` includes the input and output widths, e.g. ``[20, 64, 64, 2]``
    creates two hidden layers with 64 units and a linear output layer.  The
    deterministic ``seed`` fully defines the initial weights.
    """

    def __init__(self, sizes: Sequence[int], activation: str = "relu",
                 seed: int = 2026, lr: float = 1e-3, init: str = "he"):
        self.sizes: List[int] = [int(s) for s in sizes]
        if len(self.sizes) < 2:
            raise ValueError("MLP needs at least input and output sizes")
        self.activation_name = str(activation)
        self.init = str(init)
        rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        self.layers = []
        for i in range(len(self.sizes) - 1):
            self.layers.append(Dense(
                self.sizes[i], self.sizes[i + 1], init=init,
                seed=int(rng.integers(0, 2 ** 31 - 1))))
            if i < len(self.sizes) - 2:
                self.layers.append(_activation(self.activation_name))
        self.opt = Adam(self.params(), lr=lr)
        self.out = None

    # ------------------------------------------------------------------ core
    def params(self):
        out = []
        for layer in self.layers:
            if isinstance(layer, Dense):
                out.extend(layer.params())
        return out

    def grads(self):
        out = []
        for layer in self.layers:
            if isinstance(layer, Dense):
                out.extend(layer.grads())
        return out

    def forward(self, X: np.ndarray) -> np.ndarray:
        h = np.asarray(X, dtype=np.float64)
        if h.ndim == 1:
            h = h[None, :]
        for layer in self.layers:
            h = layer.forward(h)
        self.out = h
        return h

    def backward(self, dY: np.ndarray) -> np.ndarray:
        g = np.asarray(dY, dtype=np.float64)
        for layer in reversed(self.layers):
            g = layer.backward(g)
        return g

    def predict(self, X: np.ndarray) -> np.ndarray:
        X = np.asarray(X, dtype=np.float64)
        single = X.ndim == 1
        y = self.forward(X[None, :] if single else X)
        return y[0] if single else y

    def step(self) -> None:
        self.opt.step(self.grads())

    def set_lr(self, lr: float) -> None:
        self.opt.lr = float(lr)

    # -------------------------------------------------------------- training
    def train_step(self, X: np.ndarray, Y: np.ndarray, loss: str = "mse"):
        """One forward / backward / Adam update; returns the batch loss."""
        pred = self.forward(X)
        if loss == "mse":
            diff = pred - np.asarray(Y, dtype=np.float64)
            value = float(np.mean(diff * diff))
            grad = 2.0 * diff / diff.size
        elif loss == "mae":
            diff = pred - np.asarray(Y, dtype=np.float64)
            value = float(np.mean(np.abs(diff)))
            grad = np.sign(diff) / diff.size
        elif loss == "ce":
            value, grad = softmax_cross_entropy(pred, Y)
        else:
            raise ValueError("unknown loss %r" % (loss,))
        self.backward(grad)
        self.step()
        return value

    # ------------------------------------------------------------------- io
    def save(self, path: str, extra: dict = None) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        payload = {
            "sizes": np.asarray(self.sizes, dtype=np.int64),
            "activation": np.asarray(self.activation_name),
            "init": np.asarray(self.init),
            "lr": np.asarray(float(self.opt.lr)),
            "n_layers": np.asarray(len(self.layers), dtype=np.int64),
        }
        for i, layer in enumerate(self.layers):
            if isinstance(layer, Dense):
                payload["W_%d" % i] = layer.W
                payload["b_%d" % i] = layer.b
        np.savez(path, **payload)
        if extra is not None:
            with open(os.path.splitext(path)[0] + ".json", "w",
                      encoding="utf-8") as fh:
                json.dump(extra, fh, indent=2, sort_keys=True)

    @classmethod
    def load(cls, path: str) -> "MLP":
        with np.load(path, allow_pickle=False) as data:
            sizes = [int(s) for s in data["sizes"]]
            act = str(data["activation"])
            init = str(data["init"])
            lr = float(data["lr"])
            net = cls(sizes, activation=act, seed=0, lr=lr, init=init)
            for i, layer in enumerate(net.layers):
                if isinstance(layer, Dense):
                    layer.W[...] = data["W_%d" % i]
                    layer.b[...] = data["b_%d" % i]
        return net

    def copy_params_from(self, other: "MLP") -> None:
        if self.sizes != other.sizes:
            raise ValueError("cannot copy weights between different architectures")
        src = [l for l in other.layers if isinstance(l, Dense)]
        dst = [l for l in self.layers if isinstance(l, Dense)]
        for a, b in zip(dst, src):
            a.copy_params_from(b)
