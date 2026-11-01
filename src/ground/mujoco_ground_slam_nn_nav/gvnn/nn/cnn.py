"""Small convolutional network implemented with im2col (NumPy only)."""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Sequence

import numpy as np

from .layers import Dense, relu, relu_grad, sigmoid, tanh


def _im2col(x: np.ndarray, k: int, stride: int, padding: int):
    if padding:
        xp = np.pad(x, ((0, 0), (padding, padding), (padding, padding), (0, 0)),
                    mode="constant")
    else:
        xp = x
    b, h, w, c = xp.shape
    ho = (h - k) // stride + 1
    wo = (w - k) // stride + 1
    windows = np.lib.stride_tricks.sliding_window_view(xp, (k, k), axis=(1, 2))
    windows = windows[:, :ho * stride:stride, :wo * stride:stride]
    cols = windows.transpose(0, 1, 2, 4, 5, 3).reshape(b * ho * wo, k * k * c)
    return cols, xp.shape, ho, wo


class Conv2D:
    def __init__(self, cin: int, cout: int, k: int, stride: int = 1,
                 padding: int = 0, activation: str = "relu",
                 rng: Optional[np.random.Generator] = None):
        rng = rng or np.random.default_rng(0)
        self.cin, self.cout = int(cin), int(cout)
        self.k, self.stride, self.padding = int(k), int(stride), int(padding)
        self.activation = activation
        self.W = rng.normal(0.0, math.sqrt(2.0 / (k * k * cin)),
                            size=(k, k, cin, cout)).astype(np.float64)
        self.b = np.zeros(cout, dtype=np.float64)
        self.grads: Dict[str, np.ndarray] = {}
        self._cols = None
        self._xp_shape = None
        self._ho = self._wo = 0
        self._z = None
        self._out_shape = None

    def forward(self, x: np.ndarray, cache: bool = True) -> np.ndarray:
        b = x.shape[0]
        cols, xp_shape, ho, wo = _im2col(x, self.k, self.stride, self.padding)
        z = cols @ self.W.reshape(-1, self.cout) + self.b
        z = z.reshape(b, ho, wo, self.cout)
        if self.activation == "relu":
            out = relu(z)
        elif self.activation == "tanh":
            out = tanh(z)
        elif self.activation == "sigmoid":
            out = sigmoid(z)
        else:
            out = z
        if cache:
            self._cols = cols
            self._xp_shape = xp_shape
            self._ho, self._wo = ho, wo
            self._z = z
            self._out_shape = (b, ho, wo, self.cout)
        return out

    def backward(self, grad: np.ndarray) -> np.ndarray:
        if self.activation == "relu":
            grad = relu_grad(self._z, grad)
        elif self.activation == "tanh":
            grad = grad * (1.0 - np.tanh(self._z) ** 2)
        elif self.activation == "sigmoid":
            s = sigmoid(self._z)
            grad = grad * s * (1.0 - s)
        b = grad.shape[0]
        flat = grad.reshape(-1, self.cout)
        self.grads["W"] = (self._cols.T @ flat).reshape(self.k, self.k, self.cin, self.cout)
        self.grads["b"] = flat.sum(axis=0)
        dcols = (flat @ self.W.reshape(-1, self.cout).T).reshape(
            b, self._ho, self._wo, self.k, self.k, self.cin)
        bshape = self._xp_shape
        dxp = np.zeros(bshape, dtype=np.float64)
        for i in range(self.k):
            for j in range(self.k):
                dxp[:, i:i + self._ho * self.stride:self.stride,
                    j:j + self._wo * self.stride:self.stride, :] += dcols[:, :, :, i, j, :]
        if self.padding:
            p = self.padding
            return dxp[:, p:bshape[1] - p, p:bshape[2] - p, :]
        return dxp


class MaxPool2D:
    def __init__(self, k: int = 2):
        self.k = int(k)
        self._argmax = None
        self._shape = None
        self._out_shape = None

    def forward(self, x: np.ndarray, cache: bool = True) -> np.ndarray:
        b, h, w, c = x.shape
        k = self.k
        ho, wo = h // k, w // k
        xr = x[:, :ho * k, :wo * k, :].reshape(b, ho, k, wo, k, c)
        xt = xr.transpose(0, 1, 3, 5, 2, 4).reshape(b, ho, wo, c, k * k)
        out = xt.max(axis=-1)
        if cache:
            self._argmax = xt.argmax(axis=-1)
            self._shape = (b, h, w, c)
            self._out_shape = (b, ho, wo, c)
        return out

    def backward(self, grad: np.ndarray) -> np.ndarray:
        b, ho, wo, c = self._out_shape
        k = self.k
        dxr = np.zeros((b, ho, wo, c, k * k), dtype=np.float64)
        np.put_along_axis(dxr, self._argmax[..., None], grad[..., None], axis=-1)
        dx = dxr.reshape(b, ho, wo, c, k, k).transpose(0, 1, 4, 2, 5, 3)
        dx = dx.reshape(b, ho * k, wo * k, c)
        out = np.zeros(self._shape, dtype=np.float64)
        out[:, :ho * k, :wo * k, :] = dx
        return out


class Flatten:
    def __init__(self):
        self._shape = None

    def forward(self, x: np.ndarray, cache: bool = True) -> np.ndarray:
        if cache:
            self._shape = x.shape
        return x.reshape(x.shape[0], -1)

    def backward(self, grad: np.ndarray) -> np.ndarray:
        return grad.reshape(self._shape)


class CNN:
    """Sequential CNN built from a declarative spec."""

    def __init__(self, spec: Sequence[dict], seed: int = 0):
        rng = np.random.default_rng(seed)
        self.spec = list(spec)
        self.layers: List[object] = []
        for s in self.spec:
            typ = s["type"]
            if typ == "conv":
                self.layers.append(Conv2D(s["cin"], s["cout"], s["k"],
                                          s.get("s", 1), s.get("p", 0),
                                          s.get("act", "relu"), rng))
            elif typ == "pool":
                self.layers.append(MaxPool2D(s.get("k", 2)))
            elif typ == "flatten":
                self.layers.append(Flatten())
            elif typ == "dense":
                self.layers.append(Dense(s["nin"], s["nout"], rng, s.get("act", "relu")))
            else:
                raise ValueError(f"unknown layer type {typ!r}")

    def forward(self, x: np.ndarray, cache: bool = True) -> np.ndarray:
        out = np.asarray(x, dtype=np.float64)
        for layer in self.layers:
            out = layer.forward(out, cache=cache)
        return out

    def backward(self, grad: np.ndarray) -> None:
        for layer in reversed(self.layers):
            grad = layer.backward(grad)

    def params(self) -> Dict[str, np.ndarray]:
        p: Dict[str, np.ndarray] = {}
        for i, layer in enumerate(self.layers):
            if isinstance(layer, (Conv2D, Dense)):
                p[f"W{i}"] = layer.W
                p[f"b{i}"] = layer.b
        return p

    def grads(self) -> Dict[str, np.ndarray]:
        g: Dict[str, np.ndarray] = {}
        for i, layer in enumerate(self.layers):
            if isinstance(layer, (Conv2D, Dense)) and "W" in layer.grads:
                g[f"W{i}"] = layer.grads["W"]
                g[f"b{i}"] = layer.grads["b"]
        return g

    def predict(self, x: np.ndarray) -> np.ndarray:
        return self.forward(x, cache=False)

    # ------------------------------------------------------------------ io
    def save(self, path: str) -> None:
        arrays = {}
        for i, layer in enumerate(self.layers):
            if isinstance(layer, (Conv2D, Dense)):
                arrays[f"W{i}"] = layer.W
                arrays[f"b{i}"] = layer.b
        np.savez_compressed(path, **arrays)
        with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as f:
            json.dump({"type": "cnn", "spec": self.spec}, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str) -> "CNN":
        with open(os.path.splitext(path)[0] + ".json", "r", encoding="utf-8") as f:
            meta = json.load(f)
        data = np.load(path)
        model = cls(meta["spec"])
        for i, layer in enumerate(model.layers):
            if isinstance(layer, (Conv2D, Dense)):
                layer.W = np.asarray(data[f"W{i}"], dtype=np.float64)
                layer.b = np.asarray(data[f"b{i}"], dtype=np.float64)
        return model
