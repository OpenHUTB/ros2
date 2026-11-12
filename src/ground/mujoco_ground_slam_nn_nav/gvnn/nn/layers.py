"""Dense layers and activation functions."""

from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def relu_grad(x: np.ndarray, g: np.ndarray) -> np.ndarray:
    return g * (x > 0.0)


def sigmoid(x: np.ndarray) -> np.ndarray:
    out = np.empty_like(x, dtype=np.float64)
    pos = x >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-x[pos]))
    ex = np.exp(x[~pos])
    out[~pos] = ex / (1.0 + ex)
    return out


def tanh(x: np.ndarray) -> np.ndarray:
    return np.tanh(x)


class Dense:
    def __init__(self, n_in: int, n_out: int, rng: np.random.Generator,
                 activation: str = "relu"):
        self.n_in = int(n_in)
        self.n_out = int(n_out)
        self.activation = activation
        if activation == "relu":
            scale = math.sqrt(2.0 / n_in)
        else:
            scale = math.sqrt(6.0 / (n_in + n_out))
        self.W = rng.normal(0.0, scale, size=(n_in, n_out)).astype(np.float64)
        self.b = np.zeros(n_out, dtype=np.float64)
        self.grads: Dict[str, np.ndarray] = {}
        self._x = None
        self._z = None

    def forward(self, x: np.ndarray, cache: bool = True) -> np.ndarray:
        z = x @ self.W + self.b
        if self.activation == "relu":
            out = relu(z)
        elif self.activation == "tanh":
            out = tanh(z)
        elif self.activation == "sigmoid":
            out = sigmoid(z)
        else:
            out = z
        if cache:
            self._x = x
            self._z = z
        return out

    def backward(self, grad: np.ndarray) -> np.ndarray:
        if self.activation == "relu":
            grad = relu_grad(self._z, grad)
        elif self.activation == "tanh":
            grad = grad * (1.0 - np.tanh(self._z) ** 2)
        elif self.activation == "sigmoid":
            s = sigmoid(self._z)
            grad = grad * s * (1.0 - s)
        self.grads["W"] = self._x.T @ grad
        self.grads["b"] = grad.sum(axis=0)
        return grad @ self.W.T
