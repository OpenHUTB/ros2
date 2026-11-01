"""Loss functions with analytic gradients."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def mse(y_true: np.ndarray, y_pred: np.ndarray) -> Tuple[float, np.ndarray]:
    diff = y_pred - y_true
    n = max(1, diff.size)
    return float(np.mean(diff * diff)), (2.0 / n) * diff


def softmax(logits: np.ndarray) -> np.ndarray:
    z = logits - logits.max(axis=1, keepdims=True)
    e = np.exp(z)
    return e / e.sum(axis=1, keepdims=True)


def softmax_cross_entropy(y_true: np.ndarray, logits: np.ndarray) -> Tuple[float, np.ndarray]:
    p = softmax(logits)
    n = len(logits)
    eps = 1e-12
    loss = -float(np.mean(np.log(p[np.arange(n), y_true] + eps)))
    grad = p.copy()
    grad[np.arange(n), y_true] -= 1.0
    return loss, grad / n


def binary_cross_entropy(y_true: np.ndarray, logits: np.ndarray) -> Tuple[float, np.ndarray]:
    p = 1.0 / (1.0 + np.exp(-logits))
    eps = 1e-12
    loss = -float(np.mean(y_true * np.log(p + eps) + (1 - y_true) * np.log(1 - p + eps)))
    grad = (p - y_true) / max(1, y_true.size)
    return loss, grad
