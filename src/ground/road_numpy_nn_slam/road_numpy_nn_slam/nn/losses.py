"""Small loss / activation helpers shared by the training tools."""

from __future__ import annotations

import numpy as np


def mse(pred: np.ndarray, target: np.ndarray) -> float:
    d = np.asarray(pred, dtype=np.float64) - np.asarray(target, dtype=np.float64)
    return float(np.mean(d * d))


def mae(pred: np.ndarray, target: np.ndarray) -> float:
    d = np.asarray(pred, dtype=np.float64) - np.asarray(target, dtype=np.float64)
    return float(np.mean(np.abs(d)))


def rmse(pred: np.ndarray, target: np.ndarray) -> float:
    return float(np.sqrt(mse(pred, target)))


def softmax(logits: np.ndarray) -> np.ndarray:
    z = np.asarray(logits, dtype=np.float64)
    z = z - z.max(axis=-1, keepdims=True)
    e = np.exp(z)
    return e / e.sum(axis=-1, keepdims=True)


def softmax_cross_entropy(logits: np.ndarray, labels: np.ndarray):
    """Return (loss, dLogits) for integer labels, batch averaged."""
    logits = np.asarray(logits, dtype=np.float64)
    labels = np.asarray(labels, dtype=np.int64).reshape(-1)
    probs = softmax(logits)
    n = probs.shape[0]
    picked = np.clip(probs[np.arange(n), labels], 1e-12, 1.0)
    loss = float(-np.mean(np.log(picked)))
    grad = probs.copy()
    grad[np.arange(n), labels] -= 1.0
    return loss, grad / n
