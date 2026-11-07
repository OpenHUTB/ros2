"""间隙接受多层感知机。"""

from __future__ import annotations

import numpy as np


class GapPolicy:
    """Classify a radar snapshot as WAIT or ENTER using a small MLP."""

    def __init__(self, input_size: int = 4, hidden_size: int = 12, seed: int = 7) -> None:
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0.0, 0.2, (input_size, hidden_size))
        self.b1 = np.zeros(hidden_size)
        self.w2 = rng.normal(0.0, 0.2, (hidden_size, 2))
        self.b2 = np.zeros(2)

    def probabilities(self, features: np.ndarray) -> np.ndarray:
        x = np.atleast_2d(np.asarray(features, dtype=np.float64))
        hidden = np.tanh(x @ self.w1 + self.b1)
        logits = hidden @ self.w2 + self.b2
        logits -= logits.max(axis=1, keepdims=True)
        exp = np.exp(logits)
        return exp / exp.sum(axis=1, keepdims=True)

    def train(self, features: np.ndarray, labels: np.ndarray, epochs: int = 6000, learning_rate: float = 0.1) -> float:
        x = np.asarray(features, dtype=np.float64)
        y = np.asarray(labels, dtype=np.int64)
        if x.ndim != 2 or y.shape != (len(x),) or len(x) == 0:
            raise ValueError("features must be NxD and labels must contain N classes")
        target = np.eye(2)[y]
        for _ in range(epochs):
            hidden = np.tanh(x @ self.w1 + self.b1)
            logits = hidden @ self.w2 + self.b2
            logits -= logits.max(axis=1, keepdims=True)
            probs = np.exp(logits)
            probs /= probs.sum(axis=1, keepdims=True)
            grad = (probs - target) / len(x)
            gw2, gb2 = hidden.T @ grad, grad.sum(axis=0)
            gh = (grad @ self.w2.T) * (1.0 - hidden * hidden)
            gw1, gb1 = x.T @ gh, gh.sum(axis=0)
            self.w2 -= learning_rate * gw2
            self.b2 -= learning_rate * gb2
            self.w1 -= learning_rate * gw1
            self.b1 -= learning_rate * gb1
        return float((self.probabilities(x).argmax(axis=1) == y).mean())

    def decide(self, features: np.ndarray, threshold: float = 0.65) -> str:
        probs = self.probabilities(features)[0]
        return "ENTER" if probs[1] >= threshold else "WAIT"
