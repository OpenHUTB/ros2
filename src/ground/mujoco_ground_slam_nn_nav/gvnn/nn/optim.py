"""Adam optimiser for dictionary-based parameter sets."""

from __future__ import annotations

from typing import Dict

import numpy as np


class Adam:
    def __init__(self, params: Dict[str, np.ndarray], lr: float = 1e-3,
                 beta1: float = 0.9, beta2: float = 0.999, eps: float = 1e-8):
        self.params = params
        self.lr = float(lr)
        self.beta1 = float(beta1)
        self.beta2 = float(beta2)
        self.eps = float(eps)
        self.m = {k: np.zeros_like(v) for k, v in params.items()}
        self.v = {k: np.zeros_like(v) for k, v in params.items()}
        self.t = 0

    def step(self, grads: Dict[str, np.ndarray]) -> None:
        self.t += 1
        b1t = 1.0 - self.beta1 ** self.t
        b2t = 1.0 - self.beta2 ** self.t
        for k, p in self.params.items():
            g = grads.get(k)
            if g is None:
                continue
            self.m[k] = self.beta1 * self.m[k] + (1.0 - self.beta1) * g
            self.v[k] = self.beta2 * self.v[k] + (1.0 - self.beta2) * (g * g)
            mhat = self.m[k] / b1t
            vhat = self.v[k] / b2t
            p -= self.lr * mhat / (np.sqrt(vhat) + self.eps)
