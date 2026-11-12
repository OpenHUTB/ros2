"""Adam optimiser operating on plain NumPy parameter / gradient lists."""

from __future__ import annotations

from typing import List, Sequence

import numpy as np


class Adam:
    """Adam with bias correction; parameters are updated in place."""

    def __init__(self, params: Sequence[np.ndarray], lr: float = 1e-3,
                 beta1: float = 0.9, beta2: float = 0.999,
                 eps: float = 1e-8, weight_decay: float = 0.0):
        self.params: List[np.ndarray] = list(params)
        self.lr = float(lr)
        self.beta1 = float(beta1)
        self.beta2 = float(beta2)
        self.eps = float(eps)
        self.weight_decay = float(weight_decay)
        self.m = [np.zeros_like(p) for p in self.params]
        self.v = [np.zeros_like(p) for p in self.params]
        self.t = 0

    def step(self, grads: Sequence[np.ndarray]) -> None:
        if len(grads) != len(self.params):
            raise ValueError("gradient count does not match parameter count")
        self.t += 1
        b1, b2 = self.beta1, self.beta2
        c1 = 1.0 - b1 ** self.t
        c2 = 1.0 - b2 ** self.t
        for p, g, m, v in zip(self.params, grads, self.m, self.v):
            g = np.asarray(g, dtype=np.float64)
            if self.weight_decay > 0.0:
                g = g + self.weight_decay * p
            m *= b1
            m += (1.0 - b1) * g
            v *= b2
            v += (1.0 - b2) * (g * g)
            m_hat = m / c1
            v_hat = v / c2
            p -= self.lr * m_hat / (np.sqrt(v_hat) + self.eps)

    def zero_grad(self) -> None:
        for m, v in zip(self.m, self.v):
            m[...] = 0.0
            v[...] = 0.0
