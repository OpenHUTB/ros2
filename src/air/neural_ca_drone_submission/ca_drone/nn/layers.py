"""Reusable layers built on the local autodiff engine."""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

from .autograd import Tensor, conv2d, he_normal, maxpool2d, upsample2d, xavier
from .optim import Module


class Linear(Module):
    def __init__(self, n_in: int, n_out: int, rng: Optional[np.random.Generator] = None,
                 bias: bool = True, init: str = "he"):
        rng = rng or np.random.default_rng(0)
        w = he_normal((n_out, n_in), rng=rng) if init == "he" else xavier((n_out, n_in), rng=rng)
        self.w = Tensor(w, requires_grad=True)
        self.b = Tensor(np.zeros(n_out), requires_grad=True) if bias else None

    def forward(self, x: Tensor) -> Tensor:
        out = x @ self.w.transpose()
        if self.b is not None:
            out = out + self.b
        return out

    def parameters(self) -> Dict[str, Tensor]:
        d = {"w": self.w}
        if self.b is not None:
            d["b"] = self.b
        return d


class Conv2d(Module):
    def __init__(self, c_in: int, c_out: int, k: int = 3, stride: int = 1,
                 padding: Optional[int] = None, rng: Optional[np.random.Generator] = None,
                 bias: bool = True):
        rng = rng or np.random.default_rng(0)
        padding = k // 2 if padding is None else padding
        self.stride = stride
        self.padding = padding
        self.w = Tensor(he_normal((c_out, c_in, k, k), rng=rng), requires_grad=True)
        self.b = Tensor(np.zeros(c_out), requires_grad=True) if bias else None

    def forward(self, x: Tensor) -> Tensor:
        return conv2d(x, self.w, self.b, stride=self.stride, padding=self.padding)

    def parameters(self) -> Dict[str, Tensor]:
        d = {"w": self.w}
        if self.b is not None:
            d["b"] = self.b
        return d


class MLP(Module):
    def __init__(self, sizes, rng: Optional[np.random.Generator] = None, out_init: float = 0.01):
        rng = rng or np.random.default_rng(0)
        self.layers = []
        for i in range(len(sizes) - 1):
            lin = Linear(sizes[i], sizes[i + 1], rng=rng,
                         init="xavier" if i == len(sizes) - 2 else "he")
            if i == len(sizes) - 2 and lin.b is not None:
                lin.b.data *= 0.0
                lin.w.data *= out_init / max(1e-6, float(np.std(lin.w.data)))
            self.layers.append(lin)

    def forward(self, x: Tensor) -> Tensor:
        h = x
        for i, lin in enumerate(self.layers):
            h = lin(h)
            if i < len(self.layers) - 1:
                h = h.relu()
        return h

    def parameters(self) -> Dict[str, Tensor]:
        d = {}
        for i, lin in enumerate(self.layers):
            for k, v in lin.parameters().items():
                d[f"l{i}.{k}"] = v
        return d


__all__ = ["Linear", "Conv2d", "MLP", "maxpool2d", "upsample2d"]
