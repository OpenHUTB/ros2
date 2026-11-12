"""Optimiser, parameter containers and (de)serialisation helpers."""

from __future__ import annotations

import json
import os
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

import numpy as np

from .autograd import Tensor


class Module:
    """Base class: subclasses return an ordered ``name -> Tensor`` mapping."""

    def parameters(self) -> Dict[str, Tensor]:  # pragma: no cover - interface
        raise NotImplementedError

    def __call__(self, *args, **kwargs):
        return self.forward(*args, **kwargs)

    def forward(self, *args, **kwargs):  # pragma: no cover
        raise NotImplementedError

    def state_dict(self) -> Dict[str, np.ndarray]:
        return {k: v.data.copy() for k, v in self.parameters().items()}

    def load_state_dict(self, state: Mapping[str, np.ndarray]):
        params = self.parameters()
        for k, v in state.items():
            if k not in params:
                raise KeyError(f"unexpected parameter {k!r}")
            if params[k].data.shape != np.asarray(v).shape:
                raise ValueError(f"shape mismatch for {k}: {params[k].data.shape} vs {np.asarray(v).shape}")
            params[k].data[...] = np.asarray(v)

    def num_params(self) -> int:
        return int(sum(v.data.size for v in self.parameters().values()))

    def save(self, path: str, meta: Optional[dict] = None):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        arrays = {f"p_{k}": v.data for k, v in self.parameters().items()}
        if meta:
            arrays["meta_json"] = np.array(json.dumps(meta, ensure_ascii=False))
        np.savez_compressed(path, **arrays)

    @classmethod
    def load(cls, path: str):
        data = np.load(path, allow_pickle=False)
        model = cls()
        state = {k[2:]: data[k] for k in data.files if k.startswith("p_")}
        model.load_state_dict(state)
        meta = {}
        if "meta_json" in data.files:
            meta = json.loads(str(data["meta_json"]))
        return model, meta


class Adam:
    def __init__(self, params: Mapping[str, Tensor], lr: float = 1e-3,
                 betas: Tuple[float, float] = (0.9, 0.999), eps: float = 1e-8,
                 weight_decay: float = 0.0, clip_norm: float = 5.0):
        self.params = params
        self.lr = lr
        self.b1, self.b2 = betas
        self.eps = eps
        self.wd = weight_decay
        self.clip_norm = clip_norm
        self.m = {k: np.zeros_like(v.data) for k, v in params.items()}
        self.v = {k: np.zeros_like(v.data) for k, v in params.items()}
        self.t = 0

    def zero_grad(self):
        for p in self.params.values():
            p.grad = None

    def step(self) -> float:
        self.t += 1
        grads = []
        total = 0.0
        for k, p in self.params.items():
            if p.grad is None:
                continue
            g = p.grad
            grads.append((k, g))
            total += float(np.sum(g * g))
        norm = float(np.sqrt(total))
        scale = 1.0
        if self.clip_norm and norm > self.clip_norm:
            scale = self.clip_norm / (norm + 1e-12)
        for k, g in grads:
            p = self.params[k]
            g = g * scale
            if self.wd:
                g = g + self.wd * p.data
            self.m[k] = self.b1 * self.m[k] + (1 - self.b1) * g
            self.v[k] = self.b2 * self.v[k] + (1 - self.b2) * (g * g)
            mh = self.m[k] / (1 - self.b1 ** self.t)
            vh = self.v[k] / (1 - self.b2 ** self.t)
            p.data -= self.lr * mh / (np.sqrt(vh) + self.eps)
        return norm


def iterate_batches(n: int, batch: int, rng: np.random.Generator, shuffle: bool = True):
    idx = np.arange(n)
    if shuffle:
        rng.shuffle(idx)
    for start in range(0, n, batch):
        yield idx[start:start + batch]
