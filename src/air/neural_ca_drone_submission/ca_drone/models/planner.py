"""Neural cost-to-go heuristic used inside A*."""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from ..nn.autograd import Tensor
from ..nn.layers import Linear
from ..nn.optim import Module

PATCH = 11


class NeuralHeuristic(Module):
    """MLP on an 11x11 local occupancy patch plus the goal offset."""

    def __init__(self, patch: int = PATCH, hidden: int = 128):
        self.patch = patch
        n_in = patch * patch + 3
        self.fc1 = Linear(n_in, hidden)
        self.fc2 = Linear(hidden, 64)
        self.out = Linear(64, 1, init="xavier")
        self.out.w.data *= 0.1
        self.out.b.data[:] = 0.0

    def forward(self, x: Tensor) -> Tensor:
        h = self.fc1(x).relu()
        h = self.fc2(h).relu()
        return self.out(h)

    def parameters(self) -> Dict[str, Tensor]:
        return {"fc1.w": self.fc1.w, "fc1.b": self.fc1.b,
                "fc2.w": self.fc2.w, "fc2.b": self.fc2.b,
                "out.w": self.out.w, "out.b": self.out.b}


def _patches(occ: np.ndarray, k: int) -> np.ndarray:
    r = k // 2
    padded = np.pad(occ.astype(np.float32), r, constant_values=1.0)
    from numpy.lib.stride_tricks import sliding_window_view
    sw = sliding_window_view(padded, (k, k))
    return sw.reshape(occ.shape[0] * occ.shape[1], k * k)


def heuristic_features(occ: np.ndarray, goal_cell: Tuple[int, int],
                       res: float = 0.5, patch: int = PATCH) -> np.ndarray:
    nx, ny = occ.shape
    gx, gy = goal_cell
    xs = np.arange(nx)[:, None]
    ys = np.arange(ny)[None, :]
    dx = (gx - xs) * res / 25.0
    dy = (gy - ys) * res / 25.0
    dist = np.sqrt(((gx - xs) * res) ** 2 + ((gy - ys) * res) ** 2) / 50.0
    extra = np.stack([np.broadcast_to(dx, (nx, ny)),
                      np.broadcast_to(dy, (nx, ny)),
                      np.broadcast_to(dist, (nx, ny))], axis=-1).reshape(nx * ny, 3)
    p = _patches(occ, patch)
    return np.concatenate([p, extra], axis=1)


def predict_field(model: NeuralHeuristic, occ: np.ndarray, goal_cell: Tuple[int, int],
                  res: float = 0.5, batch: int = 16384) -> np.ndarray:
    feats = heuristic_features(occ, goal_cell, res, model.patch)
    n = feats.shape[0]
    out = np.empty(n, dtype=np.float32)
    for s in range(0, n, batch):
        x = Tensor(feats[s:s + batch].astype(np.float64))
        out[s:s + batch] = model(x).data[:, 0]
    h = out.reshape(occ.shape) * 50.0
    h[occ] = np.inf
    return np.maximum(h, 0.0)
