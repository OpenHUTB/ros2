"""End-to-end depth-to-command CNN."""

from __future__ import annotations

from typing import Dict

import numpy as np

from ..nn.autograd import Tensor, concat, maxpool2d
from ..nn.layers import Conv2d, Linear
from ..nn.optim import Module


class E2ENet(Module):
    """1x48x64 depth + goal vector -> body (vx, vy) command."""

    def __init__(self):
        self.c1 = Conv2d(1, 16, 3, stride=2)
        self.c2 = Conv2d(16, 32, 3, stride=2)
        self.c3 = Conv2d(32, 32, 3, stride=2)
        self.c4 = Conv2d(32, 48, 3, stride=2)
        self.fc1 = Linear(48 * 3 * 4 + 3, 128)
        self.fc2 = Linear(128, 64)
        self.out = Linear(64, 2, init="xavier")
        self.out.w.data *= 0.1
        self.out.b.data[:] = 0.0

    def forward(self, depth: Tensor, goal: Tensor) -> Tensor:
        h = self.c1(depth).relu()
        h = self.c2(h).relu()
        h = self.c3(h).relu()
        h = self.c4(h).relu()
        b = h.shape[0]
        h = h.reshape(b, -1)
        h = concat([h, goal], axis=1)
        h = self.fc1(h).relu()
        h = self.fc2(h).relu()
        y = self.out(h).tanh()
        return y * Tensor(np.array([1.5, 1.5]))

    def parameters(self) -> Dict[str, Tensor]:
        d = {}
        for name in ("c1", "c2", "c3", "c4", "fc1", "fc2", "out"):
            for k, v in getattr(self, name).parameters().items():
                d[f"{name}.{k}"] = v
        return d


def goal_features(goal_body: np.ndarray, dist: float) -> np.ndarray:
    g = np.clip(np.asarray(goal_body, dtype=np.float32) / 10.0, -1.0, 1.0)
    return np.concatenate([g, [min(dist, 50.0) / 50.0]]).astype(np.float32)
