"""Imitation-learned velocity controller (the control module)."""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from ..nn.autograd import Tensor
from ..nn.layers import Linear
from ..nn.optim import Module

PATCH = 9
MAX_SPEED = 1.5


class ControlNet(Module):
    """MLP: local occupancy + goal/velocity errors -> body velocity command."""

    def __init__(self, patch: int = PATCH, hidden: int = 128):
        self.patch = patch
        n_in = patch * patch + 5
        self.fc1 = Linear(n_in, hidden)
        self.fc2 = Linear(hidden, hidden)
        self.out = Linear(hidden, 2, init="xavier")
        self.out.w.data *= 0.1
        self.out.b.data[:] = 0.0

    def forward(self, x: Tensor) -> Tensor:
        h = self.fc1(x).relu()
        h = self.fc2(h).relu()
        return (self.out(h)).tanh() * MAX_SPEED

    def parameters(self) -> Dict[str, Tensor]:
        return {"fc1.w": self.fc1.w, "fc1.b": self.fc1.b,
                "fc2.w": self.fc2.w, "fc2.b": self.fc2.b,
                "out.w": self.out.w, "out.b": self.out.b}


def control_features(patch: np.ndarray, goal_body: np.ndarray, vel_body: np.ndarray,
                     clearance: float) -> np.ndarray:
    g = np.clip(np.asarray(goal_body, dtype=np.float32) / 5.0, -1.0, 1.0)
    v = np.clip(np.asarray(vel_body, dtype=np.float32) / MAX_SPEED, -1.0, 1.0)
    c = np.array([min(float(clearance), 3.0) / 3.0], dtype=np.float32)
    return np.concatenate([g, v, patch.reshape(-1).astype(np.float32), c])


def predict_command(model: ControlNet, x: np.ndarray) -> np.ndarray:
    return model(Tensor(x[None, :])).data[0]
