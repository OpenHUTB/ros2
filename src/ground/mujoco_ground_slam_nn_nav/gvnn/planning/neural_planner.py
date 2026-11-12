"""Neural local planner: local occupancy patch + goal -> 8-way waypoint."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from ..datasets import DIRECTION_BINS, goal_polar, local_patch
from ..nn.mlp import MLP


class NeuralPlanner:
    def __init__(self, model_path: str):
        self.model = MLP.load(model_path)

    def waypoint(self, pose: Sequence[float], occupied: np.ndarray,
                 unknown: np.ndarray, goal: Sequence[float],
                 distance: float = 2.5) -> Tuple[Optional[Tuple[float, float]], int, float]:
        patch = local_patch(occupied, unknown, pose, size=21, resolution=0.2)
        features = np.concatenate([patch.reshape(-1), goal_polar(pose, goal)])[None, :]
        logits = self.model.predict(features)[0]
        idx = int(np.argmax(logits))
        z = logits - logits.max()
        p = np.exp(z) / np.exp(z).sum()
        angle = idx * (2.0 * math.pi / DIRECTION_BINS)
        wx = float(pose[0]) + distance * math.cos(float(pose[2]) + angle)
        wy = float(pose[1]) + distance * math.sin(float(pose[2]) + angle)
        return (wx, wy), idx, float(p[idx])
