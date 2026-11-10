"""Trained MLP local planner for occupancy-grid navigation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import torch
from torch import nn

from navigation import Cell, OccupancyGrid


@dataclass(frozen=True)
class NeuralAction:
    linear: float
    angular: float
    label: str


class LocalPolicy(nn.Module):
    """Small feed-forward policy over relative goal and local clearance."""

    def __init__(self, hidden_size: int = 16) -> None:
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(5, hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, 3),
        )

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return self.network(features)


class NeuralPlanner:
    """Load and run a label-validated neural steering policy."""

    labels = ("LEFT", "STRAIGHT", "RIGHT")
    feature_names = ("front_free", "left_free", "right_free", "goal_forward", "goal_left")
    _turns = {"LEFT": (0.6, 0.8), "STRAIGHT": (0.6, 0.0), "RIGHT": (0.6, -0.8)}

    def __init__(self, checkpoint: Path | None = None) -> None:
        path = checkpoint or self._default_checkpoint()
        payload: dict[str, Any] = torch.load(path, map_location="cpu", weights_only=True)
        if tuple(payload.get("labels", ())) != self.labels:
            raise ValueError("neural planner checkpoint label order mismatch")
        if tuple(payload.get("feature_names", ())) != self.feature_names:
            raise ValueError("neural planner checkpoint feature contract mismatch")
        if payload.get("version") != 1:
            raise ValueError("unsupported neural planner checkpoint version")
        self.model = LocalPolicy(int(payload.get("hidden_size", 16)))
        self.model.load_state_dict(payload["state_dict"], strict=True)
        self.model.eval()

    @staticmethod
    def _default_checkpoint() -> Path:
        local = Path(__file__).resolve().parent / "models" / "weights" / "neural_navigation.pt"
        if local.is_file():
            return local
        try:
            from ament_index_python.packages import get_package_share_directory
            return Path(get_package_share_directory(
                "mujoco_abandoned_park_uav_visual_steering"
            )) / "models" / "weights" / local.name
        except (ImportError, LookupError):
            return local

    @classmethod
    def features(
        cls, grid: OccupancyGrid, current: Cell, goal: Cell, heading_yaw: float
    ) -> tuple[float, float, float, float, float]:
        fx, fy = math.cos(heading_yaw), math.sin(heading_yaw)
        lx, ly = -fy, fx

        def free(dx: float, dy: float) -> float:
            cell = (current[0] + round(dx), current[1] + round(dy))
            return float(grid.in_bounds(cell) and cell not in grid.occupied)

        dx, dy = goal[0] - current[0], goal[1] - current[1]
        distance = math.hypot(dx, dy)
        if distance:
            dx, dy = dx / distance, dy / distance
        return (free(fx, fy), free(lx, ly), free(-lx, -ly), dx * fx + dy * fy, dx * lx + dy * ly)

    def predict(
        self, grid: OccupancyGrid, current: Cell, goal: Cell, heading_yaw: float = 0.0
    ) -> NeuralAction:
        features = self.features(grid, current, goal, heading_yaw)
        with torch.inference_mode():
            logits = self.model(torch.tensor(features, dtype=torch.float32))
        available = torch.tensor(features[:3], dtype=torch.bool)
        if not bool(available.any()):
            return NeuralAction(0.0, 0.0, "STOP")
        logits = logits.masked_fill(~available, -torch.inf)
        label = self.labels[int(logits.argmax().item())]
        linear, angular = self._turns[label]
        return NeuralAction(linear, angular, label)
