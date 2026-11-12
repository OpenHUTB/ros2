"""PyTorch MLP model used by lidar_obstacle_nn.

The model takes a fixed-length polar LiDAR scan as input and predicts
one of three obstacle-proximity classes:
    0  -> ``near``   (obstacle inside the ``danger_distance`` radius)
    1  -> ``far``    (obstacle visible but outside the danger radius)
    2  -> ``free``   (no obstacle within the scan range)

The architecture is intentionally small so it can be trained on a
laptop GPU in a couple of minutes and remains well below the 3 MB
binary-file guidance of the OpenHUTB/ros2 contribution rules.
"""

from __future__ import annotations

import torch
from torch import nn


class LidarMLP(nn.Module):
    """Three-layer MLP for LiDAR scan classification.

    Args:
        input_size:   Number of beams in the input scan (e.g. 360).
        hidden_size:   Width of the two hidden layers.
        num_classes:   Number of output classes (default 3: near/far/free).
        dropout:       Dropout probability applied after each hidden layer.
    """

    def __init__(
        self,
        input_size: int = 360,
        hidden_size: int = 128,
        num_classes: int = 3,
        dropout: float = 0.1,
    ) -> None:
        super().__init__()
        self.input_size = input_size

        self.net = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward pass.

        Args:
            x: FloatTensor of shape ``(batch, input_size)``. Each row is
               one scan. ``inf`` and ``nan`` values are clamped to the
               scan range maximum (treated as "no return").

        Returns:
            Logits of shape ``(batch, num_classes)``.
        """
        # Sanitise ranges coming straight from sensor_msgs/LaserScan.
        x = torch.nan_to_num(x, nan=30.0, posinf=30.0, neginf=0.0)
        x = torch.clamp(x, min=0.0, max=30.0)
        # Normalise to [0, 1] so the network trains stably regardless
        # of whether the scan range is 10 m or 30 m.
        x = x / 30.0
        return self.net(x)

    @torch.no_grad()
    def predict_proba(self, x: torch.Tensor) -> torch.Tensor:
        """Return softmax probabilities for a batch of scans."""
        self.eval()
        logits = self.forward(x)
        return torch.softmax(logits, dim=-1)
