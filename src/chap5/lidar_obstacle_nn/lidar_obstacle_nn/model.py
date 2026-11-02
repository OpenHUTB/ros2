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

from lidar_obstacle_nn.scan_utils import DEFAULT_SCAN_MAX


def normalise_scan(x: torch.Tensor, scan_max: float = DEFAULT_SCAN_MAX) -> torch.Tensor:
    """Sanitize ranges and scale them to the network's 0..1 input range."""
    if scan_max <= 0:
        raise ValueError('scan_max must be positive')
    x = torch.nan_to_num(
        x,
        nan=float(scan_max),
        posinf=float(scan_max),
        neginf=0.0,
    )
    return torch.clamp(x, min=0.0, max=float(scan_max)) / float(scan_max)


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
        # Sanitise ranges coming straight from sensor_msgs/LaserScan and
        # normalise so the network trains stably across scan-range limits.
        x = normalise_scan(x)
        return self.net(x)

    @torch.no_grad()
    def predict_proba(self, x: torch.Tensor) -> torch.Tensor:
        """Return softmax probabilities for a batch of scans."""
        self.eval()
        logits = self.forward(x)
        return torch.softmax(logits, dim=-1)
