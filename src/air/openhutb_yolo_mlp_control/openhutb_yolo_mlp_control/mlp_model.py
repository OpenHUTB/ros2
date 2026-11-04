import numpy as np
import torch.nn as nn


DEFAULT_INPUT_SCALE = np.array(
    [15.0, 15.0, 6.0, 4.0, 4.0, 2.0],
    dtype=np.float32,
)

DEFAULT_OUTPUT_SCALE = np.array(
    [4.0, 4.0, 2.0],
    dtype=np.float32,
)


class MLPController(nn.Module):
    """将六维状态映射为三维速度指令的控制器。"""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(6, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 3),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.net(x)
