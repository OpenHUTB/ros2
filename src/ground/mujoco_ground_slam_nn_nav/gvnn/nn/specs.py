"""Network architectures used by the four learning tasks."""

from __future__ import annotations

from .cnn import CNN
from .mlp import MLP


E2E_SPEC = [
    {"type": "conv", "cin": 3, "cout": 8, "k": 5, "s": 2, "p": 0, "act": "relu"},
    {"type": "pool", "k": 2},
    {"type": "conv", "cin": 8, "cout": 16, "k": 3, "s": 1, "p": 1, "act": "relu"},
    {"type": "pool", "k": 2},
    {"type": "flatten"},
    {"type": "dense", "nin": 16 * 7 * 7, "nout": 64, "act": "relu"},
    {"type": "dense", "nin": 64, "nout": 2, "act": "tanh"},
]


def e2e_cnn(seed: int = 0) -> CNN:
    return CNN(E2E_SPEC, seed=seed)


def perception_mlp(seed: int = 0) -> MLP:
    return MLP([360, 256, 256, 360], activation="relu", out_activation="linear", seed=seed)


# 1-D convolutional denoiser implemented as a 1-row 2-D CNN.  The fully
# convolutional structure preserves the 360-beam angular ordering.
PERCEPTION_SPEC = [
    {"type": "conv", "cin": 1, "cout": 8, "k": 7, "s": 1, "p": 3, "act": "relu"},
    {"type": "conv", "cin": 8, "cout": 8, "k": 5, "s": 1, "p": 2, "act": "relu"},
    {"type": "conv", "cin": 8, "cout": 1, "k": 3, "s": 1, "p": 1, "act": "linear"},
]


def perception_cnn(seed: int = 0) -> CNN:
    return CNN(PERCEPTION_SPEC, seed=seed)


def control_mlp(seed: int = 0) -> MLP:
    return MLP([5, 64, 64, 2], activation="relu", out_activation="tanh", seed=seed)


def planner_mlp(seed: int = 0) -> MLP:
    return MLP([21 * 21 + 3, 256, 128, 8], activation="relu", out_activation="linear", seed=seed)


def control_features(pose, path, front_clearance: float, speed: float,
                     pure_pursuit) -> list:
    """Feature vector used by both the control expert and the NN controller."""
    cmd = pure_pursuit.control(pose, path, front_clearance)
    return [
        float(speed) / 1.1,
        float(cmd["cross_track"]),
        float(cmd["heading_error"]) / 3.141592653589793,
        float(cmd["curvature"]) / 2.0,
        float(front_clearance) / 9.0,
    ]
