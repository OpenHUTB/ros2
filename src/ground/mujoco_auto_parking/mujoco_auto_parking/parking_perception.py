"""Parking-slot perception: relative pose from a 2D LiDAR scan."""

import math
from pathlib import Path
from typing import Tuple

import numpy as np

from .nn_core import NumpyMLP, load_npz

TARGET_SLOT_XY = (0.25, 3.75)
TARGET_SLOT_YAW = -0.5 * math.pi
MAX_RANGE = 8.0
NUM_BEAMS = 120
PERCEPTION_SIZES = (NUM_BEAMS, 64, 64, 4)
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 24
IMAGE_SIZES = (IMAGE_WIDTH * IMAGE_HEIGHT * 3, 128, 64, 4)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def perception_features(
    ranges,
    max_range: float = MAX_RANGE,
    num_beams: int = NUM_BEAMS,
) -> np.ndarray:
    """Normalize and resample a LaserScan range array."""
    values = np.asarray(ranges, dtype=np.float64).reshape(-1)
    if values.size == 0:
        values = np.full(num_beams, max_range)
    if values.size != num_beams:
        source = np.linspace(0.0, 1.0, values.size)
        target = np.linspace(0.0, 1.0, num_beams)
        values = np.interp(target, source, values)
    values = np.nan_to_num(
        values, nan=max_range, posinf=max_range, neginf=0.0
    )
    return np.clip(values, 0.0, max_range) / max_range


def relative_slot_pose(
    x: float,
    y: float,
    yaw: float,
    target_xy: Tuple[float, float] = TARGET_SLOT_XY,
    target_yaw: float = TARGET_SLOT_YAW,
) -> Tuple[float, float, float]:
    """Target slot pose in the car frame: (dx, dy, dtheta)."""
    dx_world = float(target_xy[0]) - float(x)
    dy_world = float(target_xy[1]) - float(y)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx = cos_yaw * dx_world + sin_yaw * dy_world
    dy = -sin_yaw * dx_world + cos_yaw * dy_world
    dtheta = wrap_angle(float(target_yaw) - float(yaw))
    return dx, dy, dtheta


def encode_label(dx: float, dy: float, dtheta: float) -> np.ndarray:
    """Encode a relative pose for tanh regression."""
    return np.array([
        np.clip(dx / 5.0, -1.0, 1.0),
        np.clip(dy / 5.0, -1.0, 1.0),
        math.sin(dtheta),
        math.cos(dtheta),
    ], dtype=np.float64)


def decode_output(output: np.ndarray) -> Tuple[float, float, float]:
    """Decode (dx/5, dy/5, sin, cos) to (dx, dy, dtheta)."""
    values = np.asarray(output, dtype=np.float64).reshape(-1)
    dx = float(values[0]) * 5.0
    dy = float(values[1]) * 5.0
    dtheta = math.atan2(float(values[2]), float(values[3]))
    return dx, dy, dtheta


class PerceptionNetwork:
    """Load the trained MLP and predict the target slot relative pose from LiDAR."""

    def __init__(
        self,
        weights_path: Path | str | None = None,
        sizes: Tuple[int, ...] = PERCEPTION_SIZES,
        zero_weights: bool = False,
    ) -> None:
        if zero_weights:
            model = NumpyMLP(sizes, hidden_act='tanh', output_act='tanh')
            for weight in model.W:
                weight[...] = 0.0
            for bias in model.b:
                bias[...] = 0.0
            self.model = model
            return
        if weights_path is None:
            raise ValueError('weights_path is required unless zero_weights=True')
        payload = load_npz(weights_path)
        self.model = NumpyMLP(
            sizes, hidden_act='tanh', output_act='tanh', weights=payload
        )

    def predict(self, ranges) -> Tuple[float, float, float]:
        features = perception_features(ranges)
        return decode_output(self.model.forward(features.reshape(1, -1))[0])

    def predict_raw(self, ranges) -> np.ndarray:
        features = perception_features(ranges)
        return self.model.forward(features.reshape(1, -1))[0]


def image_features(
    image_rgb,
    width: int = IMAGE_WIDTH,
    height: int = IMAGE_HEIGHT,
) -> np.ndarray:
    """Downsample an RGB image and flatten it to [0, 1] features."""
    image = np.asarray(image_rgb, dtype=np.float32)
    if image.ndim == 2:
        image = np.repeat(image[..., None], 3, axis=2)
    if image.shape[2] > 3:
        image = image[..., :3]
    source_h, source_w = image.shape[:2]
    if source_h < 1 or source_w < 1:
        return np.zeros(width * height * 3, dtype=np.float64)
    ys = np.linspace(0, source_h - 1, height).astype(np.int64)
    xs = np.linspace(0, source_w - 1, width).astype(np.int64)
    small = image[ys][:, xs, :3] / 255.0
    return small.reshape(-1).astype(np.float64)


class ImagePerceptionNetwork:
    """Predict the target slot relative pose from the front camera image."""

    def __init__(
        self,
        weights_path: Path | str | None = None,
        zero_weights: bool = False,
    ) -> None:
        sizes = IMAGE_SIZES
        if zero_weights:
            model = NumpyMLP(sizes, hidden_act='tanh', output_act='tanh')
            for weight in model.W:
                weight[...] = 0.0
            for bias in model.b:
                bias[...] = 0.0
            self.model = model
        else:
            if weights_path is None:
                raise ValueError(
                    'weights_path is required unless zero_weights=True'
                )
            self.model = NumpyMLP(
                sizes,
                hidden_act='tanh',
                output_act='tanh',
                weights=load_npz(weights_path),
            )

    def predict(self, image_rgb) -> Tuple[float, float, float]:
        features = image_features(image_rgb)
        return decode_output(self.model.forward(features.reshape(1, -1))[0])

    def predict_raw(self, image_rgb) -> np.ndarray:
        features = image_features(image_rgb)
        return self.model.forward(features.reshape(1, -1))[0]
