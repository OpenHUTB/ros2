"""生成确定性的仿真灰度观测，并训练图像到控制指令的 MLP。"""

from __future__ import annotations

import math

import numpy as np


def render_state_observation(
    ego_state: tuple[float, float, float, float],
    traffic_states: np.ndarray | list[tuple[float, float, float, float]],
    lane_offset_m: float = 0.0,
    image_size: int = 16,
    forward_range_m: float = 24.0,
    lateral_range_m: float = 12.0,
) -> np.ndarray:
    """Render a deterministic bird's-eye grayscale observation from simulated state.

    ``ego_state`` is ``(x, y, yaw, speed)`` and each traffic row is
    ``(x, y, vx, vy)`` in world coordinates. The image is a synthetic sensor
    representation, not a camera render from a physics engine.
    """
    if image_size < 4 or forward_range_m <= 0 or lateral_range_m <= 0:
        raise ValueError("image_size must be >= 4 and observation ranges must be positive")
    traffic = np.asarray(traffic_states, dtype=np.float64)
    if traffic.size == 0:
        traffic = traffic.reshape(0, 4)
    if traffic.ndim != 2 or traffic.shape[1] != 4:
        raise ValueError("traffic_states must have shape Nx4: x, y, vx, vy")

    x, y, yaw, speed = map(float, ego_state)
    image = np.full((image_size, image_size), 24, dtype=np.uint8)
    forward_step = forward_range_m / image_size
    lateral_step = lateral_range_m / image_size

    for row in range(1, image_size - 1):
        forward = (image_size - row - 0.5) * forward_step
        center = -lane_offset_m - forward * math.tan(yaw)
        for column in range(image_size):
            lateral = (column + 0.5) * lateral_step - lateral_range_m / 2.0
            if abs(lateral - center) <= 3.5:
                image[row, column] = 72
            if abs(abs(lateral - center) - 3.5) <= lateral_step * 0.55:
                image[row, column] = 132

    speed_columns = int(np.clip(speed / 12.0, 0.0, 1.0) * image_size + 0.5)
    image[0, :] = 32
    image[0, :speed_columns] = 176

    cosine, sine = math.cos(yaw), math.sin(yaw)
    for tx, ty, vx, vy in traffic:
        dx, dy = tx - x, ty - y
        forward = dx * cosine + dy * sine
        lateral = -dx * sine + dy * cosine
        if not (0.0 <= forward <= forward_range_m and abs(lateral) <= lateral_range_m / 2.0):
            continue
        closing_speed = speed - (vx * cosine + vy * sine)
        intensity = int(np.clip(176.0 + closing_speed * 5.0, 112.0, 248.0))
        row = image_size - 1 - int(forward / forward_step)
        column = int((lateral + lateral_range_m / 2.0) / lateral_step)
        row_radius = max(1, int(math.ceil(1.3 / forward_step)))
        column_radius = max(1, int(math.ceil(0.9 / lateral_step)))
        image[max(1, row - row_radius):min(image_size - 1, row + row_radius + 1),
              max(0, column - column_radius):min(image_size, column + column_radius + 1)] = intensity

    image[-1, image_size // 2] = 255
    return image


def _teacher_control(
    speed_mps: float,
    lane_offset_m: float,
    yaw_error_rad: float,
    local_traffic: list[tuple[float, float, float]],
) -> np.ndarray:
    steer = float(np.clip(-0.55 * yaw_error_rad - 0.18 * lane_offset_m, -1.0, 1.0))
    danger_ttc = math.inf
    nearest_clearance = math.inf
    for forward, lateral, closing_speed in local_traffic:
        if forward <= 0.0 or abs(lateral) > 2.8:
            continue
        clearance = max(0.0, forward - 2.2)
        nearest_clearance = min(nearest_clearance, clearance)
        if closing_speed > 1e-3:
            danger_ttc = min(danger_ttc, clearance / closing_speed)

    if nearest_clearance < 2.0 or danger_ttc < 1.6:
        throttle, brake = 0.0, 1.0
    elif nearest_clearance < 6.0 or danger_ttc < 3.0:
        throttle, brake = 0.1, 0.45
    else:
        throttle = float(np.clip((8.0 - speed_mps) / 8.0, 0.0, 0.75))
        brake = 0.0
    return np.array([steer, throttle, brake], dtype=np.float64)


def generate_teacher_dataset(
    seed: int = 7,
    samples: int = 256,
    image_size: int = 16,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate reproducible image/control pairs from randomized simulated states.

    Labels come from a compact geometric driving rule. They are suitable for
    exercising the image-to-control pipeline, not for claiming real-world skill.
    """
    if samples < 8:
        raise ValueError("samples must be at least 8")
    rng = np.random.default_rng(seed)
    images = np.empty((samples, image_size, image_size), dtype=np.uint8)
    controls = np.empty((samples, 3), dtype=np.float64)
    for index in range(samples):
        lane_offset = float(rng.uniform(-1.4, 1.4))
        yaw = float(rng.uniform(-0.24, 0.24))
        speed = float(rng.uniform(0.0, 12.0))
        ego = (0.0, 0.0, yaw, speed)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        local_traffic: list[tuple[float, float, float]] = []
        world_traffic: list[tuple[float, float, float, float]] = []
        for _ in range(int(rng.integers(0, 5))):
            forward = float(rng.uniform(2.5, 23.0))
            lateral = float(rng.uniform(-5.5, 5.5))
            closing_speed = float(rng.uniform(-2.0, max(0.5, speed + 1.0)))
            traffic_speed = max(0.0, speed - closing_speed)
            tx = forward * cosine - lateral * sine
            ty = forward * sine + lateral * cosine
            vx, vy = traffic_speed * cosine, traffic_speed * sine
            local_traffic.append((forward, lateral, closing_speed))
            world_traffic.append((tx, ty, vx, vy))
        images[index] = render_state_observation(
            ego, world_traffic, lane_offset_m=lane_offset, image_size=image_size
        )
        controls[index] = _teacher_control(speed, lane_offset, yaw, local_traffic)
    return images, controls


class ImageToControl:
    """16x16 灰度前视图像 -> (steering, throttle, brake) 的两层 MLP。

    输入图像形状为 (N, 16, 16) 或 (N, 256)，像素归一化到 [0,1]。
    输出形状 (N, 3)：steering ∈ [-1,1]，throttle/brake ∈ [0,1]。
    """

    def __init__(self, pixels: int = 16 * 16, hidden: int = 24, seed: int = 11) -> None:
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0, 0.08, (pixels, hidden))
        self.b1 = np.zeros(hidden)
        self.w2 = rng.normal(0, 0.08, (hidden, 3))
        self.b2 = np.zeros(3)

    def _features(self, images: np.ndarray) -> np.ndarray:
        x = np.asarray(images, dtype=np.float64).reshape(len(images), -1) / 255.0
        if x.shape[1] != self.w1.shape[0]:
            raise ValueError(f"expected {self.w1.shape[0]} pixels per image, got {x.shape[1]}")
        return x

    def predict(self, images: np.ndarray) -> np.ndarray:
        x = self._features(images)
        hidden = np.tanh(x @ self.w1 + self.b1)
        raw = np.tanh(hidden @ self.w2 + self.b2)
        steering = raw[:, 0]
        throttle = (raw[:, 1] + 1.0) / 2.0
        brake = (raw[:, 2] + 1.0) / 2.0
        return np.column_stack((steering, throttle, brake))

    def train(
        self,
        images: np.ndarray,
        controls: np.ndarray,
        epochs: int = 200,
        learning_rate: float = 0.02,
    ) -> list[float]:
        x = self._features(images)
        y = np.asarray(controls, dtype=np.float64).copy()
        if x.shape[1] != self.w1.shape[0] or y.shape != (len(x), 3):
            raise ValueError("expected N grayscale images and an Nx3 control matrix")
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
            raise ValueError("images and controls must contain finite values")
        y[:, 1:] = y[:, 1:] * 2.0 - 1.0
        losses: list[float] = []
        for _ in range(epochs):
            hidden = np.tanh(x @ self.w1 + self.b1)
            output = np.tanh(hidden @ self.w2 + self.b2)
            losses.append(float(np.mean((output - y) ** 2)))
            grad = 2.0 * (output - y) * (1.0 - output * output) / len(x)
            gw2, gb2 = hidden.T @ grad, grad.sum(axis=0)
            gh = (grad @ self.w2.T) * (1.0 - hidden * hidden)
            self.w2 -= learning_rate * gw2
            self.b2 -= learning_rate * gb2
            self.w1 -= learning_rate * (x.T @ gh)
            self.b1 -= learning_rate * gh.sum(axis=0)
        return losses

    def save(self, path: str) -> None:
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)

    @classmethod
    def load(cls, path: str) -> "ImageToControl":
        data = np.load(path)
        model = cls(pixels=data["w1"].shape[0], hidden=data["w1"].shape[1])
        model.w1, model.b1 = data["w1"], data["b1"]
        model.w2, model.b2 = data["w2"], data["b2"]
        return model

    def evaluate(self, images: np.ndarray, controls: np.ndarray) -> dict[str, float]:
        """Return regression errors for an explicit, held-out label set."""
        expected = np.asarray(controls, dtype=np.float64)
        predicted = self.predict(images)
        if expected.shape != predicted.shape or len(expected) == 0:
            raise ValueError("controls must be a non-empty Nx3 matrix matching images")
        if not np.all(np.isfinite(expected)):
            raise ValueError("controls must contain finite values")
        errors = np.abs(predicted - expected)
        return {
            "mae": float(errors.mean()),
            "mse": float(np.square(errors).mean()),
            "steering_mae": float(errors[:, 0].mean()),
            "throttle_mae": float(errors[:, 1].mean()),
            "brake_mae": float(errors[:, 2].mean()),
        }


def train_and_evaluate_teacher(
    seed: int = 7,
    samples: int = 256,
    image_size: int = 16,
    hidden: int = 24,
    epochs: int = 400,
) -> tuple[ImageToControl, dict[str, float | int]]:
    """Train on one deterministic split and return held-out teacher metrics."""
    images, controls = generate_teacher_dataset(seed=seed, samples=samples, image_size=image_size)
    split_rng = np.random.default_rng(seed + 1)
    order = split_rng.permutation(samples)
    split = max(4, min(samples - 4, int(samples * 0.8)))
    train_indices, test_indices = order[:split], order[split:]
    model = ImageToControl(pixels=image_size * image_size, hidden=hidden, seed=seed)
    model.train(images[train_indices], controls[train_indices], epochs=epochs)
    train_metrics = model.evaluate(images[train_indices], controls[train_indices])
    test_metrics = model.evaluate(images[test_indices], controls[test_indices])
    metrics: dict[str, float | int] = {
        "train_samples": len(train_indices),
        "test_samples": len(test_indices),
        **{f"train_{key}": value for key, value in train_metrics.items()},
        **{f"test_{key}": value for key, value in test_metrics.items()},
    }
    return model, metrics
