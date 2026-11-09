"""Neural trajectory tracking for a differential-drive car."""

import math
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

from .nn_core import NumpyMLP, load_npz

CONTROL_SIZES = (5, 64, 64, 2)
V_REF_DEFAULT = 0.45
LOOKAHEAD_DEFAULT = 0.8


def wrap_angle(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def build_s_curve(
    start_x: float,
    start_y: float,
    start_yaw: float,
    length: float = 4.0,
    amplitude: float = 1.0,
    num_points: int = 200,
) -> np.ndarray:
    """Build a smooth S-curve in world coordinates: Nx3 (x, y, yaw)."""
    s = np.linspace(0.0, float(length), int(num_points))
    y_local = amplitude * np.sin(np.pi * s / float(length))
    dy_ds = amplitude * np.pi / float(length) * np.cos(
        np.pi * s / float(length)
    )
    yaw_local = np.arctan2(dy_ds, 1.0)
    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)
    x = start_x + cos_yaw * s - sin_yaw * y_local
    y = start_y + sin_yaw * s + cos_yaw * y_local
    yaw = (
        (start_yaw + yaw_local + math.pi) % (2.0 * math.pi) - math.pi
    )
    return np.column_stack([x, y, yaw])


class ReferenceTrajectory:
    """A polyline reference with nearest-point and lookahead queries."""

    def __init__(self, points: np.ndarray) -> None:
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
            raise ValueError('trajectory must be Nx3 with at least two points')
        self.points = points

    def nearest(self, x: float, y: float) -> Tuple[int, float, np.ndarray]:
        distance = np.hypot(
            self.points[:, 0] - float(x), self.points[:, 1] - float(y)
        )
        index = int(np.argmin(distance))
        return index, float(distance[index]), self.points[index]

    def lookahead(
        self,
        x: float,
        y: float,
        distance: float = LOOKAHEAD_DEFAULT,
    ) -> Tuple[int, float, np.ndarray]:
        index, nearest_distance, _ = self.nearest(x, y)
        accumulated = 0.0
        cursor = index
        while cursor < len(self.points) - 1 and accumulated < distance:
            step = np.hypot(
                self.points[cursor + 1, 0] - self.points[cursor, 0],
                self.points[cursor + 1, 1] - self.points[cursor, 1],
            )
            accumulated += float(step)
            cursor += 1
        return cursor, nearest_distance, self.points[cursor]


class TrajectoryController:
    """MLP that maps tracking errors to (v, yaw-rate) commands."""

    def __init__(
        self,
        weights_path: Path | str | None = None,
        zero_weights: bool = False,
    ) -> None:
        sizes = CONTROL_SIZES
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

    @staticmethod
    def features(state, reference, v_ref: float) -> np.ndarray:
        x, y, yaw, velocity = state
        ref_x, ref_y, ref_yaw = reference
        dx = float(ref_x) - float(x)
        dy = float(ref_y) - float(y)
        cos_yaw = math.cos(float(yaw))
        sin_yaw = math.sin(float(yaw))
        ex = cos_yaw * dx + sin_yaw * dy
        ey = -sin_yaw * dx + cos_yaw * dy
        epsi = wrap_angle(float(ref_yaw) - float(yaw))
        return np.array([
            ex / 1.0,
            ey / 0.5,
            epsi / math.pi,
            float(velocity) / 1.0,
            float(v_ref) / 0.6,
        ], dtype=np.float64)

    @staticmethod
    def expert_from_features(
        features: np.ndarray,
        v_ref: float,
    ) -> Tuple[float, float]:
        ey = float(features[1]) * 0.5
        epsi = float(features[2]) * math.pi
        alpha = wrap_angle(epsi + math.atan2(ey, 1.0))
        speed = v_ref * max(0.15, 1.0 - 0.6 * min(abs(epsi), 1.0))
        yaw_rate = 2.0 * speed * math.sin(alpha) / 1.0
        yaw_rate = float(np.clip(yaw_rate, -1.2, 1.2))
        return float(speed), yaw_rate

    def predict(
        self,
        state,
        reference,
        v_ref: float,
    ) -> Tuple[float, float]:
        features = self.features(state, reference, v_ref)
        output = self.model.forward(features.reshape(1, -1))[0]
        speed = float(v_ref) * (0.5 * (float(output[0]) + 1.0))
        yaw_rate = 1.2 * float(output[1])
        speed = float(np.clip(speed, 0.0, float(v_ref) * 1.1))
        yaw_rate = float(np.clip(yaw_rate, -1.2, 1.2))
        return speed, yaw_rate

    def expert_command(
        self,
        state,
        reference,
        v_ref: float,
    ) -> Tuple[float, float]:
        features = self.features(state, reference, v_ref)
        return self.expert_from_features(features, v_ref)


def evaluate_tracking(
    controller: TrajectoryController,
    simulation,
    geometry,
    v_ref: float = V_REF_DEFAULT,
    max_time: float = 15.0,
) -> Dict[str, float]:
    """Run closed-loop S-curve tracking and return error metrics."""
    start_x, start_y, start_yaw = -6.0, 0.0, 0.0
    trajectory = ReferenceTrajectory(
        build_s_curve(start_x, start_y, start_yaw)
    )
    simulation.reset()
    simulation.set_pose(start_x, start_y, start_yaw)
    errors = []
    heading_errors = []
    control_period = 0.02
    steps_per_control = max(1, int(round(control_period / simulation.timestep)))
    time = 0.0
    reached = False

    while time < max_time:
        state = simulation.planar_state()
        index, nearest_distance, reference = trajectory.lookahead(
            state.x, state.y
        )
        speed, yaw_rate = controller.predict(
            (state.x, state.y, state.yaw, state.linear_velocity),
            reference,
            v_ref,
        )
        left, right = geometry.wheel_speeds(speed, yaw_rate)
        for _ in range(steps_per_control):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()
        errors.append(nearest_distance)
        heading_errors.append(
            abs(wrap_angle(float(reference[2]) - state.yaw))
        )
        time += control_period
        if index >= len(trajectory.points) - 4 and nearest_distance < 0.25:
            reached = True
            break

    errors_array = np.asarray(errors, dtype=np.float64)
    heading_array = np.asarray(heading_errors, dtype=np.float64)
    return {
        'rmse': float(np.sqrt(np.mean(errors_array ** 2))),
        'max_error': float(np.max(errors_array)),
        'rmse_heading': float(np.sqrt(np.mean(heading_array ** 2))),
        'time': float(time),
        'reached': float(1.0 if reached else 0.0),
        'samples': int(len(errors_array)),
    }
