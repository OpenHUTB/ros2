"""Camera and expert tests / 相机与专家测试。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT))

from mujoco_abandoned_park_uav_visual_steering import (  # noqa: E402
    PathCenterExpert,
    SimulationConfig,
    SteeringLabel,
    UavSimulation,
)


def _expert() -> PathCenterExpert:
    return PathCenterExpert(
        path_half_width_m=1.4,
        steering_threshold=0.18,
        stop_margin_m=0.12,
        path_end_y_m=33.0,
    )


def test_expert_labels_path_correction_direction() -> None:
    expert = _expert()
    assert expert.decide(x_m=0.8, y_m=0.0, yaw_radians=0.0).label is SteeringLabel.LEFT
    assert expert.decide(x_m=-0.8, y_m=0.0, yaw_radians=0.0).label is SteeringLabel.RIGHT
    assert expert.decide(x_m=0.0, y_m=0.0, yaw_radians=0.0).label is SteeringLabel.STRAIGHT


def test_expert_stops_outside_safe_corridor_and_at_path_end() -> None:
    expert = _expert()
    assert expert.decide(x_m=1.3, y_m=0.0, yaw_radians=0.0).label is SteeringLabel.STOP
    assert expert.decide(x_m=0.0, y_m=33.0, yaw_radians=0.0).label is SteeringLabel.STOP


def test_forward_camera_returns_stable_rgb_frame() -> None:
    config = SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml")
    with UavSimulation(MODULE_ROOT / "models" / "scene.xml", config) as simulation:
        first = simulation.capture_rgb()
        second = simulation.capture_rgb()
    assert first.shape == (180, 320, 3)
    assert first.dtype == np.uint8
    frame_delta = np.abs(first.astype(np.int16) - second.astype(np.int16))
    assert float(frame_delta.mean()) < 0.001
    assert int(frame_delta.max()) <= 1
    assert float(first.std()) > 10.0

