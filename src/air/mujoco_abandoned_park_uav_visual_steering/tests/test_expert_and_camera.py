"""Camera and expert tests / 相机与专家测试。"""

from __future__ import annotations

import sys
from dataclasses import replace
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
    assert expert.decide(x_m=0.8, y_m=0.0, yaw_radians=0.0).steering > 0.0
    assert expert.decide(x_m=-0.8, y_m=0.0, yaw_radians=0.0).steering < 0.0


def test_expert_feedback_returns_both_sides_towards_center() -> None:
    config = SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml")
    expert = _expert()
    for offset in (-0.9, 0.9):
        with UavSimulation(MODULE_ROOT / "models" / "scene.xml", config) as simulation:
            simulation.reset(offset)
            for _ in range(80):
                state = simulation.state()
                decision = expert.decide(state["x_m"], state["y_m"], state["yaw_radians"])
                assert decision.label is not SteeringLabel.STOP
                simulation.step(decision.steering)
            assert abs(simulation.state()["x_m"]) < abs(offset) / 2


def test_terminal_start_reaches_real_stop_without_leaving_corridor() -> None:
    config = SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml")
    with UavSimulation(MODULE_ROOT / "models" / "scene.xml", config) as simulation:
        simulation.reset(0.35, initial_y_m=32.9)
        for _ in range(10):
            state = simulation.state()
            decision = _expert().decide(state["x_m"], state["y_m"], state["yaw_radians"])
            if decision.label is SteeringLabel.STOP:
                assert state["y_m"] >= 33.0
                assert abs(state["x_m"]) < 1.28
                break
            simulation.step(decision.steering)
        else:
            raise AssertionError("terminal trajectory did not reach STOP")


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


def test_configured_camera_dimensions_are_used() -> None:
    config = SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml")
    with UavSimulation(MODULE_ROOT / "models" / "scene.xml", replace(
        config, render_width=64, render_height=36
    )) as simulation:
        assert simulation.capture_rgb().shape == (36, 64, 3)

