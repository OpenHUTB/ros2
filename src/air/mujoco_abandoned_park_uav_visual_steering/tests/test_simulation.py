"""Simulation contract tests / 仿真契约测试。"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT))

from mujoco_abandoned_park_uav_visual_steering import (  # noqa: E402
    SimulationConfig,
    UavSimulation,
)


@pytest.fixture()
def configured_simulation() -> UavSimulation:
    config = SimulationConfig.from_yaml(MODULE_ROOT / "config" / "simulation.yaml")
    return UavSimulation(MODULE_ROOT / "models" / "scene.xml", config)


def test_scene_exposes_required_camera_and_uav(configured_simulation: UavSimulation) -> None:
    assert configured_simulation.camera_id >= 0
    assert configured_simulation.body_id >= 0
    assert configured_simulation.model.ngeom >= 20
    assert np.allclose(configured_simulation.position, (0.0, -7.0, 1.4))


def test_steering_changes_heading_and_lateral_position(
    configured_simulation: UavSimulation,
) -> None:
    for _ in range(20):
        configured_simulation.step(0.5)
    state = configured_simulation.state()
    assert state["time_seconds"] == pytest.approx(1.0)
    assert state["yaw_radians"] > 0.0
    assert state["x_m"] < 0.0
    assert state["y_m"] > -7.0


def test_invalid_steering_is_rejected(configured_simulation: UavSimulation) -> None:
    with pytest.raises(ValueError, match="steering"):
        configured_simulation.step(1.01)


def test_main_entry_runs_complete_simulation() -> None:
    completed = subprocess.run(
        [sys.executable, str(MODULE_ROOT / "main.py"), "--mode", "simulation", "--steps", "4"],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    payload = json.loads(completed.stdout)
    assert payload["status"] == "ok"
    assert payload["steps"] == 4
    assert payload["state"]["y_m"] > -7.0

