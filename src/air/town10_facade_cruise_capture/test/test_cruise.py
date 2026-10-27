"""核心数值边界、真实积分、图像/位姿一致性与任务产物检查。"""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from town10_capture.config import CruiseConfig, ned_to_enu, town10_route
from town10_capture.mission import CruiseMission, run_cruise
from town10_capture.scene import MujocoBackend


@pytest.mark.parametrize(
    "field,value",
    [("speed", float("nan")), ("duration", float("inf")), ("rate", 0), ("speed", 3)],
)
def test_nonfinite_parameters(field, value):
    with pytest.raises(ValueError):
        CruiseConfig(**{field: value}).validate()


def test_exact_route_and_coordinate_transform():
    route = town10_route()
    assert route.shape == (36, 4)
    np.testing.assert_allclose(route[0], [110, -22, -32, 270])
    np.testing.assert_allclose(ned_to_enu(ned_to_enu([1, 2, 3])), [1, 2, 3])


def test_integrated_force_changes_position():
    sim = MujocoBackend()
    try:
        before = sim.data.qpos.copy()
        sim.step([0.4, 0, 0])
        assert sim.data.time == pytest.approx(0.05)
        assert sim.data.qpos[0] > before[0]
        assert sim.data.qvel[0] > 0
    finally:
        sim.close()


def test_source_texture_hashes():
    assets = Path(__file__).resolve().parents[1] / "town10_capture/assets"
    provenance = json.loads((assets / "provenance.json").read_text(encoding="utf-8"))
    assert len(provenance) == 3
    for record in provenance:
        assert (
            hashlib.sha256((assets / record["texture_file"]).read_bytes()).hexdigest()
            == record["texture_sha256"]
        )


def test_full_neural_scan_manifest(tmp_path):
    result = run_cruise(CruiseConfig(output=tmp_path, capture_rate=1))
    assert result["status"] == "complete"
    assert result["waypoints_completed"] == result["images_captured"] == 12
    assert result["maximum_speed_m_s"] <= 0.81
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    for capture in manifest["captures"]:
        assert (tmp_path / capture["file"]).is_file()
        assert (
            np.linalg.norm(
                np.array(capture["target_enu_m"]) - capture["position_enu_m"]
            )
            < 0.15
        )
    assert len((tmp_path / "trajectory.csv").read_text().splitlines()) > 100


def test_timeout_is_saved(tmp_path):
    mission = CruiseMission(CruiseConfig(output=tmp_path, duration=0.1))
    while mission.tick():
        pass
    assert mission.reason == "duration_limit"
    assert (
        json.loads((tmp_path / "metrics.json").read_text())["status"]
        == "duration_limit"
    )
