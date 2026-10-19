"""MuJoCo scene tests / MuJoCo 场景测试。"""

from pathlib import Path

import mujoco
import pytest

from rooftop_pv.scene import SCENARIOS, load_model, scenario_path


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_each_scene_loads_with_camera_and_quadrotor(scenario: str) -> None:
    model, data = load_model(scenario)
    assert data.qpos.shape[0] >= 7
    assert model.nu == 4
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "downward_rgb") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "uav") >= 0


def test_scenarios_have_distinct_panel_densities_and_obstacles() -> None:
    counts = {}
    for scenario in SCENARIOS:
        model, _ = load_model(scenario)
        counts[scenario] = sum((model.geom(index).name or "").startswith("pv_panel_") for index in range(model.ngeom))
    assert counts["dense"] > counts["occluded"] > counts["sparse"]
    occluded, _ = load_model("occluded")
    assert mujoco.mj_name2id(occluded, mujoco.mjtObj.mjOBJ_GEOM, "skylight") >= 0


def test_unknown_scenario_is_rejected() -> None:
    with pytest.raises(ValueError):
        scenario_path("unknown")


def test_scenario_files_are_small_and_self_contained() -> None:
    for scenario in SCENARIOS:
        path = scenario_path(scenario)
        assert path.stat().st_size < 20_000
        assert "mesh" not in path.read_text(encoding="utf-8")
