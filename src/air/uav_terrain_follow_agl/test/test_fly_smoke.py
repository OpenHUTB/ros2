# -*- coding: utf-8 -*-
"""闭环冒烟：短航程下三种策略都能飞完，且间隙排序与设计预期一致。

MuJoCo 用例在没装 mujoco 时自动跳过；运动学路径永远可跑。
"""
import math

import pytest

from uav_terrain_follow_agl.core.cases import Case, get_case
from uav_terrain_follow_agl.core.dem import IDEAL_DEM
from uav_terrain_follow_agl.core.flyer import KinematicPlant, fly, fly_all, speed_profile


def _require_mujoco():
    """只给真正需要 MuJoCo 的用例加跳过标记：导入失败整份文件会被跳过，
    连纯运动学的用例都跑不了（离线 CI 里那些用例本来是可以跑的）。"""
    pytest.importorskip("mujoco", reason="需要 mujoco 才能跑闭环")
    pytest.importorskip("uav_mujoco_sim.core.quadrotor", reason="需要同仓库 uav_mujoco_sim")


def _short_case(**kwargs) -> Case:
    base = get_case("ridge_8ms")
    params = dict(name="short", title="短航程冒烟", terrain_name=base.terrain_name,
                  speed_mps=8.0, agl_set_m=15.0, safety_m=5.0, length_m=360.0,
                  ramp_s=3.0, seed=base.seed, dem=base.dem,
                  limits=base.limits, altimeter=base.altimeter)
    params.update(kwargs)
    return Case(**params)


# --------------------------------------------------------------------------- #
# 与仿真器无关的部分
# --------------------------------------------------------------------------- #
def test_speed_profile_matches_cruise():
    x, vx, ax = speed_profile(10.0, 8.0, 4.0)
    assert vx == pytest.approx(8.0)
    assert ax == pytest.approx(0.0)
    assert x == pytest.approx(0.5 * 8.0 * 4.0 + 8.0 * 6.0)
    assert speed_profile(-1.0, 8.0, 4.0) == (0.0, 0.0, 0.0)


def test_kinematic_plant_lags_velocity_command():
    plant = KinematicPlant()
    plant.reset((0.0, 0.0, 15.0))
    for _ in range(500):
        pos, vel = plant.step((0.0, 0.0, 15.0), (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    assert vel[2] == pytest.approx(1.0, abs=0.02)   # 一阶滞后收敛到指令


def test_kinematic_flight_completes():
    case = _short_case()
    f = fly(case, "dem_preview", plant="kinematic")
    assert not f.crashed
    assert f.samples[-1].x >= case.length_m - 0.5
    assert f.metrics()["min_clearance_m"] > case.safety_m


# --------------------------------------------------------------------------- #
# MuJoCo 闭环
# --------------------------------------------------------------------------- #
def test_mujoco_flight_records_truth_and_commands():
    _require_mujoco()
    case = _short_case(length_m=60.0)
    f = fly(case, "reactive_agl", plant="mujoco")
    assert len(f.samples) > 100
    for s in f.samples[:5]:
        assert math.isfinite(s.z) and math.isfinite(s.clearance)
        assert s.clearance == pytest.approx(s.z - s.terrain_z)


def test_three_strategies_rank_as_expected():
    _require_mujoco()
    """同一段山脊：定高撞地 < 纯反馈 < DEM 前馈。"""
    case = _short_case()
    gaps = {name: fly(case, name, plant="mujoco").metrics()["min_clearance_m"]
            for name in ("constant_msl", "reactive_agl", "dem_preview")}
    assert gaps["constant_msl"] < 0.0
    assert gaps["dem_preview"] > gaps["reactive_agl"]
    assert gaps["dem_preview"] > case.safety_m


def test_stop_on_intrusion_terminates_early():
    case = _short_case()
    full = fly(case, "constant_msl", plant="kinematic")
    cut = fly(case, "constant_msl", plant="kinematic", stop_on_intrusion=True)
    assert len(cut.samples) < len(full.samples)
    assert cut.metrics()["min_clearance_m"] > full.metrics()["min_clearance_m"] - 1e-9
    assert abs(cut.metrics()["min_clearance_m"]) < 1.0


def test_kinematic_and_mujoco_agree_within_a_metre():
    _require_mujoco()
    case = _short_case()
    a = fly(case, "dem_preview", plant="kinematic").metrics()
    b = fly(case, "dem_preview", plant="mujoco").metrics()
    assert abs(a["min_clearance_m"] - b["min_clearance_m"]) < 1.0
    assert abs(a["rms_agl_error_m"] - b["rms_agl_error_m"]) < 1.0


def test_ideal_dem_does_not_change_safety_conclusion():
    case = _short_case(dem=IDEAL_DEM)
    f = fly(case, "dem_preview", plant="kinematic")
    assert f.metrics()["min_clearance_m"] > case.safety_m


def test_fly_all_returns_each_strategy():
    case = _short_case(length_m=40.0)
    flights = fly_all(case, plant="kinematic")
    assert [f.strategy for f in flights] == ["constant_msl", "reactive_agl", "dem_preview"]


def test_unknown_plant_raises():
    with pytest.raises(ValueError):
        fly(_short_case(), "dem_preview", plant="gazebo")


def test_base_model_really_has_no_ground_contact():
    _require_mujoco()
    """底座模型机身 geom 关掉了碰撞——所以地形只以解析场进入评估。"""
    from uav_mujoco_sim.core.quadrotor import Quadrotor

    quad = Quadrotor(dt=0.01)
    m = quad.model
    for i in range(m.ngeom):
        name = m.geom(i).name
        if name.startswith(("body", "arm")):
            assert m.geom_contype[i] == 0 and m.geom_conaffinity[i] == 0
