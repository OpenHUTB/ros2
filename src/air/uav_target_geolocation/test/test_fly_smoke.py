# -*- coding: utf-8 -*-
"""闭环冒烟：真跑一次 MuJoCo，确认能飞到位、采样数够、解算不离谱。"""
import numpy as np
import pytest

pytest.importorskip("mujoco")

from uav_target_geolocation.core.cases import GeoCase, Station
from uav_target_geolocation.core.flyer import _LineRef, fly_case, summarize_flight

SMOKE = GeoCase(
    name="smoke_single",
    title="冒烟：单站 40 m 高",
    stations=(Station(0.0, radius_m=20.0, height_m=40.0),),
    samples=20,
)


def test_line_ref_follows_a_trapezoid_profile():
    ref = _LineRef((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), v_max=2.0, a_max=1.0)
    speeds = []
    for _ in range(2000):
        _p, v, _a = ref.step(0.01)
        speeds.append(float(np.linalg.norm(v)))
        if ref.done:
            break
    assert ref.done
    assert max(speeds) <= 2.0 + 1e-9
    assert speeds[-1] < 0.2                      # 到点前已经减速
    assert 1.0 < speeds.index(max(speeds)) * 0.01   # 先加速再匀速，不是一步到位


def test_flight_reaches_station_and_collects_samples():
    res = fly_case(SMOKE, seed=11)
    assert len(res.visits) == 1
    v = res.visits[0]
    assert v.n_samples == SMOKE.samples
    assert v.hover_error_m < 1.0                 # 悬停要稳
    assert float(np.linalg.norm(v.pos_mean[:2] - v.target[:2])) < 1.0


def test_geolocation_from_the_flight_is_close_to_truth():
    res = fly_case(SMOKE, seed=12)
    s = summarize_flight(SMOKE, res)
    assert s["single_ref_m"] is not None
    assert s["single_ref_m"] < 5.0               # 40 m 斜距、0.2° 偏置 ⇒ 亚米级
    assert s["station_fused"]["ok"]
    assert s["per_sample"]["single"]["n"] == SMOKE.samples


def test_flying_with_wind_tilts_the_body():
    """有水平外力时机体必然倾转，倾角大小应满足 tanθ = F/(mg)。"""
    windy = GeoCase(name="windy", title="有风", stations=SMOKE.stations,
                    samples=20, wind_n=(1.5, 0.0))
    res = fly_case(windy, seed=13)
    pitch = abs(res.visits[0].tilt_mean_deg[1])
    expect = float(np.degrees(np.arctan2(1.5, 0.95 * 9.81)))
    assert pitch == pytest.approx(expect, abs=1.5)
