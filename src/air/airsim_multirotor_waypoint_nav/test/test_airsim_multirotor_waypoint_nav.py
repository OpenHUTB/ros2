"""航点换算、三维误差与配置的单元测试（不需要 AirSim 服务端）。

app.py 顶层不 import airsim（只在 run() 里 import），所以可以直接按包导入被测函数。
"""
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from airsim_multirotor_waypoint_nav.app import (  # noqa: E402
    absolute_waypoints,
    distance_3d,
    load_params,
)


@pytest.fixture(scope="module")
def cfg():
    return load_params(os.path.join(ROOT, "config", "params.yaml"))


def test_distance_is_3d():
    # 水平 3 m、高度差 4 m -> 5 m（含高度，评审指出不能只算平面误差）
    assert distance_3d((3.0, 0.0, -4.0), (0.0, 0.0, 0.0)) == pytest.approx(5.0)


def test_distance_zero_on_target():
    assert distance_3d((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == pytest.approx(0.0)


def test_waypoints_are_relative_to_takeoff_point():
    home = (10.0, -5.0, -0.5)
    wps = absolute_waypoints(home, [(25.0, 0.0), (0.0, 25.0)], 6.0)
    assert wps[0][0] == pytest.approx(35.0)
    assert wps[0][1] == pytest.approx(-5.0)
    assert wps[1][0] == pytest.approx(10.0)
    assert wps[1][1] == pytest.approx(20.0)
    assert all(p[2] == pytest.approx(-6.0) for p in wps)


def test_waypoints_config_shape(cfg):
    wps = cfg["flight"]["waypoints"]
    assert len(wps) == 4
    assert all(len(w) == 2 for w in wps)


def test_arrival_rule_is_strict(cfg):
    """容差要有连续命中要求，且 PASS 阈值不能形同虚设。"""
    assert int(cfg["flight"]["consecutive_hits"]) >= 2
    assert 0 < float(cfg["evaluation"]["pass_max_error"]) < float(cfg["flight"]["arrive_tolerance"])
    assert float(cfg["flight"]["max_flight_time"]) > 0


def test_sampling_interval_is_dense(cfg):
    """采样间隔要足够密，避免日志只有几行。"""
    assert float(cfg["flight"]["log_interval"]) <= 0.2
