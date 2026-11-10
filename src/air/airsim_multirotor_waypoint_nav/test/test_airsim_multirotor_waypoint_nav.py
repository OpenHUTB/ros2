"""航点换算与到达判定的单元测试（不需要 AirSim 服务端）。"""
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def _load_helpers():
    """只加载 app.py 里的纯函数，避免 import airsim。"""
    app = os.path.join(ROOT, "airsim_multirotor_waypoint_nav", "app.py")
    src = open(app, encoding="utf-8").read()
    start = src.index("def distance_3d(")
    end = src.index("def save_plot(")
    ns = {"math": __import__("math")}
    exec(compile(src[start:end], app, "exec"), ns)  # noqa: S102
    return ns


def _cfg():
    with open(os.path.join(ROOT, "config", "params.yaml"), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_distance_is_3d():
    ns = _load_helpers()
    # 水平 3 m、高度差 4 m -> 5 m（含高度，评审指出不能只算平面误差）
    assert ns["distance_3d"]((3.0, 0.0, -4.0), (0.0, 0.0, 0.0)) == pytest.approx(5.0)


def test_distance_zero_on_target():
    ns = _load_helpers()
    assert ns["distance_3d"]((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == pytest.approx(0.0)


def test_waypoints_are_relative_to_takeoff_point():
    ns = _load_helpers()
    home = (10.0, -5.0, -0.5)
    wps = ns["absolute_waypoints"](home, [(25.0, 0.0), (0.0, 25.0)], 6.0)
    assert wps[0][0] == pytest.approx(35.0)
    assert wps[0][1] == pytest.approx(-5.0)
    assert wps[1][0] == pytest.approx(10.0)
    assert wps[1][1] == pytest.approx(20.0)
    # 高度固定为 -takeoff_height
    assert all(p[2] == pytest.approx(-6.0) for p in wps)


def test_waypoints_config_shape():
    cfg = _cfg()
    wps = cfg["flight"]["waypoints"]
    assert len(wps) == 4
    assert all(len(w) == 2 for w in wps)


def test_arrival_rule_is_strict():
    """容差要有连续命中要求，且 PASS 阈值不能形同虚设。"""
    cfg = _cfg()
    assert int(cfg["flight"]["consecutive_hits"]) >= 2
    assert 0 < float(cfg["evaluation"]["pass_max_error"]) < float(cfg["flight"]["arrive_tolerance"])
    assert float(cfg["flight"]["max_flight_time"]) > 0


def test_sampling_interval_is_dense():
    """采样间隔要足够密，避免日志只有几行。"""
    cfg = _cfg()
    assert float(cfg["flight"]["log_interval"]) <= 0.2
