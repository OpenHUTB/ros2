"""禁飞区检测的几何与判定测试。"""

from __future__ import annotations

import json
import math

import pytest

import main as nfz


def test_distance_zero_for_same_point():
    assert nfz.distance_m(47.65, -122.14, 47.65, -122.14) == 0.0


def test_distance_symmetric():
    a = nfz.distance_m(47.65, -122.14, 47.60, -122.20)
    b = nfz.distance_m(47.60, -122.20, 47.65, -122.14)
    assert a == pytest.approx(b, rel=1e-12)


def test_distance_one_milli_degree_latitude():
    # 1e-3 度纬度 ≈ 111.19 m（R = 6 371 000 m）
    d = nfz.distance_m(47.0, -122.0, 47.001, -122.0)
    assert d == pytest.approx(math.radians(0.001) * nfz.EARTH_RADIUS_M, rel=1e-9)
    assert 110.0 < d < 112.0


def test_distance_longitude_scales_with_latitude():
    at_equator = nfz.distance_m(0.0, 0.0, 0.0, 0.001)
    at_47 = nfz.distance_m(47.0, 0.0, 47.0, 0.001)
    assert at_47 < at_equator
    assert at_47 == pytest.approx(at_equator * math.cos(math.radians(47.0)), rel=1e-3)


def test_distance_known_city_pair():
    # 机场中心 -> 基地中心，约 7.1 km
    d = nfz.distance_m(47.65, -122.14, 47.60, -122.20)
    assert 6500 < d < 7800


def test_point_inside_zone_center():
    name, dist = nfz.check_no_fly_zone(47.65, -122.14)
    assert name == "Airport"
    assert dist == pytest.approx(0.0, abs=1e-6)


def test_point_just_outside_radius():
    # 机场半径 2000 m，正北 2100 m 处应判为安全
    dlat = math.degrees(2100.0 / nfz.EARTH_RADIUS_M)
    name, dist = nfz.check_no_fly_zone(47.65 + dlat, -122.14)
    assert name is None and dist is None


def test_point_just_inside_radius_is_flagged():
    """边界判定是 <=：半径内侧 1 m 仍算“在内”。"""
    dlat = math.degrees(1999.0 / nfz.EARTH_RADIUS_M)
    name, dist = nfz.check_no_fly_zone(47.65 + dlat, -122.14)
    assert name == "Airport"
    assert dist == pytest.approx(1999.0, abs=1.0)


def test_custom_zone_list_is_respected():
    zones = [{"name": "Tiny", "lat": 30.0, "lon": 120.0, "radius_m": 100}]
    name, _ = nfz.check_no_fly_zone(30.0, 120.0, zones=zones)
    assert name == "Tiny"
    assert nfz.check_no_fly_zone(47.65, -122.14, zones=zones) == (None, None)


def test_empty_zone_list_is_safe():
    assert nfz.check_no_fly_zone(47.65, -122.14, zones=[]) == (None, None)


# ---------------------------------------------------------------- route


ZONE = {"name": "Test", "lat": 47.65, "lon": -122.14, "radius_m": 2000}


def test_segment_crossing_has_zero_distance():
    # 东西向直线正好穿过圆心
    d = nfz.segment_min_distance_m((47.65, -122.17), (47.65, -122.11), ZONE)
    assert d < 1.0


def test_segment_far_away_uses_perpendicular_distance():
    d = nfz.segment_min_distance_m((47.70, -122.17), (47.70, -122.11), ZONE)
    expected = nfz.distance_m(47.70, -122.14, ZONE["lat"], ZONE["lon"])
    assert d == pytest.approx(expected, rel=0.02)


def test_segment_endpoint_is_closest():
    # 线段整体在禁区东侧，最近点是终点
    d = nfz.segment_min_distance_m((47.65, -122.05), (47.65, -122.08), ZONE)
    expected = nfz.distance_m(47.65, -122.08, ZONE["lat"], ZONE["lon"])
    assert d == pytest.approx(expected, rel=0.02)


def test_segment_degenerate_is_point_distance():
    d = nfz.segment_min_distance_m((47.65, -122.14), (47.65, -122.14), ZONE)
    assert d < 1.0


def test_segment_distance_symmetric():
    a = nfz.segment_min_distance_m((47.70, -122.17), (47.70, -122.11), ZONE)
    b = nfz.segment_min_distance_m((47.70, -122.11), (47.70, -122.17), ZONE)
    assert a == pytest.approx(b, rel=1e-9)


def test_route_crossing_is_unsafe():
    """两端都在区外、但航线穿过禁区 —— 旧版只查端点会漏判。"""
    result = nfz.check_route((47.65, -122.17), (47.65, -122.11))
    assert result["safe"] is False
    reasons = {v["reason"] for v in result["violations"]}
    assert "path_crosses" in reasons
    assert result["zones_checked"] == len(nfz.NO_FLY_ZONES)


def test_route_pickup_inside():
    result = nfz.check_route((47.65, -122.14), (47.70, -122.05))
    assert result["safe"] is False
    assert result["violations"][0]["reason"] == "pickup_inside"
    assert result["violations"][0]["zone"] == "Airport"


def test_route_delivery_inside():
    result = nfz.check_route((47.70, -122.05), (47.62, -122.10))
    assert result["safe"] is False
    reasons = {v["reason"] for v in result["violations"]}
    assert "delivery_inside" in reasons


def test_route_safe():
    result = nfz.check_route((47.70, -122.25), (47.70, -122.05))
    assert result["safe"] is True
    assert result["violations"] == []


def test_route_with_custom_zones():
    zones = [{"name": "Small", "lat": 47.65, "lon": -122.14, "radius_m": 100}]
    # 东西向航线从圆心上方 500 m 掠过，半径只有 100 m -> 安全
    dlat = math.degrees(500.0 / nfz.EARTH_RADIUS_M)
    result = nfz.check_route((47.65 + dlat, -122.17), (47.65 + dlat, -122.11),
                             zones=zones)
    assert result["safe"] is True
    assert result["zones_checked"] == 1


def test_mission_zones_compat():
    ok, msg = nfz.check_mission_zones((47.70, -122.25), (47.70, -122.05))
    assert ok is True
    assert "safe" in msg.lower()

    ok, msg = nfz.check_mission_zones((47.65, -122.17), (47.65, -122.11))
    assert ok is False
    assert "航线穿越禁飞区" in msg


# ---------------------------------------------------------------- cli


def test_parse_latlon_valid():
    assert nfz.parse_latlon("47.65,-122.14") == (47.65, -122.14)
    assert nfz.parse_latlon(" 47.65， -122.14 ") == (47.65, -122.14)


@pytest.mark.parametrize("text", ["47.65", "a,b", "91.0,0", "0,181"])
def test_parse_latlon_invalid(text):
    with pytest.raises(ValueError):
        nfz.parse_latlon(text)


def test_cli_json_crossing_returns_1(capsys):
    code = nfz.main(["--pickup", "47.65,-122.17", "--dropoff", "47.65,-122.11",
                     "--json"])
    assert code == 1
    payload = json.loads(capsys.readouterr().out)
    assert payload["safe"] is False
    assert payload["zones_checked"] == 3


def test_cli_safe_returns_0(capsys):
    code = nfz.main(["--pickup", "47.70,-122.25", "--dropoff", "47.70,-122.05",
                     "--json"])
    assert code == 0
    assert json.loads(capsys.readouterr().out)["safe"] is True


def test_cli_requires_both_points(capsys):
    with pytest.raises(SystemExit) as exc:
        nfz.main(["--pickup", "47.65,-122.17"])
    assert exc.value.code == 2
