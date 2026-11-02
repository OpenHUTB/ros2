"""nfz_core 单元测试

重点回归三类问题：
1. 旧版 bug：端点安全但航线段穿越禁飞区 —— check_path/check_mission 必须拦截；
2. 球面几何已知答案：1° 纬度弧长、正东/正北方位角、端点/线段归属；
3. NED → GPS 换算的 cos(φ) 修正（旧版经度偏大约 33%）。

运行：python -m pytest test_nfz_core.py -v
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import nfz_core as nfz

ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]


# ---------- haversine 已知答案 ----------

def test_one_degree_latitude():
    # 1° 纬度 ≈ πR/180 ≈ 111194.9 m
    d = nfz.haversine_m(0.0, 0.0, 1.0, 0.0)
    assert abs(d - nfz.M_PER_DEG_LAT) < 1e-6
    assert abs(d - 111194.926) < 0.5


def test_one_degree_longitude_on_equator():
    # 赤道上 1° 经度 ≈ 111194.9 m；北纬 47.65° 处缩为 cos(φ) 倍
    d_eq = nfz.haversine_m(0.0, 0.0, 0.0, 1.0)
    assert abs(d_eq - nfz.M_PER_DEG_LAT) < 1.0
    d_mid = nfz.haversine_m(47.65, 0.0, 47.65, 1.0)
    expect = nfz.M_PER_DEG_LAT * math.cos(math.radians(47.65))
    assert abs(d_mid - expect) < 1.0


def test_haversine_symmetry_and_zero():
    assert nfz.haversine_m(47.6, -122.3, 47.6, -122.3) == 0.0
    d1 = nfz.haversine_m(30.0, 120.0, 40.0, -100.0)
    d2 = nfz.haversine_m(40.0, -100.0, 30.0, 120.0)
    assert abs(d1 - d2) < 1e-6


# ---------- bearing 已知答案 ----------

def test_bearing_north_and_east():
    assert abs(nfz.bearing_rad(0.0, 0.0, 1.0, 0.0)) < 1e-9          # 正北
    assert abs(nfz.bearing_rad(0.0, 0.0, 0.0, 1.0) - math.pi / 2) < 1e-9  # 正东


# ---------- 旧版核心 bug：航线穿越检测 ----------

def test_path_crossing_zone_is_detected():
    # 旧版 bug 场景：取货/送货点都在禁区外(约 2245m)，但航线正穿机场中心
    pickup = (47.65, -122.17)
    delivery = (47.65, -122.11)
    airport = ZONES[0]
    # 前置条件确认：两端点确实在禁区外
    for pt in (pickup, delivery):
        d = nfz.haversine_m(pt[0], pt[1], airport['lat'], airport['lon'])
        assert d > airport['radius_m'], '前置条件不成立：端点已在禁区内'
    violations = nfz.check_path([pickup, delivery], ZONES)
    names = [v['name'] for v in violations]
    assert 'Airport' in names
    # 航线正对中心穿过，最短距离应很小
    v = [v for v in violations if v['name'] == 'Airport'][0]
    assert v['min_dist_m'] < 500.0
    assert v['leg'] == 0


def test_path_parallel_offset_is_safe():
    # 航线向北偏 0.03°(约 3336m)，大于机场半径 2000m —— 不判违规
    pickup = (47.68, -122.17)
    delivery = (47.68, -122.11)
    violations = nfz.check_path([pickup, delivery], ZONES)
    assert all(v['name'] != 'Airport' for v in violations)


def test_zone_beyond_segment_endpoint_is_safe():
    # 禁区在线段延长线方向之外：最近点是端点，距离约 7.5km —— 安全
    pickup = (47.65, -122.30)
    delivery = (47.65, -122.24)
    violations = nfz.check_path([pickup, delivery], ZONES)
    assert all(v['name'] != 'Airport' for v in violations)


def test_buffer_margin_enlarges_zone():
    # 距机场中心约 2669m 的航线：不加裕量安全，加 1000m 裕量后违规
    points = [(47.674, -122.17), (47.674, -122.11)]
    assert nfz.check_path(points, [ZONES[0]]) == []
    v = nfz.check_path(points, [ZONES[0]], buffer_m=1000.0)
    assert any(x['name'] == 'Airport' for x in v)


def test_mission_catches_endpoints_and_path():
    # 端点在禁区内的老能力仍在
    ok, report = nfz.check_mission((47.65, -122.14), (30.0, 120.0), ZONES)
    assert not ok
    assert any(r['kind'] == 'point' and r['name'] == 'Airport' for r in report)
    # 新能力：端点安全但航线穿越
    ok, report = nfz.check_mission((47.65, -122.17), (47.65, -122.11), ZONES)
    assert not ok
    assert any(r['kind'] == 'path' and r['name'] == 'Airport' for r in report)


def test_mission_safe_case():
    ok, report = nfz.check_mission((30.0, 120.0), (31.0, 121.0), ZONES)
    assert ok and report == []


def test_mission_with_waypoint_detour_is_safe():
    # 直线穿越机场，但绕行航路点使每段都避开 —— 整体安全
    pickup = (47.65, -122.17)
    delivery = (47.65, -122.11)
    detour = (47.72, -122.14)   # 机场以北约 7.8km
    ok, report = nfz.check_mission(pickup, delivery, ZONES,
                                   waypoints=[detour])
    assert ok and report == []


def test_check_path_requires_two_points():
    try:
        nfz.check_path([(47.65, -122.14)], ZONES)
        assert False, '应抛出 ValueError'
    except ValueError:
        pass


# ---------- min_dist_to_segment 的端点/线段归属 ----------

def test_segment_distance_projection_inside():
    # 点在线段正中垂线上：交叉距离应约等于偏移量
    offset = 0.03  # 约 3336 m
    dist, where = nfz.min_dist_to_segment_m(
        47.65 + offset, -122.14,      # 中心正北侧
        47.65, -122.17, 47.65, -122.11)
    assert where == 'segment'
    assert abs(dist - offset * nfz.M_PER_DEG_LAT) < 20.0


def test_segment_distance_endpoint_cases():
    # 禁区中心在线段 A→B 延长线方向、B 点以东 → 归属 endpoint_b
    dist, where = nfz.min_dist_to_segment_m(
        47.65, -122.05, 47.65, -122.20, 47.65, -122.10)
    assert where == 'endpoint_b'
    assert abs(dist - 0.05 * nfz.M_PER_DEG_LAT *
               math.cos(math.radians(47.65))) < 5.0
    # 在 A 点以西 → 归属 endpoint_a
    dist, where = nfz.min_dist_to_segment_m(
        47.65, -122.25, 47.65, -122.20, 47.65, -122.10)
    assert where == 'endpoint_a'
    assert abs(dist - 0.05 * nfz.M_PER_DEG_LAT *
               math.cos(math.radians(47.65))) < 5.0


def test_segment_distance_perpendicular_accuracy():
    # 点在线段中点正北约 5559.7m：无论归类 segment 还是 endpoint，
    # 距离值都必须准确（球面几何在垂直边界处归属存在固有模糊）。
    dist, _where = nfz.min_dist_to_segment_m(
        47.70, -122.15, 47.65, -122.20, 47.65, -122.10)
    assert abs(dist - 0.05 * nfz.M_PER_DEG_LAT) < 5.0


def test_point_on_zone_center_hits():
    hits = nfz.check_point(47.65, -122.14, ZONES)
    assert len(hits) == 1 and hits[0]['name'] == 'Airport'
    assert hits[0]['dist_m'] < 1e-6


def test_point_outside_all_zones():
    assert nfz.check_point(30.0, 120.0, ZONES) == []


# ---------- NED → GPS 换算修正 ----------

def test_ned_to_gps_north_and_east():
    # 向北 1000m：纬度增加约 1000/111195 ≈ 0.00899°
    lat, lon = nfz.ned_to_gps(1000.0, 0.0, 47.65, -122.14)
    assert abs((lat - 47.65) - 1000.0 / nfz.M_PER_DEG_LAT) < 1e-9
    # 向东 1000m：经度增加要除以 cos(φ)
    lat, lon = nfz.ned_to_gps(0.0, 1000.0, 47.65, -122.14)
    expect = 1000.0 / (nfz.M_PER_DEG_LAT * math.cos(math.radians(47.65)))
    assert abs((lon - (-122.14)) - expect) < 1e-9


def test_ned_to_gps_old_scale_error_is_corrected():
    # 旧版把经度按 0.00001°/m 换算（隐含 1° 经度 = 100km），
    # 真实值在北纬 47.65° 处为 1° ≈ 74.9km：从"每度米数"看旧版
    # 高估约 33%，从"度/米换算系数"看低估约 25%。新换算应无偏差。
    y = 1000.0
    _, lon_new = nfz.ned_to_gps(0.0, y, 47.65, -122.14)
    lon_old = -122.14 + y * 0.00001          # 旧版写法
    lon_truth = -122.14 + y / (nfz.M_PER_DEG_LAT * math.cos(math.radians(47.65)))
    err_new = abs(lon_new - lon_truth)
    err_old = abs(lon_old - lon_truth)
    assert err_new < 1e-9
    assert err_old > 1e-6                    # 旧版确实存在可观偏差
    rel = err_old / abs(lon_truth + 122.14)
    assert rel > 0.25                        # 度数偏差超过 25%
