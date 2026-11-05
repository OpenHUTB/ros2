#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""禁飞区检测模块的本地测试（纯标准库，不需要 AirSim / ROS / 网络）.

覆盖：
    A. Haversine 距离
    B. 单点检测（含边界 = 半径）
    C. 航线到禁区中心的最短距离（点到线段）
    D. 航线穿越检测（两端在外、路径穿过 / 路径擦边在外 / 端点在内）
    E. 命令行入口：参数解析、退出码、JSON 输出
    F. 工程文件：README 与文档已同步说明新功能

用法: python3 tests/test_no_fly_zone_check_local.py
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import os
import sys

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "no_fly_zone_check")
sys.path.insert(0, PKG)

import main as nfz  # noqa: E402

R = nfz.EARTH_RADIUS_M
ZONE_T = {'name': 'TestZone', 'lat': 0.0, 'lon': 0.0, 'radius_m': 1000}


def offset_lat(deg):
    """赤道上纬度偏移（度）对应的米数。"""
    return math.radians(deg) * R


# ============================================================ A. 距离
def test_distance():
    print("== A. Haversine 距离 ==")
    check(nfz.distance_m(30.0, 120.0, 30.0, 120.0) == 0.0, "同一点距离为 0")
    one_deg = nfz.distance_m(0.0, 0.0, 1.0, 0.0)
    check(abs(one_deg - math.radians(1.0) * R) < 1e-6, "赤道 1° 纬差 = R·π/180")
    forward = nfz.distance_m(47.0, -122.0, 47.5, -121.5)
    backward = nfz.distance_m(47.5, -121.5, 47.0, -122.0)
    check(abs(forward - backward) < 1e-9, "距离对称")
    # 北京 -> 上海 约 1067 km（容差 3%）
    bj_sh = nfz.distance_m(39.90, 116.40, 31.23, 121.47)
    check(abs(bj_sh - 1067000.0) / 1067000.0 < 0.03, "北京-上海距离量级正确（%.0f km）" % (bj_sh / 1000))


# ============================================================ B. 单点检测
def test_point_check():
    print("== B. 单点检测 ==")
    zone, dist = nfz.check_no_fly_zone(47.65, -122.14)          # Airport 中心
    check(zone == 'Airport' and dist < 1.0, "禁区中心 -> 命中 Airport")
    zone2, _ = nfz.check_no_fly_zone(30.0, 120.0)               # 杭州附近
    check(zone2 is None, "远处坐标 -> 不命中")
    inside_name, _ = nfz.check_no_fly_zone(0.0, 0.0, [ZONE_T])
    check(inside_name == 'TestZone', "自定义禁区：中心命中")

    edge = math.degrees(1000.0 / R)                             # 恰好半径处（北向）
    hit, d = nfz.check_no_fly_zone(edge, 0.0, [ZONE_T])
    check(hit == 'TestZone', "恰好等于半径 -> 判为在区内（<=）")
    outside_lat = math.degrees(1000.0 / R) + 1e-6               # 略超半径
    miss, _ = nfz.check_no_fly_zone(outside_lat, 0.0, [ZONE_T])
    check(miss is None, "略超半径 -> 判为在区外")


# ============================================================ C. 最短距离
def test_segment_distance():
    print("== C. 航线到禁区中心的最短距离 ==")
    through = nfz.segment_min_distance_m((0.0, -0.02), (0.0, 0.02), ZONE_T)
    check(through < 1e-6, "穿过中心的航线 -> 最短距离 ≈ 0")
    parallel = nfz.segment_min_distance_m((0.01, -0.02), (0.01, 0.02), ZONE_T)
    check(abs(parallel - offset_lat(0.01)) < 1.0, "平行航线 -> 最短距离 = 垂直偏移")
    diagonal = nfz.segment_min_distance_m((0.02, 0.0), (0.0, 0.02), ZONE_T)
    expected = offset_lat(0.02) / math.sqrt(2.0)
    check(abs(diagonal - expected) < 1.0, "斜穿航线 -> 垂足落在线段内时距离正确")
    beyond = nfz.segment_min_distance_m((0.02, 0.02), (0.03, 0.03), ZONE_T)
    check(abs(beyond - math.hypot(offset_lat(0.02), offset_lat(0.02))) < 1.0,
          "垂足在线段外 -> 取最近端点")
    degenerate = nfz.segment_min_distance_m((0.0, 0.01), (0.0, 0.01), ZONE_T)
    check(abs(degenerate - offset_lat(0.01)) < 1.0, "起终点重合 -> 退化为点距离")


# ============================================================ D. 航线检测
def test_route_check():
    print("== D. 航线穿越检测 ==")
    zones = [ZONE_T]

    result = nfz.check_route((0.0, -0.02), (0.0, 0.02), zones)
    check(result['safe'] is False, "两端在区外但航线穿过 -> 不安全")
    check(len(result['violations']) == 1, "每命中一个禁区记一条违规")
    check(result['violations'][0]['reason'] == 'path_crosses', "reason = path_crosses")
    check(result['violations'][0]['zone'] == 'TestZone', "违规记录带禁区名")
    check(result['zones_checked'] == 1, "zones_checked 正确")

    safe = nfz.check_route((0.01, -0.02), (0.01, 0.02), zones)
    check(safe['safe'] is True and safe['violations'] == [], "航线在禁区外 -> 安全")

    diagonal = nfz.check_route((0.02, 0.0), (0.0, 0.02), zones)
    check(diagonal['safe'] is True, "斜穿但最近距离 1.57 km > 半径 1 km -> 安全")

    pickup_in = nfz.check_route((0.0, 0.0), (0.0, 0.02), zones)
    check(pickup_in['violations'][0]['reason'] == 'pickup_inside', "取货点在区内 -> pickup_inside")
    delivery_in = nfz.check_route((0.0, 0.02), (0.0, 0.0), zones)
    check(delivery_in['violations'][0]['reason'] == 'delivery_inside', "送货点在区内 -> delivery_inside")

    both = nfz.check_route((0.0, 0.0), (0.0, 0.0), zones)
    check(len(both['violations']) == 1, "端点在内时不重复记 path_crosses")

    two = [ZONE_T, {'name': 'Second', 'lat': 0.001, 'lon': 0.0, 'radius_m': 500}]
    multi = nfz.check_route((0.0, -0.02), (0.0, 0.02), two)
    check(len(multi['violations']) == 2 and multi['zones_checked'] == 2,
          "多个禁区都被穿越 -> 各记一条")

    legacy_ok, legacy_msg = nfz.check_mission_zones((47.70, -122.25), (47.70, -122.05))
    check(legacy_ok is True and 'safe' in legacy_msg.lower(), "旧接口：安全航线返回 True")
    legacy_bad, _ = nfz.check_mission_zones((47.65, -122.17), (47.65, -122.11))
    check(legacy_bad is False, "旧接口：穿越航线返回 False")


# ============================================================ E. 命令行
def test_cli():
    print("== E. 命令行入口 ==")
    check(nfz.parse_latlon('47.65,-122.14') == (47.65, -122.14), "解析 \"纬度,经度\"")
    check(nfz.parse_latlon(' 47.65，-122.14 ') == (47.65, -122.14), "兼容中文逗号与空格")
    for bad in ('47.65', 'a,b', '91,0', '0,181'):
        try:
            nfz.parse_latlon(bad)
            check(False, "非法坐标应报错: %r" % bad)
        except ValueError:
            check(True, "非法坐标报 ValueError: %r" % bad)

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        code = nfz.main(['--pickup', '47.65,-122.17', '--dropoff', '47.65,-122.11'])
    text = buffer.getvalue()
    check(code == 1, "穿越航线退出码 = 1")
    check('WARNING' in text and 'Airport' in text, "穿越航线打印警告")

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        code = nfz.main(['--pickup', '47.70,-122.25', '--dropoff', '47.70,-122.05'])
    check(code == 0 and '[OK]' in buffer.getvalue(), "安全航线退出码 = 0 且打印 OK")

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        code = nfz.main(['--pickup', '47.65,-122.17', '--dropoff', '47.65,-122.11', '--json'])
    payload = json.loads(buffer.getvalue())
    check(code == 1 and payload['safe'] is False, "--json 输出可解析且为不安全")
    check(payload['violations'][0]['reason'] == 'path_crosses', "--json 包含 reason")


# ============================================================ F. 工程文件
def test_files():
    print("== F. 工程文件 ==")
    readme = open(os.path.join(PKG, 'README.md'), encoding='utf-8').read()
    check('航线' in readme and '--pickup' in readme and '穿越' in readme,
          "README 说明航线穿越检测与命令行用法")
    doc_path = os.path.join(ROOT, 'docs', 'no_fly_zone_check', 'index.md')
    doc = open(doc_path, encoding='utf-8').read()
    check('航线穿越' in doc and ('点到线段' in doc or 'segment' in doc.lower()),
          "文档补充航线穿越原理")
    check('--pickup' in doc, "文档给出命令行示例")


def main():
    test_distance()
    test_point_check()
    test_segment_distance()
    test_route_check()
    test_cli()
    test_files()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for item in FAIL:
            print("  FAIL: %s" % item)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
