"""禁飞区检测模块 (No-Fly Zone Check)

修复说明：旧版 check_mission_zones 只检查取货点/送货点两个端点是否
落在禁飞区内——两端点都安全、但两点之间的直线航线正穿禁飞区的任务
会被误判为"安全"。本版接入共享核心 nfz_core（位于 src/air/no_fly_zone_check/），
对起降点与每一段航线分别做球面最短距离检测，同时支持安全裕量 buffer。

用法：python main.py
"""

import math
import os
import sys

# 引入共享核心（src/air/no_fly_zone_check/nfz_core.py，纯标准库实现）
sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'air',
                    'no_fly_zone_check'))
import nfz_core as nfz  # noqa: E402

# ============ 禁飞区定义 ============
NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]

# 安全裕量：航线距禁区中心 <= radius + buffer 即判违规
SAFETY_BUFFER_M = 200.0


def distance_m(lat1, lon1, lat2, lon2):
    """Haversine 公式：计算两点间球面距离（米）"""
    return nfz.haversine_m(lat1, lon1, lat2, lon2)


def check_no_fly_zone(lat, lon):
    """判断单个坐标是否落在禁飞区内（保留旧接口）"""
    hits = nfz.check_point(lat, lon, NO_FLY_ZONES)
    if hits:
        return hits[0]['name'], hits[0]['dist_m']
    return None, None


def check_mission_zones(pickup, delivery, buffer_m=SAFETY_BUFFER_M,
                        waypoints=None):
    """任务级校验：起降点端点 + 航线段逐段检测（修复旧版漏报）"""
    ok, report = nfz.check_mission(pickup, delivery, NO_FLY_ZONES,
                                   buffer_m=buffer_m, waypoints=waypoints)
    if ok:
        return True, 'Both points and the route are safe'
    msgs = []
    for v in report:
        if v['kind'] == 'point':
            msgs.append('%s point is inside no-fly zone [%s] (%.1f m from '
                        'center)' % (v.get('label', '?'), v['name'],
                                     v['dist_m']))
        else:
            msgs.append('route leg %d passes within %.1f m of no-fly zone '
                        '[%s]' % (v['leg'] + 1, v['min_dist_m'], v['name']))
    return False, '; '.join(msgs)


def main():
    print('=' * 50)
    print('No-Fly Zone Check Module (route-aware version)')
    print('=' * 50)
    print('Loaded %d no-fly zones:' % len(NO_FLY_ZONES))
    for z in NO_FLY_ZONES:
        print("  - %-13s center=(%s, %s) radius=%sm"
              % (z['name'], z['lat'], z['lon'], z['radius_m']))
    print()

    test_cases = [
        ('Airport center',       47.65, -122.14),
        ('Near Airport',         47.66, -122.13),
        ('Military base center', 47.60, -122.20),
        ('School center',        47.62, -122.10),
        ('Safe location (CN)',   30.00, 120.00),
        ('Safe location (US)',   40.00, -100.00),
    ]

    print('-' * 50)
    print('Point Test Results')
    print('-' * 50)
    for name, lat, lon in test_cases:
        zone, dist = check_no_fly_zone(lat, lon)
        if zone:
            print('[WARNING] %-24s (%s, %s) -> INSIDE zone [%s] (%.1f m)'
                  % (name, lat, lon, zone, dist))
        else:
            print('[OK]      %-24s (%s, %s) -> safe' % (name, lat, lon))

    print()
    print('-' * 50)
    print('Mission check: old bug regression (endpoints safe, route crosses)')
    print('-' * 50)
    pickup = (47.65, -122.17)    # 机场以西约 2.2km，端点安全
    delivery = (47.65, -122.11)  # 机场以东约 2.2km，端点安全
    ok, msg = check_mission_zones(pickup, delivery)
    print('Pickup=%s, Delivery=%s' % (pickup, delivery))
    print('Result: %s' % msg)

    print()
    print('-' * 50)
    print('Mission check with detour waypoints (route avoids the zone)')
    print('-' * 50)
    # 三段绕行：正北飞离机场经度圈 -> 机场以北 7.8km 处正东穿越 -> 正南下降
    # 每一段与机场中心的最短距离都 >= 约 2245m > 半径 2000m + 裕量 200m
    w1 = (47.72, -122.17)
    w2 = (47.72, -122.11)
    ok, msg = check_mission_zones(pickup, delivery, waypoints=[w1, w2])
    print('Waypoints: %s -> %s -> %s -> %s' % (pickup, w1, w2, delivery))
    print('Result: %s' % msg)


if __name__ == '__main__':
    main()
