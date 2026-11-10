"""
禁飞区检测模块 (No-Fly Zone Check)

功能：
1) 单点检测：给定一个地理坐标，判断它是否落在预设的禁飞区内；
2) 航线检测：给定取货点 / 送货点，判断**整条直线航线**是否穿越禁飞区。
   旧版本只检查两个端点，航线从禁飞区中间穿过（两端都在区外）时会漏判。

算法：
* 球面距离：Haversine 公式；
* 航线穿越：以禁飞区中心为原点做局部等距投影，再算“点到线段”的最短距离。

用法：
    python main.py                                          # 内置示例
    python main.py --pickup 47.65,-122.17 --dropoff 47.65,-122.11
    python main.py --pickup ... --dropoff ... --json         # 机器可读输出
"""

import argparse
import json
import math
import sys

# ============ 禁飞区定义 ============
NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]

EARTH_RADIUS_M = 6371000

REASON_LABELS = {
    'pickup_inside': '取货点在禁飞区内',
    'delivery_inside': '送货点在禁飞区内',
    'path_crosses': '航线穿越禁飞区',
}


def distance_m(lat1, lon1, lat2, lon2):
    """Haversine 公式：计算两点间球面距离（米）"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def check_no_fly_zone(lat, lon, zones=None):
    """判断坐标是否在禁飞区内，返回 (zone_name, distance) 或 (None, None)。"""
    for zone in (NO_FLY_ZONES if zones is None else zones):
        d = distance_m(lat, lon, zone['lat'], zone['lon'])
        if d <= zone['radius_m']:
            return zone['name'], d
    return None, None


# ============ 航线穿越检测 ============
def _project_to_local(lat, lon, zone):
    """把经纬度投影到以禁飞区中心为原点的局部平面（米，等距近似）。

    禁飞区半径通常只有几百米到几公里，这个近似在几公里范围内的误差
    远小于“是否穿越”的判定裕度；比逐点采样更简单，也不会漏掉细长的穿越。
    """
    x = math.radians(lon - zone['lon']) * math.cos(math.radians(zone['lat'])) * EARTH_RADIUS_M
    y = math.radians(lat - zone['lat']) * EARTH_RADIUS_M
    return x, y


def segment_min_distance_m(pickup, delivery, zone):
    """航线（球面直线在局部平面的投影）到禁飞区中心的最短距离（米）。"""
    ax, ay = _project_to_local(pickup[0], pickup[1], zone)
    bx, by = _project_to_local(delivery[0], delivery[1], zone)
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(ax, ay)
    # 把禁区中心 (0,0) 投影到线段上，t 夹到 [0,1] 即“点到线段”
    t = max(0.0, min(1.0, -(ax * dx + ay * dy) / length_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(cx, cy)


def check_route(pickup, delivery, zones=None):
    """检查整条航线：端点是否在禁飞区内，以及航线是否从禁飞区中穿过。

    返回 ``{'safe': bool, 'zones_checked': int, 'violations': [...]}``；
    每条违规包含 zone / reason / distance_m / radius_m。
    """
    zones = NO_FLY_ZONES if zones is None else zones
    violations = []
    for zone in zones:
        inside = None
        for reason, point in (('pickup_inside', pickup), ('delivery_inside', delivery)):
            d = distance_m(point[0], point[1], zone['lat'], zone['lon'])
            if d <= zone['radius_m']:
                inside = (reason, d)
                break
        if inside is not None:
            violations.append({
                'zone': zone['name'],
                'reason': inside[0],
                'distance_m': round(inside[1], 1),
                'radius_m': zone['radius_m'],
            })
            continue
        d = segment_min_distance_m(pickup, delivery, zone)
        if d <= zone['radius_m']:
            violations.append({
                'zone': zone['name'],
                'reason': 'path_crosses',
                'distance_m': round(d, 1),
                'radius_m': zone['radius_m'],
            })
    return {'safe': not violations, 'zones_checked': len(zones),
            'violations': violations}


def check_mission_zones(pickup, delivery):
    """兼容旧接口：只返回 (ok, message)，内部已升级为整条航线检测。"""
    result = check_route(pickup, delivery)
    if result['safe']:
        return True, 'Both points and the route are safe'
    first = result['violations'][0]
    return False, '{} [{zone}] ({distance_m:.1f} m, radius {radius_m} m)'.format(
        REASON_LABELS[first['reason']], **first)


def parse_latlon(text):
    """解析 "纬度,经度" 文本，返回 (lat, lon)。"""
    parts = [p.strip() for p in str(text).replace('，', ',').split(',')]
    if len(parts) != 2:
        raise ValueError('坐标格式应为 "纬度,经度"：%r' % text)
    try:
        lat, lon = float(parts[0]), float(parts[1])
    except ValueError:
        raise ValueError('坐标必须是数字：%r' % text)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError('经纬度超出范围：%r' % text)
    return lat, lon


def print_route_report(pickup, delivery, result):
    """打印航线检测结果。"""
    print('=' * 50)
    print('航线禁飞区检测')
    print('=' * 50)
    print('取货点: ({:.5f}, {:.5f})'.format(pickup[0], pickup[1]))
    print('送货点: ({:.5f}, {:.5f})'.format(delivery[0], delivery[1]))
    print('检查禁飞区数量: {}'.format(result['zones_checked']))
    print('-' * 50)
    if result['safe']:
        print('[OK] 航线安全：起止点与整条航线都不在禁飞区内')
        return
    for item in result['violations']:
        print('[WARNING] {}【{}】：距离中心 {:.1f} m（半径 {} m）'.format(
            REASON_LABELS[item['reason']], item['zone'],
            item['distance_m'], item['radius_m']))


def _demo():
    print('=' * 50)
    print('No-Fly Zone Check Module')
    print('=' * 50)
    print('Loaded {} no-fly zones:'.format(len(NO_FLY_ZONES)))
    for z in NO_FLY_ZONES:
        print("  - {name:14s} center=({lat}, {lon}) radius={radius_m}m".format(**z))
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
    print('单点检测')
    print('-' * 50)
    for name, lat, lon in test_cases:
        zone, dist = check_no_fly_zone(lat, lon)
        if zone:
            print('[WARNING] {name:24s} ({lat}, {lon}) -> INSIDE zone [{zone}] ({dist:.1f} m)'
                  .format(name=name, lat=lat, lon=lon, zone=zone, dist=dist))
        else:
            print('[OK]      {name:24s} ({lat}, {lon}) -> safe'.format(
                name=name, lat=lat, lon=lon))

    print()
    print('-' * 50)
    print('航线穿越检测（两端都在区外，但航线从禁区中穿过）')
    print('-' * 50)
    crossing = ((47.65, -122.17), (47.65, -122.11))
    print_route_report(crossing[0], crossing[1], check_route(*crossing))

    print()
    print('-' * 50)
    print('航线安全示例')
    print('-' * 50)
    safe = ((47.70, -122.25), (47.70, -122.05))
    print_route_report(safe[0], safe[1], check_route(*safe))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='禁飞区检测：单点 / 航线穿越（Haversine + 点到线段距离）')
    parser.add_argument('--pickup', help='取货点 "纬度,经度"，例如 47.65,-122.17')
    parser.add_argument('--dropoff', help='送货点 "纬度,经度"，例如 47.65,-122.11')
    parser.add_argument('--json', action='store_true', help='以 JSON 输出结果')
    args = parser.parse_args(argv)

    if args.pickup or args.dropoff:
        if not (args.pickup and args.dropoff):
            parser.error('--pickup 与 --dropoff 需要同时提供')
        try:
            pickup = parse_latlon(args.pickup)
            delivery = parse_latlon(args.dropoff)
        except ValueError as exc:
            parser.error(str(exc))
        result = check_route(pickup, delivery)
        if args.json:
            print(json.dumps(result, ensure_ascii=False, indent=2))
        else:
            print_route_report(pickup, delivery, result)
        return 0 if result['safe'] else 1

    _demo()
    return 0


if __name__ == '__main__':
    sys.exit(main())
