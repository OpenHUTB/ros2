"""
禁飞区检测模块 (No-Fly Zone Check)

功能：给定一个地理坐标，判断它是否落在预设的禁飞区内。
算法：Haversine 公式计算球面距离。
用法：python main.py
"""

import math

# ============ 禁飞区定义 ============
NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]

EARTH_RADIUS_M = 6371000


def distance_m(lat1, lon1, lat2, lon2):
    """Haversine 公式：计算两点间球面距离（米）"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def check_no_fly_zone(lat, lon):
    for zone in NO_FLY_ZONES:
        d = distance_m(lat, lon, zone['lat'], zone['lon'])
        if d <= zone['radius_m']:
            return zone['name'], d
    return None, None


def check_mission_zones(pickup, delivery):
    for label, (lat, lon) in [('Pickup', pickup), ('Delivery', delivery)]:
        zone, dist = check_no_fly_zone(lat, lon)
        if zone:
            return False, f'{label} point is inside no-fly zone [{zone}] ' \
                          f'({dist:.1f} m from center)'
    return True, 'Both points are safe'


def main():
    print('=' * 50)
    print('No-Fly Zone Check Module')
    print('=' * 50)
    print(f'Loaded {len(NO_FLY_ZONES)} no-fly zones:')
    for z in NO_FLY_ZONES:
        print(f"  - {z['name']:14s} center=({z['lat']}, {z['lon']}) "
              f"radius={z['radius_m']}m")
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
    print('Test Results')
    print('-' * 50)
    for name, lat, lon in test_cases:
        zone, dist = check_no_fly_zone(lat, lon)
        if zone:
            print(f'[WARNING] {name:24s} ({lat}, {lon}) '
                  f'-> INSIDE zone [{zone}] ({dist:.1f} m)')
        else:
            print(f'[OK]      {name:24s} ({lat}, {lon}) -> safe')

    print()
    print('-' * 50)
    print('Mission check example')
    print('-' * 50)
    pickup = (47.65, -122.14)
    delivery = (30.00, 120.00)
    ok, msg = check_mission_zones(pickup, delivery)
    print(f'Pickup={pickup}, Delivery={delivery}')
    print(f'Result: {msg}')


if __name__ == '__main__':
    main()