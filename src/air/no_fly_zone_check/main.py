"""禁飞区检测（AirSim 在线版 / 离线演示版）

修复说明（相对旧版）：
1. NED → 经纬度换算改用 nfz.ned_to_gps：旧版 x/y 同比例折算，
   忽略经度方向 1° = 111.32·cos(φ) km，北纬 47.65° 处经度偏大 ~33%；
2. 任务校验从"只查起降点"升级为"起降点 + 航线段逐段检测"
   （nfz.check_mission），端点安全但航线穿越禁飞区的任务会被拦截。

用法：
  python main.py            # 优先连接 AirSim；连不上则自动进入离线演示
  python main.py --offline  # 强制离线演示（不尝试连接模拟器）
"""
import math
import sys

import nfz_core as nfz

# ============ 禁飞区定义 ============
NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]

ORIGIN_LAT, ORIGIN_LON = 47.65, -122.14   # NED 原点（示例场景）
WAYPOINT_M_LAT = 0.00001 * 1.0            # 兼容旧版场景的坐标尺度参考


def _print_zones():
    print('=' * 62)
    print('No-Fly Zone Check (航线穿越校验版)')
    print('=' * 62)
    print('已加载 %d 个禁飞区:' % len(NO_FLY_ZONES))
    for z in NO_FLY_ZONES:
        print("  - %-13s center=(%.4f, %.4f) radius=%dm"
              % (z['name'], z['lat'], z['lon'], z['radius_m']))
    print()


def _get_drone_position_offline():
    """离线演示：返回一个固定的 NED 位置（模拟模拟器读数）"""
    return {'x_val': -800.0, 'y_val': 600.0, 'z_val': -120.0}


def run_with_airsim():
    """连接 AirSim，读取无人机位置并检测"""
    import airsim
    print('[INFO] 正在连接 AirSim 模拟器 (127.0.0.1:41451)...')
    client = airsim.MultirotorClient(ip='127.0.0.1', port=41451)
    client.confirmConnection()
    print('[INFO] AirSim 模拟器连接成功！')
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    print('[INFO] 模拟器返回 NED 坐标系: x=%.2fm, y=%.2fm, z=%.2fm'
          % (pos.x_val, pos.y_val, pos.z_val))
    return {'x_val': pos.x_val, 'y_val': pos.y_val, 'z_val': pos.z_val}


def check_drone_position(ned):
    """NED 位置 → 经纬度（修正换算）→ 单点检测"""
    drone_lat, drone_lon = nfz.ned_to_gps(
        ned['x_val'], ned['y_val'], ORIGIN_LAT, ORIGIN_LON)
    print('[INFO] 换算为经纬度坐标(修正 cos(φ) 后): (%.6f, %.6f)'
          % (drone_lat, drone_lon))
    hits = nfz.check_point(drone_lat, drone_lon, NO_FLY_ZONES)
    if hits:
        h = hits[0]
        print('[警告] 无人机在禁飞区【%s】内，距中心 %.1f 米！请立即返航！'
              % (h['name'], h['dist_m']))
    else:
        print('[安全] 无人机不在任何禁飞区内。')


def demo_mission_check():
    """演示旧版漏报场景：端点安全但直线航线穿越机场禁飞区"""
    print('-' * 62)
    print('任务级校验演示（旧版只查端点，本版逐段查航线）')
    print('-' * 62)
    pickup = (47.65, -122.17)     # 机场以西约 2.2km，端点安全
    delivery = (47.65, -122.11)   # 机场以东约 2.2km，端点安全
    for label, (lat, lon) in (('Pickup', pickup), ('Delivery', delivery)):
        hits = nfz.check_point(lat, lon, NO_FLY_ZONES)
        status = '安全' if not hits else '位于 ' + hits[0]['name']
        print('  %-8s (%.4f, %.4f) -> %s' % (label, lat, lon, status))

    # 旧版逻辑到此为止会判定"安全"；下面逐段检查航线
    ok, report = nfz.check_mission(pickup, delivery, NO_FLY_ZONES)
    if ok:
        print('  [通过] 航线未穿越任何禁飞区')
    else:
        for v in report:
            if v['kind'] == 'path':
                print('  [拦截] 第 %d 段航线距禁飞区【%s】中心仅 %.0f 米'
                      ' (< 半径 %dm)，航线穿越禁飞区！'
                      % (v['leg'] + 1, v['name'], v['min_dist_m'],
                         [z for z in NO_FLY_ZONES
                          if z['name'] == v['name']][0]['radius_m']))

    # 绕行航路点方案：向东北绕开机场后校验通过
    detour = (47.72, -122.14)
    ok2, _ = nfz.check_mission(pickup, delivery, NO_FLY_ZONES,
                               waypoints=[detour])
    print('  [对比] 增加绕行航路点 (%.4f, %.4f) 后 -> %s'
          % (detour[0], detour[1], '校验通过' if ok2 else '仍不通过'))


def main():
    offline = '--offline' in sys.argv
    _print_zones()
    ned = None
    if not offline:
        try:
            ned = run_with_airsim()
        except Exception as exc:
            print('[提示] 无法连接 AirSim（%s），转入离线演示模式。' % exc)
    if ned is None:
        ned = _get_drone_position_offline()
        print('[离线演示] 使用模拟 NED 位置: x=%.1fm, y=%.1fm'
              % (ned['x_val'], ned['y_val']))
    check_drone_position(ned)
    demo_mission_check()


if __name__ == '__main__':
    main()
