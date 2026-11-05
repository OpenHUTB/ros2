import math
import airsim  # 引入 AirSim 模拟器官方 Python 客户端库

# ============ 禁飞区定义 ============
NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]
EARTH_RADIUS_M = 6371000

def distance_m(lat1, lon1, lat2, lon2):
    """Haversine 公式：计算两点间球面距离（米）"""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))

def check_no_fly_zone(lat, lon):
    """判断坐标是否在禁飞区内"""
    for zone in NO_FLY_ZONES:
        d = distance_m(lat, lon, zone['lat'], zone['lon'])
        if d <= zone['radius_m']:
            return zone['name'], d
    return None, None

def main():
    # ========== 关键修改：连接 AirSim 模拟器 ==========
    print("[INFO] 正在连接 AirSim 模拟器 (127.0.0.1:41451)...")
    # 使用 AirSim 官方接口实例化无人机客户端
    client = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
    
    try:
        # 确认连接（这是判断模拟器是否真实运行的核心接口）
        client.confirmConnection()
        print("[INFO] AirSim 模拟器连接成功！")
    except Exception as e:
        print(f"[ERROR] 无法连接 AirSim 模拟器。请确保已启动 AirSim 1.8.1 模拟环境。")
        print(f"[ERROR] 异常信息: {e}")
        return

    # ========== 从模拟器获取无人机位置 ==========
    print("[INFO] 正在从模拟器获取无人机状态...")
    state = client.getMultirotorState()
    
    # 获取 NED 坐标系下的位置数据（单位：米）
    pos = state.kinematics_estimated.position
    print(f"[INFO] 模拟器返回 NED 坐标系: x={pos.x_val:.2f}m, y={pos.y_val:.2f}m, z={pos.z_val:.2f}m")

    # 将模拟器坐标映射为经纬度（这里假定禁飞区附近的初始经纬度作为原点进行换算）
    # 严谨说明：实际场景应使用无人机搭载的 GPS 模块获取真实经纬度
    drone_lat = 47.65 + pos.x_val * 0.00001
    drone_lon = -122.14 + pos.y_val * 0.00001
    print(f"[INFO] 换算为经纬度坐标: ({drone_lat:.5f}, {drone_lon:.5f})")

    # ========== 执行禁飞区检测 ==========
    zone, dist = check_no_fly_zone(drone_lat, drone_lon)
    if zone:
        print(f"[警告] 无人机在禁飞区【{zone}】内，距离中心 {dist:.1f} 米！请立即返航！")
    else:
        print("[安全] 无人机不在禁飞区内。")

if __name__ == '__main__':
    main()