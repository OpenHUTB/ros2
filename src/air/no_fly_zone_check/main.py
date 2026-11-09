import math
import airsim
import csv
from datetime import datetime

NO_FLY_ZONES = [
    {'name': 'Airport',      'lat': 47.65, 'lon': -122.14, 'radius_m': 2000},
    {'name': 'MilitaryBase', 'lat': 47.60, 'lon': -122.20, 'radius_m': 1500},
    {'name': 'School',       'lat': 47.62, 'lon': -122.10, 'radius_m': 500},
]
EARTH_RADIUS_M = 6371000

def distance_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))

def check_no_fly_zone(lat, lon):
    for zone in NO_FLY_ZONES:
        d = distance_m(lat, lon, zone['lat'], zone['lon'])
        if d <= zone['radius_m']:
            return zone['name'], d
    return None, None

def check_proximity(lat, lon, threshold=500):
    for zone in NO_FLY_ZONES:
        d = distance_m(lat, lon, zone['lat'], zone['lon'])
        dist_to_edge = d - zone['radius_m']
        if 0 < dist_to_edge <= threshold:
            return zone['name'], dist_to_edge
    return None, None

def log_violation(zone_name, lat, lon, dist):
    file_exists = False
    try:
        with open('violation_log.csv', 'r', encoding='utf-8') as f:
            file_exists = True
    except FileNotFoundError:
        pass
    
    with open('violation_log.csv', 'a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['Time', 'Zone', 'Latitude', 'Longitude', 'Distance(m)'])
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        writer.writerow([current_time, zone_name, round(lat, 5), round(lon, 5), round(dist, 2)])

def main():
    print("[INFO] 正在连接 AirSim 模拟器 (127.0.0.1:41451)...")
    client = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
    
    try:
        client.confirmConnection()
        print("[INFO] AirSim 模拟器连接成功！")
    except Exception as e:
        print(f"[ERROR] 无法连接 AirSim 模拟器。请确保已启动 AirSim 1.8.1 模拟环境。")
        return

    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    drone_lat = 47.65 + pos.x_val * 0.00001
    drone_lon = -122.14 + pos.y_val * 0.00001
    
    print(f"[INFO] 从模拟器获取到无人机位置: ({drone_lat:.5f}, {drone_lon:.5f})")

    zone, dist = check_no_fly_zone(drone_lat, drone_lon)
    if zone:
        print(f"[警告] 无人机在禁飞区【{zone}】内，距离中心 {dist:.1f} 米！请立即返航！")
        log_violation(zone, drone_lat, drone_lon, dist)
        print("[INFO] 违规记录已保存至 violation_log.csv")
    else:
        near_zone, near_dist = check_proximity(drone_lat, drone_lon)
        if near_zone:
            print(f"[预警] 无人机接近禁飞区【{near_zone}】，距离边界 {near_dist:.1f} 米，请谨慎驾驶！")
        else:
            print("[安全] 无人机不在禁飞区内且距离安全。")

if __name__ == '__main__':
    main()