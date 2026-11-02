"""禁飞区检测纯逻辑核心（nfz_core）

旧版 main.py 的两个安全隐患（本模块修复的对象）：

1. 任务校验只检查起降点端点 —— 取货点/送货点都安全，但两点之间的
   直线航线可能直穿禁飞区（例如正对机场跑道方向），旧版会误判"安全"。
2. AirSim 版的 NED → 经纬度换算把 x/y 用同一比例折算，忽略了纬度方向
   与经度方向的"每度米数"不同（经度 1 度 = 111.32·cos(φ) km），
   在北纬 47.65° 处经度换算偏大约 33%。

本模块只做纯计算（无 ROS / AirSim 依赖），提供：

- haversine_m            两点球面距离
- bearing_rad            两点间初始方位角
- cross_track_m          点到航线段的符号交叉距离
- min_dist_to_segment_m  点到大圆航线段的最短距离（含端点情形）
- check_point            单点禁飞区检测
- check_path             多段航线的逐段禁飞区检测（核心修复）
- check_mission          任务级校验：端点 + 航线段一起查
- ned_to_gps             NED(米) → 经纬度(度)，含 cos(φ) 修正
"""

import math

EARTH_RADIUS_M = 6371000.0
#: 1 度纬度对应的地表弧长（米）≈ 111194.9
M_PER_DEG_LAT = math.pi * EARTH_RADIUS_M / 180.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Haversine 公式：计算两点间球面距离（米）"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, a)))


def bearing_rad(lat1, lon1, lat2, lon2):
    """从点 1 指向点 2 的初始方位角（弧度，北为 0，顺时针为正）"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return math.atan2(y, x)


def _cross_along_track_m(clat, clon, alat, alon, blat, blon):
    """计算点 C 相对大圆航线 A→B 的符号交叉距离与沿线距离（米）。

    返回 (dxt, dat)：
      dxt —— 交叉距离，C 在航线左侧为正（符号按方位角差），0 表示在航线上；
      dat —— C 的投影沿航线距 A 的弧长，可为负（投影在 A 之前）。
    """
    d13 = haversine_m(alat, alon, clat, clon)
    if d13 < 1e-9:
        return 0.0, 0.0
    theta13 = bearing_rad(alat, alon, clat, clon)
    theta12 = bearing_rad(alat, alon, blat, blon)
    dtheta = theta13 - theta12
    sin_d13 = math.sin(d13 / EARTH_RADIUS_M)
    # 符号交叉距离：|dxt| 是点到无限长大圆的距离
    dxt = math.asin(max(-1.0, min(1.0, sin_d13 * math.sin(dtheta)))) * EARTH_RADIUS_M
    cos_dxt = math.cos(dxt / EARTH_RADIUS_M)
    if abs(cos_dxt) < 1e-12:      # 交叉距离接近半个周长，退化情形
        dat = d13 * math.cos(dtheta)
    else:
        ratio = math.cos(d13 / EARTH_RADIUS_M) / cos_dxt
        dat = math.acos(max(-1.0, min(1.0, ratio))) * EARTH_RADIUS_M
        if math.cos(dtheta) < 0:  # 投影在 A 的反方向
            dat = -dat
    return dxt, dat


def min_dist_to_segment_m(clat, clon, alat, alon, blat, blon):
    """点 C 到大圆航线段 A→B 的最短距离（米）。

    投影落在线段内且交叉距离更小时返回交叉距离；否则返回到较近
    端点的距离。返回 (min_dist, where)，
    where ∈ {'segment', 'endpoint_a', 'endpoint_b'}。
    """
    dxt, dat = _cross_along_track_m(clat, clon, alat, alon, blat, blon)
    d12 = haversine_m(alat, alon, blat, blon)
    d_a = haversine_m(clat, clon, alat, alon)
    d_b = haversine_m(clat, clon, blat, blon)
    best = (d_a, 'endpoint_a')
    if d_b < best[0]:
        best = (d_b, 'endpoint_b')
    if 0.0 < dat < d12:
        d_seg = abs(dxt)
        if d_seg < best[0]:
            best = (d_seg, 'segment')
    return best


def check_point(lat, lon, zones, buffer_m=0.0):
    """单点检测：返回命中的禁飞区列表。

    每个元素为 dict(name, dist_m, kind='point')，按距离升序。
    buffer_m 为安全裕量（正值把禁区向外扩 buffer_m 米）。
    """
    hits = []
    for zone in zones:
        d = haversine_m(lat, lon, zone['lat'], zone['lon'])
        if d <= zone['radius_m'] + buffer_m:
            hits.append({'name': zone['name'], 'dist_m': d, 'kind': 'point'})
    hits.sort(key=lambda h: h['dist_m'])
    return hits


def check_path(points, zones, buffer_m=0.0):
    """多段航线逐段检测（旧版缺失的能力）。

    points 为 [(lat, lon), ...]，至少 2 个点；相邻两点构成一段大圆航线。
    返回违规列表，每个元素为 dict(name, leg, min_dist_m, kind='path')，
    按最短距离升序。min_dist_m 是禁飞区中心到该航段的最短距离，
    小于等于 radius_m + buffer_m 即判违规。
    """
    if len(points) < 2:
        raise ValueError('航线至少需要 2 个航路点')
    violations = []
    for leg in range(len(points) - 1):
        alat, alon = points[leg]
        blat, blon = points[leg + 1]
        for zone in zones:
            min_d, _where = min_dist_to_segment_m(
                zone['lat'], zone['lon'], alat, alon, blat, blon)
            if min_d <= zone['radius_m'] + buffer_m:
                violations.append({
                    'name': zone['name'],
                    'leg': leg,
                    'min_dist_m': min_d,
                    'kind': 'path',
                })
    violations.sort(key=lambda v: v['min_dist_m'])
    return violations


def check_mission(pickup, delivery, zones, buffer_m=0.0, waypoints=None):
    """任务级校验：起降点端点 + 航线段一起查。

    返回 (ok, report)。ok 为 False 时 report 是违规描述列表
    （含 'point' 与 'path' 两类）；ok 为 True 时 report 为空列表。
    waypoints 可选：途经航路点 [(lat, lon), ...]，航线按
    pickup → waypoints... → delivery 分段检测。
    """
    path = [tuple(pickup)] + [tuple(w) for w in (waypoints or [])] + \
           [tuple(delivery)]
    violations = []
    for label, pt in (('Pickup', pickup), ('Delivery', delivery)):
        for hit in check_point(pt[0], pt[1], zones, buffer_m):
            v = dict(hit)
            v['label'] = label
            violations.append(v)
    for v in check_path(path, zones, buffer_m):
        violations.append(v)
    ok = len(violations) == 0
    return ok, violations


def ned_to_gps(x_north_m, y_east_m, origin_lat, origin_lon):
    """NED 平面坐标（米）→ 经纬度（度）。

    旧版把 x、y 用同一比例换算，忽略了经度方向 1 度只有
    111.32·cos(φ) 千米；在北纬 47.65° 处经度会偏大约 33%。
    本函数按方向分别换算：
      dlat = y 北向米数 / M_PER_DEG_LAT
      dlon = 东向米数 / (M_PER_DEG_LAT · cos(φ))
    """
    dlat = x_north_m / M_PER_DEG_LAT
    cos_phi = max(1e-6, math.cos(math.radians(origin_lat)))
    dlon = y_east_m / (M_PER_DEG_LAT * cos_phi)
    return origin_lat + dlat, origin_lon + dlon
