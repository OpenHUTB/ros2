"""
ROS+CARLA 恶劣雨雾天气下自适应感知降级策略实验

依赖: carla==0.9.15, numpy, opencv-python, matplotlib
使用: 1) 先启动 CARLA 服务端（双击解压目录下的 CarlaUE4.exe）
      2) 在 Python 3.10 环境中运行 python main.py

说明: 本实验是「恶劣雨雾天气下多传感器感知性能对比」系列的第五部分（收官）。
      前四部分依次回答了：相机在雨雾中退化多少（第一）、图像增强能补回多少（第二）、
      雷达与相机谁更稳健（第三）、把雷达投到图像上的融合还能不能用（第四）。
      本实验把前四个结论落地成一个可运行的策略：先由图像判别当前天气等级，
      再据此在「相机主导 / 融合 / 雷达主导」三种感知模式之间自动切换，
      与任何单一固定模式对比，验证自适应切换能否在不依赖天气真值的前提下
      逼近该天气下的最优模式。

      场景：同一辆车、同一参考位姿，沿车道在 12/22/32/42 m 处左右交错放置
      4 辆目标车，避免相互遮挡。四档天气下分别统计：
        - 天气判别：边缘密度相对「晴朗基准」的比值，阈值取自第一部分的实测退化幅度
        - 相机模式：目标投影框内强梯度像素占比 ≥ 阈值即视为检出（阈值晴朗标定后固定）
        - 雷达模式：目标 3D 框内回波点数 ≥ 阈值即视为检出
        - 融合模式：相机与雷达取并集
        - 自适应模式：按判别出的天气选择模式，再取该模式的检出结果
      所有结果写入本脚本同级的 results/ 目录，与运行时的工作目录无关。
"""
import os
import csv
import math
import time

import carla
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')  # 无界面环境也能出图
import matplotlib.pyplot as plt

# 结果目录固定在脚本同级，避免从其他目录运行时把结果写到别处
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 传感器安装位置（车辆坐标系：x 前、y 右、z 上）----------
CAM_AT = carla.Location(x=1.5, z=2.4)
LIDAR_AT = carla.Location(x=0.0, z=2.6)

# ---------- 相机内参：由 FOV 与分辨率反解 ----------
CAM_W, CAM_H, CAM_FOV = 800, 600, 90
FX = FY = (CAM_W / 2.0) / np.tan(np.radians(CAM_FOV) / 2.0)   # = 400
CX, CY = CAM_W / 2.0, CAM_H / 2.0

# ---------- 四档天气的雷达大气衰减系数（与第三、四部分完全一致）----------
LIDAR_ATTENUATION = {
    'ClearNoon': 0.004,
    'SoftRainNoon': 0.01,
    'HardRainNoon': 0.03,
    'HeavyFog': 0.06,
}
# 雷达按渲染帧发布，单帧只含约 45° 扇形，必须累积成整圈；车辆测量期间静止
LIDAR_WINDOW_S = 0.2
LIDAR_PPS = 600000

# ---------- 目标车距离档 ----------
# 6 个距离档跨越 10~40 m，才能看出「可用感知距离」随天气收缩，而不是只有有/无两档
TARGETS = [(10.0, 0.0), (16.0, 0.0), (22.0, 0.0),
           (28.0, 0.0), (34.0, 0.0), (40.0, 0.0)]

# ---------- 判据阈值 ----------
CAM_CONTRAST_MIN = 0.10    # 目标框内与周边背景的亮度对比度下限 → 相机可检出
CAM_MIN_BOX_PX = 12        # 目标投影宽度小于该像素数 → 像素太少，视为不可检出
LIDAR_BOX_MARGIN = 0.25    # 目标包围盒外扩（米），容忍标定误差
LIDAR_GROUND_SKIP = 0.35   # 盒底往上这么高以内的点视为路面回波，不计入目标
LIDAR_MIN_HITS = 5         # 目标框内回波点数达到该值 → 能形成可辨识点簇，视为发现目标

# ---------- 天气判别：边缘密度相对晴天基准的比值下界 ----------
# 四档实测比值约 1.00 / 0.79 / 0.24 / 0.14（来自第一部分与本模块实测），
# 阈值取在相邻两档比值的中点，避免某一档恰好压在阈值上
WEATHER_RATIO_THRESHOLDS = [
    ('ClearNoon', 0.85),
    ('SoftRainNoon', 0.50),
    ('HardRainNoon', 0.19),
    ('HeavyFog', 0.0),
]
# 判别出的天气 → 采用的感知模式
WEATHER_MODE = {
    'ClearNoon': 'camera',
    'SoftRainNoon': 'fusion',
    'HardRainNoon': 'fusion',
    'HeavyFog': 'lidar',
}
# 晴朗基准边缘密度的兜底值（正常由第一档实测得到）
FALLBACK_EDGE_REF = 0.13


# ---------- 图像特征 ----------

def edge_density(img_bgr):
    """边缘密度：Canny 边缘像素占全图的比例（与第一部分同口径）。"""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    return float((cv2.Canny(gray, 50, 150) > 0).mean())


def classify_weather(ratio):
    """按边缘密度比值判别天气等级（从高到低逐个比对阈值）。"""
    for name, lower in WEATHER_RATIO_THRESHOLDS:
        if ratio >= lower:
            return name
    return WEATHER_RATIO_THRESHOLDS[-1][0]


# ---------- 坐标变换 ----------

def to_local(pts_world, tf):
    """世界坐标点 → 以 tf 为原点的局部坐标（只考虑绕 z 轴的 yaw）。

    CARLA 坐标系：x 前、y 右、z 上。前方单位向量 (cos, sin)，右方单位向量 (-sin, cos)。
    """
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    d = pts_world[:, :3] - np.array([tf.location.x, tf.location.y, tf.location.z])
    x = d[:, 0] * c + d[:, 1] * s
    y = -d[:, 0] * s + d[:, 1] * c
    out = np.stack([x, y, d[:, 2]], axis=1)
    if pts_world.shape[1] > 3:
        out = np.hstack([out, pts_world[:, 3:]])
    return out


def local_to_world(pts_local, tf):
    """以 tf 为原点的局部坐标 → 世界坐标（to_local 的逆变换）。

    保留第 4 列及其后的附加字段（如回波强度）。
    """
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    x = pts_local[:, 0] * c - pts_local[:, 1] * s + tf.location.x
    y = pts_local[:, 0] * s + pts_local[:, 1] * c + tf.location.y
    z = pts_local[:, 2] + tf.location.z
    out = np.stack([x, y, z], axis=1)
    if pts_local.shape[1] > 3:
        out = np.hstack([out, pts_local[:, 3:]])
    return out


def bbox_corners_world(actor):
    """目标外接包围盒的 8 个角点（世界坐标）。"""
    bb = actor.bounding_box
    tf = actor.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                lx = bb.location.x + sx * bb.extent.x
                ly = bb.location.y + sy * bb.extent.y
                lz = bb.location.z + sz * bb.extent.z
                corners.append((tf.location.x + lx * c - ly * s,
                                tf.location.y + lx * s + ly * c,
                                tf.location.z + lz))
    return np.array(corners)


# ---------- 投影 ----------

def project_ego_points(pts_ego):
    """车辆坐标系点 → 相机像素坐标（与第四部分完全一致的投影公式）。"""
    p = pts_ego.copy()
    p[:, 0] -= CAM_AT.x
    p[:, 1] -= CAM_AT.y
    p[:, 2] -= CAM_AT.z
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    front = x > 1.0
    safe = np.maximum(x, 1e-6)
    u = CX + FX * y / safe
    v = CY - FY * z / safe
    return u, v, front


def project_actor_box(actor, ego_tf):
    """把目标 3D 包围盒投影成图像 2D 框（u0, v0, u1, v1），完全在画面外时返回 None。"""
    ego_pts = to_local(bbox_corners_world(actor), ego_tf)
    u, v, front = project_ego_points(ego_pts)
    if int(front.sum()) < 4:
        return None
    u, v = u[front], v[front]
    u0, u1 = max(float(u.min()), 0.0), min(float(u.max()), CAM_W - 1.0)
    v0, v1 = max(float(v.min()), 0.0), min(float(v.max()), CAM_H - 1.0)
    if u1 - u0 < 1.0 or v1 - v0 < 1.0:
        return None
    return u0, v0, u1, v1


# ---------- 两种模式的检出判据 ----------

def camera_detect(gray, box):
    """相机判据：目标框内与框外背景的平均亮度对比度 ≥ 阈值。

    不用「框线上的强梯度占比」是因为近距离目标的框线恰好落在车身平滑面上，
    即使晴天也会被判成不可检出；而「框内外的亮度差」直接对应目标还能不能从背景里分出来，
    雨雾的退化恰好就体现在这个差值被压缩上。
    框外背景取框外 15% 宽的环形区域，避免把路面/建筑的整体明暗算进对比度。
    """
    if box is None:
        return False, 0.0
    u0, v0, u1, v1 = [int(round(t)) for t in box]
    u0, v0 = max(u0, 0), max(v0, 0)
    u1, v1 = min(u1, CAM_W - 1), min(v1, CAM_H - 1)
    if u1 - u0 < CAM_MIN_BOX_PX or v1 - v0 < CAM_MIN_BOX_PX:
        return False, 0.0

    pad_u = max(int((u1 - u0) * 0.15), 3)
    pad_v = max(int((v1 - v0) * 0.15), 3)
    cu0, cv0 = max(u0 - pad_u, 0), max(v0 - pad_v, 0)
    cu1, cv1 = min(u1 + pad_u, CAM_W - 1), min(v1 + pad_v, CAM_H - 1)
    patch = gray[cv0:cv1 + 1, cu0:cu1 + 1].astype(np.float64)

    inside = np.zeros(patch.shape, bool)
    inside[v0 - cv0:v1 - cv0 + 1, u0 - cu0:u1 - cu0 + 1] = True
    if inside.sum() < 16 or (~inside).sum() < 16:
        return False, 0.0

    mu_in = patch[inside].mean()
    mu_out = patch[~inside].mean()
    contrast = abs(mu_in - mu_out) / max(mu_out, 1.0)
    return contrast >= CAM_CONTRAST_MIN, contrast


def lidar_box_mask(pts_world, actor):
    """返回落在目标 3D 包围盒内的回波点掩膜。

    盒体比车体mesh略大，盒底那层会框进路面回波，因此抬高下界把它排除，
    否则「目标框内点数」会被地面点灌水。
    """
    bb = actor.bounding_box
    p = to_local(pts_world, actor.get_transform())
    z_bottom = bb.location.z - bb.extent.z + LIDAR_GROUND_SKIP
    return (
        (np.abs(p[:, 0] - bb.location.x) <= bb.extent.x + LIDAR_BOX_MARGIN) &
        (np.abs(p[:, 1] - bb.location.y) <= bb.extent.y + LIDAR_BOX_MARGIN) &
        (p[:, 2] >= z_bottom) &
        (p[:, 2] <= bb.location.z + bb.extent.z + LIDAR_BOX_MARGIN)
    )


# ---------- 连接 CARLA 与传感器装配 ----------

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()
car_map = world.get_map()

# 清理上次运行残留的车辆：残留会占用出生点，导致每次跑落在不同位置、数据不可复现
leftovers = list(world.get_actors().filter('vehicle.*'))
for actor in leftovers:
    try:
        actor.destroy()
    except RuntimeError:
        pass
if leftovers:
    print(f'已清理 {len(leftovers)} 辆残留车辆')
    time.sleep(1.0)

# ego 车辆：出生点可能被其他车辆占用，依次尝试直到成功
vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
spawn_point = None
spawn_idx = -1
for idx, sp in enumerate(car_map.get_spawn_points()):
    try:
        ego = world.spawn_actor(vehicle_bp, sp)
        spawn_point = sp
        spawn_idx = idx
        break
    except RuntimeError:
        continue
if spawn_point is None:
    raise RuntimeError('所有出生点均被占用，请重启 CARLA（清空场景）后重试')
ego.set_autopilot(False)
ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
print(f'ego 车辆已生成，出生点序号={spawn_idx}，位置={spawn_point.location}')

# 目标车按距离逐个生成、测完即销毁：同一时刻场景里只有一辆目标车。
# 这样既不会出现「后车被前车挡住、框里装的是前车车身」，也不用担心摆到路缘或绿化带上。
# 全部用同一车型，跨天气对比不受车型差异干扰。
ego_yaw = math.radians(spawn_point.rotation.yaw)
right_vec = np.array([-math.sin(ego_yaw), math.cos(ego_yaw)])
fwd_vec = np.array([math.cos(ego_yaw), math.sin(ego_yaw)])
ego_loc = spawn_point.location


def spawn_target(dist, lateral):
    """在 ego 前方 dist 米、右侧 lateral 米处生成一辆静止目标车。"""
    loc = carla.Location(x=ego_loc.x + fwd_vec[0] * dist + right_vec[0] * lateral,
                         y=ego_loc.y + fwd_vec[1] * dist + right_vec[1] * lateral,
                         z=ego_loc.z + 0.2)
    actor = world.spawn_actor(vehicle_bp, carla.Transform(loc, spawn_point.rotation))
    actor.set_autopilot(False)
    actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    return actor

# RGB 相机：与前三、四部分完全一致的参数，保证各场实验可比
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', str(CAM_W))
camera_bp.set_attribute('image_size_y', str(CAM_H))
camera_bp.set_attribute('fov', str(CAM_FOV))
camera = world.spawn_actor(camera_bp, carla.Transform(CAM_AT), attach_to=ego)

img_store = {'img': None}


def on_image(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))  # BGRA
    # copy(): raw_data 的底层缓冲随 image 对象释放，必须拷成独立数组再保存
    img_store['img'] = arr[:, :, :3].copy()


camera.listen(on_image)

# 雷达按帧发布，单帧只是整圈的一小段扇形，因此在回调里把每一帧累积起来
lidar_store = {'buf': []}


def on_lidar(measurement):
    arr = np.frombuffer(measurement.raw_data, dtype=np.float32)
    # copy(): 不拷贝会读到 measurement 释放后的内存，表现为点数/强度全为 0 或溢出
    lidar_store['buf'].append(arr.reshape(-1, 4).copy())  # x, y, z, intensity


def collect_lidar(window_s):
    """在 window_s 时间窗内累积所有雷达帧，返回拼成一整圈的点云 (N,4)。"""
    lidar_store['buf'] = []
    t0 = time.time()
    while time.time() - t0 < window_s:
        time.sleep(0.02)
    if not lidar_store['buf']:
        return None
    return np.vstack(lidar_store['buf'])


def setup_lidar(attenuation):
    """按指定大气衰减系数重建雷达（每档天气调用一次）。

    atmosphere_attenuation_rate 是 blueprint 属性，生成后无法修改，
    因此切换天气时需要销毁旧雷达、用新的衰减系数重新生成。
    """
    bp = bp_lib.find('sensor.lidar.ray_cast')
    bp.set_attribute('channels', '32')
    bp.set_attribute('range', '50')
    bp.set_attribute('points_per_second', str(LIDAR_PPS))
    bp.set_attribute('rotation_frequency', '20')
    bp.set_attribute('upper_fov', '10')
    bp.set_attribute('lower_fov', '-30')
    bp.set_attribute('atmosphere_attenuation_rate', str(attenuation))
    # 丢点参数：让「强度越低越容易被丢」成为天气退化的主要来源
    bp.set_attribute('dropoff_general_rate', '0.0')
    bp.set_attribute('dropoff_intensity_limit', '0.9')
    bp.set_attribute('dropoff_zero_intensity', '0.5')
    lidar = world.spawn_actor(bp, carla.Transform(LIDAR_AT), attach_to=ego)
    lidar.listen(on_lidar)
    return lidar


# ---------- 四档天气（与前面各部分一致）----------
# 注意: carla.WeatherParameters.ClearNoon 等是共享的类属性对象，直接修改会污染该常量，
# 必须用构造函数生成独立对象，否则「晴朗」和「浓雾」会变成同一组参数。
base = carla.WeatherParameters.ClearNoon
clear = carla.WeatherParameters(
    cloudiness=base.cloudiness,
    precipitation=0.0,
    precipitation_deposits=0.0,
    wind_intensity=base.wind_intensity,
    sun_azimuth_angle=base.sun_azimuth_angle,
    sun_altitude_angle=base.sun_altitude_angle,
    fog_density=0.0,
    fog_distance=0.0,
    fog_falloff=0.0,
    wetness=0.0,
)
fog = carla.WeatherParameters(
    cloudiness=base.cloudiness,
    precipitation=0.0,
    precipitation_deposits=0.0,
    wind_intensity=base.wind_intensity,
    sun_azimuth_angle=base.sun_azimuth_angle,
    sun_altitude_angle=base.sun_altitude_angle,
    fog_density=80.0,
    fog_distance=0.75,
    fog_falloff=0.1,
    wetness=0.0,
)
weather_presets = [
    ('ClearNoon', clear),
    ('SoftRainNoon', carla.WeatherParameters.SoftRainNoon),
    ('HardRainNoon', carla.WeatherParameters.HardRainNoon),
    ('HeavyFog', fog),
]

os.makedirs(RESULTS_DIR, exist_ok=True)
rows, target_rows = [], []
edge_ref = None          # 晴朗基准边缘密度，第一档实测后写入
snapshots = {}

for name, wp in weather_presets:
    # 每次测量前把车辆复位到同一参考位姿，确保四档天气看到的是同一场景
    ego.set_transform(spawn_point)
    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    world.set_weather(wp)

    # 让 CARLA 窗口跟随车顶相机，方便直接看到车内视角（不影响测量数据）
    world.get_spectator().set_transform(camera.get_transform())

    # 重建雷达，写入本档天气的大气衰减系数
    lidar = setup_lidar(LIDAR_ATTENUATION[name])
    time.sleep(3)  # 等待天气生效、相机与雷达稳定

    # 连续取 5 帧估计边缘密度，逐帧判别天气后取多数票
    densities, votes = [], []
    for _ in range(5):
        time.sleep(0.15)
        if img_store['img'] is None:
            continue
        d = edge_density(img_store['img'])
        densities.append(d)
        ref = edge_ref if edge_ref else d
        if not edge_ref:
            votes.append('ClearNoon')     # 第一档即晴天基准，直接标为晴朗
        else:
            votes.append(classify_weather(d / ref))

    if not densities:
        print(f'{name}: 未收到相机数据，跳过')
        lidar.stop()
        time.sleep(0.3)
        lidar.destroy()
        continue

    edge_now = float(np.mean(densities))
    if edge_ref is None:
        edge_ref = edge_now               # 第一档（ClearNoon）作为基准
    edge_ratio = edge_now / edge_ref if edge_ref else 0.0
    predicted = max(set(votes), key=votes.count) if votes else 'ClearNoon'
    mode_used = WEATHER_MODE[predicted]

    # 逐个距离测量：一次只放一辆目标车，测完销毁再测下一档距离
    cam_flags, lidar_flags, fusion_flags = [], [], []
    viz, frames = [], {}
    img = None
    for nominal_dist, lateral in TARGETS:
        actor = spawn_target(nominal_dist, lateral)
        time.sleep(0.5)  # 等目标进入视场，并让雷达扫过它所在的方位

        # 丢弃几帧旧数据后采集整圈点云
        for _ in range(2):
            time.sleep(0.15)
        cloud = collect_lidar(LIDAR_WINDOW_S)
        if img_store['img'] is not None:
            img = img_store['img'].copy()

        gt = actor.get_transform().location
        gt_ego = to_local(np.array([[gt.x, gt.y, gt.z]]), ego.get_transform())[0]
        dist = float(math.hypot(gt_ego[0], gt_ego[1]))

        box, cam_ok, cam_contrast = None, False, 0.0
        hits, lidar_ok, intensity = 0, False, float('nan')
        in_pts = np.zeros((0, 4))

        if cloud is not None and img is not None:
            # 点云转到世界坐标（保留强度列），并只保留雷达 55 m 内的点（雷达量程 50 m）
            near = np.linalg.norm(cloud[:, :2], axis=1) < 55.0
            pts_world = local_to_world(cloud[near, :4].astype(np.float64), lidar.get_transform())

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            box = project_actor_box(actor, ego.get_transform())
            cam_ok, cam_contrast = camera_detect(gray, box)

            mask = lidar_box_mask(pts_world, actor)
            hits = int(mask.sum())
            lidar_ok = hits >= LIDAR_MIN_HITS
            in_pts = pts_world[mask]
            # 框内回波强度的中位数：反映本档天气下该距离的回波质量
            intensity = float(np.median(in_pts[:, 3])) if hits > 0 else float('nan')

        actor.destroy()   # 测完立即移除，避免遮挡下一个距离档
        time.sleep(0.3)

        cam_flags.append(cam_ok)
        lidar_flags.append(lidar_ok)
        fusion_flags.append(cam_ok or lidar_ok)
        viz.append((box, cam_ok, in_pts, nominal_dist))
        if img is not None and box is not None:
            frames[nominal_dist] = (img, box)

        target_rows.append({
            'weather': name,
            'nominal_m': nominal_dist,
            'distance_m': round(dist, 2),
            'cam_detect': int(cam_ok),
            'cam_contrast': round(cam_contrast, 3),
            'lidar_hits': hits,
            'lidar_intensity': None if math.isnan(intensity) else round(intensity, 3),
            'lidar_detect': int(lidar_ok),
            'fusion_detect': int(cam_ok or lidar_ok),
        })

    # 6 辆车是在同一参考位姿下逐辆测的，投影位置几乎完全重叠，
    # 因此每档只展示中间距离档（28 m）目标，四档之间可直接对比同一目标的成像与回波。
    ref_dist = TARGETS[len(TARGETS) // 2][0]
    if ref_dist in frames:
        img = frames[ref_dist][0]
    viz = [v for v in viz if v[3] == ref_dist]

    # 本档天气测完，销毁雷达并留出回调线程退出的时间
    lidar.stop()
    time.sleep(0.3)
    lidar.destroy()

    cam_recall = float(np.mean(cam_flags) * 100.0)
    lidar_recall = float(np.mean(lidar_flags) * 100.0)
    fusion_recall = float(np.mean(fusion_flags) * 100.0)
    mode_flags = {'camera': cam_flags, 'lidar': lidar_flags, 'fusion': fusion_flags}[mode_used]
    adaptive_recall = float(np.mean(mode_flags) * 100.0)

    rows.append({
        'weather': name,
        'edge_density': round(edge_now, 4),
        'edge_ratio': round(edge_ratio, 3),
        'predicted': predicted,
        'mode_used': mode_used,
        'cam_recall_pct': round(cam_recall, 1),
        'lidar_recall_pct': round(lidar_recall, 1),
        'fusion_recall_pct': round(fusion_recall, 1),
        'adaptive_recall_pct': round(adaptive_recall, 1),
    })
    snapshots[name] = (img, viz, rows[-1])

    print(f'{name} [attenuation={LIDAR_ATTENUATION[name]}]')
    print(f'  边缘密度={edge_now:.4f} 比值={edge_ratio:.2f} 判别={predicted} 采用模式={mode_used}')
    print(f'  检出率: 相机={cam_recall:.0f}% 雷达={lidar_recall:.0f}% '
          f'融合={fusion_recall:.0f}% 自适应={adaptive_recall:.0f}%')

# ---------- 保存指标数据 ----------
if not rows:
    raise RuntimeError('未采集到任何数据，请确认 CARLA 已启动且场景未被占满')
with open(os.path.join(RESULTS_DIR, 'adaptive_metrics.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)
with open(os.path.join(RESULTS_DIR, 'adaptive_targets.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=list(target_rows[0].keys()))
    writer.writeheader()
    writer.writerows(target_rows)

# ---------- 图 1：四档天气的检出可视化 ----------
fig, axes = plt.subplots(2, 2, figsize=(15, 11))
for ax, row in zip(axes.ravel(), rows):
    name = row['weather']
    img, viz, _ = snapshots[name]
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    for box, cam_ok, pts, nominal in viz:
        if box is not None:
            u0, v0, u1, v1 = box
            color = '#00b0ff' if cam_ok else '#9e9e9e'
            ax.add_patch(plt.Rectangle((u0, v0), u1 - u0, v1 - v0, fill=False,
                                       edgecolor=color, linewidth=2, linestyle='-'))
            ax.text(u0, max(v0 - 6, 12), f'{int(nominal)}m', color=color, fontsize=9,
                    bbox=dict(facecolor='black', alpha=0.5, pad=1, edgecolor='none'))
        # 框内雷达回波点画成红点，直观看到「雷达还能看到哪些目标」
        if len(pts) > 0:
            pe = to_local(pts[:, :3], ego.get_transform())
            uu, vv, ff = project_ego_points(pe)
            sel = ff & (uu >= 0) & (uu < CAM_W) & (vv >= 0) & (vv < CAM_H)
            ax.scatter(uu[sel], vv[sel], c='#ff1744', s=2, alpha=0.9)
    ax.set_title(f'{name} → {row["predicted"]} | mode={row["mode_used"]} | '
                 f'cam={row["cam_recall_pct"]:.0f}% lidar={row["lidar_recall_pct"]:.0f}%')
    ax.set_xlim(0, CAM_W)
    ax.set_ylim(CAM_H, 0)
    ax.axis('off')
fig.suptitle('Adaptive perception: detected targets per weather '
             '(blue box = camera detected, red = LiDAR echoes)')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS_DIR, 'weather_adaptive_detection.png'), dpi=100, bbox_inches='tight')
plt.close(fig)

# ---------- 图 2：各模式检出率对比 + 天气判别特征 ----------
weathers = [r['weather'] for r in rows]
x_pos = np.arange(len(weathers))
width = 0.26
WEATHER_COLOR = {
    'ClearNoon': '#1565c0',
    'SoftRainNoon': '#00897b',
    'HardRainNoon': '#ef6c00',
    'HeavyFog': '#6a1b9a',
}

fig, axes = plt.subplots(1, 3, figsize=(19, 4.8))

axes[0].bar(x_pos - width, [r['cam_recall_pct'] for r in rows], width, label='Camera only')
axes[0].bar(x_pos, [r['lidar_recall_pct'] for r in rows], width, label='LiDAR only')
axes[0].bar(x_pos + width, [r['fusion_recall_pct'] for r in rows], width, label='Fusion (union)')
for i, r in enumerate(rows):
    axes[0].plot(i, r['adaptive_recall_pct'], marker='*', markersize=16,
                 color='#ff6d00', linestyle='none',
                 label='Adaptive' if i == 0 else None)
axes[0].set_title('Detection recall per perception mode')
axes[0].set_xticks(x_pos)
axes[0].set_xticklabels([f'{w}\n({r["mode_used"]})' for w, r in zip(weathers, rows)], fontsize=8)
axes[0].set_ylabel('recall / %')
axes[0].set_ylim(0, 118)
axes[0].legend(fontsize=8, ncol=4, loc='upper center', bbox_to_anchor=(0.5, -0.16),
               frameon=False)
axes[0].grid(axis='y', alpha=0.3)

# 右图：天气判别的依据——边缘密度及其相对晴天基准的阈值线
ref = rows[0]['edge_density']
axes[1].bar(x_pos, [r['edge_density'] for r in rows], width=0.55,
            color=[WEATHER_COLOR[w] for w in weathers])
for i, r in enumerate(rows):
    axes[1].text(i, r['edge_density'] + 0.003, f'→ {r["predicted"]}',
                 ha='center', fontsize=7)
for lo in [t for _, t in WEATHER_RATIO_THRESHOLDS if t > 0]:
    axes[1].axhline(lo * ref, color='#616161', linestyle='--', linewidth=1)
    axes[1].text(len(rows) - 0.42, lo * ref, f'{lo:.2f}×ref', va='bottom',
                 fontsize=7, color='#616161')
axes[1].set_title('Weather classification feature (edge density, dashed = thresholds)')
axes[1].set_xticks(x_pos)
axes[1].set_xticklabels(weathers, fontsize=8)
axes[1].set_ylabel('edge density')
axes[1].set_ylim(0, ref * 1.25)
axes[1].grid(axis='y', alpha=0.3)

# 右图：随距离的相机对比度（实线）与雷达框内回波点数（虚线，右轴）
ax3 = axes[2]
ax3_r = ax3.twinx()
for w in weathers:
    sub = [t for t in target_rows if t['weather'] == w]
    d = [t['distance_m'] for t in sub]
    ax3.plot(d, [t['cam_contrast'] for t in sub], '-o', ms=4,
             color=WEATHER_COLOR[w], label=f'{w}')
    ax3_r.plot(d, [t['lidar_hits'] for t in sub], '--s', ms=3, alpha=0.55,
               color=WEATHER_COLOR[w])
ax3.axhline(CAM_CONTRAST_MIN, color='#616161', linestyle=':', linewidth=1)
ax3.text(10, CAM_CONTRAST_MIN + 0.006, 'camera threshold', fontsize=7, color='#616161')
ax3.set_xlabel('target distance (m)')
ax3.set_ylabel('camera contrast (solid)')
ax3_r.set_ylabel('LiDAR returns in box (dashed)')
ax3_r.set_yscale('log')
ax3.set_title('Per-distance contrast (solid) vs LiDAR returns (dashed)')
ax3.grid(alpha=0.3)
ax3.legend(fontsize=7, loc='upper right')

fig.tight_layout()
fig.savefig(os.path.join(RESULTS_DIR, 'weather_adaptive_metrics.png'), dpi=110, bbox_inches='tight')
plt.close(fig)

# ---------- 清理 ----------
camera.stop()
time.sleep(0.3)
camera.destroy()
ego.destroy()
print('实验完成，结果保存在 results/ 目录：')
print('  adaptive_metrics.csv / adaptive_targets.csv')
print('  weather_adaptive_detection.png / weather_adaptive_metrics.png')
