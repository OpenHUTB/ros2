"""
ROS+CARLA 恶劣雨雾天气下相机-激光雷达融合可行性实验

依赖: carla==0.9.15, numpy, opencv-python, matplotlib
使用: 1) 先启动 CARLA 服务端（双击解压目录下的 CarlaUE4.exe）
      2) 在 Python 3.10 环境中运行 python main.py

说明: 本实验是「恶劣雨雾天气下多传感器感知性能对比」系列的第四部分。
      前三部分分别量化了相机的退化、图像增强的恢复上限、以及雷达与相机的
      横向退化对比。本实验回答一个更实际的问题：把雷达点云投影到相机图像上，
      这种融合在雨雾中还能不能用——几何上投得上，语义上对得上吗？

      做法：同一辆车、同一参考位姿、同一四档天气下同时挂载 RGB 相机、
      深度相机（仅用于区分天空与地面/目标像素）与激光雷达，把 0.3 s 窗口内
      累积的整圈点云按固定外参投影到相机像素平面，统计投影有效率、图像覆盖率
      与边缘命中率（投影点是否落在图像的强梯度处），衡量融合的几何可用性与
      语义可用性。

      所有结果写入本脚本同级的 results/ 目录，与运行时的工作目录无关。
"""
import os
import csv
import time

import carla
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')  # 无界面环境也能出图
import matplotlib.pyplot as plt

# 结果目录固定在脚本同级，避免从其他目录运行时把结果写到别处
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 相机与雷达的安装位置（车辆坐标系：x 前、y 右、z 上）----------
CAM_AT = carla.Location(x=1.5, z=2.4)     # RGB 相机 / 深度相机共用同一机位
LIDAR_AT = carla.Location(x=0.0, z=2.6)   # 雷达在车辆中心上方，避免被相机遮挡

# ---------- 相机内参：由 FOV 与分辨率反解 ----------
CAM_W, CAM_H, CAM_FOV = 800, 600, 90
FX = FY = (CAM_W / 2.0) / np.tan(np.radians(CAM_FOV) / 2.0)   # = 400
CX, CY = CAM_W / 2.0, CAM_H / 2.0

# ---------- 四档天气的雷达大气衰减系数（与第三部分完全一致）----------
LIDAR_ATTENUATION = {
    'ClearNoon': 0.004,
    'SoftRainNoon': 0.01,
    'HardRainNoon': 0.03,
    'HeavyFog': 0.06,
}

# 雷达点云累积时间窗（秒）。雷达按渲染帧发布，单帧只含约 45° 扇形，
# 必须累积多圈才能得到整圈点云；车辆在测量期间静止，四档窗口完全一致。
# 本实验要把点云投到 800×600 图像上做逐像素比对，因此把点密度提到
# 1,000,000 点/秒、窗口放宽到 0.3 s，避免采样过稀导致覆盖率随机波动。
LIDAR_WINDOW_S = 0.3
LIDAR_PPS = 1000000

# 深度相机把深度编码进 RGB 三通道，量程上限（米）
DEPTH_FAR = 1000.0
# 深度大于该值视为天空（深度相机对天空输出量程上限）
SKY_DEPTH = 900.0
# 覆盖率的近距离档（米）：只统计这个距离内的点，衡量近场标注能力
NEAR_COVER_M = 20.0
# 覆盖率的远距离档（米）：只统计这个距离外的点，衡量远端标注能力
FAR_COVER_M = 30.0
# 边缘命中的判定半径（像素）：投影点落在图像边缘的 3 像素邻域内即算命中
EDGE_NEIGHBOR_PX = 3
# 距离分档（米），用于画「边缘命中率随距离衰减」的曲线
RANGE_BINS = np.arange(0, 51, 10)


# ---------- 深度解码 ----------

def decode_depth(raw_data, height, width):
    """把深度相机 BGRA 原始数据解码成米。

    CARLA 深度相机把深度归一化后编码进 RGB 三通道：
    depth = (R + G*256 + B*65536 - 1) / (256^3 - 1) * DEPTH_FAR
    这里只用它区分「天空」与「真实表面」，不做绝对精度比对。
    """
    a = np.frombuffer(raw_data, dtype=np.uint8).reshape(height, width, 4)
    r = a[:, :, 2].astype(np.float64)          # BGRA 中的 R 是最低位字节
    g = a[:, :, 1].astype(np.float64)
    b = a[:, :, 0].astype(np.float64)
    return (r + g * 256.0 + b * 65536.0 - 1.0) / (256.0 ** 3 - 1.0) * DEPTH_FAR


# ---------- 点云投影 ----------

def project_cloud(cloud):
    """把雷达点云投影到相机像素平面。

    cloud: (N,4)，列为 x(前) / y(右) / z(上) / 强度，位于雷达坐标系。
    两个传感器刚性固定在车上、姿态相同，因此雷达坐标系到相机坐标系
    只差一个平移量，按安装位置直接换算即可。
    投影公式：u = CX + FX * y/x ，v = CY - FY * z/x（z 向上，故 v 取负）。
    """
    p = cloud[:, :3].astype(np.float64).copy()
    p[:, 0] += (CAM_AT.x - LIDAR_AT.x)
    p[:, 1] += (CAM_AT.y - LIDAR_AT.y)
    p[:, 2] += (CAM_AT.z - LIDAR_AT.z)

    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    front = x > 1.0                                   # 只保留相机前方的点
    safe = np.maximum(x, 1e-6)
    u = CX + FX * y / safe
    v = CY - FY * z / safe
    inside = front & (u >= 0) & (u < CAM_W) & (v >= 0) & (v < CAM_H)
    return u, v, inside, front, x


def coverage_pct(u, v, inside, dist, mask, dmin=0.0, dmax=None):
    """图像覆盖率：前景像素中被至少一个投影点覆盖的比例（%）。

    dmin / dmax 用来只统计某个距离区间的点，从而分别衡量近场与远端的标注能力。
    """
    sel = inside.copy()
    if dmin > 0:
        sel &= dist >= dmin
    if dmax is not None:
        sel &= dist <= dmax
    if not sel.any():
        return 0.0
    covered = np.zeros((CAM_H, CAM_W), dtype=bool)
    covered[v[sel].astype(int), u[sel].astype(int)] = True
    denom = int(mask.sum())
    return float(covered[mask].sum() / denom * 100.0) if denom else 0.0


def edge_hit_pct(u, v, inside, dist, edge_zone, band=None):
    """边缘命中率：投影点落在图像强梯度邻域内的比例（%）。

    这一步衡量的是「语义可用性」：几何上投得上是一回事，落点是否对应
    真实结构（建筑轮廓、车辆边界、标线）是另一回事。雾天图像边缘大面积
    消失，同样的点就落不到结构上，命中率随之下降。
    band 给出时只统计该距离区间内的点。
    """
    sel = inside.copy()
    if band is not None:
        lo, hi = band
        sel &= (dist >= lo) & (dist < hi)
    if not sel.any():
        return 0.0
    return float(edge_zone[v[sel].astype(int), u[sel].astype(int)].mean() * 100.0)


# ---------- 连接 CARLA 与传感器装配 ----------

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# ego 车辆：出生点可能被其他车辆占用，依次尝试直到成功
vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
spawn_point = None
for sp in world.get_map().get_spawn_points():
    try:
        vehicle = world.spawn_actor(vehicle_bp, sp)
        spawn_point = sp
        break
    except RuntimeError:
        continue
if spawn_point is None:
    raise RuntimeError('所有出生点均被占用，请重启 CARLA（清空场景）后重试')
vehicle.set_autopilot(False)
vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
print(f'车辆已生成，位置={spawn_point.location}')

# RGB 相机：与前三部分完全一致的参数，保证四场实验可比
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', str(CAM_W))
camera_bp.set_attribute('image_size_y', str(CAM_H))
camera_bp.set_attribute('fov', str(CAM_FOV))
camera = world.spawn_actor(camera_bp, carla.Transform(CAM_AT), attach_to=vehicle)

# 深度相机：与 RGB 相机同机位同内参，只用来把天空像素从覆盖率分母中剔除
depth_bp = bp_lib.find('sensor.camera.depth')
depth_bp.set_attribute('image_size_x', str(CAM_W))
depth_bp.set_attribute('image_size_y', str(CAM_H))
depth_bp.set_attribute('fov', str(CAM_FOV))
depth_camera = world.spawn_actor(depth_bp, carla.Transform(CAM_AT), attach_to=vehicle)

image_store = {'img': None, 'depth': None}


def on_image(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))  # BGRA
    # copy(): raw_data 的底层缓冲随 image 对象释放，必须拷成独立数组再保存
    image_store['img'] = arr[:, :, :3].copy()               # BGR


def on_depth(image):
    # 同样是 BGRA 编码，解码成米后保存
    image_store['depth'] = decode_depth(image.raw_data, image.height, image.width).copy()


camera.listen(on_image)
depth_camera.listen(on_depth)

# 雷达按帧发布，单帧只是整圈的一小段扇形，因此在回调里把每一帧都累积起来，
# 由采集循环在固定时间窗结束后拼成一整圈点云
lidar_store = {'buf': []}


def on_lidar(measurement):
    arr = np.frombuffer(measurement.raw_data, dtype=np.float32)
    # copy(): raw_data 的底层缓冲随 measurement 对象释放，不拷贝会读到已释放内存，
    # 表现为点数/距离/强度全为 0 或溢出成巨大值
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
    lidar = world.spawn_actor(bp, carla.Transform(LIDAR_AT), attach_to=vehicle)
    lidar.listen(on_lidar)
    return lidar


# ---------- 四档天气（与第一、三部分一致）----------
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
rows, overlays, edge_curves = [], {}, {}

for name, wp in weather_presets:
    # 每次测量前把车辆复位到同一参考位姿，确保四档天气看到的是同一场景
    vehicle.set_transform(spawn_point)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    world.set_weather(wp)

    # 让 CARLA 窗口跟随车顶相机，方便直接看到车内视角（不影响测量数据）
    world.get_spectator().set_transform(camera.get_transform())

    # 重建雷达，写入本档天气的大气衰减系数
    lidar = setup_lidar(LIDAR_ATTENUATION[name])
    time.sleep(3)  # 等待天气生效、相机与雷达稳定

    for _ in range(3):        # 丢弃几帧旧数据
        time.sleep(0.2)
    cloud = collect_lidar(LIDAR_WINDOW_S)

    # 停止后留出时间让回调线程退出，否则紧接着 destroy 会触发客户端原生崩溃
    lidar.stop()
    time.sleep(0.3)
    lidar.destroy()

    if image_store['img'] is None or image_store['depth'] is None or cloud is None:
        print(f'{name}: 未收到相机或雷达数据，跳过')
        continue

    img = image_store['img']
    depth = image_store['depth']
    fg_mask = depth < SKY_DEPTH          # 非天空像素，即可被雷达标注的区域
    sky_mask = ~fg_mask

    # 图像强梯度区：Canny 边缘再膨胀成 3 像素邻域，用于判断投影点是否落在结构上
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    kernel = np.ones((2 * EDGE_NEIGHBOR_PX + 1,) * 2, np.uint8)
    edge_zone = cv2.dilate(edges, kernel) > 0

    u, v, inside, front, dist = project_cloud(cloud)

    valid_pct = float(inside.sum() / max(int(front.sum()), 1) * 100.0)
    cover_pct = coverage_pct(u, v, inside, dist, fg_mask)
    cover_near_pct = coverage_pct(u, v, inside, dist, fg_mask, dmax=NEAR_COVER_M)
    cover_far_pct = coverage_pct(u, v, inside, dist, fg_mask, dmin=FAR_COVER_M)
    edge_pct = edge_hit_pct(u, v, inside, dist, edge_zone)

    # 天空误投率：投到天空像素的点占比，用于校验标定是否成立（正常应接近 0）
    ui, vi = u[inside].astype(int), v[inside].astype(int)
    sky_hit_pct = float(sky_mask[vi, ui].mean() * 100.0)

    rows.append({'weather': name,
                 'lidar_points': int(len(cloud)),
                 'valid_pct': round(valid_pct, 2),
                 'sky_hit_pct': round(sky_hit_pct, 2),
                 'cover_pct': round(cover_pct, 2),
                 'cover_near_pct': round(cover_near_pct, 2),
                 'cover_far_pct': round(cover_far_pct, 2),
                 'edge_hit_pct': round(edge_pct, 2)})
    overlays[name] = (img, u, v, inside, dist)
    edge_curves[name] = [
        ((lo + hi) / 2.0, edge_hit_pct(u, v, inside, dist, edge_zone, band=(lo, hi)))
        for lo, hi in zip(RANGE_BINS[:-1], RANGE_BINS[1:])
    ]

    print(f'{name} [attenuation={LIDAR_ATTENUATION[name]}]')
    print(f'  点数={len(cloud)} 前方点={int(front.sum())} 投影有效={valid_pct:.1f}% '
          f'天空误投={sky_hit_pct:.2f}%')
    print(f'  覆盖率={cover_pct:.2f}% (≤{NEAR_COVER_M:.0f}m {cover_near_pct:.2f}% / '
          f'≥{FAR_COVER_M:.0f}m {cover_far_pct:.2f}%) 边缘命中={edge_pct:.1f}%')

# ---------- 保存指标数据 ----------
if not rows:
    raise RuntimeError('未采集到任何数据，请确认 CARLA 已启动且场景未被占满')
with open(os.path.join(RESULTS_DIR, 'fusion_metrics.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

# ---------- 图 1：四档天气的投影叠加图 ----------
fig, axes = plt.subplots(2, 2, figsize=(14, 10.5))
for ax, row in zip(axes.ravel(), rows):
    name = row['weather']
    img, u, v, inside, dist = overlays[name]
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    # 按距离着色，让「远端点先消失」这件事在图上直接可见
    sc = ax.scatter(u[inside], v[inside], c=dist[inside], cmap='turbo',
                    vmin=0, vmax=50, s=0.6, alpha=0.8)
    ax.set_title(f'{name}  valid={row["valid_pct"]:.1f}%  cover={row["cover_pct"]:.1f}%')
    ax.set_xlim(0, CAM_W)
    ax.set_ylim(CAM_H, 0)
    ax.axis('off')
fig.colorbar(sc, ax=axes, shrink=0.6, label='LiDAR range / m')
fig.suptitle('LiDAR points projected onto the camera image (four weathers)')
fig.savefig(os.path.join(RESULTS_DIR, 'weather_fusion_overlay.png'), dpi=100, bbox_inches='tight')
plt.close(fig)

# ---------- 图 2：融合指标随天气的变化 + 边缘命中率随距离的衰减 ----------
weathers = [r['weather'] for r in rows]
x_pos = np.arange(len(weathers))
width = 0.2

fig, axes = plt.subplots(1, 2, figsize=(15, 4.8))
axes[0].bar(x_pos - 1.5 * width, [r['cover_pct'] for r in rows], width, label='Coverage (all)')
axes[0].bar(x_pos - 0.5 * width, [r['cover_near_pct'] for r in rows], width,
            label=f'Coverage (<= {NEAR_COVER_M:.0f} m)')
axes[0].bar(x_pos + 0.5 * width, [r['cover_far_pct'] for r in rows], width,
            label=f'Coverage (>= {FAR_COVER_M:.0f} m)')
axes[0].bar(x_pos + 1.5 * width, [r['edge_hit_pct'] for r in rows], width,
            label='Edge hit rate')
axes[0].set_title('Sensor-fusion metrics under four weathers')
axes[0].set_xticks(x_pos)
axes[0].set_xticklabels(weathers, rotation=15, fontsize=9)
axes[0].set_ylabel('%')
axes[0].legend(fontsize=8)

for name in weathers:
    pts = edge_curves.get(name) or []
    if not pts:
        continue
    axes[1].plot([p[0] for p in pts], [p[1] for p in pts], marker='o', label=name)
axes[1].set_title('Edge hit rate vs. range')
axes[1].set_xlabel('range / m')
axes[1].set_ylabel('edge hit rate / %')
axes[1].set_ylim(0, 105)
axes[1].grid(alpha=0.3)
axes[1].legend(fontsize=8)

fig.tight_layout()
fig.savefig(os.path.join(RESULTS_DIR, 'weather_fusion_metrics.png'), dpi=110, bbox_inches='tight')
plt.close(fig)

# ---------- 演示：自动驾驶行驶，CARLA 窗口跟随车顶相机 ----------
# 上面的测量全部在车辆刹停状态下完成，所以窗口里的视角是静止的。这一段只为
# 直观看看雨雾中行驶时的第一视角，不产生任何指标，删掉或注释都不影响结果。
DEMO_SECONDS = 10.0
print(f'演示：车辆自动驾驶 {DEMO_SECONDS:.0f} 秒（浓雾），CARLA 窗口跟随车顶相机')
world.set_weather(fog)
vehicle.set_transform(spawn_point)
vehicle.set_autopilot(True)
t0 = time.time()
while time.time() - t0 < DEMO_SECONDS:
    world.get_spectator().set_transform(camera.get_transform())
    time.sleep(0.05)
vehicle.set_autopilot(False)
vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

# ---------- 清理 ----------
camera.stop()
depth_camera.stop()
time.sleep(0.3)
camera.destroy()
depth_camera.destroy()
vehicle.destroy()
print('实验完成，结果保存在 results/ 目录：')
print('  fusion_metrics.csv / weather_fusion_overlay.png / weather_fusion_metrics.png')
