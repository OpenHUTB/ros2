"""
ROS+CARLA 恶劣雨雾天气下相机与激光雷达感知性能对比实验

依赖: carla==0.9.15, numpy, matplotlib
使用: 1) 先启动 CARLA 服务端（双击解压目录下的 CarlaUE4.exe）
      2) 在 Python 3.10 环境中运行 python main.py

说明: 本实验是「恶劣雨雾天气下相机感知性能对比实验」系列的第二部分传感器扩展：
      前两场实验都以相机（sensor.camera.rgb）为感知对象，本实验在同一辆车、
      同一参考位姿、同一四档天气下，同时挂载相机与激光雷达
      （sensor.lidar.ray_cast），一次采集得到两套数据，用「相对晴朗基准的
      退化率」把两个传感器的退化程度放进同一口径做横向对比，验证
      「相机在雨雾中退化明显、激光雷达受影响较小」的互补性。

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

# 四档天气的雷达大气衰减系数（单位：/m，越大回波越弱）。
# CARLA 雷达 blueprint 用 atmosphere_attenuation_rate 描述激光穿过大气时的
# 回波衰减，默认值为 0.004。雨滴与雾滴会对 ~905nm 激光产生散射与吸收，
# 因此按天气档位递增设定：清晰保持默认，雨越大 / 雾越浓取值越大。
# 该参数在传感器生成时写入 blueprint，运行中不能修改，
# 故每档天气前重建一次雷达（见下方 setup_lidar）。
LIDAR_ATTENUATION = {
    'ClearNoon': 0.004,
    'SoftRainNoon': 0.01,
    'HardRainNoon': 0.03,
    'HeavyFog': 0.06,
}

# 近 / 中 / 远距离分段的边界（米），需小于雷达量程
NEAR_M, MID_M = 15.0, 30.0

# 雷达点云累积时间窗（秒）。CARLA 的雷达按渲染帧发布，一帧只含约 45° 的扇形点云
# （本机世界频率约 160 FPS，20 Hz 转速下每帧约 1000 点），单帧无法代表整圈。
# 车辆在测量期间静止，因此在 0.2 s（约 4 圈）窗口内把连续多帧点云累积成并集，
# 即可得到覆盖整圈、约 3 万点的完整扫描；四档天气用完全相同的窗口，保证可比。
LIDAR_WINDOW_S = 0.2


# ---------- 相机指标（与实验1完全相同的口径）----------

def camera_metrics(frames):
    """亮度（灰度均值）、对比度（灰度标准差）、边缘密度（Canny 边缘占比 %）。"""
    gray = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    brightness = float(np.mean([np.mean(g) for g in gray]))
    contrast = float(np.mean([np.std(g) for g in gray]))
    edge = float(np.mean([
        np.sum(cv2.Canny(g, 50, 150) > 0) / (g.shape[0] * g.shape[1]) * 100
        for g in gray]))
    return brightness, contrast, edge


# ---------- 雷达指标 ----------

def lidar_metrics(pts):
    """对一次扫描点云给出四项指标。

    pts: (N,4) float32 数组，列依次为 x(前) / y(左) / z(上) / intensity。
         这里的一次「扫描」是固定时间窗内累积的点云并集（见下方 LIDAR_WINDOW_S），
         因为 CARLA 雷达按渲染帧发布，单帧只是一小段扇形，无法代表整圈。
    指标口径：
      count      —— 有效点数（回波衰减会触发传感器丢点，点数下降）
      intensity  —— 平均回波强度
      range_p95  —— 距离的 95 分位（近似「能看多远」）
      near/mid/far—— 三个距离区间的点数占比（%）
    """
    if pts is None or len(pts) == 0:
        return (0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rng = np.linalg.norm(pts[:, :3], axis=1)  # 到传感器的距离
    return (float(len(pts)), float(pts[:, 3].mean()), float(np.percentile(rng, 95)),
            float((rng < NEAR_M).mean() * 100),
            float(((rng >= NEAR_M) & (rng < MID_M)).mean() * 100),
            float((rng >= MID_M).mean() * 100))


def degradation(value, base):
    """退化率 = (晴朗基准 - 退化值) / 晴朗基准 × 100%，正值表示指标下降。"""
    if base == 0:
        return 0.0
    return (base - value) / base * 100.0


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

# 相机：与实验 1 / 实验 2 完全相同的参数，保证三场实验可比
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '90')
camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=1.5, z=2.4)),
                           attach_to=vehicle)

image_store = {'img': None}


def on_image(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))  # BGRA
    # copy(): raw_data 的底层缓冲随 image 对象释放，必须拷成独立数组再保存
    image_store['img'] = arr[:, :, :3].copy()               # BGR


camera.listen(on_image)

# 雷达按帧发布，单帧只是整圈的一小段扇形，因此在回调里把每一帧都累积起来，
# 由采集循环在固定时间窗结束后拼成一整圈点云
lidar_store = {'buf': []}


def on_lidar(measurement):
    arr = np.frombuffer(measurement.raw_data, dtype=np.float32)
    # copy(): raw_data 的底层缓冲随 measurement 对象释放，不拷贝会读到已释放内存，
    # 表现为点数/距离/强度全为 0 或溢出成巨大值（numpy 会报 overflow 警告）
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
    bp.set_attribute('points_per_second', '200000')
    bp.set_attribute('rotation_frequency', '20')
    bp.set_attribute('upper_fov', '10')
    bp.set_attribute('lower_fov', '-30')
    bp.set_attribute('atmosphere_attenuation_rate', str(attenuation))
    # 丢点参数：让「强度越低越容易被丢」成为天气退化的主要来源
    bp.set_attribute('dropoff_general_rate', '0.0')
    bp.set_attribute('dropoff_intensity_limit', '0.9')
    bp.set_attribute('dropoff_zero_intensity', '0.5')
    # 雷达放在车辆中心上方，避免被前置相机遮挡
    lidar = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.0, z=2.6)),
                              attach_to=vehicle)
    lidar.listen(on_lidar)
    return lidar


# ---------- 四档天气（与实验 1 一致）----------
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
rows, bev = [], {}

for name, wp in weather_presets:
    # 每次测量前把车辆复位到同一参考位姿，确保四档天气看到的是同一场景
    vehicle.set_transform(spawn_point)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    world.set_weather(wp)

    # 重建雷达，写入本档天气的大气衰减系数
    lidar = setup_lidar(LIDAR_ATTENUATION[name])
    time.sleep(3)  # 等待天气生效、相机与雷达稳定

    # 相机：连续取 5 帧；雷达：在固定时间窗内累积多帧，拼成一整圈点云
    frames = []
    for _ in range(5):
        if image_store['img'] is not None:
            frames.append(image_store['img'])
        time.sleep(0.3)

    cloud = collect_lidar(LIDAR_WINDOW_S)

    # 停止后留出时间让回调线程退出，否则紧接着 destroy 会触发客户端原生崩溃
    lidar.stop()
    time.sleep(0.3)
    lidar.destroy()

    if not frames or cloud is None or len(cloud) == 0:
        print(f'{name}: 未收到相机或雷达数据，跳过')
        continue

    cb, cc, ce = camera_metrics(frames)
    lc, li, lr, ln, lm, lf = lidar_metrics(cloud)

    rows.append({'weather': name,
                 'cam_brightness': round(cb, 2), 'cam_contrast': round(cc, 2),
                 'cam_edge': round(ce, 2),
                 'lidar_count': int(lc), 'lidar_intensity': round(li, 3),
                 'lidar_range_p95': round(lr, 2),
                 'lidar_near_pct': round(ln, 2), 'lidar_mid_pct': round(lm, 2),
                 'lidar_far_pct': round(lf, 2)})
    bev[name] = cloud

    print(f'{name} [attenuation={LIDAR_ATTENUATION[name]}]')
    print(f'  相机  亮度={cb:.1f} 对比度={cc:.1f} 边缘密度={ce:.2f}%')
    print(f'  雷达  点数={lc:.0f} 平均强度={li:.3f} 距离95分位={lr:.2f}m '
          f'近/中/远={ln:.1f}/{lm:.1f}/{lf:.1f}%')

# ---------- 保存指标数据 ----------
if not rows:
    raise RuntimeError('未采集到任何数据，请确认 CARLA 已启动且场景未被占满')
with open(os.path.join(RESULTS_DIR, 'metrics.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

# 晴朗档作为所有退化率的基准（正常应为第一行）
clear_row = next((r for r in rows if r['weather'] == 'ClearNoon'), rows[0])

# ---------- 图 1：四档天气点云 BEV 俯视 ----------
fig, axes = plt.subplots(2, 2, figsize=(11, 11))
sc = None
for ax, name in zip(axes.ravel(), [r['weather'] for r in rows]):
    pts = bev.get(name)
    if pts is None or len(pts) == 0:
        continue
    # BEV：以 x（前）为横轴、y（左）为纵轴，按高度 z 着色
    sc = ax.scatter(pts[:, 1], pts[:, 0], c=pts[:, 2], s=0.4, cmap='turbo', vmin=-2, vmax=6)
    ax.set_title(f'{name}  (points={len(pts)})')
    ax.set_xlabel('y / m (left)')
    ax.set_ylabel('x / m (forward)')
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
if sc is not None:
    fig.colorbar(sc, ax=axes, shrink=0.6, label='height z / m')
fig.suptitle('LiDAR point cloud in four weather conditions (bird-eye view)')
fig.savefig(os.path.join(RESULTS_DIR, 'weather_lidar_bev.png'), dpi=110, bbox_inches='tight')
plt.close(fig)

# ---------- 图 2：相机 vs 雷达退化率对照 ----------
weathers = [r['weather'] for r in rows if r['weather'] != 'ClearNoon']
x = np.arange(len(weathers))
width = 0.2

cam_metrics = [('cam_brightness', 'Brightness'), ('cam_contrast', 'Contrast'),
               ('cam_edge', 'Edge density')]
lidar_metrics = [('lidar_count', 'Point count'), ('lidar_intensity', 'Mean intensity'),
                 ('lidar_range_p95', 'Range p95'), ('lidar_far_pct', 'Far-point share')]

fig, axes = plt.subplots(1, 3, figsize=(17, 4.6))

# 相机三个指标的退化率
for i, (key, label) in enumerate(cam_metrics):
    vals = [degradation(next(r[key] for r in rows if r['weather'] == w), clear_row[key])
            for w in weathers]
    axes[0].bar(x + (i - 1) * width, vals, width, label=label)
axes[0].set_title('Camera degradation rate (%)')
axes[0].set_xticks(x); axes[0].set_xticklabels(weathers, rotation=15, fontsize=8)
axes[0].axhline(0, color='gray', lw=0.8); axes[0].legend(fontsize=8)

# 雷达四个指标的退化率
for i, (key, label) in enumerate(lidar_metrics):
    vals = [degradation(next(r[key] for r in rows if r['weather'] == w), clear_row[key])
            for w in weathers]
    axes[1].bar(x + (i - 1.5) * width, vals, width, label=label)
axes[1].set_title('LiDAR degradation rate (%)')
axes[1].set_xticks(x); axes[1].set_xticklabels(weathers, rotation=15, fontsize=8)
axes[1].axhline(0, color='gray', lw=0.8); axes[1].legend(fontsize=8)

# 传感器级平均退化率：把各自各指标的退化率取平均，得到单个可对比的数
cam_avg = [np.mean([abs(degradation(next(r[k] for r in rows if r['weather'] == w), clear_row[k]))
                    for k, _ in cam_metrics]) for w in weathers]
lidar_avg = [np.mean([abs(degradation(next(r[k] for r in rows if r['weather'] == w), clear_row[k]))
                      for k, _ in lidar_metrics]) for w in weathers]
axes[2].bar(x - 0.2, cam_avg, 0.4, label='Camera (avg of metrics)', color='steelblue')
axes[2].bar(x + 0.2, lidar_avg, 0.4, label='LiDAR (avg of metrics)', color='darkorange')
axes[2].set_title('Cross-sensor average degradation rate (%)')
axes[2].set_xticks(x); axes[2].set_xticklabels(weathers, rotation=15, fontsize=8)
axes[2].legend(fontsize=8)

fig.suptitle('Camera vs LiDAR perception degradation under adverse weather')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS_DIR, 'weather_multisensor_degradation.png'),
            dpi=110, bbox_inches='tight')
plt.close(fig)

# ---------- 清理 ----------
camera.stop()
time.sleep(0.3)
camera.destroy()
vehicle.destroy()
print('实验完成，结果保存在 results/ 目录：')
print('  metrics.csv / weather_lidar_bev.png / weather_multisensor_degradation.png')
