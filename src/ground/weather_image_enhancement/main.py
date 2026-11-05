"""
ROS+CARLA 恶劣雨雾天气下相机图像增强恢复实验

依赖: carla==0.9.15, numpy, opencv-python, matplotlib
使用: 1) 先启动 CARLA 服务端（双击解压目录下的 CarlaUE4.exe）
      2) 在 Python 3.10 环境中运行 python main.py
说明: 本实验是「恶劣雨雾天气下相机感知性能对比实验」的续作：
      实验1量化了四种天气下相机图像的退化程度，本实验在同样的采集流程之上，
      对退化图像施加经典的图像增强算法（雨天 CLAHE 对比度增强、浓雾暗通道
      先验去雾），再用相同的三项指标量化「增强能恢复多少感知能力」。
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

# ---------- 图像增强算法 ----------

def clahe_enhance(img_bgr, clip_limit=2.0, tile=8):
    """CLAHE 限制对比度自适应直方图均衡：作用于 LAB 空间的 L 通道，
    只拉伸亮度分布而不扰动色彩，适合改善雨天的低对比度。"""
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile, tile)).apply(l)
    return cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)


def dark_channel(img_bgr, patch=15):
    """暗通道：取 RGB 三通道最小值后做最小值滤波（腐蚀实现）。"""
    min_c = img_bgr.min(axis=2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (patch, patch))
    return cv2.erode(min_c, kernel)


def defog_dark_channel_prior(img_bgr, omega=0.95, t0=0.1, patch=15):
    """暗通道先验去雾（He et al. CVPR 2009 的简化实现）：
    1) 由暗通道估计透射率 t；2) 取暗通道最亮 0.1% 像素对应原图强度估计大气光 A；
    3) 用均值滤波粗精炼透射率（代替论文中的软抠图/导向滤波，换取速度）；
    4) 按大气散射模型 I = J*t + A 反解无雾图像 J。"""
    img = img_bgr.astype(np.float64) / 255.0
    dc = dark_channel(img_bgr, patch) / 255.0
    # 大气光 A：暗通道最亮的 0.1% 像素在原图中的逐通道均值
    n = max(int(dc.size * 0.001), 1)
    idx = np.argpartition(dc.ravel(), -n)[-n:]
    A = img.reshape(-1, 3)[idx].mean(axis=0)
    # 透射率并做粗精炼
    t = 1.0 - omega * dc / A.mean()
    t = cv2.blur(t, (patch, patch))
    t = np.clip(t, t0, 1.0)
    J = (img - A) / t[..., None] + A
    return (np.clip(J, 0.0, 1.0) * 255).astype(np.uint8)


def enhance(name, img):
    """按天气选择增强方法：雨天 CLAHE，浓雾去雾，晴朗保持原样作为基准。"""
    if name == 'HeavyFog':
        return defog_dark_channel_prior(img), 'Dehaze (Dark Channel Prior)'
    if name in ('SoftRainNoon', 'HardRainNoon'):
        return clahe_enhance(img), 'CLAHE'
    return img.copy(), 'None (baseline)'


# ---------- 指标计算（与实验1完全相同的口径）----------

def metrics(frames):
    gray = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    brightness = float(np.mean([np.mean(g) for g in gray]))
    contrast = float(np.mean([np.std(g) for g in gray]))
    edge = float(np.mean([
        np.sum(cv2.Canny(g, 50, 150) > 0) / (g.shape[0] * g.shape[1]) * 100
        for g in gray]))
    return brightness, contrast, edge


def montage(orig, enhanced, tag, method, path):
    """原图与增强图并排对比，顶部标注处理方式。"""
    h, w = orig.shape[:2]
    bar = np.full((40, w * 2 + 4, 3), 255, dtype=np.uint8)
    sep = np.full((h, 4, 3), 255, dtype=np.uint8)
    pair = np.hstack([orig, sep, enhanced])
    cv2.putText(bar, f'{tag} | left: original  right: {method}',
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    out = np.vstack([bar, pair])
    ok, buf = cv2.imencode('.png', out)
    buf.tofile(path)  # 兼容含中文/空格的路径（cv2.imwrite 在此类路径下会静默失败）


# ---------- 连接 CARLA ----------
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# ---------- 生成车辆与相机（与实验1相同的配置，保证两场实验场景可比）----------
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
# 出生点可能被其他车辆占用，依次尝试直到成功
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

camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '90')
camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

image_store = {'img': None}


def on_image(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))  # BGRA
    image_store['img'] = arr[:, :, :3]                      # BGR


camera.listen(on_image)

# ---------- 四档天气（与实验1一致）----------
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
rows = []
clear_ref = None

for name, wp in weather_presets:
    # 每次测量前把车辆复位到同一参考位姿，确保四档天气看到的是同一场景
    vehicle.set_transform(spawn_point)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    world.set_weather(wp)
    time.sleep(3)  # 等待天气生效、相机刷新

    # 同一场景下连续采集 5 帧：原图取平均口径，增强图对同样 5 帧分别增强后取平均
    frames = []
    for _ in range(5):
        img = image_store['img']
        if img is not None:
            frames.append(img)
        time.sleep(0.3)
    if not frames:
        print(f'{name}: 未收到图像，跳过')
        continue

    b0, c0, e0 = metrics(frames)

    enhanced_frames, method = [], None
    for f in frames:
        fe, method = enhance(name, f)
        enhanced_frames.append(fe)
    b1, c1, e1 = metrics(enhanced_frames)

    row = {'weather': name, 'method': method,
           'brightness_orig': round(b0, 2), 'brightness_enh': round(b1, 2),
           'contrast_orig': round(c0, 2), 'contrast_enh': round(c1, 2),
           'edge_orig': round(e0, 2), 'edge_enh': round(e1, 2)}
    # 恢复率=(增强-退化)/(晴朗基准-退化)，对三个指标方向通用
    if clear_ref is None:
        clear_ref = (b0, c0, e0)
        row['brightness_rec'] = row['contrast_rec'] = row['edge_rec'] = ''
    else:
        row['brightness_rec'] = round((b1 - b0) / (clear_ref[0] - b0) * 100, 1)
        row['contrast_rec'] = round((c1 - c0) / (clear_ref[1] - c0) * 100, 1)
        row['edge_rec'] = round((e1 - e0) / (clear_ref[2] - e0) * 100, 1)
    rows.append(row)

    print(f'{name} [{method}]:')
    print(f'  原图   亮度={b0:.1f} 对比度={c0:.1f} 边缘密度={e0:.2f}%')
    print(f'  增强后 亮度={b1:.1f} 对比度={c1:.1f} 边缘密度={e1:.2f}%')
    if row['contrast_rec'] != '':
        print(f"  恢复率 亮度={row['brightness_rec']}% 对比度={row['contrast_rec']}%"
              f" 边缘密度={row['edge_rec']}%")

    if name != 'ClearNoon':
        # 第三段为文件名后缀（与教程 docs/img/ground/ 中的命名一致）
        suffix = {'SoftRainNoon': 'soft_rain',
                  'HardRainNoon': 'hard_rain',
                  'HeavyFog': 'heavy_fog'}[name]
        montage(frames[-1], enhanced_frames[-1], name, method,
                os.path.join(RESULTS_DIR, f'enhancement_compare_{suffix}.png'))

# ---------- 保存结果 ----------
with open(os.path.join(RESULTS_DIR, 'metrics.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

# ---------- 分组柱状图：每个指标一组，原图 vs 增强后 ----------
names = [r['weather'] for r in rows]
x = np.arange(len(names))
width = 0.38
fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
for ax, (k_orig, k_enh, title) in zip(axes, [
        ('brightness_orig', 'brightness_enh', 'Brightness (mean)'),
        ('contrast_orig', 'contrast_enh', 'Contrast (std)'),
        ('edge_orig', 'edge_enh', 'Edge Density (%)')]):
    ax.bar(x - width / 2, [r[k_orig] for r in rows], width, label='Original', color='steelblue')
    ax.bar(x + width / 2, [r[k_enh] for r in rows], width, label='Enhanced', color='orange')
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, fontsize=8)
    ax.set_title(title)
    ax.legend(fontsize=8)
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, 'weather_enhancement_metrics.png'), dpi=100)

# ---------- 清理 ----------
camera.stop()
camera.destroy()
vehicle.destroy()
print('实验完成，结果保存在 results/ 目录')
