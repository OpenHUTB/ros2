"""
UWB 布站几何定位 —— CARLA 实景运行出图脚本

依赖: carla==0.9.16, numpy, opencv-python, matplotlib, Pillow
使用:
  1) 先启动 CARLA 服务端（双击解压目录下的 CarlaUE4.exe，等待地图加载并监听 2000 端口）
  2) 在 Python 3.10 环境安装: pip install carla==0.9.16 opencv-python
  3) 运行: python carla_demo.py

说明:
  - 本脚本连接 CARLA 服务端（默认 localhost:2000，可用环境变量 CARLA_HOST 覆盖），
    在 Town 内生成一辆地面载具作为定位真值目标；以载具出生点为局部原点，按正方形
    构型布置四个虚拟 UWB 基站。
  - 读取载具真实位姿作为真值，按到各基站的水平距离叠加高斯噪声模拟测距，再用
    阻尼高斯-牛顿三边定位求解估计位置，定位算法与纯仿真 main.py 保持一致。
  - 高空俯视相机采集一帧 CARLA 真实渲染画面，与二维定位叠加层合成一张实景效果图，
    写入本脚本同级的 results/ 目录。
  - 本脚本必须在已运行 CARLA 服务端的环境中执行；效果图须由真实运行产生，
    不可用纯仿真图像替代。
"""
import os
import time

import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')  # 无图形界面环境下后台出图
import matplotlib.pyplot as plt
import carla

# ---------- 参数 ----------
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
HOST = os.environ.get('CARLA_HOST', 'localhost')
PORT = 2000
SIGMA_RANGE = 0.3          # 测距噪声标准差 (m)
GN_ITERS = 6               # 三边定位高斯-牛顿迭代次数
GN_DAMPING = 0.6           # 迭代步长阻尼，抑制病态几何下跳飞
SEED = 20261111

# 正方形基站相对出生点的水平偏移 (m)：四站布置在四周，观测方向最分散
STATION_OFFSETS = [(-12.0, -12.0), (12.0, -12.0),
                   (12.0, 12.0), (-12.0, 12.0)]


def trilaterate(baselines, ranges, iters=GN_ITERS, damping=GN_DAMPING):
    """带阻尼的高斯-牛顿迭代求解二维位置，初值取基站质心。"""
    baselines = np.asarray(baselines, dtype=float)
    p = baselines.mean(axis=0)
    for _ in range(iters):
        H, h = [], []
        for b, r in zip(baselines, ranges):
            d = np.linalg.norm(p - b)
            H.append((p - b) / max(d, 1e-6))
            h.append(r - d)
        H = np.asarray(H)
        h = np.asarray(h)
        dp = np.linalg.lstsq(H, h, rcond=None)[0]
        p = p + damping * dp
    return p


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rng = np.random.default_rng(SEED)

    # ---------- 连接服务端 ----------
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # ---------- 生成真值载具 ----------
    vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
    vehicle = None
    spawn_point = None
    for sp in world.get_map().get_spawn_points():
        try:
            vehicle = world.spawn_actor(vehicle_bp, sp)
            spawn_point = sp
            break
        except RuntimeError:
            continue
    if vehicle is None:
        raise RuntimeError('所有出生点均被占用，请重启 CARLA（清空场景）后重试')

    # 测量阶段刹停，保证实景帧与定位真值对应同一参考位姿
    vehicle.set_autopilot(False)
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

    # 基站世界坐标（CARLA 世界 xy 平面）
    base_loc = spawn_point.location
    stations_world = np.array([
        [base_loc.x + ox, base_loc.y + oy] for ox, oy in STATION_OFFSETS
    ])

    # ---------- 俯视 RGB 相机 ----------
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=0.0, z=28.0),
                                       carla.Rotation(pitch=-90.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    image_store = {'img': None}

    def on_image(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape(image.height, image.width, 4)  # BGRA
        image_store['img'] = arr[:, :, :3]              # BGR

    camera.listen(on_image)
    time.sleep(2.0)  # 等待相机刷新一帧

    # ---------- 读取真值并三边定位 ----------
    loc = vehicle.get_transform().location
    true_xy = np.array([loc.x, loc.y])
    true_dist = np.linalg.norm(stations_world - true_xy, axis=1)
    ranges = true_dist + rng.normal(0.0, SIGMA_RANGE, size=len(stations_world))
    est_xy = trilaterate(stations_world, ranges)
    error = float(np.linalg.norm(est_xy - true_xy))

    # 局部坐标（以出生点为原点），用于二维叠加层
    origin = np.array([base_loc.x, base_loc.y])
    true_local = true_xy - origin
    est_local = est_xy - origin
    stations_local = stations_world - origin

    frame = image_store['img']
    if frame is None:
        raise RuntimeError('未收到相机图像，请检查相机挂载与服务端渲染')

    # ---------- 合成实景效果图 ----------
    fig, (ax_cam, ax_map) = plt.subplots(
        1, 2, figsize=(12, 5.2), gridspec_kw={'width_ratios': [1.2, 1]})

    # 左：CARLA 真实渲染帧
    ax_cam.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    ax_cam.set_title('CARLA Town — real render (top-down camera)')
    ax_cam.axis('off')

    # 右：二维定位叠加层（局部坐标）
    ax_map.scatter(stations_local[:, 0], stations_local[:, 1],
                   c='black', marker='^', s=90, label='UWB base station')
    for k, s in enumerate(stations_local):
        ax_map.annotate(f'B{k+1}', (s[0] + 1.0, s[1] + 1.0), fontsize=9)
    ax_map.scatter(true_local[0], true_local[1], c='gray', marker='*',
                   s=160, label='true position')
    ax_map.scatter(est_local[0], est_local[1], c='tab:orange', marker='o',
                   s=90, label='estimated')
    ax_map.plot([true_local[0], est_local[0]], [true_local[1], est_local[1]],
                'r--', lw=1.2)
    ax_map.set_title(f'UWB positioning (error = {error:.3f} m)')
    ax_map.set_xlabel('local x (m)')
    ax_map.set_ylabel('local y (m)')
    ax_map.legend(fontsize=8)
    ax_map.grid(alpha=0.3)
    ax_map.set_aspect('equal')
    lim = 16.0
    ax_map.set_xlim(-lim, lim)
    ax_map.set_ylim(-lim, lim)
    fig.tight_layout()

    out_path = os.path.join(RESULTS_DIR, 'carla_uwb_layout.png')
    fig.savefig(out_path, dpi=110)
    plt.close(fig)

    print(f'真值={true_local.round(3).tolist()} '
          f'估计={est_local.round(3).tolist()} 误差={error:.3f} m')
    print(f'实景效果图已保存: {out_path}')

    # ---------- 清理 ----------
    camera.destroy()
    vehicle.destroy()


if __name__ == '__main__':
    main()
