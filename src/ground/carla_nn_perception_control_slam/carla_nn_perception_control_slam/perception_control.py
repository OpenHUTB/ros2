#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务② 传感器感知 + 给定轨迹控制 演示
感知: RGB 相机 -> YOLOv5 目标检测; 深度相机 -> 前向障碍距离/TTC
控制: Carla 地图生成参考轨迹(waypoints) -> 纯跟踪(Pure Pursuit)转向 + TTC 自适应油门
输出: task2_perception.gif + task2_metrics.json
"""
import os
import carla, time, queue, json, math
import numpy as np
import cv2
from PIL import Image
from ultralytics import YOLO

HOST, PORT = '127.0.0.1', 2000
# ---- 启动参数支持: launch 文件通过 args 传入 host/port ----
import argparse as _argparse
_ap = _argparse.ArgumentParser(add_help=False)
_ap.add_argument('--host', default=HOST)
_ap.add_argument('--port', type=int, default=PORT)
_ap.add_argument('--out', default='output', help='输出目录(相对当前工作目录)')
_known, _unknown = _ap.parse_known_args()
HOST, PORT = _known.host, _known.port
OUT_DIR = _known.out
os.makedirs(OUT_DIR, exist_ok=True)

OUT_GIF = os.path.join(OUT_DIR, 'task2_perception_tracking.gif')
IMG_W, IMG_H = 640, 480
TOTAL_FRAMES = 70
YOLO_IMGSZ = 320

client = carla.Client(HOST, PORT)
client.set_timeout(30.0)
world = client.get_world()
carla_map = world.get_map()
print('MAP:', carla_map.name, flush=True)

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
settings.no_rendering_mode = False
world.apply_settings(settings)

for a in world.get_actors().filter('vehicle.*'):
    try:
        a.destroy()
    except Exception:
        pass

spawn_points = carla_map.get_spawn_points()
ego_bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
ego_tf = spawn_points[0]
ego = world.spawn_actor(ego_bp, ego_tf)
print('EGO:', ego.id, flush=True)
ego_loc = ego_tf.location

# ---- NPC 车辆（感知目标），放在 ego 周边 ----
npcs = []
veh_bps = world.get_blueprint_library().filter('vehicle.*')
world.tick()
world.tick()
fwd = ego_tf.get_forward_vector()
cand = []
for sp in spawn_points[1:]:
    d = sp.location.distance(ego_loc)
    if not (10.0 < d < 120.0):
        continue
    dx = sp.location.x - ego_loc.x
    dy = sp.location.y - ego_loc.y
    dot = (dx * fwd.x + dy * fwd.y) / max(d, 1e-3)
    score = (-dot) * 100 + d        # 前方优先，再近优先
    cand.append((score, sp))
cand.sort(key=lambda x: x[0])
for i, (d, sp) in enumerate(cand[:20]):
    if len(npcs) >= 14:
        break
    bp = veh_bps[i % len(veh_bps)]
    v = world.try_spawn_actor(bp, sp)
    if v is not None:
        try:
            v.set_autopilot(True)
        except Exception:
            pass
        npcs.append(v)
print('NPCS:', len(npcs), 'CAND:', len(cand), 'FRONT:', sum(1 for (sc,_) in cand if sc<0), flush=True)
for _ in range(5):
    world.tick()

# ---- 传感器 ----
cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', str(IMG_W))
cam_bp.set_attribute('image_size_y', str(IMG_H))
cam_bp.set_attribute('fov', '90')
cam = world.spawn_actor(cam_bp, carla.Transform(
    carla.Location(x=1.5, z=2.30), carla.Rotation(pitch=-8)), attach_to=ego)

dep_bp = world.get_blueprint_library().find('sensor.camera.depth')
dep_bp.set_attribute('image_size_x', str(IMG_W))
dep_bp.set_attribute('image_size_y', str(IMG_H))
dep_bp.set_attribute('fov', '90')
dep = world.spawn_actor(dep_bp, carla.Transform(
    carla.Location(x=1.5, z=2.30), carla.Rotation(pitch=-8)), attach_to=ego)

q_img, q_dep = queue.Queue(), queue.Queue()
cam.listen(q_img.put)
dep.listen(q_dep.put)

model = YOLO(os.environ.get('YOLO_WEIGHTS', 'yolov5m.pt'))
print('MODEL_READY', flush=True)
TARGET_CLS = {'car', 'truck', 'bus', 'motorcycle', 'bicycle', 'person', 'train'}

# ---- 参考轨迹: 沿当前车道向前取 waypoints ----
def reference_path(vehicle, n=25, step=3.0):
    wp = carla_map.get_waypoint(vehicle.get_location(),
                                project_to_road=True,
                                lane_type=carla.LaneType.Driving)
    path = []
    cur = wp
    for _ in range(n):
        nxt = cur.next(step)
        if not nxt:
            break
        cur = nxt[0]
        path.append(cur.transform.location)
    return path

def depth_to_meters(image):
    arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        (image.height, image.width, 4)).astype(np.float32)
    b, g, r = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    norm = (r + g * 256.0 + b * 256.0 * 256.0) / (256.0 ** 3 - 1.0)
    return norm * 1000.0

def front_min_depth(depth_m, x0=0.30, x1=0.70, y0=0.18, y1=0.50):
    h, w = depth_m.shape
    roi = depth_m[int(h * y0):int(h * y1), int(w * x0):int(w * x1)]
    return float(np.percentile(roi, 5)) if roi.size else 100.0

def pure_pursuit(vehicle, path, lookahead=6.0):
    if not path:
        return 0.0
    loc = vehicle.get_location()
    tf = vehicle.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    target = path[-1]
    for p in path:
        if p.distance(loc) >= lookahead:
            target = p
            break
    dx, dy = target.x - loc.x, target.y - loc.y
    # 转到车体系
    lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    dist = max(math.hypot(lx, ly), 1e-3)
    alpha = math.atan2(ly, lx)
    ld = max(dist, 2.0)
    steer = math.atan2(2.0 * 4.0 * math.sin(alpha), ld)  # L=4.0m 轴距
    return max(-1.0, min(1.0, steer / math.radians(35)))

frames, det_counts, ttcs = [], [], []
prev = 0.0
t_start = time.time()

for i in range(TOTAL_FRAMES):
    world.tick()
    try:
        img = q_img.get(timeout=5.0)
        dep_img = q_dep.get(timeout=5.0)
    except queue.Empty:
        continue

    rgb = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(
        (img.height, img.width, 4))[:, :, :3].copy()
    depth_m = depth_to_meters(dep_img)
    d_front = front_min_depth(depth_m)

    res = model.predict(rgb, imgsz=YOLO_IMGSZ, conf=0.35, verbose=False)[0]
    boxes, n_det = [], 0
    for b in res.boxes:
        name = res.names[int(b.cls)]
        if name not in TARGET_CLS:
            continue
        n_det += 1
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        conf = float(b.conf)
        boxes.append((x1, y1, x2, y2, name, conf))
        cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(rgb, f'{name} {conf:.2f}', (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ---- 控制: 纯跟踪 + TTC 自适应 ----
    vel = ego.get_velocity()
    speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    path = reference_path(ego)
    steer = pure_pursuit(ego, path)
    ttc = d_front / max(speed, 0.1)
    throttle = 0.75 if speed < 8.0 else 0.35
    brake = 0.0
    if d_front < 6.0 or (ttc < 1.2 and speed > 2.0):
        throttle, brake = 0.0, 0.6
    ctrl = carla.VehicleControl()
    ctrl.throttle, ctrl.steer, ctrl.brake = throttle, steer, brake
    ego.apply_control(ctrl)

    det_counts.append(n_det)
    ttcs.append(round(ttc, 2))

    hud = (f'frame {i+1}/{TOTAL_FRAMES}  det={n_det}  '
           f'v={speed*3.6:5.1f}km/h  steer={steer:+.2f}  '
           f'd={d_front:5.1f}m  TTC={ttc:4.1f}s')
    cv2.rectangle(rgb, (0, 0), (IMG_W, 26), (0, 0, 0), -1)
    cv2.putText(rgb, hud, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 255, 255), 1)
    frames.append(rgb)
    if i == 4:
        cv2.imwrite(os.path.join(OUT_DIR, 't2_debug_frame.png'), rgb)
        cv2.imwrite(os.path.join(OUT_DIR, 't2_debug_depth.png'),
                    cv2.applyColorMap(cv2.convertScaleAbs(depth_m, alpha=2.0),
                                      cv2.COLORMAP_JET))

    if (i + 1) % 10 == 0:
        print(f'[{i+1}/{TOTAL_FRAMES}] det={n_det} v={speed*3.6:.1f}km/h '
              f'd={d_front:.1f}m', flush=True)

elapsed = time.time() - t_start
print('FRAMES:', len(frames), 'ELAPSED:', round(elapsed, 1), flush=True)

if frames:
    imgs = [Image.fromarray(cv2.resize(f, (400, 300))) for f in frames]
    imgs[0].save(OUT_GIF, save_all=True, append_images=imgs[1:],
                 duration=100, loop=0, optimize=True)

metrics = {
    'map': carla_map.name,
    'frames': len(frames),
    'elapsed_s': round(elapsed, 1),
    'avg_fps': round(len(frames) / max(elapsed, 1e-3), 2),
    'avg_detections': round(float(np.mean(det_counts)), 2) if det_counts else 0,
    'max_detections': int(max(det_counts)) if det_counts else 0,
    'min_front_distance_m': round(min(ttcs) if ttcs else 0, 2),
    'min_ttc_s': round(min(ttcs), 2) if ttcs else 0,
}
with open(os.path.join(OUT_DIR, 'task2_metrics.json'), 'w') as f:
    json.dump(metrics, f, indent=2, ensure_ascii=False)
print('METRICS:', json.dumps(metrics, ensure_ascii=False), flush=True)

try:
    settings.synchronous_mode = False
    world.apply_settings(settings)
except Exception as e:
    print('cleanup settings warn:', e, flush=True)
for _a in [cam, dep] + list(npcs) + [ego]:
    try:
        _a.destroy()
    except Exception:
        pass
print('DONE', flush=True)
