#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务④ 端到端神经网络 (PilotNet 复现): 图像 -> 控制指令
用法:
  python3 task4_e2e.py collect    # 自动驾驶采集 (图像, steer, throttle, brake)
  python3 task4_e2e.py train      # 训练 CNN 并保存 loss 曲线/评估指标
  python3 task4_e2e.py drive      # 端到端推理驾驶 + 录制 gif
"""
import os, sys, time, json, queue, math
import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None
from PIL import Image, ImageDraw

try:
    import carla
except ImportError:
    carla = None

HOST, PORT = '127.0.0.1', 2000
# ---- 启动参数支持: launch 文件通过 args 传入 host/port ----
import argparse as _argparse
_ap = _argparse.ArgumentParser(add_help=False)
_ap.add_argument('--host', default=HOST)
_ap.add_argument('--port', type=int, default=PORT)
_ap.add_argument('--out', default='output', help='输出目录(相对当前工作目录)')
_ap.add_argument('--data-dir', default='data', help='采集数据/训练产物目录')
_known, _unknown = _ap.parse_known_args()
HOST, PORT = _known.host, _known.port
OUT_DIR = _known.out
DATA_DIR = _known.data_dir
os.makedirs(OUT_DIR, exist_ok=True)

IMG_W, IMG_H = 400, 300          # 采集分辨率
NET_W, NET_H = 96, 96            # 网络输入分辨率
# PilotNet 权重路径抽到 paths.py（只依赖 os + 可选 ament，无 carla/cv2），
# 让 CI 无 CARLA 环境也能 import 并断言权重存在
from .paths import MODEL
NPZ = os.path.join(DATA_DIR, 'e2e_data.npz')
HIST = os.path.join(DATA_DIR, 'train_history.json')


def ensure_dir():
    os.makedirs(DATA_DIR, exist_ok=True)


def connect():
    c = carla.Client(HOST, PORT)
    c.set_timeout(30.0)
    w = c.get_world()
    return c, w


def cleanup_vehicles(world):
    for a in world.get_actors().filter('vehicle.*'):
        try:
            a.destroy()
        except Exception:
            pass


def spawn_ego(world, autopilot=False):
    sps = world.get_map().get_spawn_points()
    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    for sp in sps:
        v = world.try_spawn_actor(bp, sp)
        if v is not None:
            if autopilot:
                v.set_autopilot(True)
            return v
    raise RuntimeError('cannot spawn ego')


def attach_cam(world, ego, w=IMG_W, h=IMG_H, pitch=-8.0, z=1.5):
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(w))
    bp.set_attribute('image_size_y', str(h))
    bp.set_attribute('fov', '90')
    cam = world.spawn_actor(bp, carla.Transform(
        carla.Location(x=1.5, z=z), carla.Rotation(pitch=pitch)), attach_to=ego)
    q = queue.Queue()
    cam.listen(q.put)
    return cam, q


def img_to_array(image):
    a = np.frombuffer(image.raw_data, dtype=np.uint8)
    return a.reshape((image.height, image.width, 4))[:, :, :3].copy()


def set_sync(world, on=True, dt=0.05):
    s = world.get_settings()
    s.synchronous_mode = on
    s.fixed_delta_seconds = dt
    world.apply_settings(s)


# ------------------------------------------------------------------ collect
def _teacher_control(world, ego, target_speed=8.5):
    """经典控制教师: 地图参考轨迹 + 纯跟踪转向 + P 速度闭环
    返回 (steer, throttle, brake) —— 作为神经网络的模仿学习标签
    """
    m = world.get_map()
    loc = ego.get_location()
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    wp = m.get_waypoint(loc, project_to_road=True,
                        lane_type=carla.LaneType.Driving)
    path, cur = [], wp
    for _ in range(20):
        nx = cur.next(2.5)
        if not nx:
            break
        cur = nx[0]
        path.append(cur.transform.location)
    # 取 8m 前视点
    target = path[-1] if path else loc
    for p in path:
        if p.distance(loc) >= 8.0:
            target = p
            break
    dx, dy = target.x - loc.x, target.y - loc.y
    lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    alpha = math.atan2(ly, max(abs(lx), 1e-3))
    steer = math.atan2(2.0 * 2.9 * math.sin(alpha), max(math.hypot(lx, ly), 2.0))
    steer = float(np.clip(steer / math.radians(35.0), -1.0, 1.0))
    v = ego.get_velocity().length()
    err = target_speed - v
    throttle = float(np.clip(0.55 + 0.25 * err, 0.0, 1.0))
    brake = float(np.clip(-0.6 * err, 0.0, 1.0)) if err < -1.5 else 0.0
    return steer, throttle, brake


def collect(n_frames=1000):
    """教师控制器驾驶 + 采集图像与标签; 卡住自动换出生点"""
    ensure_dir()
    client, world = connect()
    print('MAP:', world.get_map().name, flush=True)
    set_sync(world, True)
    cleanup_vehicles(world)
    sps = world.get_map().get_spawn_points()
    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]

    state = {'i': 0, 'ego': None, 'cam': None, 'q': None}

    def place():
        if state['ego'] is not None:
            try:
                state['cam'].destroy()
                state['ego'].destroy()
            except Exception:
                pass
        ego = cam = q = None
        for k in range(len(sps)):
            sp = sps[(state['i'] + k) % len(sps)]
            ego = world.try_spawn_actor(bp, sp)
            if ego is not None:
                state['i'] = (state['i'] + k + 1) % len(sps)
                break
        if ego is None:
            raise RuntimeError('no spawn point available')
        cam, q = attach_cam(world, ego)
        state['ego'], state['cam'], state['q'] = ego, cam, q
        for _ in range(6):
            world.tick()
        while not q.empty():
            q.get()

    place()
    X, Y = [], []
    t0 = time.time()
    stuck, tries = 0, 0
    last_speed = 0.0
    while len(X) < n_frames and tries < n_frames * 6:
        tries += 1
        ego = state['ego']
        steer, throttle, brake = _teacher_control(world, ego, target_speed=7.0)
        c = carla.VehicleControl()
        c.steer, c.throttle, c.brake = steer, throttle, brake
        ego.apply_control(c)
        world.tick()
        try:
            image = state['q'].get(timeout=5.0)
        except queue.Empty:
            continue
        speed = ego.get_velocity().length()
        # 卡住检测
        if speed < 0.6:
            stuck += 1
        else:
            stuck = 0
        if stuck > 25:
            print(f'stuck at {len(X)} frames, respawn', flush=True)
            place()
            stuck = 0
            continue
        if speed < 0.6 and len(X) > 50:
            continue          # 静止帧不作为训练样本
        rgb = img_to_array(image)
        X.append(cv2.resize(rgb, (NET_W, NET_H)))
        Y.append([steer, throttle, brake, speed])
        last_speed = speed
        if len(X) % 100 == 0:
            print(f'[{len(X)}/{n_frames}] steer={steer:+.3f} '
                  f'thr={throttle:.2f} v={speed*3.6:.1f}km/h', flush=True)

    X = np.asarray(X, dtype=np.uint8)
    Y = np.asarray(Y, dtype=np.float32)
    np.savez_compressed(NPZ, images=X, labels=Y)
    print('SAVED', NPZ, X.shape, Y.shape,
          'elapsed', round(time.time() - t0, 1), flush=True)
    print('STATS steer_std=%.3f steer_nonzero=%.2f speed_mean=%.2f' % (
        float(np.std(Y[:, 0])), float(np.mean(np.abs(Y[:, 0]) > 0.02)),
        float(np.mean(Y[:, 3]))), flush=True)

    try:
        set_sync(world, False)
        state['cam'].destroy()
        state['ego'].destroy()
    except Exception:
        pass


# -------------------------------------------------------------------- train
def draw_curve(hist, path):
    W, H, pad = 560, 320, 40
    im = Image.new('RGB', (W, H), (255, 255, 255))
    d = ImageDraw.Draw(im)
    d.rectangle([pad, pad, W - 10, H - pad], outline=(120, 120, 120))
    tr, va = hist['train_loss'], hist['val_loss']
    ymax = max(max(tr), max(va)) * 1.1 or 1.0
    for series, color in ((tr, (30, 110, 220)), (va, (220, 80, 60))):
        pts = []
        for i, v in enumerate(series):
            x = pad + (W - 10 - pad) * i / max(len(series) - 1, 1)
            y = H - pad - (H - 2 * pad) * (v / ymax)
            pts.append((x, y))
        if len(pts) > 1:
            d.line(pts, fill=color, width=2)
            for p in pts:
                d.ellipse([p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2], fill=color)
    d.text((pad, 12), 'PilotNet training curve (blue=train, red=val)', fill=(0, 0, 0))
    d.text((pad, H - 26), f'epoch 1..{len(tr)}   ymax={ymax:.4f}', fill=(0, 0, 0))
    im.save(path)


def train(epochs=15, batch=64, lr=1e-3):
    import torch
    import torch.nn as nn
    from torch.utils.data import TensorDataset, DataLoader

    ensure_dir()
    data = np.load(NPZ)
    X = torch.from_numpy(data['images'].astype(np.float32) / 255.0).permute(0, 3, 1, 2)
    Y = torch.from_numpy(data['labels'][:, :3])     # steer, throttle, brake
    n = X.shape[0]
    idx = torch.randperm(n)
    n_val = max(int(n * 0.15), 20)
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    tr_ds = TensorDataset(X[tr_idx], Y[tr_idx])
    va_ds = TensorDataset(X[val_idx], Y[val_idx])
    tr_dl = DataLoader(tr_ds, batch_size=batch, shuffle=True)
    va_dl = DataLoader(va_ds, batch_size=batch)

    class PilotNet(nn.Module):
        """NVIDIA PilotNet 结构的复现 (适配 96x96 输入)"""
        def __init__(self):
            super().__init__()
            self.features = nn.Sequential(
                nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),      # 46
                nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),     # 21
                nn.Conv2d(36, 48, 3, stride=2), nn.ReLU(),     # 10
                nn.Conv2d(48, 64, 3, stride=1), nn.ReLU(),     # 8
                nn.Conv2d(64, 64, 3, stride=1), nn.ReLU(),     # 6
                nn.Flatten(),
            )
            self.head = nn.Sequential(
                nn.Linear(64 * 6 * 6, 100), nn.ReLU(), nn.Dropout(0.2),
                nn.Linear(100, 50), nn.ReLU(),
                nn.Linear(50, 3), nn.Tanh(),
            )

        def forward(self, x):
            return self.head(self.features(x))

    model = PilotNet()
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    lossf = nn.MSELoss()
    hist = {'train_loss': [], 'val_loss': []}
    best = 1e9
    for ep in range(1, epochs + 1):
        model.train()
        tot = 0.0
        for xb, yb in tr_dl:
            opt.zero_grad()
            out = model(xb)
            loss = lossf(out, yb)
            loss.backward()
            opt.step()
            tot += loss.item() * xb.size(0)
        tr_loss = tot / len(tr_ds)
        model.eval()
        vtot = 0.0
        abs_sum = 0.0
        cnt = 0
        with torch.no_grad():
            for xb, yb in va_dl:
                out = model(xb)
                vtot += lossf(out, yb).item() * xb.size(0)
                abs_sum += (out[:, 0] - yb[:, 0]).abs().sum().item()
                cnt += xb.size(0)
        va_loss = vtot / len(va_ds)
        steer_mae = abs_sum / max(cnt, 1)
        hist['train_loss'].append(round(tr_loss, 6))
        hist['val_loss'].append(round(va_loss, 6))
        print(f'epoch {ep:2d}/{epochs} train={tr_loss:.5f} val={va_loss:.5f} '
              f'steer_MAE={steer_mae:.4f}', flush=True)
        if va_loss < best:
            best = va_loss
            torch.save(model.state_dict(), MODEL)

    hist['best_val_loss'] = round(best, 6)
    hist['n_train'] = len(tr_ds)
    hist['n_val'] = len(va_ds)
    hist['steer_mae_deg'] = round(steer_mae * 35.0, 2)   # steer 归一化 -> 角度
    with open(HIST, 'w') as f:
        json.dump(hist, f, indent=2)
    draw_curve(hist, os.path.join(DATA_DIR, 'train_curve.png'))
    print('TRAIN_DONE best_val=', best, 'steer_MAE_deg=', hist['steer_mae_deg'], flush=True)


# -------------------------------------------------------------------- drive
def drive(n_frames=60):
    import torch
    import torch.nn as nn

    class PilotNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.features = nn.Sequential(
                nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
                nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
                nn.Conv2d(36, 48, 3, stride=2), nn.ReLU(),
                nn.Conv2d(48, 64, 3, stride=1), nn.ReLU(),
                nn.Conv2d(64, 64, 3, stride=1), nn.ReLU(),
                nn.Flatten(),
            )
            self.head = nn.Sequential(
                nn.Linear(64 * 6 * 6, 100), nn.ReLU(), nn.Dropout(0.2),
                nn.Linear(100, 50), nn.ReLU(),
                nn.Linear(50, 3), nn.Tanh(),
            )

        def forward(self, x):
            return self.head(self.features(x))

    model = PilotNet()
    print('WEIGHTS:', MODEL, flush=True)
    model.load_state_dict(torch.load(MODEL, map_location='cpu'))
    model.eval()
    print('MODEL_LOADED', flush=True)

    client, world = connect()
    set_sync(world, True)
    cleanup_vehicles(world)
    ego = spawn_ego(world, autopilot=False)
    cam, q = attach_cam(world, ego)
    for _ in range(10):
        world.tick()

    frames, preds, speeds = [], [], []
    t0 = time.time()
    for i in range(n_frames):
        world.tick()
        try:
            image = q.get(timeout=5.0)
        except queue.Empty:
            continue
        rgb = img_to_array(image)
        small = cv2.resize(rgb, (NET_W, NET_H)).astype(np.float32) / 255.0
        t = torch.from_numpy(small).permute(2, 0, 1).unsqueeze(0)
        with torch.no_grad():
            out = model(t)[0].numpy()
        steer = float(np.clip(out[0] * 1.2, -1, 1))
        throttle = float(np.clip(abs(out[1]) * 1.2 + 0.15, 0, 1))
        brake = 0.0
        c = carla.VehicleControl()
        c.steer, c.throttle, c.brake = steer, throttle, brake
        ego.apply_control(c)
        v = ego.get_velocity().length()
        preds.append(steer)
        speeds.append(v * 3.6)

        disp = rgb.copy()
        cv2.rectangle(disp, (0, 0), (IMG_W, 26), (0, 0, 0), -1)
        cv2.putText(disp, f'E2E PilotNet  steer={steer:+.2f} thr={throttle:.2f} '
                          f'v={v*3.6:4.1f}km/h',
                    (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 120), 1)
        # 神经网络输入可视化 (右下角)
        vis = cv2.resize((small * 255).astype(np.uint8), (96, 72))
        disp[IMG_H - 80:IMG_H - 8, IMG_W - 104:IMG_W - 8] = vis
        frames.append(disp)

    elapsed = time.time() - t0
    print('FRAMES:', len(frames), 'ELAPSED:', round(elapsed, 1), flush=True)

    if frames:
        imgs = [Image.fromarray(cv2.resize(f, (400, 300))) for f in frames]
        gif = os.path.join(OUT_DIR, 'task4_e2e_pilotnet.gif')
        imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                     duration=100, loop=0, optimize=True)
        met = {
            'frames': len(frames),
            'elapsed_s': round(elapsed, 1),
            'avg_speed_kmh': round(float(np.mean(speeds)), 2),
            'max_speed_kmh': round(float(np.max(speeds)), 2),
            'mean_abs_steer': round(float(np.mean(np.abs(preds))), 3),
            'inference_fps': round(len(frames) / max(elapsed, 1e-3), 2),
        }
        with open(os.path.join(OUT_DIR, 'task4_metrics.json'), 'w') as f:
            json.dump(met, f, indent=2)
        print('METRICS:', json.dumps(met), flush=True)

    try:
        set_sync(world, False)
        cam.destroy()
        ego.destroy()
    except Exception:
        pass
    print('DONE', flush=True)


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'collect'
    if cmd == 'collect':
        collect(int(sys.argv[2]) if len(sys.argv) > 2 else 1000)
    elif cmd == 'train':
        train(int(sys.argv[2]) if len(sys.argv) > 2 else 15)
    elif cmd == 'drive':
        drive(int(sys.argv[2]) if len(sys.argv) > 2 else 60)
    else:
        print('usage: collect|train|drive')
