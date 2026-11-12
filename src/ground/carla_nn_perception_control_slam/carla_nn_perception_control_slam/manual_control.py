#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务① 键盘运动控制演示 (manual_control 程序化等价实现)
按键映射: W=throttle, S=brake, A/D=steer
用队列确保每帧相机图像都被捕获
"""
import os
import carla
import time
import queue
import numpy as np
from PIL import Image

HOST = '127.0.0.1'
PORT = 2000

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

client = carla.Client(HOST, PORT)
client.set_timeout(30.0)
world = client.get_world()
print('MAP:', world.get_map().name)

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

bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
sp = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(bp, sp)
print('VEHICLE:', vehicle.id)

cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '800')
cam_bp.set_attribute('image_size_y', '600')
cam = world.spawn_actor(cam_bp, carla.Transform(
    carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=-15)), attach_to=vehicle)

image_queue = queue.Queue()
cam.listen(image_queue.put)

frames = []

def grab_one(timeout=5.0):
    image = image_queue.get(timeout=timeout)
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = arr.reshape((image.height, image.width, 4))[:, :, :3].copy()
    frames.append(arr)

# 预热: 丢弃前几帧
for _ in range(5):
    world.tick()
    try:
        image_queue.get(timeout=3.0)
    except queue.Empty:
        pass

sequence = [
    (1.0, 0.0, 0.0, 'W: accelerate', 25),
    (0.5, -0.5, 0.0, 'A: steer left', 25),
    (1.0, 0.0, 0.0, 'W: straight', 20),
    (0.5, 0.5, 0.0, 'D: steer right', 25),
    (0.0, 0.0, 1.0, 'S: brake', 15),
]

for throttle, steer, brake, desc, n in sequence:
    c = carla.VehicleControl()
    c.throttle = throttle
    c.steer = steer
    c.brake = brake
    vehicle.apply_control(c)
    print('KEY:', desc)
    for _ in range(n):
        world.tick()
        try:
            grab_one(timeout=5.0)
        except queue.Empty:
            pass

settings.synchronous_mode = False
world.apply_settings(settings)
time.sleep(0.3)
cam.destroy()
vehicle.destroy()

print('FRAMES:', len(frames))
if frames:
    imgs = [Image.fromarray(f).resize((400, 300)) for f in frames]
    out_gif = os.path.join(OUT_DIR, 'task1_manual_control.gif')
    imgs[0].save(out_gif, save_all=True,
                 append_images=imgs[1:], duration=50, loop=0)
    print('GIF_SAVED')
print('DONE')
