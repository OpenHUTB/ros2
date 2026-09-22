#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业一 · CARLA 无人车键盘运动控制（任务 1）

在 CARLA 0.9.16 中物理仿真无人车，通过键盘实时控制：
    W / ↑    油门（加速）
    S / ↓    刹车
    A / ←    左转
    D / →    右转
    Q / E    转向微调（同向/反向）——可选
    SPACE    手刹
    ESC      退出

说明：
  * 依赖已启动的 CARLA 服务端（./CarlaUE4.sh -carla-rpc-port=2000 + ./CarlaUE4 -quality-level=Low）。
  * 默认用 RGB 相机渲染自车前向视角并显示在 pygame 窗口，提供实时 3D 画面。
  * CARLA 本身自带 spectator 视角可看车辆；本模块额外把车头相机画面送到 pygame。

用法（独立运行）：
    python 01_control/main.py --host 127.0.0.1 --port 2000

ROS launch（示例，在已配 roslaunch 的主机上）：
    roslaunch 01_control/launch/main.launch.py
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import (  # noqa: E402
    connect, reset_sync, spawn_vehicle, make_rgb_camera, make_lidar,
    apply_control, get_location, get_yaw, get_speed, _tick_once,
    set_spectator_follow,
    DEFAULT_HOST, DEFAULT_PORT, DEFAULT_MAP, DT,
)

import pygame

KEYS = {
    "up": False, "down": False, "left": False, "right": False,
    "fwd": False, "rev": False, "left_fine": False, "right_fine": False,
}


def _handle_event(ev):
    if ev.type == pygame.KEYDOWN or ev.type == pygame.KEYUP:
        down = (ev.type == pygame.KEYDOWN)
        k = ev.key
        mapping = {
            pygame.K_w: "fwd", pygame.K_s: "rev",
            pygame.K_a: "left", pygame.K_d: "right",
            pygame.K_UP: "up", pygame.K_DOWN: "down",
            pygame.K_LEFT: "left", pygame.K_RIGHT: "right",
            pygame.K_q: "left_fine", pygame.K_e: "right_fine",
        }
        if k in mapping:
            KEYS[mapping[k]] = down


def keyboard_control(world, vehicle, sensor_cb, sim_time=0.0, dt=DT, follow=False):
    """主循环：pygame 显示车头相机画面 + 键盘映射控制。

    sim_time <= 0 => 不限时，直到 ESC。
    """
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("CARLA 键控无人车  W/S=油门刹车 A/D=转向  ESC=退出")
    clock = pygame.time.Clock()

    start_t = time.time()
    elapsed = 0.0
    throttle = 0.0
    steer = 0.0
    try:
        while True:
            if sim_time > 0 and elapsed >= sim_time:
                print("[提示] 达到 sim_time 时长，自动结束。")
                break

            # 先处理窗口关闭/ESC（事件轮询）
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    print("退出")
                    return
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    print("退出")
                    return
            # 逐帧直读键盘状态（比事件式更可靠，焦点在 pygame 窗口即可读到）
            ks = pygame.key.get_pressed()
            KEYS.update({
                "fwd":  bool(ks[pygame.K_w]),    "rev":  bool(ks[pygame.K_s]),
                "left": bool(ks[pygame.K_a]) or bool(ks[pygame.K_LEFT]),
                "right": bool(ks[pygame.K_d]) or bool(ks[pygame.K_RIGHT]),
                "left_fine": bool(ks[pygame.K_q]),
                "right_fine": bool(ks[pygame.K_e]),
            })

            # 按键 -> 控制量
            throttle = 0.6 if KEYS["fwd"] else 0.0
            brake = 0.8 if KEYS["rev"] else 0.0
            steer = 0.0
            if KEYS["left"]:
                steer -= 0.6
            if KEYS["right"]:
                steer += 0.6
            if KEYS["left_fine"]:
                steer -= 0.15
            if KEYS["right_fine"]:
                steer += 0.15

            apply_control(vehicle, throttle=throttle, steer=steer, brake=brake)
            world.tick()  # 同步步进一帧

            # 可选：让 CARLA 大窗口镜头跟随自车
            if follow:
                set_spectator_follow(world, vehicle)

            # 显示车头相机最近一帧（若已回调）
            frame = sensor_cb.frame
            if frame is not None:
                surf = pygame.image.frombuffer(
                    np.ascontiguousarray(frame), (frame.shape[1], frame.shape[0]), "RGB")
                surf = pygame.transform.smoothscale(surf, (640, 480))
                screen.blit(surf, (0, 0))
            # HUD
            x, y = get_location(vehicle)
            speed = get_speed(vehicle)
            font = pygame.font.Font(None, 28)
            screen.blit(font.render(f"x={x:.1f} y={y:.1f}  v={speed:.1f} m/s",
                                    True, (0, 255, 0)), (10, 10))
            screen.blit(font.render(f"ctrl th={throttle:.1f} st={steer:.1f} br={brake:.1f}",
                                    True, (0, 255, 0)), (10, 40))
            # 按键诊断：实时显示各键是否被读到（用于排查“键盘没反应”）
            kd = " ".join(f"{k[0]}{int(v)}" for k, v in KEYS.items())
            screen.blit(font.render(f"keys: {kd}", True, (255, 200, 0)), (10, 70))
            pygame.display.flip()
            clock.tick_busy_loop(int(1.0 / dt))
            elapsed = time.time() - start_t
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


class _SensorHolder:
    def __init__(self):
        self.frame = None


def main():
    p = argparse.ArgumentParser(description="CARLA 键控无人车 任务1")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--town", default=DEFAULT_MAP)
    p.add_argument("--sim_time", type=float, default=0.0,
                   help="仿真秒数，0=不限时直到 ESC（默认）。")
    p.add_argument("--follow", action="store_true",
                   help="让 CARLA 大窗口镜头自动跟随自车（便于观察/录屏）。")
    p.add_argument("--with_lidar", action="store_true", help="额外挂载雷达用于演示")
    args = p.parse_args()

    client, world = connect(args.host, args.port, args.town)
    vehicle, tf = spawn_vehicle(world)

    holder = _SensorHolder()
    # 传感器必须常驻引用，否则 Python 会回收导致画面消失
    _sensors = [make_rgb_camera(world, vehicle,
                                lambda rgb: setattr(holder, "frame", rgb),
                                tick=True)]
    if args.with_lidar:
        _sensors.append(make_lidar(world, vehicle, lambda pc: None, tick=True))

    print(f"[就绪] 自车已生成 @ {tf.location}. 按 W/S/A/D/方向键控制，ESC 退出。")
    keyboard_control(world, vehicle, holder, sim_time=args.sim_time, follow=args.follow)
    print("[完成] 释放资源")
    vehicle.destroy()


if __name__ == "__main__":
    main()
