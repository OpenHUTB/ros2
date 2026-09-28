#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 0.9.16 地面载具物理仿真与键盘运动控制 —— 主入口（main.* 约定）。

本模块对应课程任务（1）：对车辆进行物理仿真，通过键盘对车辆进行运动控制。

两种运行方式：
  1. 独立模式（默认）：单进程直连 CARLA，pygame 显示前视画面并读键控制，
     适合 Windows 原生 / 有图形界面的 Ubuntu。
       python3 main.py
       python3 main.py --host 192.168.8.1 --follow
  2. ROS 2 launch 模式：拉起 carla_control_node + keyboard_teleop_node 两个节点，
     适合 ROS 2 Humble 环境（虚拟机 / Ubuntu）。
       python3 main.py --launch
       python3 main.py --ros1          # ROS 1 Noetic（roslaunch）
  也可直接用标准命令启动：
       ros2 launch carla_keyboard_control main.launch.py
"""

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_keyboard_control import carla_common as cc  # noqa: E402


# ------------------------------------------------------------------ 独立模式
def run_standalone(args):
    """单进程直连 CARLA：渲染前视画面 + 键盘控制（真实驾驶逻辑）。"""
    import numpy as np
    import pygame

    client, world = cc.connect(args.host, args.port, args.town)
    vehicle, tf = cc.spawn_vehicle(world, args.ego_blueprint)
    holder = {'frame': None}
    # 传感器须常驻引用，否则被 Python 回收后画面消失
    _sensors = [cc.make_rgb_camera(
        world, vehicle, lambda rgb: holder.__setitem__('frame', rgb),
        width=args.width, height=args.height, fov=args.fov, tick=True)]
    print(f"[就绪] 自车已生成 @ {tf.location}。W/S/A/D 控制，ESC 退出。")

    pygame.init()
    screen = pygame.display.set_mode((args.width, args.height))
    pygame.display.set_caption("CARLA 键控无人车  W/S=油门/刹车(倒车) A/D=转向")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 26)
    keys = {k: False for k in
            ('fwd', 'rev', 'left', 'right', 'left_fine', 'right_fine')}
    start = time.time()
    try:
        while True:
            if args.sim_time > 0 and time.time() - start >= args.sim_time:
                print("[提示] 达到 sim_time，自动结束。")
                break
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    return
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    return
            ks = pygame.key.get_pressed()
            keys.update({
                'fwd': bool(ks[pygame.K_w]),
                'rev': bool(ks[pygame.K_s]),
                'left': bool(ks[pygame.K_a]) or bool(ks[pygame.K_LEFT]),
                'right': bool(ks[pygame.K_d]) or bool(ks[pygame.K_RIGHT]),
                'left_fine': bool(ks[pygame.K_q]),
                'right_fine': bool(ks[pygame.K_e]),
            })

            # 真实驾驶逻辑：有速度时 S 刹车，低速时 S 挂倒挡
            speed = cc.get_speed(vehicle)
            throttle = brake = 0.0
            reverse = False
            if keys['fwd']:
                throttle = args.throttle_max
            elif keys['rev']:
                if speed < args.rev_threshold:
                    reverse, throttle = True, args.throttle_max * 0.8
                else:
                    brake = args.brake_max
            steer = 0.0
            if keys['left']:
                steer -= args.steer_max
            if keys['right']:
                steer += args.steer_max
            if keys['left_fine']:
                steer -= args.steer_max * 0.25
            if keys['right_fine']:
                steer += args.steer_max * 0.25

            cc.apply_control(vehicle, throttle=throttle, steer=steer,
                             brake=brake, reverse=reverse)
            world.tick()
            if args.follow:
                cc.set_spectator_follow(world, vehicle)

            frame = holder['frame']
            if frame is not None:
                surf = pygame.image.frombuffer(
                    np.ascontiguousarray(frame),
                    (frame.shape[1], frame.shape[0]), "RGB")
                surf = pygame.transform.smoothscale(
                    surf, (args.width, args.height))
                screen.blit(surf, (0, 0))
            x, y, _ = cc.get_location(vehicle)
            screen.blit(font.render(
                f"x={x:.1f} y={y:.1f}  v={speed:.1f} m/s", True, (0, 255, 0)), (10, 10))
            screen.blit(font.render(
                f"ctrl th={throttle:.1f} st={steer:.1f} br={brake:.1f} rev={int(reverse)}",
                True, (0, 255, 0)), (10, 40))
            kd = " ".join(f"{k[0]}{int(v)}" for k, v in keys.items())
            screen.blit(font.render(f"keys: {kd}", True, (255, 200, 0)), (10, 70))
            pygame.display.flip()
            clock.tick_busy_loop(int(1.0 / cc.DT))
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        for sensor in _sensors:
            sensor.stop()
            sensor.destroy()
        vehicle.destroy()
        print("[完成] 已释放资源")


# ------------------------------------------------------------------ ROS 模式
def run_launch(ros_version):
    """用 ROS 1 / ROS 2 的 launch 启动节点集群。"""
    if ros_version == 1:
        cmd = ['roslaunch', 'carla_keyboard_control', 'main.launch']
    else:
        cmd = ['ros2', 'launch', 'carla_keyboard_control', 'main.launch.py']
    print(f"[INFO] 通过 launch 启动：{' '.join(cmd)}")
    return subprocess.call(cmd)


def main():
    parser = argparse.ArgumentParser(
        description="CARLA 0.9.16 地面载具物理仿真与键盘运动控制")
    parser.add_argument('--host', default=cc.DEFAULT_HOST,
                        help="CARLA 服务端地址（虚拟机填宿主机 IP）")
    parser.add_argument('--port', type=int, default=cc.DEFAULT_PORT)
    parser.add_argument('--town', default=cc.DEFAULT_TOWN)
    parser.add_argument('--ego_blueprint', default=cc.DEFAULT_EGO_BLUEPRINT)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fov', type=float, default=90.0)
    parser.add_argument('--sim_time', type=float, default=0.0,
                        help="仿真秒数，0=不限时直到 ESC")
    parser.add_argument('--follow', action='store_true',
                        help="让 CARLA 大窗口镜头跟随自车（便于录屏）")
    parser.add_argument('--throttle_max', type=float, default=0.6)
    parser.add_argument('--brake_max', type=float, default=0.8)
    parser.add_argument('--steer_max', type=float, default=0.6)
    parser.add_argument('--rev_threshold', type=float, default=0.5,
                        help="低于该速度(m/s)按 S 视为挂倒挡")
    parser.add_argument('--launch', action='store_true',
                        help="用 ros2 launch 启动 ROS 2 节点集群")
    parser.add_argument('--ros1', action='store_true',
                        help="用 roslaunch 启动 ROS 1 Noetic 节点集群")
    args, unknown = parser.parse_known_args()

    if args.launch or args.ros1:
        return run_launch(1 if args.ros1 else 2)
    return run_standalone(args)


if __name__ == '__main__':
    sys.exit(main())
