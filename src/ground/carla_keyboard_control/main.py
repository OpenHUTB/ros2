#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 0.9.16 地面载具物理仿真与键盘运动控制 —— 主入口（main.* 约定）。

本模块对应课程任务（1）：对车辆进行物理仿真，通过键盘对车辆进行运动控制。

三种运行方式：
  1. 独立交互模式（默认）：单进程直连 CARLA，pygame 显示前视画面并读键控制。
       python3 main.py --host 192.168.8.1 --follow
  2. 无窗口取证模式（headless）：不依赖任何图形界面，自动执行一段控制序列并
     把相机画面逐帧存为 PNG，同时打印状态表 —— 适用于**虚拟机无 3D 加速**时
     生成"可运行"证据图。
       python3 main.py --host 192.168.8.1 --headless --demo --save_dir ~/shots
  3. ROS launch 模式：拉起 ROS 2 / ROS 1 节点集群。
       python3 main.py --launch          # ROS 2 Humble
       python3 main.py --ros1            # ROS 1 Noetic
  也可直接用标准命令启动：
       ros2 launch carla_keyboard_control main.launch.py
"""

import argparse
import os
import struct
import subprocess
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_keyboard_control import carla_common as cc  # noqa: E402


# ------------------------------------------------------------------ PNG 输出
def _save_png(path, rgb):
    """把 (H,W,3) uint8 RGB 数组写成 PNG。

    纯 zlib 实现，**不依赖 pygame / Pillow，也不需要图形界面**，
    因此在无 3D 加速的虚拟机里也能稳定导出证据图片。
    """
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def _chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += _chunk(b"IDAT", zlib.compress(raw, 6))
    png += _chunk(b"IEND", b"")
    with open(path, "wb") as fp:
        fp.write(png)


# ------------------------------------------------------------------ 控制合成
def compose_control(keys, speed, args):
    """按键 + 当前车速 → (throttle, steer, brake, reverse)。

    逻辑实现集中在 `carla_common.compose_control`（纯函数），本函数只负责把
    命令行参数转成对应标定值，保证 standalone 与 ROS 2 节点行为完全一致。
    """
    return cc.compose_control(
        keys, speed,
        throttle_max=args.throttle_max,
        brake_max=args.brake_max,
        steer_max=args.steer_max,
        rev_threshold=args.rev_threshold,
    )


# 自动演示序列：(持续秒数, 说明, 按下的键)
DEMO_SEQUENCE = [
    (2.5, "直线加速", {"fwd"}),
    (2.0, "右转", {"fwd", "right"}),
    (1.5, "刹车减速", {"rev"}),
    (1.0, "停稳", set()),
    (2.5, "挂倒挡后退", {"rev"}),
]


def _blank_keys():
    return cc.blank_keys()


# ------------------------------------------------------------------ 独立模式
def run_standalone(args):
    """单进程直连 CARLA：交互控制，或 headless 自动取证。"""
    import numpy as np

    pygame = None
    screen = font = None
    if not args.headless:
        import pygame as _pg
        pygame = _pg
        pygame.init()
        # 虚拟机无 3D 加速时强制软件渲染，避免黑屏
        screen = pygame.display.set_mode((args.width, args.height))
        pygame.display.set_caption("CARLA 键控无人车  W/S=油门/刹车(倒车) A/D=转向")
        font = pygame.font.Font(None, 26)

    client, world = cc.connect(args.host, args.port, args.town)
    vehicle, tf = cc.spawn_vehicle(world, args.ego_blueprint)
    holder = {"frame": None}
    # 传感器须常驻引用，否则被 Python 回收后画面消失
    _sensors = [cc.make_rgb_camera(
        world, vehicle, lambda rgb: holder.__setitem__("frame", rgb),
        width=args.width, height=args.height, fov=args.fov, tick=True)]

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    print(f"[就绪] 自车已生成 @ {tf.location}；地图 {args.town}")
    print("[就绪] 控制键：W 前进 / S 刹车(静止时倒车) / A 左 / D 右 / ESC 退出")

    keys = _blank_keys()
    clock = pygame.time.Clock() if pygame else None
    start = time.time()
    shot = 0
    frames_seen = 0
    warned_no_frame = False
    demo_idx = -1
    demo_t0 = time.time()
    next_shot = 0.0

    try:
        while True:
            now = time.time()
            if args.sim_time > 0 and now - start >= args.sim_time:
                print("[提示] 达到 sim_time，自动结束。")
                break

            # ---------------- 按键来源 ----------------
            if args.demo:
                # 自动演示：按序列切换按键
                elapsed = now - demo_t0
                acc = 0.0
                idx = len(DEMO_SEQUENCE) - 1
                for i, (dur, _desc, _k) in enumerate(DEMO_SEQUENCE):
                    if elapsed < acc + dur:
                        idx = i
                        break
                    acc += dur
                else:
                    print("[提示] 演示序列执行完毕。")
                    break
                if idx != demo_idx:
                    demo_idx = idx
                    keys = _blank_keys()
                    for k in DEMO_SEQUENCE[idx][2]:
                        keys[k] = True
                    print(f"[演示 {idx + 1}/{len(DEMO_SEQUENCE)}] "
                          f"{DEMO_SEQUENCE[idx][1]}")
            elif pygame:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        return 0
                    if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                        return 0
                ks = pygame.key.get_pressed()
                keys.update({
                    "fwd": bool(ks[pygame.K_w]),
                    "rev": bool(ks[pygame.K_s]),
                    "left": bool(ks[pygame.K_a]) or bool(ks[pygame.K_LEFT]),
                    "right": bool(ks[pygame.K_d]) or bool(ks[pygame.K_RIGHT]),
                    "left_fine": bool(ks[pygame.K_q]),
                    "right_fine": bool(ks[pygame.K_e]),
                })

            # ---------------- 控制与物理步进 ----------------
            speed = cc.get_speed(vehicle)
            throttle, steer, brake, reverse = compose_control(keys, speed, args)
            cc.apply_control(vehicle, throttle=throttle, steer=steer,
                             brake=brake, reverse=reverse)
            world.tick()
            if args.follow:
                cc.set_spectator_follow(world, vehicle)

            x, y, _z = cc.get_location(vehicle)
            line = (f"x={x:7.2f} y={y:7.2f} v={speed:5.2f} m/s | "
                    f"th={throttle:.1f} st={steer:+.2f} br={brake:.1f} rev={int(reverse)}")
            print(line)

            # ---------------- 取证截图 ----------------
            frame = holder["frame"]
            if frame is not None:
                frames_seen += 1
            elif not warned_no_frame and now - start > 3.0:
                warned_no_frame = True
                print("[警告] 3 秒内未收到任何相机帧。请确认：")
                print("       ① CARLA 服务端窗口已完全加载（未在切换地图）；")
                print("       ② --host 指向的是运行 CarlaUE4 的宿主机；")
                print("       ③ 该地图上车辆出生点未被占用。")
            if args.save_dir and frame is not None and now >= next_shot:
                shot += 1
                name = os.path.join(args.save_dir, f"carla_run_{shot:02d}.png")
                _save_png(name, np.ascontiguousarray(frame))
                print(f"[截图] {name}")
                next_shot = now + 0.5

            # ---------------- 画面显示 ----------------
            if pygame and frame is not None:
                surf = pygame.image.frombuffer(
                    np.ascontiguousarray(frame),
                    (frame.shape[1], frame.shape[0]), "RGB")
                surf = pygame.transform.smoothscale(
                    surf, (args.width, args.height))
                screen.blit(surf, (0, 0))
                screen.blit(font.render(
                    f"x={x:.1f} y={y:.1f}  v={speed:.1f} m/s", True, (0, 255, 0)),
                    (10, 10))
                screen.blit(font.render(
                    f"ctrl th={throttle:.1f} st={steer:.1f} "
                    f"br={brake:.1f} rev={int(reverse)}", True, (0, 255, 0)),
                    (10, 40))
                kd = " ".join(f"{k[0]}{int(v)}" for k, v in keys.items())
                screen.blit(font.render(f"keys: {kd}", True, (255, 200, 0)),
                            (10, 70))
                pygame.display.flip()
                clock.tick_busy_loop(int(1.0 / cc.DT))
    except KeyboardInterrupt:
        pass
    finally:
        if pygame:
            pygame.quit()
        for sensor in _sensors:
            sensor.stop()
            sensor.destroy()
        vehicle.destroy()
        print(f"[完成] 已释放资源；共收到相机帧 {frames_seen} 张，导出截图 {shot} 张")
    return 0


# ------------------------------------------------------------------ ROS 模式
def run_launch(ros_version):
    """用 ROS 1 / ROS 2 的 launch 启动节点集群。"""
    if ros_version == 1:
        cmd = ["roslaunch", "carla_keyboard_control", "main.launch"]
    else:
        cmd = ["ros2", "launch", "carla_keyboard_control", "main.launch.py"]
    print(f"[INFO] 通过 launch 启动：{' '.join(cmd)}")
    return subprocess.call(cmd)


def _parse_bool(value):
    """Parse a boolean that arrives as text.

    ROS 1 launch files can only pass arguments as plain text (e.g.
    ``--follow true`` / ``--follow false``) -- unlike ROS 2, which passes a real
    parameter and keeps the bool type.  Without this, ``--follow false`` makes
    ``store_true`` set follow=True and the stray ``false`` is swallowed by
    ``parse_known_args`` as an unknown positional.
    """
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "y", "t"):
        return True
    if text in ("0", "false", "no", "off", "n", "f", ""):
        return False
    raise argparse.ArgumentTypeError(f"无法识别的布尔值：{value!r}")


def _add_bool(parser, name, help_text):
    """Boolean switch: bare ``--follow`` or explicit ``--follow true|false``."""
    parser.add_argument(name, type=_parse_bool, nargs="?", const=True,
                        default=False, help=help_text)


def main():
    parser = argparse.ArgumentParser(
        description="CARLA 0.9.16 地面载具物理仿真与键盘运动控制")
    parser.add_argument("--host", default=cc.DEFAULT_HOST,
                        help="CARLA 服务端地址（虚拟机填宿主机 IP）")
    parser.add_argument("--port", type=int, default=cc.DEFAULT_PORT)
    parser.add_argument("--town", default=cc.DEFAULT_TOWN)
    parser.add_argument("--ego_blueprint", default=cc.DEFAULT_EGO_BLUEPRINT)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fov", type=float, default=90.0)
    parser.add_argument("--sim_time", type=float, default=0.0,
                        help="仿真秒数，0=不限时直到 ESC")
    _add_bool(parser, "--follow",
              "让 CARLA 大窗口镜头跟随自车（便于录屏）")
    parser.add_argument("--throttle_max", type=float, default=cc.DEFAULT_THROTTLE_MAX)
    parser.add_argument("--brake_max", type=float, default=cc.DEFAULT_BRAKE_MAX)
    parser.add_argument("--steer_max", type=float, default=cc.DEFAULT_STEER_MAX)
    parser.add_argument("--rev_threshold", type=float,
                        default=cc.DEFAULT_REV_THRESHOLD,
                        help="低于该速度(m/s)按 S 视为挂倒挡")
    _add_bool(parser, "--headless",
              "不开图形窗口（虚拟机无 3D 加速时使用）")
    _add_bool(parser, "--demo",
              "自动执行一段控制序列（加速/转向/刹车/倒车）")
    parser.add_argument("--save_dir", nargs="?", const="", default="",
                        help="把相机画面逐帧存为 PNG 的目录"
                             "（取证用；留空=不导出）")
    _add_bool(parser, "--launch",
              "用 ros2 launch 启动 ROS 2 节点集群")
    _add_bool(parser, "--ros1",
              "用 roslaunch 启动 ROS 1 Noetic 节点集群")
    args, _unknown = parser.parse_known_args()

    if args.launch or args.ros1:
        return run_launch(1 if args.ros1 else 2)
    return run_standalone(args)


if __name__ == "__main__":
    sys.exit(main())
