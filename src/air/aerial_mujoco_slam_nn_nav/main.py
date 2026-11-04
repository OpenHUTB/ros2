#!/usr/bin/env python3
"""空域载具模块一键入口（不依赖 ROS）。

    python3 main.py --mode teleop --scene warehouse     # 键盘飞行
    python3 main.py --mode nav    --scene corridor      # 建图导航（经典栈）
    python3 main.py --mode neural --scene corridor      # 神经规划 + 控制
    python3 main.py --mode e2e    --scene corridor      # 端到端深度图
    python3 main.py --mode test                         # 单元测试
    python3 main.py --mode demo   --out media/demo.gif  # 录一段演示

按键（teleop）：W/S 前后，A/D 左右，Q/E 偏航，R/F 升降，空格悬停，X 退出。
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from aerial_mujoco_slam_nn_nav.nav import NavConfig, Navigation      # noqa: E402
from aerial_mujoco_slam_nn_nav.sim import DroneSim                   # noqa: E402


def run_teleop(scene: str, viewer: bool, seconds: float = 120.0):
    """按住 W/S/A/D 飞行，松开 0.35 s 自动悬停。"""
    import termios
    import tty

    sim = DroneSim(scene=scene, render=viewer)
    dt = 0.02
    vx = vy = yaw_rate = 0.0
    z = 1.2
    last_key = 0.0
    keys = {"w": (0.6, 0.0), "s": (-0.5, 0.0), "a": (0.0, 0.6), "d": (0.0, -0.6)}
    settings = termios.tcgetattr(sys.stdin)
    print("W/S 前后，A/D 左右，Q/E 偏航，R/F 升降，空格悬停，X 退出")
    try:
        tty.setcbreak(sys.stdin.fileno())
        t = 0.0
        while t < seconds:
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1).lower()
                if k == "x":
                    break
                if k == " ":
                    vx = vy = yaw_rate = 0.0
                elif k in keys:
                    vx, vy = keys[k]
                elif k == "q":
                    yaw_rate = 0.8
                elif k == "e":
                    yaw_rate = -0.8
                elif k == "r":
                    z = min(2.2, z + 0.2)
                elif k == "f":
                    z = max(0.6, z - 0.2)
                last_key = time.time()
            if time.time() - last_key > 0.35:
                vx = vy = yaw_rate = 0.0
            sim.set_command(vx=vx, vy=vy, yaw_rate=yaw_rate, z_ref=z)
            sim.step_dt(dt)
            t += dt
            if viewer:
                sim.follow_view(480, 360)
            if sim.is_fallen():
                print("坠机，重新起飞")
                sim.reset()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sim.set_command()
    print("结束")


def run_nav(scene: str, policy: str, viewer: bool, seconds: float = 90.0):
    sim = DroneSim(scene=scene, render=viewer)
    nav = Navigation(sim, NavConfig(), policy=policy, with_depth=(policy == "e2e"))
    last = 0.0
    while nav.t < seconds and nav.mode != "done" and not sim.is_fallen():
        nav.step()
        if viewer and nav.t - last > 0.1:
            last = nav.t
            sim.follow_view(480, 360)
    print(f"scene={scene} policy={policy} -> {nav.summary()}")
    return nav.summary()


def run_demo(out: str, seconds: float = 18.0):
    from PIL import Image
    from aerial_mujoco_slam_nn_nav import model as M
    sim = DroneSim(scene="warehouse", render=True)
    nav = Navigation(sim, NavConfig())
    frames, last = [], -1.0
    while nav.t < seconds and nav.mode != "done":
        nav.step()
        if nav.t - last >= 0.1:
            last = nav.t
            frames.append(sim.follow_view(480, 360, distance=2.1, azimuth=100, elevation=-35))
    imgs = [Image.fromarray(f) for f in frames[::2]]
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=120, loop=0)
    print(f"saved {out} frames={len(frames)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="nav",
                    choices=["teleop", "nav", "neural", "e2e", "test", "demo"])
    ap.add_argument("--scene", default="warehouse")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--out", default="media/demo.gif")
    ap.add_argument("--no-viewer", action="store_true")
    args = ap.parse_args()
    viewer = not args.no_viewer

    if args.mode == "test":
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests"))
        import run_all
        return run_all.main()
    if args.mode == "demo":
        return run_demo(args.out, args.seconds)
    if args.mode == "teleop":
        return run_teleop(args.scene, viewer, args.seconds)
    return run_nav(args.scene, args.mode, viewer, args.seconds)


if __name__ == "__main__":
    sys.exit(main() or 0)
