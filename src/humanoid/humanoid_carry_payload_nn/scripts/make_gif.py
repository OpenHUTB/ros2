#!/usr/bin/env python3
"""make_gif.py —— 渲染搬运演示 GIF（用于 README 与文档）。

    python3 scripts/make_gif.py                       # 默认 10 s / 20 fps / 480x360
    python3 scripts/make_gif.py --seconds 6 --fps 15 --width 400 --height 300

注意：需要可用的 OpenGL 上下文（虚拟机桌面可跑；纯 SSH 无图形会失败）。
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import BalanceMLP, PDExpert                        # noqa: E402
from sim_core import CarryPayloadSim, package_path            # noqa: E402


def default_output():
    """默认写入仓库的 docs/img/humanoid/humanoid_carry_payload_nn/demo.gif；
    脱离仓库运行时退回当前目录的 demo.gif。"""
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo = os.path.dirname(os.path.dirname(os.path.dirname(pkg)))
    docs = os.path.join(repo, "docs")
    if os.path.isdir(docs):
        return os.path.join(docs, "img", "humanoid", "humanoid_carry_payload_nn", "demo.gif")
    return os.path.join(os.getcwd(), "demo.gif")


def main():
    ap = argparse.ArgumentParser(description="渲染演示 GIF")
    ap.add_argument("--controller", default="nn", choices=["open", "pd", "nn"])
    ap.add_argument("--mass", type=float, default=2.0)
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--camera", default="overview")
    ap.add_argument("--output", default=default_output())
    args = ap.parse_args()

    try:
        import imageio
    except ImportError:
        print("[gif] 需要 imageio：pip install imageio pillow")
        return 1

    ctrl = None
    if args.controller == "pd":
        ctrl = PDExpert()
    elif args.controller == "nn":
        weights = package_path("models/balance_mlp.npz")
        ctrl = BalanceMLP.load(weights) if os.path.isfile(weights) else PDExpert()
        if not os.path.isfile(weights):
            print("[gif] 未找到权重，改用 PD 专家")

    sim = CarryPayloadSim(payload_mass=args.mass)
    sim.reset(args.mass)
    every = max(1, int(1.0 / (args.fps * sim.dt)))
    frames = []
    n = int(args.seconds / sim.dt)
    try:
        for i in range(n):
            v_cmd = args.speed * min(1.0, (i * sim.dt) / 0.5)
            sim.step(v_cmd=v_cmd, w_cmd=0.0, controller=ctrl)
            if i % every == 0:
                frames.append(sim.render(camera=args.camera, height=args.height,
                                         width=args.width))
            if not sim.standing():
                print("[gif] t=%.2f s 跌倒，提前结束录制" % (i * sim.dt))
                break
    finally:
        sim.close()

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    imageio.mimsave(args.output, frames, duration=1.0 / args.fps, loop=0)
    size_mb = os.path.getsize(args.output) / 1024.0 / 1024.0
    print("[gif] %d 帧 -> %s（%.1f MB）" % (len(frames), args.output, size_mb))
    return 0


if __name__ == "__main__":
    sys.exit(main())
