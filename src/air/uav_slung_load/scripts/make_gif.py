#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成效果图 GIF（离屏渲染，不需要显示器）。

- ``demo_swing.gif``：摆角反馈控制器执行 3.5 m 平移的完整过程；
- ``compare_pid_vs_swing.gif``：上下两格同帧对比（上：PD 基线，下：摆角反馈）。

用法：``python3 scripts/make_gif.py --seconds 10 --fps 12``
"""

import argparse
import os
import sys

import imageio
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controllers import CONTROLLERS  # noqa: E402
from sim_core import DT, SlungSim  # noqa: E402

# 与仓库其他模块一致：GIF 放在 docs/img/<模块分类>/<模块名>/ 下，供 mkdocs 引用
ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..",
                      "docs", "img", "air", "uav_slung_load")
CAMERA_FOLLOW_Z = 0.95
CAMERA_DISTANCE = 1.9
CAMERA_AZIMUTH = 130
CAMERA_ELEVATION = -10


def make_camera():
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0.0, 0.0, CAMERA_FOLLOW_Z)
    cam.distance = CAMERA_DISTANCE
    cam.azimuth = CAMERA_AZIMUTH
    cam.elevation = CAMERA_ELEVATION
    return cam


def render_episode(controller_name, seconds=8.0, fps=8, size=(480, 360),
                   payload_mass=0.1, distance=3.5, step_time=1.0):
    sim = SlungSim(payload_mass)
    ctrl = CONTROLLERS[controller_name]()
    sim.reset(payload_mass)
    ctrl.reset()
    renderer = mujoco.Renderer(sim.model, height=size[1], width=size[0])
    cam = make_camera()
    frames = []
    hold = sim.base_pos.copy()
    target = np.array([distance, 0.0, 1.5])
    next_frame = 0.0
    while sim.time < seconds:
        tgt = hold if sim.time < step_time else target
        sim.step(tgt, ctrl)
        if sim.time >= next_frame:
            cam.lookat[0] = float(sim.base_pos[0])
            renderer.update_scene(sim.data, camera=cam)
            frames.append(renderer.render().copy())
            next_frame += 1.0 / fps
    renderer.close()
    return frames


def render_pair(seconds=8.0, fps=8, size=(400, 300), payload_mass=0.1, distance=3.5):
    """PD 与 swing 同步推进，返回上下两格的合成帧。"""
    sims = {}
    ctrls = {}
    renderers = {}
    for name in ("pd", "swing"):
        sim = SlungSim(payload_mass)
        sim.reset(payload_mass)
        ctrls[name] = CONTROLLERS[name]()
        ctrls[name].reset()
        sims[name] = sim
        renderers[name] = mujoco.Renderer(sim.model, height=size[1], width=size[0])
    cam = make_camera()
    frames = []
    hold = sims["pd"].base_pos.copy()
    target = np.array([distance, 0.0, 1.5])
    next_frame = 0.0
    while sims["pd"].time < seconds:
        for name in ("pd", "swing"):
            sim = sims[name]
            tgt = hold if sim.time < 1.0 else target
            sim.step(tgt, ctrls[name])
        if sims["pd"].time >= next_frame:
            cam.lookat[0] = float(sims["pd"].base_pos[0])
            imgs = []
            for name in ("pd", "swing"):
                renderers[name].update_scene(sims[name].data, camera=cam)
                imgs.append(renderers[name].render().copy())
            gap = np.full((6, size[0], 3), 30, dtype=np.uint8)
            frames.append(np.vstack([imgs[0], gap, imgs[1]]))
            next_frame += 1.0 / fps
    for r in renderers.values():
        r.close()
    return frames


def save_gif(frames, path, fps):
    imageio.mimsave(path, frames, duration=1.0 / fps, palettesize=32,
                    subrectangles=True, optimize=True, loop=0)
    size = os.path.getsize(path) / 1024.0 / 1024.0
    print("%s：%d 帧，%.2f MB" % (path, len(frames), size))


def main(argv=None):
    parser = argparse.ArgumentParser(description="生成吊挂负载演示 GIF")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--mass", type=float, default=0.1)
    args = parser.parse_args(argv)

    os.makedirs(ASSETS, exist_ok=True)
    frames = render_episode("swing", args.seconds, args.fps, (480, 360), args.mass)
    save_gif(frames, os.path.join(ASSETS, "demo_swing.gif"), args.fps)
    pair = render_pair(args.seconds, args.fps, (400, 300), args.mass)
    save_gif(pair, os.path.join(ASSETS, "compare_pid_vs_swing.gif"), args.fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
