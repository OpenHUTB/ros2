#!/usr/bin/env python3
"""渲染 passive_walker 的演示 GIF（画面可用 render.py 复现）。

用法：
    python3 render.py --seconds 5.5 --fps 7 --width 960 --height 540 --out 01_simulation.gif

依赖见 requirements.txt（mujoco + imageio）。无显示器的 Linux 服务器先
``export MUJOCO_GL=egl``。资产里已设置 ``offwidth/offheight``，这里显式传入
宽高，保证输出尺寸稳定。
"""

import argparse
import os

import imageio.v2 as imageio
import mujoco

from passive_walker.sim import PassiveWalker


def render(seconds=5.5, fps=7, width=960, height=540, out="01_simulation.gif"):
    sim = PassiveWalker()
    model, data = sim.model, sim.data
    renderer = mujoco.Renderer(model, height=height, width=width)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.distance = 2.2
    camera.elevation = -12
    camera.azimuth = 90
    camera.lookat[:] = [0.0, 0.0, 0.4]

    steps = int(round(seconds * fps))
    steps_per_frame = max(1, int(round((1.0 / fps) / sim.dt)))
    frames = []
    for _ in range(steps):
        sim.step(steps_per_frame)
        camera.lookat[0] = sim.x          # 相机跟着轮子走，画面不会把轮子跑出视野
        renderer.update_scene(data, camera)
        frames.append(renderer.render())
    renderer.close()

    out_dir = os.path.dirname(os.path.abspath(out))
    os.makedirs(out_dir, exist_ok=True)
    imageio.mimsave(out, frames, fps=fps, macro_block_size=1)
    print("已生成 %s（%d 帧，%dx%d，%.1f s）" % (out, len(frames), width, height, seconds))


def main():
    p = argparse.ArgumentParser(description="渲染 passive_walker 演示 GIF")
    p.add_argument("--seconds", type=float, default=5.5, help="渲染时长（秒）")
    p.add_argument("--fps", type=int, default=7, help="GIF 帧率")
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    p.add_argument("--out", default="01_simulation.gif")
    a = p.parse_args()
    render(a.seconds, a.fps, a.width, a.height, a.out)


if __name__ == "__main__":
    main()
