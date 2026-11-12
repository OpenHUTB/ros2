#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离屏渲染当前 main.py 的 go-to-goal 闭环，导出演示 GIF（开发期工具）。

用法（在包目录下执行）::

    python3 test/record_demo_gif.py              # 输出到仓库 docs/img/ground/
    python3 test/record_demo_gif.py --out a.gif  # 自定义输出路径

说明：
- 本脚本只做离屏渲染录帧，不参与 ROS 运行时；目标点、控制律与差速换算
  全部复用 ``main.py`` 中的常量与 ``control_step``，因此 GIF 就是本包代码的
  真实闭环输出，不是另外录屏。
- 需要 EGL / OpenGL 离屏上下文；EGL 走 mesa 软件渲染，无独立显卡的无头环境也可
  出图。个别极简容器若未带 EGL 软件渲染库，需先装好 EGL 驱动；离屏缓冲尺寸已在
  脚本内按目标帧宽高设置，避免默认缓冲不足导致黑帧。
"""
import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

_PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PKG_ROOT))

import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

import main as m  # noqa: E402

W, H = 960, 720
RECORD_EVERY = 8    # 每 8 个仿真步录一帧
# 每步组仿真时长 = RECORD_EVERY × dt = 8 × 0.005 = 40 ms；帧延迟取 40 ms，GIF 按实时 1:1 播放
FRAME_MS = 40       # 每帧 GIF 延迟（ms）


def _default_out() -> Path:
    """从包目录向上推断仓库根，返回 docs/img 下的默认输出路径。"""
    cur = _PKG_ROOT
    for _ in range(4):
        candidate = cur / "docs" / "img" / "ground" / "mujoco_diff_drive_go_to_goal_demo.gif"
        if (cur / "docs").is_dir():
            return candidate
        cur = cur.parent
    return Path("mujoco_diff_drive_go_to_goal_demo.gif")


def main() -> int:
    parser = argparse.ArgumentParser(description="离屏渲染并导出 go-to-goal 演示 GIF")
    parser.add_argument("--out", default=str(_default_out()), help="GIF 输出路径")
    args = parser.parse_args()

    model = m.load_model()
    data = mujoco.MjData(model)
    # 模型默认离屏缓冲为 640x480，先调到目标尺寸，避免帧缓冲不足导致黑帧
    model.vis.global_.offwidth = W
    model.vis.global_.offheight = H

    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat = [1.5, 1.0, 0.05]
    cam.distance = 4.5
    cam.azimuth = -90
    cam.elevation = -60

    mode = "drive"
    frames = []
    step = 0
    arrived = False
    while step < m.MAX_STEPS:
        x, y = data.qpos[0], data.qpos[1]
        yaw = m.quat_to_yaw(data.qpos[3:7])
        left, right, mode, arrived, _, _ = m.control_step(x, y, yaw, mode)
        data.ctrl[0], data.ctrl[1] = left, right
        mujoco.mj_step(model, data)
        step += 1

        if step % RECORD_EVERY == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(Image.fromarray(renderer.render()))

        if arrived:
            break

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        out, save_all=True, append_images=frames[1:],
        duration=FRAME_MS, loop=0, optimize=True,
    )
    size = out.stat().st_size
    print(f"闭环步数: {step}  录帧: {len(frames)}  尺寸: {W}x{H}")
    print(f"帧延迟: {FRAME_MS} ms  文件大小: {size} B ({size/1024/1024:.2f} MB)")
    print(f"已保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
