#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离屏渲染当前 main.py 的正方形轨迹，导出演示 GIF（开发期工具）。

用法（在包目录下执行）::

    python3 test/record_demo_gif.py            # 输出到仓库 docs/img/
    python3 test/record_demo_gif.py --out a.gif # 自定义输出路径

说明：
- 本脚本只做离屏渲染录帧，不参与 ROS 运行时；轨迹点、IK 与杆长几何
  全部复用 ``main.py`` 中的 ``TARGET_CORNERS`` / ``linspace`` / ``ik``，
  因此 GIF 就是本包代码的真实运行输出，不是另外录屏。
- 一圈共 4 条边 × 51 点 = 204 个插值点；``STEP=2`` 隔帧抽稀后写入 GIF，
  以控制体积与帧数。
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

W, H = 1244, 702
STEP = 2            # 隔帧抽稀：每 STEP 个插值点取一帧
FRAME_MS = 30       # 每帧 GIF 延迟（ms）


def _default_out() -> Path:
    """从包目录向上推断仓库根，返回 docs/img 下的默认输出路径。"""
    cur = _PKG_ROOT
    for _ in range(4):
        candidate = cur / "docs" / "img" / "two_dof_arm_square_demo.gif"
        if (cur / "docs").is_dir():
            return candidate
        cur = cur.parent
    return Path("two_dof_arm_square_demo.gif")


def main() -> int:
    parser = argparse.ArgumentParser(description="离屏渲染并导出演示 GIF")
    parser.add_argument("--out", default=str(_default_out()), help="GIF 输出路径")
    args = parser.parse_args()

    model = m.load_model()
    model.vis.global_.offwidth = W
    model.vis.global_.offheight = H
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat = [0.45, 0.0, 0.05]
    cam.distance = 0.7
    cam.azimuth = -60
    cam.elevation = -70

    corners = m.TARGET_CORNERS + [m.TARGET_CORNERS[0]]
    frames = []
    total_points = 0
    for idx in range(len(corners) - 1):
        p0, p1 = corners[idx], corners[idx + 1]
        for tx, ty in m.linspace(p0, p1, 50):
            total_points += 1
            q1, q2 = m.ik(tx, ty)
            data.qpos[0], data.qpos[1] = q1, q2
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam)
            frames.append(Image.fromarray(renderer.render()))

    frames = frames[::STEP]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        out, save_all=True, append_images=frames[1:],
        duration=FRAME_MS, loop=0, optimize=True,
    )

    size = out.stat().st_size
    print(f"插值点总数: {total_points}  抽稀后帧数: {len(frames)}  尺寸: {W}x{H}")
    print(f"帧延迟: {FRAME_MS} ms  文件大小: {size} B ({size/1024/1024:.2f} MB)")
    print(f"已保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
