#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离屏渲染当前 main.py 的沿墙巡线闭环，导出演示 GIF（开发期工具）。

用法（在包目录下执行）::

    python3 test/record_demo_gif.py              # 默认输出到当前目录的 wall_following_demo_script.gif
    python3 test/record_demo_gif.py --out a.gif  # 自定义输出路径

说明：
- 本脚本只做离屏渲染录帧，不参与 ROS 运行时；起点、航点、走廊、贴墙参数与控制律
  全部复用 ``main.py`` 中的常量与 ``control_step``，因此输出的 GIF 是本包代码的
  离屏渲染结果。
- **仓库里入库的 docs/img/ground/mujoco_wall_following_demo.gif 是 viewer 窗口录屏
  （Peek 录制），不是本脚本产物**。本脚本默认输出到当前工作目录，并且拒绝把结果
  写到那个入库文件路径上，避免误覆盖。
- 需要 EGL / OpenGL 离屏上下文；EGL 走 mesa 软件渲染，无独立显卡的无头环境也可
  出图。个别极简容器若未带 EGL 软件渲染库，需先装好 EGL 驱动；离屏缓冲尺寸已在
  脚本内按目标帧宽高设置，避免默认缓冲不足导致黑帧。
"""
import argparse
import math
import os
import sys
from pathlib import Path
from typing import Optional

os.environ.setdefault("MUJOCO_GL", "egl")

_PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PKG_ROOT))

import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

import main as m  # noqa: E402

# 默认参数按"可接受体积/耗时"取值：768x576、每 32 步录一帧（8803 步约 275 帧）、
# 每帧 160 ms，总时长与闭环一致（约 44 s）。放大分辨率或缩小步长都会显著增大体积。
W, H = 768, 576
RECORD_EVERY = 32   # 每 32 个仿真步录一帧
FRAME_MS = 160      # 每帧 GIF 延迟（ms）


def _committed_gif() -> Optional[Path]:
    """仓库里入库的 viewer 录屏路径；找不到 docs 目录时返回 None。"""
    cur = _PKG_ROOT
    for _ in range(4):
        if (cur / "docs").is_dir():
            return cur / "docs" / "img" / "ground" / "mujoco_wall_following_demo.gif"
        cur = cur.parent
    return None


def _default_out() -> Path:
    """默认输出到当前工作目录，不指向仓库里入库的录屏文件。"""
    return Path.cwd() / "wall_following_demo_script.gif"


def main() -> int:
    parser = argparse.ArgumentParser(description="离屏渲染并导出沿墙巡线演示 GIF")
    parser.add_argument("--out", default=str(_default_out()), help="GIF 输出路径")
    args = parser.parse_args()

    out = Path(args.out)
    committed = _committed_gif()
    if committed is not None and out.resolve() == committed.resolve():
        print(f"拒绝写入 {out}：这是仓库入库的 viewer 录屏文件，"
              "本脚本产物不应覆盖它。请用 --out 指定其它路径。")
        return 2

    model = m.load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # 模型默认离屏缓冲为 640x480，先调到目标尺寸，避免帧缓冲不足导致黑帧
    model.vis.global_.offwidth = W
    model.vis.global_.offheight = H

    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat = [0.0, 0.0, 0.05]
    cam.distance = 6.5
    cam.azimuth = 90
    cam.elevation = -60

    bid = m._body_id(model, "chassis")
    state = {"wp": 0, "mode": "turn", "side_filt": m.RAY_MAX_RANGE}
    frames = []
    step = 0
    arrived = False
    collided = False
    while step < m.MAX_STEPS:
        x, y = data.qpos[0], data.qpos[1]
        yaw = m.quat_to_yaw(data.qpos[3:7])
        front_dist = m._ray_distance(model, data, yaw, 0.0, bid)
        side_dist = m._ray_distance(model, data, yaw, -math.pi / 2, bid)
        left, right, state, arrived, _ = m.control_step(
            x, y, yaw, state, front_dist, side_dist)
        data.ctrl[0], data.ctrl[1] = left, right
        mujoco.mj_step(model, data)
        step += 1

        if m._touch_wall(model, data):
            collided = True
            break

        if step % RECORD_EVERY == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(Image.fromarray(renderer.render()))

        if arrived:
            break

    if not frames:
        print("未录到任何帧，检查录制间隔是否大于闭环步数。")
        return 1
    out.parent.mkdir(parents=True, exist_ok=True)

    # 统一调色板量化：取中间帧生成 128 色全局调色板，再逐帧 quantize。
    # 不量化时 PIL 直存 RGB 帧体积可达 7 MB 级，量化后通常降到 1 MB 以内。
    ref = frames[len(frames) // 2].convert("P", palette=Image.ADAPTIVE, colors=128)
    frames_q = [f.quantize(palette=ref, dither=Image.Dither.NONE) for f in frames]
    frames_q[0].save(
        out, save_all=True, append_images=frames_q[1:],
        duration=FRAME_MS, loop=0, optimize=True,
    )
    size = out.stat().st_size
    print(f"闭环步数: {step}  到达: {arrived}  撞墙: {collided}  "
          f"录帧: {len(frames)}  尺寸: {W}x{H}")
    print(f"帧延迟: {FRAME_MS} ms  文件大小: {size} B ({size/1024/1024:.2f} MB)")
    print(f"已保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
