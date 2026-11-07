#!/usr/bin/env python3
"""录制演示动图：跑固定动作序列，用 MuJoCo 离屏渲染逐帧存成 GIF。

    python scripts/record_gif.py --robot h1 --out docs/h1_walk.gif
    python scripts/record_gif.py --robot h1_2 --seconds 12 --fps 20

渲染走 MuJoCo 的 ``Renderer``（OpenGL 离屏），无需开窗口；Linux 无显卡时可用
``MUJOCO_GL=osmesa`` 或 ``egl``。帧率与尺寸默认 18 fps / 288×216，10 s 的动图约 3 MB，
满足仓库 PR 模板「GIF 小于 5 MB」的要求。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_h1_walk import Walker, load_specs, routine_command  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot", default="h1", choices=["h1", "h1_2"])
    parser.add_argument("--out", type=Path, default=Path("h1_walk.gif"))
    parser.add_argument("--seconds", type=float, default=14.0, help="录制时长（仿真秒）")
    parser.add_argument("--skip", type=float, default=0.0, help="先空跑若干秒再开始录（跳过开场站立）")
    parser.add_argument("--fps", type=int, default=18)
    parser.add_argument("--width", type=int, default=288)
    parser.add_argument("--height", type=int, default=216)
    parser.add_argument(
        "--brighten",
        type=float,
        default=5.0,
        help="材质亮度系数。H1/H1_2 原始材质是近黑色（0.1,0.1,0.1），"
        "录屏时放大到中灰才看得清；只改渲染颜色，不影响质量与物理",
    )
    args = parser.parse_args()

    walker = Walker(args.robot, load_specs())
    model, data = walker.model, walker.data
    if args.brighten != 1.0:
        model.geom_rgba[:, :3] = np.clip(model.geom_rgba[:, :3] * args.brighten, 0.0, 0.85)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 95.0
    camera.elevation = -6.0
    camera.distance = 2.9

    every = max(1, int(round(1.0 / (args.fps * walker.control_dt))))
    frames: list[Image.Image] = []
    step = 0
    end_time = args.skip + args.seconds
    while data.time < end_time:
        cmd, _ = routine_command(data.time)
        walker.step(cmd)
        step += 1
        if data.time < args.skip or step % every:
            continue
        # 相机跟随骨盆，视线略高于腰
        camera.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2] + 0.15]
        renderer.update_scene(data, camera=camera)
        frames.append(Image.fromarray(renderer.render()))

    renderer.close()
    if not frames:
        print("没有采集到帧")
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)
    palette = [f.convert("P", palette=Image.Palette.ADAPTIVE, colors=128) for f in frames]
    palette[0].save(
        args.out,
        save_all=True,
        append_images=palette[1:],
        duration=int(1000 / args.fps),
        loop=0,
        optimize=True,
    )
    size_mb = args.out.stat().st_size / 1048576
    print(f"已保存 {args.out}（{len(frames)} 帧，{size_mb:.1f} MB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
