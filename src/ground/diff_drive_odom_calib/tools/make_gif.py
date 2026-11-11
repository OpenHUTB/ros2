#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 MuJoCo 渲染帧合成 GIF（作业要求"文档含动图"）。

为什么自己做而不装 ImageMagick/ffmpeg：
    虚拟机里只确认有 PIL 10.4，没有 convert/ffmpeg。
    PIL 的 Image.save(..., save_all=True, append_images=[...]) 就能直接写 GIF，
    零额外依赖。

用法：
    python3 tools/make_gif.py                  # 默认 320x180、8 秒、10 fps
    python3 tools/make_gif.py --seconds 12 --fps 12 --width 400
"""

import argparse
import math
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from diff_drive_odom_calib import config                       # noqa: E402
from diff_drive_odom_calib.mujoco_env import DiffDriveEnv      # noqa: E402


def square_segments(side, v, w):
    segs = []
    turn = (math.pi / 2) / w
    for _ in range(4):
        segs.append((v, 0.0, side / v))
        segs.append((0.0, w, turn))
    return segs


def main(argv=None):
    ap = argparse.ArgumentParser(description="MuJoCo 渲染帧合成 GIF")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=180)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--side", type=float, default=4.0)
    ap.add_argument("--v", type=float, default=0.30)
    ap.add_argument("--w", type=float, default=0.80)
    ap.add_argument("--out", default=str(PKG_ROOT / "docs_gif" / "square_drive.gif"))
    args = ap.parse_args(argv)

    from PIL import Image

    env = DiffDriveEnv(config.resolve_scene_path(None), verbose=False)
    segs = square_segments(args.side, args.v, args.w)
    frames = []
    n_per_frame = max(1, int(round(config.CONTROL_HZ / args.fps)))
    print("[GIF] 渲染方形路径 %.0f×%.0f m，%dx%d @ %d fps"
          % (args.side, args.side, args.width, args.height, args.fps))

    env.reset((0.0, 0.0, 0.0))
    for si, (v, w, dur) in enumerate(segs):
        env.set_twist(v, w)
        n = int(round(dur * config.CONTROL_HZ))
        for k in range(n):
            env.step()
            if k % n_per_frame == 0:
                frames.append(Image.fromarray(env.render(args.width, args.height)))
        print("   段 %d/%d 完成，累计 %d 帧" % (si + 1, len(segs), len(frames)))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000.0 / args.fps), loop=0, optimize=True)
    size_mb = out.stat().st_size / 1e6
    print("[GIF] %s（%d 帧，%.1f MB）" % (out, len(frames), size_mb))
    if size_mb > 10:
        print("[警告] 超过 10 MB，请降低 --width 或 --fps")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
