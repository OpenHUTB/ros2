#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 MuJoCo 渲染帧合成 GIF（作业要求"文档含动图"）。

PIL 直接就能写 GIF，不需要 ImageMagick/ffmpeg（虚拟机里没有）。

动作编排：站立 → 前进 → 转向 → 停止，让动图能看出"它在主动平衡"。
"""

import argparse
import math
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from balance_bot import config                                  # noqa: E402
from balance_bot.control import BalanceController               # noqa: E402
from balance_bot.env import BalanceBotEnv                       # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description="平衡车 GIF")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=180)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--out", default=str(PKG_ROOT / "docs_gif" / "balance_drive.gif"))
    args = ap.parse_args(argv)

    from PIL import Image

    env = BalanceBotEnv(config.resolve_scene_path(None), verbose=False)
    ctrl = BalanceController()
    # (目标速度, 目标转向, 秒)
    plan = [(0.0, 0.0, 1.5), (0.30, 0.0, 3.0), (0.30, 0.6, 2.0),
            (0.0, 0.0, 1.5), (0.0, -0.6, 1.5), (0.0, 0.0, 1.0)]
    frames = []
    n_per = max(1, int(round(config.CONTROL_HZ / args.fps)))
    print("[GIF] %dx%d @ %d fps，共 %.1f s 动作"
          % (args.width, args.height, args.fps, sum(p[2] for p in plan)))

    env.reset(tilt=0.05)          # 先给个 3° 倾角，动图开头能看到它"自己站起来"
    for seg, (v_t, w_t, dur) in enumerate(plan):
        for k in range(int(dur * config.CONTROL_HZ)):
            tl, tr = ctrl(env.state(), v_t, w_t)
            env.set_torque(tl, tr)
            env.step()
            if k % n_per == 0:
                frames.append(Image.fromarray(env.render(args.width, args.height)))
        print("   段 %d/%d 完成（累计 %d 帧）" % (seg + 1, len(plan), len(frames)))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000.0 / args.fps), loop=0, optimize=True)
    mb = out.stat().st_size / 1e6
    print("[GIF] %s（%d 帧，%.1f MB）" % (out, len(frames), mb))
    return 0 if mb <= 10 else 1


if __name__ == "__main__":
    raise SystemExit(main())
