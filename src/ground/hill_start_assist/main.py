#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模块统一入口：一条命令跑完整个坡道起步辅助实验。

评审指出本模块原先只有 tools/ 下的自检脚本、没有 main.* 入口，挂不上 launch，
仓库也会把它当成"不是模块"。这里补上统一入口：

    python3 main.py               # 跑自检（默认）
    python3 main.py --gif         # 顺带渲染对比动图
    python3 main.py --seconds 5   # 改测量窗口

无 GPU 机器上先装依赖（见 README / 文档 §3）：
    sudo apt install -y libosmesa6 libosmesa6-dev
    pip3 install -U "PyOpenGL>=3.1.6"
然后 MUJOCO_GL=osmesa python3 main.py --gif
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))


def _prepare_render_backend():
    """无 DISPLAY 时给渲染准备一个可用的后端（仅影响 --gif）。

    评审在干净的 20.04 上实测：
      · 什么都不设 → X11 DISPLAY 缺失
      · MUJOCO_GL=egl → PyOpenGL 3.1.0 没有 EGLDeviceEXT
      · MUJOCO_GL=osmesa → 缺 libosmesa6
    所以这里只做"有 DISPLAY 就不动、没有就设 osmesa"，
    真正的依赖安装写在 README/文档里，不在脚本里偷偷 apt install。
    """
    if os.environ.get("MUJOCO_GL"):
        return
    if not os.environ.get("DISPLAY"):
        os.environ["MUJOCO_GL"] = "osmesa"


def run_check(seconds=3.0):
    cmd = [sys.executable, str(ROOT / "tools" / "check_hill.py"),
           "--seconds", str(seconds)]
    return subprocess.call(cmd)


def run_gif():
    _prepare_render_backend()
    cmd = [sys.executable, str(ROOT / "tools" / "make_gif.py")]
    return subprocess.call(cmd, env=os.environ.copy())


def main(argv=None):
    ap = argparse.ArgumentParser(description="坡道起步辅助：模块入口")
    ap.add_argument("--gif", action="store_true", help="额外渲染对比动图")
    ap.add_argument("--seconds", type=float, default=3.0, help="测量窗口 (s)")
    ap.add_argument("--only-gif", action="store_true", help="只渲染动图")
    args = ap.parse_args(argv)

    rc = 0
    if not args.only_gif:
        print("=" * 62, flush=True)
        print("① 坡道起步辅助自检", flush=True)
        print("=" * 62, flush=True)
        rc = run_check(args.seconds)

    if args.gif or args.only_gif:
        print()
        print("=" * 62, flush=True)
        print("② 渲染对比动图", flush=True)
        print("=" * 62, flush=True)
        rc = run_gif() or rc

    print(flush=True)
    print("[main] 退出码 %d" % rc, flush=True)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
