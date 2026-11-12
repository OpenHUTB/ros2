#!/usr/bin/env python3
"""一键复现：跑全部场景、打印指标表并生成配图。

用法::

    python main.py                 # 全部场景 + 出图
    python main.py --no-sweep      # 跳过参数扫描（快）
"""
from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / "src"), str(ROOT)]

from sim.run_sim import main as run_main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(run_main(sys.argv[1:]))
