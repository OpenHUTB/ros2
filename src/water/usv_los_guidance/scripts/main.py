# -*- coding: utf-8 -*-
"""命令行入口：python3 main.py [--outdir DIR]

只依赖 Python 标准库。运行后生成 los_path.png（俯视轨迹）与 los_crosstrack.png
（横漂误差衰减），并输出末位置与最终横漂。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from simulate import main as run_sim  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="USV LOS 制导仿真")
    ap.add_argument("--outdir", default=os.path.dirname(__file__),
                    help="曲线 PNG 输出目录")
    ap.add_argument("--quick", action="store_true", help="保留接口，实现已足够快")
    args = ap.parse_args()
    run_sim(args.outdir)


if __name__ == "__main__":
    main()
