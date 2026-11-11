# -*- coding: utf-8 -*-
"""muscle_fatigue 演示入口（在 scripts/ 目录下直接运行）。"""

from __future__ import annotations

import argparse
import os

from simulate import main as sim_main

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="肌肉疲劳建模演示")
    ap.add_argument("--outdir", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--quick", action="store_true", help="缩短仿真时长")
    args = ap.parse_args()
    sim_main(args.outdir, quick=args.quick)
