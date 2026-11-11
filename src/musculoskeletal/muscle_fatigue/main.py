# -*- coding: utf-8 -*-
"""muscle_fatigue 模块入口。

运行方式与依赖
--------------
只用 Python 标准库，不需要 MuJoCo / ROS / matplotlib：

    python3 main.py                  # 完整演示，曲线输出到 scripts/
    python3 main.py --quick          # 缩短仿真时长
    python3 main.py --outdir DIR     # 指定输出目录

放进 catkin 工作空间后也可用 launch 拉起（见 launch/main.launch）。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts"))

from simulate import main as sim_main

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="肌肉疲劳建模演示")
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts"))
    ap.add_argument("--quick", action="store_true", help="缩短仿真时长")
    args = ap.parse_args()
    sim_main(args.outdir, quick=args.quick)
