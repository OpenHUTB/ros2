# -*- coding: utf-8 -*-
"""命令行入口：python3 main.py [--outdir DIR] [--quick]

只依赖 Python 标准库。运行后会生成 smc_response.png 并输出两种控制器的误差对比。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from simulate import main as run_sim  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="滑模控制 SMC 仿真")
    ap.add_argument("--outdir", default=os.path.dirname(__file__),
                    help="曲线 PNG 输出目录")
    ap.add_argument("--quick", action="store_true", help="保留接口，当前实现已足够快")
    args = ap.parse_args()
    run_sim(args.outdir)


if __name__ == "__main__":
    main()
