"""命令行入口：IDM 跟车仿真。

用法:
    python3 scripts/main.py --outdir scripts
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate


def main():
    ap = argparse.ArgumentParser(description="智能驾驶员模型（IDM）跟车仿真")
    ap.add_argument("--outdir", default="scripts", help="配图输出目录")
    args = ap.parse_args()

    res = simulate.run(args.outdir)
    print("单车跟驰: 末端间距 %.2f m, 自车速度 %.2f m/s, 前车速度 %.2f m/s"
          % (res["gap_end"], res["ego_v_end"], res["lead_v_end"]))
    print("车队最终速度: %s (最大离散度 %.3f m/s)"
          % (["%.2f" % v for v in res["platoon_final_speeds"]],
             res["max_final_speed_spread"]))
    print("已生成: %s  idm_gap_speed.png  idm_platoon.png" % args.outdir)


if __name__ == "__main__":
    main()
