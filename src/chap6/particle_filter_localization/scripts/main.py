"""一维蒙特卡洛定位（粒子滤波）演示入口。

用法::

    python3 scripts/main.py --outdir ../../../docs/img/chap6/particle_filter_localization
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim


def main():
    ap = argparse.ArgumentParser(description="粒子滤波一维蒙特卡洛定位")
    ap.add_argument("--outdir", default="../../../docs/img/chap6/particle_filter_localization")
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 62)
    print("20 m 走廊 / 3 个已知信标 / 只有测距 / 里程计速度高估 6%")
    print("=" * 62)
    print("行驶距离            : %.2f m" % r["travel"])
    print("航迹推算 RMSE       : %.4f m   末时刻误差 %+.3f m"
          % (r["odom_rmse"], r["odom_final"]))
    print("粒子滤波 RMSE       : %.4f m   末时刻误差 %+.3f m"
          % (r["pf_rmse"], r["pf_final"]))
    print("重采样次数          : %d" % r["resamples"])
    print("-" * 62)
    print("误差抑制倍数        : %.1f x" % (r["odom_rmse"] / r["pf_rmse"]))
    print("结论：里程计误差随里程线性累积（%.2f m）；粒子滤波用测距"
          % r["odom_final"])
    print("      持续纠正，误差稳定在 0.1 m 量级，不随里程发散。")
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
