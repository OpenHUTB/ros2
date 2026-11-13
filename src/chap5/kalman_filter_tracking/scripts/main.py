"""卡尔曼滤波一维目标跟踪演示入口。

用法::

    python3 scripts/main.py --outdir ../../../docs/img/chap5/kalman_filter_tracking
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim


def main():
    ap = argparse.ArgumentParser(description="卡尔曼滤波一维目标跟踪")
    ap.add_argument("--outdir", default="../../../docs/img/chap5/kalman_filter_tracking")
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 60)
    print("一维匀速目标跟踪（只测位置，测量噪声 sigma=0.5 m）")
    print("=" * 60)
    print("原始测量 RMSE      : %.4f m" % r["meas_rmse"])
    print("KF 位置 RMSE 全程   : %.4f m" % r["pos_rmse"])
    print("KF 位置 RMSE 稳态   : %.4f m  (t>=3s)" % r["ss_pos_rmse"])
    print("KF 速度 RMSE 稳态   : %.4f m/s (速度从未被直接测量)"
          % r["ss_vel_rmse"])
    print("末时刻速度：估计 %.3f / 真实 %.3f m/s" % (r["final_v"], r["true_v"]))
    print("-" * 60)
    print("改善倍数（稳态位置）: %.1f x" % (r["meas_rmse"] / r["ss_pos_rmse"]))
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
