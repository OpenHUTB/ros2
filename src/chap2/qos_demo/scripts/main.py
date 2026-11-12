"""ROS 2 QoS 服务质量演示入口。

用法::

    python3 scripts/main.py --outdir ../../../docs/img/chap2/qos_demo

输出两张 PNG 与终端统计表。
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim


def main():
    ap = argparse.ArgumentParser(description="ROS 2 QoS 服务质量演示")
    ap.add_argument("--outdir", default="scripts", help="配图输出目录")
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 66)
    print("发布 100 Hz / 回调耗时 30 ms（处理能力 33 Hz），仿真 1 s")
    print("=" * 66)
    print("%-22s %10s %12s %10s" % ("配置", "丢包率", "平均延迟(ms)", "末端积压"))
    print("-" * 66)
    print("%-22s %9.1f%% %12.1f %10d"
          % ("BEST_EFFORT depth=1", r["be1_drop"] * 100, r["be1_lat"] * 1000, 0))
    print("%-22s %9.1f%% %12.1f %10d"
          % ("BEST_EFFORT depth=10", r["be10_drop"] * 100, r["be10_lat"] * 1000, 9))
    print("%-22s %9.1f%% %12.1f %10d"
          % ("RELIABLE   depth=1", r["rel1_drop"] * 100, r["rel1_lat"] * 1000,
             r["rel1_backlog"]))
    print("-" * 66)
    print("结论：RELIABLE 一条不丢，但积压线性增长、延迟从 50 ms 涨到 %.0f ms；"
          % (r["rel1_lat"] * 1000))
    print("      加大 depth 只是用延迟换丢包，吞吐并不提高（都受 33 Hz 限制）。")
    print()
    print("keep_last 深度 -> 丢包率：")
    for d, rate in r["sweep"]:
        print("   depth=%-3d %.1f%%" % (d, rate * 100))
    print()
    lt = r["latch"]
    print("durability：发布端在订阅者加入前已发 %d 条" % lt["history_sent"])
    print("   volatile        后加入者收到历史 %d 条" % lt["volatile_got"])
    print("   transient_local 后加入者收到历史 %d 条（latched 最后一条）"
          % lt["transient_got"])
    print()
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
