"""RRT 二维路径规划演示入口。

用法::

    python3 scripts/main.py                 # 配图默认写到 docs/img/motion_planning/rrt_path_planning/
    python3 scripts/main.py --outdir /tmp/x # 指定输出目录
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim

_MODULE = "rrt_path_planning"


def _default_outdir():
    """默认输出到仓库的 docs/img/motion_planning/<模块>/。

    scripts/ 往上四级是仓库根目录；把脚本拷出去单独运行时找不到 docs/，退回 scripts/out。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, *([os.pardir] * 4)))
    if os.path.isdir(os.path.join(root, "docs")):
        return os.path.join(root, "docs", "img", "motion_planning", _MODULE)
    return os.path.join(here, "out")


def main():
    ap = argparse.ArgumentParser(description="RRT 二维路径规划")
    ap.add_argument("--outdir", default=_default_outdir(),
                    help="配图输出目录（默认 docs/img/motion_planning/%s）" % _MODULE)
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 60)
    print("100 m x 100 m 地图 / 4 个矩形障碍 / 步长 6 m")
    print("=" * 60)
    print("树节点数        : %d" % r["nodes"])
    print("树边数          : %d" % r["edges"])
    print("原始路径长度    : %.2f m" % r["raw_len"])
    print("平滑后路径长度  : %.2f m  (-%.1f%%)"
          % (r["smooth_len"], 100.0 * (1 - r["smooth_len"] / r["raw_len"])))
    print("起终点直线距离  : %.2f m（有障碍，必然绕行）" % r["straight"])
    print("绕行比          : %.2f x" % (r["smooth_len"] / r["straight"]))
    print("-" * 60)
    print("5 次独立运行（不同随机种子）：")
    print("  run  RAW(m)  SMOOTH(m)")
    for i, (seed, raw, sm) in enumerate(r["runs"], 1):
        print("  %-4d %7.2f  %8.2f" % (i, raw, sm))
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
