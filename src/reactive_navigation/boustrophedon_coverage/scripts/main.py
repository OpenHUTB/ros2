#!/usr/bin/env python3
"""牛耕式全覆盖演示入口。

用法::

    python3 scripts/main.py                    # 配图默认写到 docs/img/reactive_navigation/boustrophedon_coverage/
    python3 scripts/main.py --outdir /tmp/x   # 指定输出目录
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim

_MODULE = "boustrophedon_coverage"


def _default_outdir():
    """默认输出到仓库的 docs/img/reactive_navigation/<模块>/。

    scripts/ 往上四级是仓库根目录；把脚本拷出去单独运行时找不到 docs/，退回 scripts/out。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, *([os.pardir] * 4)))
    if os.path.isdir(os.path.join(root, "docs")):
        return os.path.join(root, "docs", "img", "reactive_navigation", _MODULE)
    return os.path.join(here, "out")


def main():
    ap = argparse.ArgumentParser(description="牛耕式全覆盖规划")
    ap.add_argument("--outdir", default=_default_outdir(),
                    help="配图输出目录（默认 docs/img/reactive_navigation/%s）" % _MODULE)
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 60)
    print("L 形房间（30x10 + 15x20） / 牛耕式全覆盖")
    print("=" * 60)
    print("基准方案（gap = 2 m）")
    print("  房间面积      : %.0f m^2" % r["area"])
    print("  扫掠行数      : %d" % r["rows"])
    print("  路径长度      : %.1f m（理论下限 面积/gap = %.1f m）"
          % (r["length"], r["area"] / 2.0))
    print("  采样覆盖率    : %.1f%%（步长 0.5 m，判定半径 = 作业宽度 2.2 m 的一半）"
          % (r["coverage"] * 100.0))
    print("  预估作业时间  : %.0f s（1 m/s 匀速）" % r["est_time"])
    print("-" * 60)
    print("行距扫描（gap 1 ~ 5 m，作业宽度固定 2.2 m）")
    print("  长度区间      : %.0f ~ %.0f m（随 gap 增大而缩短）"
          % (r["gap_len_max"], r["gap_len_min"]))
    print("  各行距覆盖率  : " + "  ".join(
        "%s m→%.0f%%" % (k, v * 100.0) for k, v in sorted(
            r["gap_cov"].items(), key=lambda kv: float(kv[0]))))
    print("  规律          : length ~ 面积/gap + 掉头开销；gap 超过作业宽度后覆盖率骤降")
    print("-" * 60)
    print("扫掠方向扫描（0~90 度共 7 组）")
    print("  最优方向      : %.0f 度（顺着房间直角边）" % r["theta_best"])
    print("  最差方向      : %.0f 度（长度 %.0f m，斜方向掉头与绕边更多）"
          % (r["theta_worst"], r["theta_len_worst"]))
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
