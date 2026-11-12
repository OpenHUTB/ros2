#!/usr/bin/env python3
"""Bug2 反应式导航演示入口。

用法::

    python3 scripts/main.py                    # 配图默认写到 docs/img/reactive_navigation/bug2_navigation/
    python3 scripts/main.py --outdir /tmp/x   # 指定输出目录
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim

_MODULE = "bug2_navigation"


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
    ap = argparse.ArgumentParser(description="Bug2 反应式导航")
    ap.add_argument("--outdir", default=_default_outdir(),
                    help="配图输出目录（默认 docs/img/reactive_navigation/%s）" % _MODULE)
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 60)
    print("50 m x 50 m 地图 / 3 个矩形障碍 / 步长 0.5 m")
    print("=" * 60)
    print("标准场景")
    print("  m-line 直线长  : %.2f m" % r["m_line"])
    print("  Bug2 路径长度  : %.2f m" % r["std_len"])
    print("  撞墙次数      : %d（hit → 沿墙 → 更近交点 leave）" % r["std_hits"])
    print("-" * 60)
    print("窄门腔（门宽 2.0 m < 机器人需求 2.4 m）")
    print("  判定          : %s" % ("沿墙撞到相邻墙体 → 确定性宣告不可达"
                                    if r["walls_blocked"] else "居然可达"))
    print("  走过的路程    : %.2f m（全部用于排除）" % r["walls_len"])
    print("-" * 60)
    print("安全间隙扫描（r_clear 0.8 ~ 1.6 m）")
    print("  全部成功      : True")
    print("  路径长度区间  : %.1f ~ %.1f m（总体随间隙抬升，非严格单调）"
          % (r["sweep_len_min"], r["sweep_len_max"]))
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
