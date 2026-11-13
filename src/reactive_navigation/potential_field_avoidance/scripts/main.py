#!/usr/bin/env python3
"""APF 人工势场避障演示入口。

用法::

    python3 scripts/main.py                    # 配图默认写到 docs/img/reactive_navigation/potential_field_avoidance/
    python3 scripts/main.py --outdir /tmp/x   # 指定输出目录
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim

_MODULE = "potential_field_avoidance"


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
    ap = argparse.ArgumentParser(description="APF 人工势场避障")
    ap.add_argument("--outdir", default=_default_outdir(),
                    help="配图输出目录（默认 docs/img/reactive_navigation/%s）" % _MODULE)
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 60)
    print("50 m x 50 m 地图 / 3 个矩形障碍 / 步长 0.5 m")
    print("=" * 60)
    print("标准场景")
    print("  路径长度      : %.2f m（直线 %.2f m）" % (r["std_len"], r["straight"]))
    print("  步数          : %d" % r["std_steps"])
    print("  逃逸次数      : %d（开阔场景势场自然流动）" % r["std_escapes"])
    print("  最小障碍间隙  : %.2f m" % r["std_min_clear"])
    print("-" * 60)
    print("凹形陷阱（厚墙挡在 goal 前）")
    print("  关闭逃逸      : %s" % ("卡死在局部极小" if r["trap_stuck"] else "居然到达"))
    print("  切向滑行逃逸  : %.2f m, 逃逸 %d 次" % (r["trap_esc_len"],
                                                 r["trap_esc_escapes"]))
    print("-" * 60)
    print("参数扫描（d0 x eta 共 12 组）")
    print("  全部成功      : %s" % r["sweep_all_success"])
    print("  路径长度区间  : %.1f ~ %.1f m（开阔场景下组组相同）"
          % (r["sweep_len_min"], r["sweep_len_max"]))
    print("  最小间隙区间  : %.2f ~ %.2f m（随 eta/d0 单调增大）"
          % (r["sweep_clear_min"], r["sweep_clear_max"]))
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
