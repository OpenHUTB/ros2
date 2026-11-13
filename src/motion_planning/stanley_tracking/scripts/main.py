"""Stanley 路径跟踪演示入口。

用法::

    python3 scripts/main.py                 # 配图默认写到 docs/img/motion_planning/stanley_tracking/
    python3 scripts/main.py --outdir /tmp/x # 指定输出目录
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim

_MODULE = "stanley_tracking"


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
    ap = argparse.ArgumentParser(description="Stanley 路径跟踪")
    ap.add_argument("--outdir", default=_default_outdir(),
                    help="配图输出目录（默认 docs/img/motion_planning/%s）" % _MODULE)
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 62)
    print("参考路径：半径 8 m 的圆（逆时针）/ 车速 2 m/s / 轴距 2.5 m")
    print("初始横向偏差 1.5 m（圆外），朝向正确")
    print("=" * 62)
    print("最终横向偏差        : %.4f m" % r["e_end"])
    if r["settle"] is not None:
        print("收敛到 0.05 m 用时  : %.2f s" % r["settle"])
    print("最大前轮转角        : %.3f rad（上限 0.6）" % r["max_steer"])
    print("-" * 62)
    print("对照组：")
    print("  去掉 k 项（只修航向）      : 稳态偏差 %.3f m" % r["e_end_k0"])
    print("  去掉曲率前馈（原版 Stanley）: 稳态偏差 %.3f m（理论 %.3f m）"
          % (r["e_end_noff"], r["steady_theory"]))
    print("结论：航向项负责把车头拉回来，k 项负责消除横向偏差；")
    print("      曲率前馈 arctan(L*kappa)，否则会留下恒定的横向偏移。")
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
