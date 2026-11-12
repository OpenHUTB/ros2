"""梯形 / S 曲线速度规划演示入口。

用法::

    python3 scripts/main.py                 # 配图默认写到 docs/img/motion_planning/trapezoidal_profile/
    python3 scripts/main.py --outdir /tmp/x # 指定输出目录
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate as sim

_MODULE = "trapezoidal_profile"


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
    ap = argparse.ArgumentParser(description="梯形与 S 曲线速度规划")
    ap.add_argument("--outdir", default=_default_outdir(),
                    help="配图输出目录（默认 docs/img/motion_planning/%s）" % _MODULE)
    args = ap.parse_args()

    r = sim.run(args.outdir)

    print("=" * 62)
    print("任务：移动 1.0 m，vmax=0.5 m/s，amax=1.0 m/s^2，jmax=5.0 m/s^3")
    print("=" * 62)
    print("梯形曲线：总时长 %.3f s（加速 %.3f + 匀速 %.3f + 减速 %.3f）"
          % (r["trap_T"], r["trap_ta"], r["trap_tc"], r["trap_ta"]))
    print("          末端位置 %.6f m，加速度是方波 -> jerk 理论上无穷大"
          % r["trap_end"])
    print("S 曲线  ：总时长 %.3f s（%.1f%% 更长）"
          % (r["s_T"], 100.0 * (r["s_T"] / r["trap_T"] - 1)))
    print("          末端位置 %.6f m，峰值 jerk %.2f m/s^3（受 jmax 约束）"
          % (r["s_end"], r["s_peak_jerk"]))
    print("          峰值加速度 %.2f m/s^2，峰值速度 %.2f m/s"
          % (r["s_peak_accel"], r["s_vpeak"]))
    print("-" * 62)
    print("短距离（0.05 m）时两者都冲不到 vmax，退化为三角形态：")
    print("  梯形峰值速度 %.3f m/s，S 曲线峰值速度 %.3f m/s"
          % (r["short_trap"], r["short_s"]))
    print("结论：梯形最短但冲击大；S 曲线用 %.0f%% 的时间换 jerk 有界，"
          % (100.0 * (r["s_T"] / r["trap_T"] - 1)))
    print("      这是机械臂 / CNC / 电梯普遍采用 S 曲线的原因。")
    print("配图已写入:", os.path.abspath(args.outdir))


if __name__ == "__main__":
    main()
