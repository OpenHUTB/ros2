"""生成速度规划教学配图：梯形 vs S 曲线。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# simple_plot 由三个模块共用一份：源码树里在 src/motion_planning/common/，
# colcon 安装后各包会把它拷进自己的 lib/<pkg>/，两个位置都试一下。
_HERE = os.path.dirname(os.path.abspath(__file__))
_COMMON = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir, "common"))
for _dir in (_HERE, _COMMON):
    if os.path.isfile(os.path.join(_dir, "simple_plot.py")):
        sys.path.insert(0, _dir)
        break

import simple_plot as sp
import profile as pf

D = 1.0        # 移动距离 (m)
VMAX = 0.5     # 速度上限 (m/s)
AMAX = 1.0     # 加速度上限 (m/s^2)
JMAX = 5.0     # jerk 上限 (m/s^3)


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    trap = pf.TrapezoidProfile(D, VMAX, AMAX)
    scurve = pf.SCurveProfile(D, VMAX, AMAX, JMAX)

    tt, _, vt, at = trap.samples(0.002)
    ts, _, vs, as_ = scurve.samples()

    # 图 1：速度曲线
    fig = sp.Figure(width=760, height=440)
    fig.plot(tt, vt, color=sp.Figure.BLUE, label="TRAP V")
    fig.plot(ts, vs, color=sp.Figure.RED, label="S-CURVE V")
    fig.xlabel = "t (s)"
    fig.ylabel = "v (m/s)"
    fig.title = "VELOCITY: TRAPEZOID VS S-CURVE"
    fig.save(os.path.join(outdir, "profile_velocity.png"))

    # 图 2：加速度曲线（梯形是方波，S 曲线是梯形）
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(tt, at, color=sp.Figure.BLUE, label="TRAP A")
    fig2.plot(ts, as_, color=sp.Figure.RED, label="S-CURVE A")
    fig2.xlabel = "t (s)"
    fig2.ylabel = "a (m/s2)"
    fig2.title = "ACCEL: SQUARE (TRAP) VS TRAPEZOID (S)"
    fig2.save(os.path.join(outdir, "profile_accel.png"))

    return {
        "trap_T": trap.T,
        "trap_ta": trap.ta,
        "trap_tc": trap.tc,
        "s_T": scurve.T,
        "s_vpeak": scurve.vpeak,
        "s_peak_jerk": scurve.peak_jerk(),
        "s_peak_accel": scurve.peak_accel(),
        "s_end": scurve.ss[-1],
        "trap_end": trap.eval(trap.T)[0],
        "short_trap": pf.TrapezoidProfile(0.05, VMAX, AMAX).vpeak,
        "short_s": pf.SCurveProfile(0.05, VMAX, AMAX, JMAX).vpeak,
    }
