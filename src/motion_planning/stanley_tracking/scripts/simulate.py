"""生成 Stanley 路径跟踪教学配图。"""

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
import stanley as st


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    path = st.CirclePath(8.0)
    r = st.simulate(path=path)
    r0 = st.simulate(path=path, k=0.0)

    # 图 1：参考圆 + 实际轨迹
    ref = st.reference_polyline(path, 400)
    fig = sp.Figure(width=720, height=720)
    fig.plot([p[0] for p in ref], [p[1] for p in ref],
             color=sp.Figure.GRAY, label="REF CIRCLE")
    fig.plot(r["x"], r["y"], color=sp.Figure.RED, label="VEHICLE")
    fig.set_xlim(-11, 11)
    fig.set_ylim(-11, 11)
    fig.xlabel = "x (m)"
    fig.ylabel = "y (m)"
    fig.title = "STANLEY TRACKING: START 1.5M OUTSIDE"
    fig.save(os.path.join(outdir, "stanley_track.png"))

    # 图 2：横向误差收敛（k=1.2 vs k=0）
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(r["ts"], r["e"], color=sp.Figure.RED, label="K=1.2")
    fig2.plot(r0["ts"], r0["e"], color=sp.Figure.BLUE, label="K=0")
    fig2.xlabel = "t (s)"
    fig2.ylabel = "e (m)"
    fig2.title = "CROSS-TRACK ERROR: WITH VS WITHOUT K TERM"
    fig2.save(os.path.join(outdir, "stanley_error.png"))

    settle = next((r["ts"][i] for i, v in enumerate(r["e"]) if abs(v) < 0.05), None)
    return {
        "e0": r["e"][0],
        "e_end": r["e"][-1],
        "settle": settle,
        "max_steer": max(abs(d) for d in r["delta"]),
        "e_end_k0": r0["e"][-1],
        "e_end_noff": st.simulate(path=path, feedforward=False)["e"][-1],
        "steady_theory": 2.5 / (1 + 1.2 * 8.0 / 2.0),
    }
