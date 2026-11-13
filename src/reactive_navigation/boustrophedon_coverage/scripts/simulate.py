"""生成牛耕式全覆盖教学配图。"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# simple_plot 由三个模块共用一份：源码树里在 src/reactive_navigation/common/，
# colcon 安装后各包会把它拷进自己的 lib/<pkg>/，两个位置都试一下。
_HERE = os.path.dirname(os.path.abspath(__file__))
_COMMON = os.path.normpath(os.path.join(_HERE, os.pardir, os.pardir, "common"))
for _dir in (_HERE, _COMMON):
    if os.path.isfile(os.path.join(_dir, "simple_plot.py")):
        sys.path.insert(0, _dir)
        break

import simple_plot as sp
import coverage as covmod


def _draw_poly(fig, poly, color):
    xs = [p[0] for p in poly] + [poly[0][0]]
    ys = [p[1] for p in poly] + [poly[0][1]]
    fig.plot(xs, ys, color=color)


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    room = covmod.ROOM_L
    area = abs(covmod.polygon_area(room))

    # 图 1：L 形房间的牛耕式覆盖路径（gap=2 m）
    res = covmod.plan(room, gap=2.0)
    fig = sp.Figure(width=720, height=720)
    _draw_poly(fig, room, sp.Figure.BLUE)
    p = res["path"]
    fig.plot([q[0] for q in p], [q[1] for q in p], color=sp.Figure.RED,
             label="COVERAGE")
    fig.set_xlim(0, 40)
    fig.set_ylim(0, 40)
    fig.xlabel = "x (m)"
    fig.ylabel = "y (m)"
    fig.title = "BOUSTROPHEDON COVERAGE OF L-SHAPED ROOM (gap=2m)"
    fig.save(os.path.join(outdir, "coverage_map.png"))

    # 覆盖率精算（采样步长 0.5 m；判定半径 = 作业宽度/2 = 1.1 m，
    # 作业宽度 w=2.2 m 与行距解耦，不随 gap 缩放）
    cov = covmod.coverage_stats(room, p, step=0.5)

    # 图 2：行距扫描 —— 长度 ≈ 面积 / gap + 掉头开销
    rows = []
    for gap in (1.0, 2.0, 3.0, 4.0, 5.0):
        r = covmod.plan(room, gap=gap)
        rows.append((gap, r["length"], r["rows"], r["path"]))
    xs = [r[0] for r in rows]
    # 只画同量纲的两个量：LENGTH 与理论值 AREA/gap（评审 #1400：
    # ROWS（6~30 行）与其共轴会被压成贴底直线，纵轴还出现负值）
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(xs, [r[1] for r in rows], color=sp.Figure.BLUE, label="LENGTH")
    fig2.plot(xs, [area / g for g in xs], color=sp.Figure.RED,
              label="AREA/gap IDEAL")
    fig2.set_xlim(0.5, 5.5)
    fig2.set_ylim(0.0, 700.0)
    fig2.xlabel = "gap (m)"
    fig2.ylabel = "path length (m)"
    fig2.title = "LENGTH ~ AREA/gap + TURN OVERHEAD"
    fig2.save(os.path.join(outdir, "coverage_gap.png"))

    # 图 2b：覆盖率随行距骤降 —— 行距超过作业宽度 w=2.2 m 后，行间出现
    # 宽 gap-w 的漏扫带（评审 #1400 复核：旧版判定半径随 gap 缩放，
    # gap=5 也"自证"100%，指标看不见漏扫；现与固定作业宽度绑定）
    covs = [(r[0], covmod.coverage_stats(room, r[3], step=0.5)) for r in rows]
    fig2b = sp.Figure(width=760, height=440)
    fig2b.plot([c[0] for c in covs], [c[1] * 100.0 for c in covs],
               color=sp.Figure.GREEN, label="COVERAGE")
    fig2b.set_xlim(0.5, 5.5)
    fig2b.set_ylim(0.0, 105.0)
    fig2b.xlabel = "gap (m)"
    fig2b.ylabel = "sampled coverage (%)"
    fig2b.title = "COVERAGE DROPS WHEN gap > TOOL WIDTH (w=2.2m)"
    fig2b.save(os.path.join(outdir, "coverage_gap_cov.png"))

    # 图 3：扫掠方向扫描 —— L 形房间沿长直边（0°）最省，斜方向掉头
    # 次数多、且要沿凹形边界绕行，长度显著上升（评审 #1400：90° 曾因
    # 行中心压在内墙上退化成漏扫的伪最优，修后 358.0 m 不再短于 0°）
    trows = []
    for deg in (0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0):
        r = covmod.plan(room, gap=2.0, theta=math.radians(deg))
        tcov = covmod.coverage_stats(room, r["path"], step=0.5)
        trows.append((deg, r["length"], r["rows"], tcov))
    xs3 = [r[0] for r in trows]
    fig3 = sp.Figure(width=760, height=440)
    fig3.plot(xs3, [r[1] for r in trows], color=sp.Figure.BLUE, label="LENGTH")
    fig3.set_xlim(-5, 95)
    fig3.set_ylim(330.0, 370.0)
    fig3.xlabel = "sweep direction theta (deg)"
    fig3.ylabel = "path length (m)"
    fig3.title = "SWEEP DIRECTION: ALIGN WITH ROOM EDGES"
    fig3.save(os.path.join(outdir, "coverage_theta.png"))

    return {
        "area": area,
        "rows": res["rows"],
        "length": res["length"],
        "est_time": res["est_time"],
        "coverage": cov,
        "gap_len_min": min(r[1] for r in rows),
        "gap_len_max": max(r[1] for r in rows),
        "gap_cov": {"%.0f" % c[0]: c[1] for c in covs},
        "theta_best": min(trows, key=lambda r: r[1])[0],
        "theta_worst": max(trows, key=lambda r: r[1])[0],
        "theta_len_worst": max(r[1] for r in trows),
    }
