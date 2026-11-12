"""生成 APF 势场避障教学配图。"""

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
import apf as apfmod


def _draw_rect(fig, rect, color):
    x, y, w, h = rect
    fig.plot([x, x + w, x + w, x, x], [y, y, y + h, y + h, y], color=color)


def _draw_cross(fig, p, color, r=1.5):
    fig.plot([p[0] - r, p[0] + r], [p[1], p[1]], color=color)
    fig.plot([p[0], p[0]], [p[1] - r, p[1] + r], color=color)


def run(outdir):
    os.makedirs(outdir, exist_ok=True)

    # 图 1：标准场景 —— 势场下降自然绕开三块障碍，无需任何逃逸
    res = apfmod.navigate(apfmod.START, apfmod.GOAL, apfmod.OBSTACLES)
    assert res["success"], "标准场景势场法应成功"
    fig = sp.Figure(width=720, height=720)
    for rect in apfmod.OBSTACLES:
        _draw_rect(fig, rect, sp.Figure.BLUE)
    p = res["path"]
    fig.plot([q[0] for q in p], [q[1] for q in p], color=sp.Figure.RED, label="PATH")
    _draw_cross(fig, apfmod.START, sp.Figure.GREEN, 2.0)
    _draw_cross(fig, apfmod.GOAL, sp.Figure.PURPLE, 2.0)
    fig.set_xlim(0, 50)
    fig.set_ylim(0, 50)
    fig.xlabel = "x (m)"
    fig.ylabel = "y (m)"
    fig.title = "APF: GRADIENT DESCENT AROUND OBSTACLES"
    fig.save(os.path.join(outdir, "apf_map.png"))

    # 图 2：凹形陷阱 —— 纯梯度卡死 vs 切向滑行逃逸
    stuck = apfmod.navigate(apfmod.TRAP_START, apfmod.TRAP_GOAL,
                            apfmod.TRAP_OBSTACLES, escape=False)
    esc = apfmod.navigate(apfmod.TRAP_START, apfmod.TRAP_GOAL,
                          apfmod.TRAP_OBSTACLES, escape=True)
    fig2 = sp.Figure(width=720, height=720)
    for rect in apfmod.TRAP_OBSTACLES:
        _draw_rect(fig2, rect, sp.Figure.BLUE)
    ps = stuck["path"]
    fig2.plot([q[0] for q in ps], [q[1] for q in ps], color=sp.Figure.GRAY,
              label="NO ESCAPE")
    pe = esc["path"]
    fig2.plot([q[0] for q in pe], [q[1] for q in pe], color=sp.Figure.RED,
              label="TANGENTIAL GLIDE")
    if stuck["stuck"] is not None:
        _draw_cross(fig2, stuck["stuck"], sp.Figure.PURPLE, 1.5)
    _draw_cross(fig2, apfmod.TRAP_START, sp.Figure.GREEN, 2.0)
    _draw_cross(fig2, apfmod.TRAP_GOAL, sp.Figure.PURPLE, 2.0)
    fig2.set_xlim(15, 45)
    fig2.set_ylim(0, 45)
    fig2.xlabel = "x (m)"
    fig2.ylabel = "y (m)"
    fig2.title = "LOCAL MINIMUM: STUCK VS TANGENTIAL ESCAPE"
    fig2.save(os.path.join(outdir, "apf_trap.png"))

    # 图 3：参数扫描 —— 斥力增益 eta 与影响半径 d0。路径长度组组相同
    # （开阔场景下势场只改方向场的形状，不改同伦类的绕行长度），但离障碍
    # 的最小间隙随 eta / d0 单调增大：参数调的不是"绕多远"，是"离多远"。
    # 长度与间隙量纲不同（57 m vs 1.4~2 m），分两张图各画一个量，
    # 避免共用纵轴时间隙序列被压成贴底直线（评审 #1400）。
    rows = []
    straight = (abs(apfmod.GOAL[0] - apfmod.START[0]) ** 2
                + abs(apfmod.GOAL[1] - apfmod.START[1]) ** 2) ** 0.5
    for d0 in (3.0, 4.0, 5.0):
        for eta in (80.0, 100.0, 150.0, 200.0):
            r = apfmod.navigate(apfmod.START, apfmod.GOAL, apfmod.OBSTACLES,
                                eta=eta, d0=d0)
            rows.append((d0, eta, r["length"], r["min_clear"], r["success"]))
    xs = list(range(1, len(rows) + 1))
    fig3 = sp.Figure(width=760, height=440)
    fig3.plot(xs, [r[2] for r in rows], color=sp.Figure.BLUE, label="LENGTH")
    fig3.set_ylim(50.0, 60.0)
    fig3.xlabel = "run  (d0=3,4,5 x eta=80,100,150,200)"
    fig3.ylabel = "path length (m)"
    fig3.title = "SWEEP: LENGTH FLAT AT 57 m FOR ALL 12 RUNS"
    fig3.save(os.path.join(outdir, "apf_sweep.png"))

    fig4 = sp.Figure(width=760, height=440)
    fig4.plot(xs, [r[3] for r in rows], color=sp.Figure.RED, label="MIN CLEAR")
    fig4.set_ylim(0.0, 2.5)
    fig4.xlabel = "run  (d0=3,4,5 x eta=80,100,150,200)"
    fig4.ylabel = "min clearance (m)"
    fig4.title = "SWEEP: CLEARANCE GROWS WITH eta / d0"
    fig4.save(os.path.join(outdir, "apf_sweep_clear.png"))

    return {
        "std_len": res["length"],
        "std_steps": res["steps"],
        "std_escapes": res["escapes"],
        "std_min_clear": res["min_clear"],
        "trap_stuck": stuck["stuck"] is not None,
        "trap_esc_len": esc["length"],
        "trap_esc_escapes": esc["escapes"],
        "sweep_all_success": all(r[4] for r in rows),
        "sweep_len_min": min(r[2] for r in rows),
        "sweep_len_max": max(r[2] for r in rows),
        "sweep_clear_min": min(r[3] for r in rows),
        "sweep_clear_max": max(r[3] for r in rows),
        "straight": straight,
    }
