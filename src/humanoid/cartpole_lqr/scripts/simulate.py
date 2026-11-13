"""运行仿真并输出配图。"""

import os
import cartpole as cp
import simple_plot as sp


def run(outdir, dt=0.01, T=8.0, theta0=0.1):
    model, K, Q, R, _ = cp.default_problem(dt)
    x0 = [0.0, 0.0, theta0, 0.0]

    ts, st_lqr, f_lqr = cp.simulate(model, K, x0, T, dt, control=True)
    _, st_pass, _ = cp.simulate(model, K, x0, T, dt, control=False)

    theta_lqr = [s[2] for s in st_lqr]
    theta_pass = [s[2] for s in st_pass]
    pos_lqr = [s[0] for s in st_lqr]

    os.makedirs(outdir, exist_ok=True)

    # 图1: 摆角对比
    fig1 = sp.Figure(width=720, height=420)
    fig1.plot(ts, theta_lqr, color=sp.Figure.RED, label="LQR")
    fig1.plot(ts, theta_pass, color=sp.Figure.GRAY, label="PASSIVE")
    fig1.xlabel = "t (s)"
    fig1.ylabel = "theta (rad)"
    fig1.title = "POLE ANGLE: LQR VS PASSIVE"
    fig1.save(os.path.join(outdir, "cartpole_angle.png"))

    # 图2: 小车位置与摆角（LQR，收敛到 0）
    fig2 = sp.Figure(width=720, height=420)
    fig2.plot(ts, pos_lqr, color=sp.Figure.BLUE, label="CART p (m)")
    fig2.plot(ts, theta_lqr, color=sp.Figure.RED, label="ANGLE (rad)")
    fig2.xlabel = "t (s)"
    fig2.ylabel = "value"
    fig2.title = "LQR FULL STATE REGULATION"
    fig2.save(os.path.join(outdir, "cartpole_state.png"))

    return {
        "K": K[0],
        "theta_end_lqr": theta_lqr[-1],
        "theta_end_passive": theta_pass[-1],
        "pos_end_lqr": pos_lqr[-1],
        "max_force": max(abs(u) for u in f_lqr),
    }
