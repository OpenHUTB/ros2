"""生成卡尔曼滤波教学配图。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simple_plot as sp
import kalman as kf


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    r = kf.simulate()

    # 图 1：位置——真值 / 原始测量 / KF 估计
    fig = sp.Figure(width=760, height=440)
    fig.plot(r["ts"], r["z"], color=sp.Figure.BLUE, label="MEAS Z")
    fig.plot(r["ts"], r["p_true"], color=sp.Figure.GRAY, label="TRUE P")
    fig.plot(r["ts"], r["p_hat"], color=sp.Figure.RED, label="KF P")
    fig.xlabel = "t (s)"
    fig.ylabel = "p (m)"
    fig.title = "KALMAN FILTER: POSITION TRACKING"
    fig.save(os.path.join(outdir, "kf_position.png"))

    # 图 2：速度——速度不可直接测量，由 KF 估出
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(r["ts"], r["v_true"], color=sp.Figure.GRAY, label="TRUE V")
    fig2.plot(r["ts"], r["v_hat"], color=sp.Figure.RED, label="KF V")
    fig2.xlabel = "t (s)"
    fig2.ylabel = "v (m/s)"
    fig2.title = "VELOCITY ESTIMATE (NOT MEASURED)"
    fig2.save(os.path.join(outdir, "kf_velocity.png"))

    idx = [i for i, t in enumerate(r["ts"]) if t >= 3.0]
    ss_pos = kf.rmse([r["p_hat"][i] for i in idx], [r["p_true"][i] for i in idx])
    ss_vel = kf.rmse([r["v_hat"][i] for i in idx], [r["v_true"][i] for i in idx])

    return {
        "meas_rmse": kf.rmse(r["z"], r["p_true"]),
        "pos_rmse": kf.rmse(r["p_hat"], r["p_true"]),
        "vel_rmse": kf.rmse(r["v_hat"], r["v_true"]),
        "ss_pos_rmse": ss_pos,
        "ss_vel_rmse": ss_vel,
        "final_v": r["v_hat"][-1],
        "true_v": r["v_true"][-1],
    }
