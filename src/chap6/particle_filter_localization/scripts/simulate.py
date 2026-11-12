"""生成粒子滤波定位教学配图。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simple_plot as sp
import pf as pfmod


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    r = pfmod.simulate()

    # 图 1：真实位置 / 航迹推算 / 粒子滤波估计
    fig = sp.Figure(width=760, height=440)
    fig.plot(r["ts"], r["x_odom"], color=sp.Figure.BLUE, label="ODOM")
    fig.plot(r["ts"], r["x_true"], color=sp.Figure.GRAY, label="TRUE")
    fig.plot(r["ts"], r["x_pf"], color=sp.Figure.RED, label="PF EST")
    fig.xlabel = "t (s)"
    fig.ylabel = "x (m)"
    fig.title = "MCL: ODOMETRY DRIFT VS PARTICLE FILTER"
    fig.save(os.path.join(outdir, "pf_localization.png"))

    # 图 2：定位误差
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(r["ts"], r["err_odom"], color=sp.Figure.BLUE, label="ODOM ERR")
    fig2.plot(r["ts"], r["err_pf"], color=sp.Figure.RED, label="PF ERR")
    fig2.xlabel = "t (s)"
    fig2.ylabel = "err (m)"
    fig2.title = "POSITION ERROR: ODOMETRY GROWS, PF BOUNDED"
    fig2.save(os.path.join(outdir, "pf_error.png"))

    return {
        "odom_rmse": pfmod.rmse(r["x_odom"], r["x_true"]),
        "pf_rmse": pfmod.rmse(r["x_pf"], r["x_true"]),
        "odom_final": r["err_odom"][-1],
        "pf_final": r["err_pf"][-1],
        "resamples": r["resamples"],
        "travel": r["x_true"][-1] - r["x_true"][0],
    }
