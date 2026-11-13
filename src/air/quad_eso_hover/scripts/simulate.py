"""运行四旋翼高度控制仿真并输出配图。"""

import os
import eso
import simple_plot as sp


def ref_fn(t):
    return 1.0            # 目标高度 1 m（t=0 起）


def dist_fn(t):
    return 0.0 if t < 4.0 else 4.0   # t=4s 起施加 4 N 持续上推扰动


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    plant = eso.Plant(m=1.0, g=9.81)
    wc = 2.5
    kp = wc * wc
    kd = 2.0 * wc
    w0 = 15.0
    dt = 0.001
    T = 12.0

    ts, zp, zdp, up = eso.simulate_pd(plant, ref_fn, dist_fn, kp, kd, T, dt)
    ts2, ze, zde, ue, z3, f_true = eso.simulate_eso(plant, ref_fn, dist_fn, kp, kd, w0, T, dt)

    # 图1：高度跟踪对比
    fig1 = sp.Figure(width=720, height=420)
    ref_series = [ref_fn(t) for t in ts2]
    fig1.plot(ts2, ref_series, color=sp.Figure.GRAY, label="REF")
    fig1.plot(ts, zp, color=sp.Figure.BLUE, label="PD ONLY")
    fig1.plot(ts2, ze, color=sp.Figure.RED, label="PD+ESO")
    fig1.xlabel = "t (s)"
    fig1.ylabel = "z (m)"
    fig1.title = "ALTITUDE: PD VS PD+ESO (DISTURB @4S)"
    fig1.save(os.path.join(outdir, "eso_altitude.png"))

    # 图2：ESO 对总扰动的估计
    fig2 = sp.Figure(width=720, height=420)
    fig2.plot(ts2, f_true, color=sp.Figure.GRAY, label="TRUE F")
    fig2.plot(ts2, z3, color=sp.Figure.RED, label="ESO Z3HAT")
    fig2.xlabel = "t (s)"
    fig2.ylabel = "f (m/s^2)"
    fig2.title = "ESO TOTAL-DISTURBANCE ESTIMATE"
    fig2.save(os.path.join(outdir, "eso_disturbance.png"))

    # 统计
    mean_est_err = 0.0
    cnt = 0
    for k, t in enumerate(ts2):
        if t > 8.0:
            mean_est_err += abs(z3[k] - f_true[k])
            cnt += 1
    mean_est_err /= max(cnt, 1)

    return {
        "pd_end_z": zp[-1],
        "pd_err": 1.0 - zp[-1],
        "eso_end_z": ze[-1],
        "eso_err": 1.0 - ze[-1],
        "z3_final": z3[-1],
        "f_final": f_true[-1],
        "mean_est_err": mean_est_err,
    }
