"""生成 QoS 教学配图（纯标准库）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simple_plot as sp
import qos_model as qm

PUB_RATE = 100.0     # 发布 100 Hz（10 ms 一帧）
PROC_TIME = 0.030    # 回调耗时 30 ms -> 处理能力约 33 Hz


def run(outdir):
    os.makedirs(outdir, exist_ok=True)

    be1 = qm.simulate(qm.QoSProfile(qm.BEST_EFFORT, depth=1), PUB_RATE, PROC_TIME)
    be10 = qm.simulate(qm.QoSProfile(qm.BEST_EFFORT, depth=10), PUB_RATE, PROC_TIME)
    rel1 = qm.simulate(qm.QoSProfile(qm.RELIABLE, depth=1), PUB_RATE, PROC_TIME)

    # 图 1：三种配置下的积压样本数
    fig = sp.Figure(width=760, height=440)
    fig.plot(be1.ts, be1.backlog, color=sp.Figure.RED,
             label="BE D=1")
    fig.plot(be10.ts, be10.backlog, color=sp.Figure.ORANGE,
             label="BE D=10")
    fig.plot(rel1.ts, rel1.backlog, color=sp.Figure.BLUE,
             label="RELIABLE D=1")
    fig.xlabel = "t (s)"
    fig.ylabel = "backlog (n)"
    fig.set_ylim(0, 70)
    fig.title = "QOS BACKLOG: PUB 100HZ VS SUB 33HZ"
    fig.save(os.path.join(outdir, "qos_backlog.png"))

    # 图 2：丢包率随 keep_last 深度的变化
    sweep = qm.sweep_depth(PUB_RATE, PROC_TIME)
    depths = [d for d, _ in sweep]
    rates = [r * 100.0 for _, r in sweep]
    fig2 = sp.Figure(width=760, height=440)
    fig2.plot(depths, rates, color=sp.Figure.RED, label="DROP (%)")
    fig2.xlabel = "history depth"
    fig2.ylabel = "drop (%)"
    fig2.set_ylim(0, 70)
    fig2.title = "BEST_EFFORT DROP RATE VS DEPTH"
    fig2.save(os.path.join(outdir, "qos_drop_depth.png"))

    return {
        "be1_drop": be1.drop_rate,
        "be1_lat": be1.mean_latency,
        "be10_drop": be10.drop_rate,
        "be10_lat": be10.mean_latency,
        "rel1_drop": rel1.drop_rate,
        "rel1_lat": rel1.mean_latency,
        "rel1_backlog": rel1.backlog[-1],
        "sweep": sweep,
        "latch": qm.latch_demo(),
    }
