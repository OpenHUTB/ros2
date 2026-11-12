#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形机器人神经网络行走控制：CPG 基线 + 四条学习路线的可行性矩阵。

    python3 main.py --version
    python3 main.py --test        离线自检
    python3 main.py --headless    跑完整矩阵 + CSV + 图
"""

import argparse
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np                                              # noqa: E402

import humanoid_ars_phase_walk.config as config                 # noqa: E402
import humanoid_ars_phase_walk.env as env                      # noqa: E402


def cmd_version():
    print(f"{config.PACKAGE_NAME} {config.VERSION}")
    print(f"  CPG: A_hip={config.A_HIP} A_knee={config.A_KNEE} f={config.FREQ} Hz")
    print(f"  策略：线性 ARS / 行为克隆 / DAgger（MLP-{config.BC_HIDDEN}）")
    return 0


EN = {"CPG 基线": "CPG baseline", "ARS 从零（线性）": "ARS from scratch",
      "行为克隆（MLP）": "Behavior cloning (MLP)"}


def _en(name):
    return EN.get(name, name.replace("DAgger 轮", "DAgger round"))


def _row(name, r):
    return dict(method=name, dx=r["dx"], steps=r["steps"], pitch_rms=r["pitch_rms"],
                speed=r["speed"], energy=r["energy"], reward=r["reward"])


def cmd_headless(outdir):
    from humanoid_ars_phase_walk import learners

    outdir.mkdir(parents=True, exist_ok=True)
    rows = []

    base = env.rollout(env.make_env(0))
    rows.append(_row("CPG 基线", base))

    lin = learners.LinearPolicy().train()
    rows.append(_row("ARS 从零（线性）", env.rollout(env.make_env(0), lin.commit())))

    X, Y = learners.collect_cpg()
    net = learners.MLPPolicy().fit(X, Y)
    rows.append(_row("行为克隆（MLP）", env.rollout(env.make_env(0), net)))

    net2, hist = learners.dagger()
    for i, r in enumerate(hist):
        rows.append(_row(f"DAgger 轮 {i}", r))

    with (outdir / "matrix.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    print(f"{'方法':22s} {'前进(m)':>8s} {'步数':>5s} {'俯仰RMS':>8s} {'速度(m/s)':>9s}")
    for r in rows:
        print(f"{r['method']:22s} {r['dx']:8.2f} {r['steps']:5d} {r['pitch_rms']:8.3f} {r['speed']:9.2f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(9.0, 3.2))
    names = [_en(r["method"]) for r in rows]
    axs[0].barh(names, [r["dx"] for r in rows], color="#1f77b4")
    axs[0].axvline(base["dx"], color="#d95f02", ls="--", lw=1.0, label="CPG baseline")
    axs[0].set_xlabel("forward distance (m)")
    axs[0].set_title("Forward distance per 4 s episode")
    axs[1].barh(names, [r["steps"] for r in rows], color="#2ca02c")
    axs[1].axvline(200, color="#d95f02", ls="--", lw=1.0, label="full episode")
    axs[1].set_xlabel("survived steps (of 200)")
    axs[1].set_title("How long the gait survives")
    for ax in axs:
        ax.grid(alpha=0.25, ls="--", axis="x")
        ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "matrix.png")
    plt.close(fig)
    print(f"\nCSV 与图已写入 {outdir}")
    return 0


def cmd_test():
    from humanoid_ars_phase_walk import learners

    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    check("CPG 周期与幅度合理", config.FREQ > 0 and abs(config.A_KNEE) > 0.1)
    r = env.rollout(env.make_env(0))
    check("CPG 基线能走 > 0.8 m", r["dx"] > 0.8, f"{r['dx']:+.2f} m")
    check("CPG 基线走满全程", r["steps"] >= config.SUCCESS_STEPS, f"{r['steps']} 步")
    check("CPG 俯仰 RMS < 0.4", r["pitch_rms"] < 0.4, f"{r['pitch_rms']:.3f}")

    X, Y = learners.collect_cpg(1)
    check("采样到训练数据", X.shape[1] == env.OBS_DIM and Y.shape[1] == env.ACT_DIM,
          f"{X.shape} -> {Y.shape}")
    net = learners.MLPPolicy(n_hidden=16).fit(X[:400], Y[:400], epochs=20)
    pred = net.predict(X[:10])
    check("MLP 输出形状正确", pred.shape == (10, env.ACT_DIM), f"{pred.shape}")

    print()
    print(f"{len(fails)} 项失败" if fails else "全部通过")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="人形机器人神经网络行走控制（可行性分析）")
    ap.add_argument("--version", action="store_true")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--outdir", type=str, default="results")
    a = ap.parse_args(argv)
    if a.version:
        return cmd_version()
    if a.test:
        return cmd_test()
    if a.headless:
        return cmd_headless(Path(a.outdir) if Path(a.outdir).is_absolute() else HERE / a.outdir)
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())