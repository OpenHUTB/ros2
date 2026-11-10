#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""移动平台降落的定量对照实验（离线，无 ROS / 无仿真器）。

跑三组实验，输出 CSV、控制台表格和论文式的对比图：

1. **场景对照**：直线 0.4 / 0.8 / 1.2 m/s、圆周 0.6 / 1.0 m/s（R=4 m）、升沉海况
   （直线 0.5 m/s + 0.15 m / 4 s 升沉），三种制导策略各 5 个噪声种子；
2. **时延扫描**：平台 0.8 m/s 直线，量测延迟 0.05 / 0.15 / 0.30 / 0.45 s；
3. **权限受限**：把无人机横向速度上限压到 0.6 m/s，让平台跑到 0.4 / 0.8 / 1.2 m/s
   —— 检验「平台比无人机快」时还能不能落。

用法::

    python3 tools/evaluate.py            # 跑全部实验 + 出图
    python3 tools/evaluate.py --quick    # 只跑主对照（种子更少）

图默认写到 ``docs/img/air/uav_moving_platform_landing/``（可用 ``--out`` 改）。
没有任何随机性以外的依赖：只用 numpy + matplotlib。
"""
from __future__ import print_function

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(PKG)))
sys.path.insert(0, HERE)

import closed_loop_sim as sim  # noqa: E402

STRATEGIES = ["static", "track", "predict", "auto"]
STRATEGY_CN = {
    "static": "追实测位置（朴素）",
    "track": "追当前位置+速度前馈",
    "predict": "相遇点+速度前馈",
    "auto": "自动（模块默认）",
}
COLORS = {"static": "#c44e52", "track": "#4c72b0", "predict": "#8172b2", "auto": "#55a868"}
AUTH_SPEEDS = (0.4, 0.8, 1.0, 1.2)


# ---------------------------------------------------------------- 实验定义
def scenario_motions():
    """主对照用的平台运动。"""
    return [
        ("直线 0.4 m/s", sim.LineMotion(v=(0.4, 0.0, 0.0))),
        ("直线 0.8 m/s", sim.LineMotion(v=(0.8, 0.0, 0.0))),
        ("直线 1.2 m/s", sim.LineMotion(v=(1.2, 0.0, 0.0))),
        ("圆周 0.6 m/s (R=4)", sim.CircleMotion(radius=4.0, speed=0.6)),
        ("圆周 1.0 m/s (R=4)", sim.CircleMotion(radius=4.0, speed=1.0)),
        ("升沉 0.5 m/s ±0.15", sim.HeaveMotion(sim.LineMotion(v=(0.5, 0.0, 0.0)))),
    ]


def run_block(motions, strategies=STRATEGIES, seeds=(0, 1, 2, 3, 4),
              latency=0.2, noise=0.04, params=None):
    rows = []
    for label, motion in motions:
        for strat in strategies:
            for seed in seeds:
                r = sim.run_trial(motion, strat, latency=latency, noise_m=noise,
                                  seed=seed, params=params)
                rows.append({
                    "scenario": label, "strategy": strat, "seed": seed,
                    "landed": int(r["landed"]), "aborted": int(r["aborted"]),
                    "ok": int(r["ok"]), "error_xy": r["error_xy"],
                    "go_around": r["go_around"], "duration_s": r["duration_s"],
                    "result": r,
                })
    return rows


def group_stats(rows, key="scenario"):
    """按 (场景, 策略) 汇总：均值 / p95 / 成功率。"""
    out = {}
    for k in sorted({r[key] for r in rows}):
        for s in STRATEGIES:
            sel = [r for r in rows if r[key] == k and r["strategy"] == s]
            if not sel:
                continue
            errs = [r["error_xy"] for r in sel if r["landed"]]
            out[(k, s)] = {
                "n": len(sel),
                "rate": sum(r["ok"] for r in sel) / float(len(sel)),
                "mean": float(np.mean(errs)) if errs else float("nan"),
                "p95": float(np.percentile(errs, 95)) if errs else float("nan"),
                "abort": sum(r["aborted"] for r in sel),
                "max": float(np.max(errs)) if errs else float("nan"),
            }
    return out


# ---------------------------------------------------------------- 绘图
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for f in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei",
              "DejaVu Sans"):
        try:
            matplotlib.font_manager.findfont(f, fallback_to_default=False)
            plt.rcParams["font.sans-serif"] = [f]
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def fig_scenarios(stats, labels, path):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(10.5, 4.6))
    x = np.arange(len(labels))
    w = 0.26
    for i, s in enumerate(STRATEGIES):
        vals = []
        for lb in labels:
            st = stats.get((lb, s))
            vals.append(np.nan if st is None else st["mean"])
        bars = ax.bar(x + (i - 1) * w, vals, w, label=STRATEGY_CN[s], color=COLORS[s])
        for b, v in zip(bars, vals):
            if np.isfinite(v):
                ax.text(b.get_x() + b.get_width() / 2, v + 0.01, "%.2f" % v,
                        ha="center", va="bottom", fontsize=8)
            else:
                ax.text(b.get_x() + b.get_width() / 2, 0.02, "落不准", ha="center",
                        va="bottom", fontsize=8, rotation=90, color=COLORS[s])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("接地时横向误差 / m")
    ax.set_title("移动平台精准降落：四种制导策略的落点误差"
                 "（5 个噪声种子均值，量测延迟 0.2 s，误差≤0.30 m 记为成功）")
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_latency(rows, path):
    plt = _plt()
    lat_set = sorted({r["latency"] for r in rows})
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for s in STRATEGIES:
        means = []
        for lat in lat_set:
            errs = [r["error_xy"] for r in rows
                    if r["strategy"] == s and r["latency"] == lat and r["landed"]]
            means.append(float(np.mean(errs)) if errs else np.nan)
        ax.plot(lat_set, means, "o-", label=STRATEGY_CN[s], color=COLORS[s])
    ax.axhline(0.30, color="gray", ls="--", lw=0.8)
    ax.text(lat_set[0], 0.31, "0.30 m（判成功阈值）", fontsize=8, color="gray")
    ax.set_xlabel("量测到执行的总时延 / s")
    ax.set_ylabel("接地横向误差 / m")
    ax.set_title("时延对落点精度的影响（平台 0.8 m/s 直线）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_authority(rows, path):
    plt = _plt()
    speeds = sorted({r["deck_speed"] for r in rows})
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for s in STRATEGIES:
        ok = []
        for sp in speeds:
            sel = [r for r in rows if r["strategy"] == s and r["deck_speed"] == sp]
            ok.append(100.0 * sum(r["ok"] for r in sel) / float(len(sel)) if sel else np.nan)
        ax.plot(speeds, ok, "o-", label=STRATEGY_CN[s], color=COLORS[s])
    ax.axvline(0.6, color="gray", ls=":", lw=1.0)
    ax.text(0.62, 40, "无人机横向速度上限 0.6 m/s", fontsize=8, color="gray", rotation=90)
    ax.set_xlabel("平台速度 / (m/s)")
    ax.set_ylabel("成功率 / %")
    ax.set_ylim(-5, 105)
    ax.set_title("平台比无人机快时：只有“相遇点”策略还能落")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_trajectory(results, path, title):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.8, 5.4))
    deck = np.array([results[0]["result"]["trace"]["deck"]])
    ax.plot(deck[0, :, 0], deck[0, :, 1], color="#999999", lw=3, alpha=0.45,
            label="平台轨迹")
    for r in results:
        tr = r["result"]["trace"]
        p = np.array(tr["drone"])
        m = r["strategy"]
        if r["result"]["landed"]:
            ax.plot(p[:, 0], p[:, 1], color=COLORS[m], lw=1.6, label=STRATEGY_CN[m])
            ax.plot(p[-1, 0], p[-1, 1], "o", color=COLORS[m], ms=7)
            ax.annotate("落差 %.2f m" % r["result"]["error_xy"],
                        (p[-1, 0], p[-1, 1]), textcoords="offset points",
                        xytext=(6, 6), fontsize=8, color=COLORS[m])
        else:
            ax.plot(p[:, 0], p[:, 1], color=COLORS[m], lw=1.2, ls="--",
                    label=STRATEGY_CN[m] + "（未落准）")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_title(title)
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_altitude(results, path, title):
    """横向误差随时间的曲线（比高度曲线更能说明问题）。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    for r in results:
        tr = r["result"]["trace"]
        t = np.asarray(tr["t"])
        drone = np.array(tr["drone"])
        deck = np.array(tr["deck"])
        err = np.hypot(drone[:, 0] - deck[:, 0], drone[:, 1] - deck[:, 1])
        ax.plot(t, err, color=COLORS[r["strategy"]], lw=1.5,
                label=(STRATEGY_CN[r["strategy"]]
                       + ("（落差 %.2f m）" % r["result"]["error_xy"]
                          if r["result"]["landed"] else "（未落准）")))
    ax.axhline(0.30, color="gray", ls="--", lw=0.8)
    ax.text(0.2, 0.32, "0.30 m 判成功阈值", fontsize=8, color="gray")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("无人机与平台中心水平距离 / m")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- 主流程
def main(argv=None):
    ap = argparse.ArgumentParser(description="移动平台降落定量对照实验")
    ap.add_argument("--quick", action="store_true", help="只跑主对照、种子数 3")
    ap.add_argument("--out", default=os.path.join(
        REPO, "docs", "img", "air", "uav_moving_platform_landing"))
    ap.add_argument("--csv", default=os.path.join(HERE, "results"))
    ap.add_argument("--no-figs", action="store_true")
    args = ap.parse_args(argv)

    seeds = (0, 1, 2) if args.quick else (0, 1, 2, 3, 4)
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.csv, exist_ok=True)

    # ---- 实验 1：场景对照
    print("[1/3] 场景对照（%d 个种子）..." % len(seeds))
    motions = scenario_motions()
    labels = [m[0] for m in motions]
    rows = run_block(motions, seeds=seeds, latency=0.2)
    stats = group_stats(rows)
    print("\n场景对照（接地横向误差 / m，成功率 = 落差≤0.30 m 的比例）")
    head = "%-22s" % "场景"
    for s in STRATEGIES:
        head += " | %-24s" % STRATEGY_CN[s]
    print(head)
    for lb in labels:
        line = "%-22s" % lb
        for s in STRATEGIES:
            st = stats.get((lb, s))
            if st is None:
                line += " | %-24s" % "-"
            else:
                line += " | %5s %5s 成功%3.0f%%" % (
                    ("%.2f" % st["mean"]) if np.isfinite(st["mean"]) else "  -  ",
                    ("%.2f" % st["p95"]) if np.isfinite(st["p95"]) else "  -  ",
                    100 * st["rate"])
        print(line)

    with open(os.path.join(args.csv, "scenarios.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "strategy", "seed", "landed", "ok", "error_xy",
                    "go_around", "duration_s"])
        for r in rows:
            w.writerow([r["scenario"], r["strategy"], r["seed"], r["landed"], r["ok"],
                        "%.4f" % r["error_xy"] if np.isfinite(r["error_xy"]) else "",
                        r["go_around"], "%.2f" % r["duration_s"]])

    # ---- 实验 2：时延扫描
    print("\n[2/3] 时延扫描（平台 0.8 m/s 直线）...")
    lat_rows = []
    for lat in (0.05, 0.15, 0.30, 0.45):
        for strat in STRATEGIES:
            for seed in seeds:
                r = sim.run_trial(sim.LineMotion(v=(0.8, 0.0, 0.0)), strat,
                                  latency=lat, seed=seed)
                lat_rows.append({"latency": lat, "strategy": strat, "seed": seed,
                                 "error_xy": r["error_xy"], "landed": r["landed"]})
    print("%-6s" % "时延" + "".join(" | %-22s" % STRATEGY_CN[s] for s in STRATEGIES))
    for lat in (0.05, 0.15, 0.30, 0.45):
        line = "%-6.2f" % lat
        for s in STRATEGIES:
            errs = [r["error_xy"] for r in lat_rows
                    if r["strategy"] == s and r["latency"] == lat and r["landed"]]
            line += " | %-22s" % (("%.3f m" % float(np.mean(errs))) if errs else "落不准")
        print(line)

    # ---- 实验 3：权限受限（无人机横向速度上限 0.6 m/s，起始位置在平台前方）
    print("\n[3/3] 权限受限（无人机横向上限 0.6 m/s，起点在平台前方 10 m）...")
    auth_rows = []
    for speed in AUTH_SPEEDS:
        for strat in STRATEGIES:
            for seed in seeds:
                r = sim.run_trial(sim.LineMotion(v=(speed, 0.0, 0.0)), strat,
                                  latency=0.2, seed=seed,
                                  start_offset=(10.0, 0.0, 8.0),
                                  params={"guide/max_xy_speed": 0.6})
                auth_rows.append({"deck_speed": speed, "strategy": strat, "seed": seed,
                                  "ok": r["ok"], "landed": r["landed"],
                                  "error_xy": r["error_xy"]})
    print("%-8s" % "平台速度" + "".join(" | %-16s" % STRATEGY_CN[s] for s in STRATEGIES))
    for speed in AUTH_SPEEDS:
        line = "%-8.1f" % speed
        for s in STRATEGIES:
            sel = [r for r in auth_rows if r["strategy"] == s and r["deck_speed"] == speed]
            rate = 100.0 * sum(r["ok"] for r in sel) / float(len(sel))
            errs = [r["error_xy"] for r in sel if r["landed"]]
            line += " | 成功%3.0f%% 误差%5s" % (
                rate, ("%.2f" % float(np.mean(errs))) if errs else " - ")
        print(line)

    if args.no_figs:
        return 0

    # ---- 出图
    print("\n生成图片到 %s ..." % args.out)
    fig_scenarios(stats, labels, os.path.join(args.out, "scenarios.png"))
    fig_latency(lat_rows, os.path.join(args.out, "latency.png"))
    fig_authority(auth_rows, os.path.join(args.out, "authority.png"))

    one = {}
    for strat in STRATEGIES:
        rs = [r for r in rows if r["strategy"] == strat and r["scenario"] == "直线 1.2 m/s"]
        one[strat] = rs[0]
    fig_trajectory([one[s] for s in STRATEGIES],
                   os.path.join(args.out, "trajectory_line.png"),
                   "直线 1.2 m/s 平台的俯视轨迹（延迟 0.2 s）")

    one2 = {}
    for strat in STRATEGIES:
        rs = [r for r in rows if r["strategy"] == strat and r["scenario"] == "直线 1.2 m/s"]
        one2[strat] = rs[0]
    fig_altitude([one2[s] for s in STRATEGIES],
                 os.path.join(args.out, "tracking_error.png"),
                 "直线 1.2 m/s：无人机与平台中心的水平距离（延迟 0.2 s）")

    wave = {}
    for strat in STRATEGIES:
        rs = [r for r in rows if r["strategy"] == strat and r["scenario"] == "升沉 0.5 m/s ±0.15"]
        wave[strat] = rs[0]
    fig_altitude([wave[s] for s in STRATEGIES],
                 os.path.join(args.out, "tracking_error_heave.png"),
                 "升沉甲板：无人机与平台中心的水平距离（±0.15 m 海况）")
    print("完成。")
    return 0


if __name__ == "__main__":
    import time
    t0 = time.time()
    rc = main()
    print("\n用时 %.1f s" % (time.time() - t0))
    sys.exit(rc)
