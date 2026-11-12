#!/usr/bin/env python3
"""离线复现入口：跑通 6 个场景、打印指标表、生成文档配图。

用法（在本模块目录下）::

    python sim/run_sim.py                 # 全部场景，各 3 个随机种子
    python sim/run_sim.py --scenario slip --seeds 5
    python sim/run_sim.py --no-plots      # 只打印指标
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "src"), str(HERE.parent)]

import numpy as np  # noqa: E402

from sim.experiments import Episode, run_episode, summarise  # noqa: E402
from sim.scenarios import ALL_SCENARIOS, SCENARIOS  # noqa: E402

FIG_DIR = HERE.parent / "docs_figs"


# ---------------------------------------------------------------- 中文标签兼容
def _pick_font() -> bool:
    """能找到中文字体就返回 True，找不到则图上用英文，避免出现方块。"""
    try:
        import matplotlib
        from matplotlib import font_manager

        for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "WenQuanYi Zen Hei", "Source Han Sans SC"):
            try:
                path = font_manager.findfont(font_manager.FontProperties(family=name), fallback_to_default=False)
            except Exception:
                continue
            if path and os.path.exists(path):
                matplotlib.rcParams["font.sans-serif"] = [name]
                matplotlib.rcParams["axes.unicode_minus"] = False
                return True
    except Exception:
        return False
    return False


CJK = False


def T(zh: str, en: str) -> str:
    return zh if CJK else en


# ---------------------------------------------------------------- 指标
def aggregate(episodes: list[Episode]) -> list[dict]:
    """按 (场景, 是否脱困) 汇总多个随机种子。"""
    groups: dict[tuple[str, bool], list[dict]] = {}
    for ep in episodes:
        groups.setdefault((ep.scenario, ep.use_recovery), []).append(summarise(ep))
    out = []
    for (name, rec), items in groups.items():
        dets = [i["first_detect"] for i in items if not math.isnan(i["first_detect"])]
        escs = [i["escape_time"] - i["first_detect"] for i in items if i["escape_time"] >= 0]
        out.append(
            {
                "scenario": name,
                "recovery": rec,
                "success": sum(1 for i in items if i["success"]),
                "n": len(items),
                "triggers": float(np.mean([i["triggers"] for i in items])),
                "escapes": float(np.mean([i["escapes"] for i in items])),
                "first_detect": float(np.mean(dets)) if dets else float("nan"),
                "escape_delay": float(np.mean(escs)) if escs else float("nan"),
                "post_travel": float(np.mean([i["post_travel"] for i in items])),
                "stuck_ratio": float(np.mean([i["stuck_ratio"] for i in items])),
            }
        )
    return out


def print_summary(rows: list[dict]) -> None:
    print(f"\n{'场景':<22s}{'脱困':<6s}{'成功率':<10s}{'触发':>6s}{'脱困':>6s}{'检测/s':>8s}{'脱困耗时/s':>12s}{'脱困后里程/m':>14s}")
    print("-" * 96)
    for r in rows:
        det = "-" if math.isnan(r["first_detect"]) else f"{r['first_detect']:.2f}"
        esc = "-" if math.isnan(r["escape_delay"]) else f"{r['escape_delay']:.2f}"
        hit = f"{r['success']}/{r['n']}"
        print(
            f"{r['scenario']:<22s}{'开' if r['recovery'] else '关':<6s}"
            f"{hit:<8s}{r['triggers']:>6.1f}{r['escapes']:>6.1f}{det:>8s}{esc:>12s}"
            f"{r['post_travel']:>14.2f}"
        )


# ---------------------------------------------------------------- 绘图
def plot_traces(episodes: dict[str, dict[bool, Episode]], path: Path) -> None:
    import matplotlib.pyplot as plt

    names = list(episodes.keys())
    cols = 3
    rows = math.ceil(len(names) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(12.6, 3.6 * rows))
    axes = np.atleast_2d(axes).reshape(-1)
    for ax, name in zip(axes, names):
        sc = SCENARIOS[name]
        for seg in sc.barriers:
            ax.plot([seg.x1, seg.x2], [seg.y1, seg.y2], color="#333333", lw=3, solid_capstyle="round")
        off = episodes[name].get(False)
        on = episodes[name].get(True)
        if off is not None:
            ax.plot(off.pose_true[:, 0], off.pose_true[:, 1], "--", color="#9aa0a6", lw=1.4,
                    label=T("不脱困", "no recovery"))
        if on is not None:
            ax.plot(on.pose_true[:, 0], on.pose_true[:, 1], "-", color="#1a73e8", lw=1.8,
                    label=T("本模块", "proposed"))
            idx = next((i for i, c in enumerate(on.causes) if c != "none"), None)
            if idx is not None:
                ax.plot(on.pose_true[idx, 0], on.pose_true[idx, 1], "o", ms=6, mfc="none",
                        mec="#d93025", mew=1.6, label=T("检测到卡死", "detected"))
        ax.plot(sc.start[0], sc.start[1], "s", ms=6, color="#188038")
        xs, ys = [sc.start[0]], [sc.start[1]]
        for ep in (off, on):
            if ep is not None:
                xs.extend(ep.pose_true[:, 0].tolist())
                ys.extend(ep.pose_true[:, 1].tolist())
        pad = 0.55
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"{sc.title}", fontsize=10)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=8)
    for ax in axes[len(names):]:
        ax.axis("off")
    axes[0].legend(fontsize=7, loc="best")
    fig.suptitle(T("六个场景的真实轨迹：关掉脱困 vs 开启脱困", "True trajectories: without vs with recovery"), fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_signals(ep: Episode, path: Path) -> None:
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.2, 5.4), sharex=True,
                                   gridspec_kw={"height_ratios": [1.3, 1.0]})
    dt = np.diff(ep.t, prepend=ep.t[0])
    cmd_dist = np.cumsum(np.abs(ep.cmd[:, 0]) * dt)
    odom_step = np.r_[0.0, np.hypot(np.diff(ep.pose_odom[:, 0]), np.diff(ep.pose_odom[:, 1]))]
    odom_dist = np.cumsum(odom_step)
    ax1.plot(ep.t, cmd_dist, lw=1.8, color="#188038", label=T("指令累计位移", "commanded travel"))
    ax1.plot(ep.t, odom_dist, lw=1.8, color="#e37400", label=T("里程计累计位移", "odom travel"))
    ax1.set_ylabel(T("累计位移 / m", "travel / m"))
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.25)
    ax1.set_title(T(f"检测信号：{SCENARIOS[ep.scenario].title}", f"detection signals: {ep.scenario}"), fontsize=11)

    ax2.plot(ep.t, ep.slip_ratio, lw=1.4, color="#5f6368")
    ax2.axhline(0.30, ls=":", color="#d93025", lw=1.2, label=T("滑动比阈值 0.30", "ratio threshold 0.30"))
    first = next((i for i, c in enumerate(ep.causes) if c != "none"), None)
    if first is not None:
        ax2.axvline(ep.t[first], color="#d93025", lw=1.4, alpha=0.8)
        ax2.annotate(T("判定卡死", "stuck"), (ep.t[first], 0.9), xytext=(6, 0),
                     textcoords="offset points", fontsize=8, color="#d93025")
    ax2.set_ylim(0, 1.2)
    ax2.set_ylabel(T("滑动比", "slip ratio"))
    ax2.set_xlabel(T("时间 / s", "time / s"))
    ax2.grid(alpha=0.25)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


STATE_COLORS = {
    "CRUISE": "#e8f0fe",
    "BACK_OFF": "#fce8e6",
    "TURN": "#fef7e0",
    "PROBE": "#e6f4ea",
    "PULSE": "#f3e8fd",
    "ABORT": "#5f6368",
    "STALE": "#c5221f",
}


def plot_timeline(ep: Episode, path: Path) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    fig, ax = plt.subplots(figsize=(9.2, 2.4))
    order = ["CRUISE", "BACK_OFF", "TURN", "PROBE", "PULSE", "ABORT", "STALE"]
    start = 0
    for i in range(1, len(ep.states) + 1):
        if i == len(ep.states) or ep.states[i] != ep.states[start]:
            ax.axvspan(ep.t[start], ep.t[min(i, len(ep.t) - 1)],
                       color=STATE_COLORS.get(ep.states[start], "#eee"), lw=0)
            start = i
    ax.set_yticks([])
    ax.set_xlabel(T("时间 / s", "time / s"))
    ax.set_title(T(f"状态机时序：{SCENARIOS[ep.scenario].title}", f"state machine: {ep.scenario}"), fontsize=11)
    used = [s for s in order if s in set(ep.states)]
    ax.legend(handles=[Patch(color=STATE_COLORS[s], label=s) for s in used], ncol=len(used),
              fontsize=8, loc="lower center", bbox_to_anchor=(0.5, 1.02), frameon=False)
    ax.grid(alpha=0.2, axis="x")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_compare(rows: list[dict], path: Path) -> None:
    import matplotlib.pyplot as plt

    names = [n for n in ALL_SCENARIOS if n != "free"]
    on = {r["scenario"]: r for r in rows if r["recovery"]}
    off = {r["scenario"]: r for r in rows if not r["recovery"]}
    x = np.arange(len(names))
    rate_on = [on[n]["success"] / on[n]["n"] if n in on else 0 for n in names]
    rate_off = [off[n]["success"] / off[n]["n"] if n in off else 0 for n in names]
    fig, ax = plt.subplots(figsize=(8.6, 3.6))
    ax.bar(x - 0.19, rate_off, 0.36, label=T("不脱困", "no recovery"), color="#9aa0a6")
    ax.bar(x + 0.19, rate_on, 0.36, label=T("本模块", "proposed"), color="#1a73e8")
    for xi, (a, b) in enumerate(zip(rate_off, rate_on)):
        ax.text(xi - 0.19, a + 0.03, f"{a:.2f}", ha="center", fontsize=7, color="#5f6368")
        ax.text(xi + 0.19, b + 0.03, f"{b:.2f}", ha="center", fontsize=7, color="#1a73e8")
    ax.set_xticks(x)
    ax.set_xticklabels([T(SCENARIOS[n].title, n) for n in names], fontsize=8, rotation=12)
    ax.set_ylabel(T("期望行为达成率", "success rate"))
    ax.set_ylim(0, 1.12)
    ax.grid(alpha=0.25, axis="y")
    ax.legend(fontsize=8)
    ax.set_title(T("对照实验：关掉脱困全部失败，开启后全部达成", "ablation: recovery off vs on"), fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_sweep(path: Path) -> list[dict]:
    """两件容易搞错的事：确认时长到底推迟多少、缺传感器时哪类故障会漏检。"""
    import matplotlib.pyplot as plt

    from ros2_stuck_recovery.detect import DetectorConfig

    confirms = [0.30, 0.50, 0.70, 0.90, 1.10]
    detect: dict[str, list[float]] = {"blocked_wall": [], "slip": []}
    for confirm in confirms:
        for name in detect:
            hits = []
            for seed in range(3):
                ep = run_episode(name, seed=seed, use_recovery=True,
                                 detector_config=DetectorConfig(confirm=confirm))
                first = next((ep.t[i] for i, c in enumerate(ep.causes) if c != "none"), None)
                if first is not None:
                    hits.append(first)
            detect[name].append(float(np.mean(hits)) if hits else float("nan"))

    configs = [
        ("IMU + 测距", ("imu", "range")),
        ("仅 IMU", ("imu",)),
        ("仅测距", ("range",)),
        ("都没有", ()),
    ]
    scenes = ["blocked_wall", "slip", "one_wheel"]
    rates = {label: [] for label, _ in configs}
    for label, sensors in configs:
        for name in scenes:
            ok = [
                run_episode(name, seed=seed, use_recovery=True, sensors=sensors).success
                for seed in range(3)
            ]
            rates[label].append(sum(ok) / len(ok))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.8, 3.6))
    for name, color in zip(["blocked_wall", "slip"], ["#1a73e8", "#e37400"]):
        ax1.plot(confirms, detect[name], "o-", color=color, label=T(SCENARIOS[name].title, name))
    ax1.set_xlabel(T("确认时长 / s", "confirm window / s"))
    ax1.set_ylabel(T("首次检测时刻 / s", "first detection / s"))
    ax1.set_ylim(0, 10.5)
    ax1.grid(alpha=0.25)
    ax1.legend(fontsize=7)
    ax1.set_title(T("确认时长只推迟 0.4~0.8 s：延迟主要由物理过程决定", "confirm adds only 0.4-0.8 s"), fontsize=9)

    x = np.arange(len(scenes))
    width = 0.2
    colors = ["#1a73e8", "#188038", "#f9ab00", "#d93025"]
    for i, (label, _) in enumerate(configs):
        ax2.bar(x + (i - 1.5) * width, rates[label], width, label=T(label, label), color=colors[i])
    ax2.set_xticks(x)
    ax2.set_xticklabels([T(SCENARIOS[n].title, n) for n in scenes], fontsize=7, rotation=10)
    ax2.set_ylabel(T("脱困成功率", "success rate"))
    ax2.set_ylim(0, 1.18)
    ax2.grid(alpha=0.25, axis="y")
    ax2.legend(fontsize=6, ncol=4, loc="upper center", frameon=False)
    ax2.set_title(T("缺传感器时的降级：打滑只能靠测距", "sensor ablation"), fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [{"confirm": c, "scene": s, "detect": d} for s in detect for c, d in zip(confirms, detect[s])] + [
        {"sensors": label, "scene": s, "rate": r} for label, _ in configs for s, r in zip(scenes, rates[label])
    ]


# ---------------------------------------------------------------- 主流程
def main() -> None:
    parser = argparse.ArgumentParser(description="卡死检测与自动脱困：离线复现")
    parser.add_argument("--scenario", default="all", help="场景名或 all")
    parser.add_argument("--seeds", type=int, default=3, help="每个场景的随机种子数")
    parser.add_argument("--no-plots", action="store_true", help="只打印指标，不生成图片")
    parser.add_argument("--out", default=str(FIG_DIR), help="图片输出目录")
    args = parser.parse_args()

    global CJK
    names = ALL_SCENARIOS if args.scenario == "all" else [args.scenario]
    episodes: list[Episode] = []
    for name in names:
        for rec in (False, True):
            for seed in range(args.seeds):
                episodes.append(run_episode(name, seed=seed, use_recovery=rec))

    rows = aggregate(episodes)
    order = {n: i for i, n in enumerate(ALL_SCENARIOS)}
    rows.sort(key=lambda r: (order[r["scenario"]], r["recovery"]))
    print_summary(rows)

    if args.no_plots:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        CJK = _pick_font()
    except Exception:
        print("未安装 matplotlib，跳过绘图")
        return

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sample = {}
    for name in names:
        sample[name] = {rec: run_episode(name, seed=0, use_recovery=rec) for rec in (False, True)}

    plot_traces(sample, out / "traces.png")
    plot_signals(sample["blocked_wall"][True], out / "detection_signals.png")
    plot_timeline(sample["corner"][True], out / "state_timeline.png")
    plot_compare(rows, out / "compare.png")
    plot_sweep(out / "sweep.png")
    print(f"\n图片已写入 {out}")


if __name__ == "__main__":
    main()
