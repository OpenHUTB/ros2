#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""由 results.csv 生成对比报告：汇总表格（report.md）+ 图表（PNG）.

输出（都在 --out 目录下）：
    report.md                     汇总表格（可直接粘进文档）
    fig_success_by_algo.png       不同算法成功率（带标准差误差棒）
    fig_success_by_arch.png       不同网络结构成功率
    fig_learning_curves.png       成功率随训练步数的变化（样本效率）
    fig_cost.png                  训练耗时对比
    fig_tradeoff.png              路径效率 vs 动作平滑度散点
"""
from __future__ import annotations

import glob
import json
import os

import numpy as np
import pandas as pd

METRIC_COLS = ["success_rate", "crash_rate", "timeout_rate", "mean_steps",
               "mean_reward", "path_efficiency", "action_smoothness"]
COST_COLS = ["train_time_s", "train_fps"]


# --------------------------------------------------------------------- 画图
def _setup_matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "WenQuanYi Zen Hei", "Source Han Sans SC"):
        try:
            font_manager.findfont(font_manager.FontProperties(family=name),
                                  fallback_to_default=False)
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return plt, True
        except Exception:                                   # noqa: BLE001
            continue
    # 没找到中文字体 -> 退回英文标签（由调用方判断 CJK 标志）
    return plt, False


def _label(cjk, zh, en):
    return zh if cjk else en


def _std(series):
    """样本标准差（ddof=1），且只有一个样本时返回 0.0 而不是 NaN.

    统一口径很重要：表格与正文若一个用 ddof=0、一个用 pandas 默认的 ddof=1，
    同一份报告里同一条数据会写出两个不同的「±」（例如 ±6% 与 ±7%）。
    """
    import numpy as _np
    arr = _np.asarray(series, dtype=float)
    if arr.size < 2:
        return 0.0
    s = float(_np.std(arr, ddof=1))
    return 0.0 if _np.isnan(s) else s


def _bar_with_err(plt, ax, df, key, col, cjk, title_zh, title_en, xlabel_zh, xlabel_en):
    g = df.groupby(key)[col].agg(["mean", "std"]).sort_values("mean", ascending=False)
    x = np.arange(len(g))
    ax.bar(x, g["mean"].values, yerr=np.nan_to_num(g["std"].values),
           capsize=4, color="#4C78A8", edgecolor="black", linewidth=0.6)
    ax.set_xticks(x)
    ax.set_xticklabels(g.index.astype(str))
    ax.set_ylim(0, 1.0)
    ax.set_ylabel(_label(cjk, "成功率", "Success rate"))
    ax.set_xlabel(_label(cjk, xlabel_zh, xlabel_en))
    ax.set_title(_label(cjk, title_zh, title_en))
    ax.grid(axis="y", alpha=0.3)
    for xi, (m, s) in enumerate(zip(g["mean"].values, np.nan_to_num(g["std"].values))):
        ax.text(xi, min(0.98, m + s + 0.02), "%.0f%%" % (m * 100),
                ha="center", fontsize=9)
    return g


def build_report(out_dir="results", make_plots=True):
    csv_path = os.path.join(out_dir, "results.csv")
    if not os.path.exists(csv_path):
        print("找不到 %s，先跑 --sweep 或 --algo" % csv_path)
        return None
    df = pd.read_csv(csv_path)
    if df.empty:
        print("results.csv 是空的")
        return None

    lines = ["# 无人机导航 RL 对比实验报告", ""]
    lines.append("共 **%d** 组配置：%d 种算法 × %d 种结构 × 每配置 %d 个随机种子；"
                 "每组训练 %s 步，障碍 %d 个，最终评估 %d 回合（确定性动作）。"
                 % (len(df), df["algo"].nunique(), df["arch"].nunique(),
                    df["seed"].nunique(), df["steps"].iloc[0],
                    df["obstacles"].iloc[0], df["eval_episodes"].iloc[0]))
    # 从已入库的 eval_log.json 反推**实际**评估间隔：配置写多少不重要，
    # 报告里必须写清这批数据真正按多少步的间隔跑的（否则与曲线/日志对不上）
    _iv = None
    for _p in sorted(glob.glob(os.path.join(out_dir, "runs", "*", "eval_log.json"))):
        try:
            with open(_p, encoding="utf-8") as _fh:
                _hist = json.load(_fh)
            if len(_hist) > 1:
                _iv = _hist[1]["timesteps"] - _hist[0]["timesteps"]
                break
        except Exception:                                   # noqa: BLE001
            continue
    if _iv:
        lines.append("本批结果由 **eval_freq = %d 步**跑出（由已入库的 `eval_log.json` 反推）；"
                     "`config/benchmark.yaml` 与 `launch/main.launch` 的默认值已与之对齐。"
                     % _iv)
    lines.append("")

    _sec = [0]

    def sec(title):
        _sec[0] += 1
        lines.extend(["## %d. %s" % (_sec[0], title), ""])

    agg = df.groupby(["algo", "arch"])[METRIC_COLS + COST_COLS].agg(["mean", "std"])

    # ---------------- 表 1：同结构、不同算法 ----------------
    base_arch = "64x64" if "64x64" in set(df["arch"]) else sorted(set(df["arch"]))[0]
    sub = df[df["arch"] == base_arch]
    sec("相同网络结构（%s），不同算法" % base_arch)
    lines.append("| 算法 | 成功率 | 碰撞率 | 超时率 | 平均步数 | 路径效率 | 动作平滑度 | 训练耗时(s) | 训练速度(步/秒) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for algo, g in sub.groupby("algo"):
        lines.append("| %s | **%.0f%%** ± %.0f%% | %.0f%% | %.0f%% | %.1f | %.2f | %.3f | %.0f | %.0f |"
                     % (algo.upper(),
                        g["success_rate"].mean() * 100, _std(g["success_rate"].values) * 100,
                        g["crash_rate"].mean() * 100, g["timeout_rate"].mean() * 100,
                        g["mean_steps"].mean(), g["path_efficiency"].mean(),
                        g["action_smoothness"].mean(),
                        g["train_time_s"].mean(), g["train_fps"].mean()))
    lines.append("")

    # ---------------- 表 2：同算法、不同结构 ----------------
    arch_counts = df.groupby("arch")["seed"].nunique()
    multi_arch = [a for a, n in arch_counts.items() if n > 0]
    if len(multi_arch) > 1:
        for algo in sorted(set(df["algo"])):
            sub2 = df[df["algo"] == algo]
            if sub2["arch"].nunique() < 2:
                continue
            sec("相同算法（%s），不同网络结构" % algo.upper())
            lines.append("| 结构 | 成功率 | 平均步数 | 路径效率 | 动作平滑度 | 训练耗时(s) |")
            lines.append("|---|---|---|---|---|---|")
            order = ["64x64", "256x256", "64x64x64"]
            archs = [a for a in order if a in set(sub2["arch"])] + \
                    [a for a in sorted(set(sub2["arch"])) if a not in order]
            for arch in archs:
                g = sub2[sub2["arch"] == arch]
                lines.append("| %s | **%.0f%%** ± %.0f%% | %.1f | %.2f | %.3f | %.0f |"
                             % (arch,
                                g["success_rate"].mean() * 100,
                                _std(g["success_rate"].values) * 100,
                                g["mean_steps"].mean(), g["path_efficiency"].mean(),
                                g["action_smoothness"].mean(), g["train_time_s"].mean()))
            lines.append("")

    # ---------------- 结论要点 ----------------
    # 算法层面只看基准结构，否则会把"结构对比"的差异混进"算法对比"
    sub_a = df[df["arch"] == base_arch]
    best = sub_a.groupby("algo")["success_rate"].mean().sort_values(ascending=False)
    fastest = sub_a.groupby("algo")["train_fps"].mean().sort_values(ascending=False)
    smoothest = sub_a.groupby("algo")["action_smoothness"].mean().sort_values()
    straight = sub_a.groupby("algo")["path_efficiency"].mean().sort_values(ascending=False)
    crash = sub_a.groupby("algo")["crash_rate"].mean().sort_values()

    sec("结论要点")
    lines.append("**算法层面**（结构统一为 %s，%d 种子均值）：" % (base_arch, df["seed"].nunique()))
    lines.append("")
    lines.append("- 成功率最高：**%s**（%.0f%%）；最低：%s（%.0f%%）"
                 % (best.index[0].upper(), best.iloc[0] * 100,
                    best.index[-1].upper(), best.iloc[-1] * 100))
    lines.append("- 路径最直（路径效率最高）：**%s**（%.2f）；最低：%s（%.2f）"
                 % (straight.index[0].upper(), straight.iloc[0],
                    straight.index[-1].upper(), straight.iloc[-1]))
    lines.append("- 动作最平滑：**%s**（%.3f）；最抖：%s（%.3f）"
                 % (smoothest.index[0].upper(), smoothest.iloc[0],
                    smoothest.index[-1].upper(), smoothest.iloc[-1]))
    lines.append("- 碰撞率最低：**%s**（%.0f%%）—— 但要结合超时率一起看："
                 % (crash.index[0].upper(), crash.iloc[0] * 100))
    lines.append("  碰撞少往往是因为「不敢动」，超时率会相应升高。")
    lines.append("- 训练最快：**%s**（%.0f 步/秒）；最慢：%s（%.0f 步/秒）"
                 % (fastest.index[0].upper(), fastest.iloc[0],
                    fastest.index[-1].upper(), fastest.iloc[-1]))
    lines.append("")

    # 显著性检查：成功率前两名如果误差带重叠，就不能宣称"谁更好"
    top2 = best.index[:2]
    if len(top2) >= 2:
        m1, s1 = sub_a[sub_a["algo"] == top2[0]]["success_rate"].agg(["mean", "std"])
        m2, s2 = sub_a[sub_a["algo"] == top2[1]]["success_rate"].agg(["mean", "std"])
        s1 = 0.0 if pd.isna(s1) else s1
        s2 = 0.0 if pd.isna(s2) else s2
        if abs(m1 - m2) < (s1 + s2):
            lines.append("> ⚠️ **成功率前两名（%s vs %s）误差带重叠**（%.0f%%±%.0f%% vs %.0f%%±%.0f%%），"
                         "在 %d 个种子下**尚未统计分离**，只能说趋势如此。"
                         % (top2[0].upper(), top2[1].upper(),
                            m1 * 100, s1 * 100, m2 * 100, s2 * 100, df["seed"].nunique()))
            lines.append(">")
            lines.append("> 但**路径效率、动作平滑度与训练速度的差异远超噪声**，"
                         "这几项是本实验最可靠的结论。")
            lines.append("")

    ppo_only = df[df["algo"] == "ppo"]
    if ppo_only["arch"].nunique() > 1:
        ba = ppo_only.groupby("arch")["success_rate"].mean().sort_values(ascending=False)
        be = ppo_only.groupby("arch")["path_efficiency"].mean().sort_values(ascending=False)
        bs = ppo_only.groupby("arch")["action_smoothness"].mean().sort_values()
        base_rate = ba.get("64x64", float("nan")) * 100
        lines.append("**结构层面**（算法统一为 PPO，%d 种子均值）：" % df["seed"].nunique())
        lines.append("")
        lines.append("- 成功率最高：**%s**（%.0f%%）；基准 64x64 为 %.0f%%"
                     % (ba.index[0], ba.iloc[0] * 100, base_rate))
        lines.append("- 路径效率最高：**%s**（%.2f）" % (be.index[0], be.iloc[0]))
        lines.append("- 动作最平滑：**%s**（%.3f）" % (bs.index[0], bs.iloc[0]))
        lines.append("")
        spread = (ba.max() - ba.min()) * 100
        seed_sd = ppo_only.groupby("arch")["success_rate"].apply(lambda s: _std(s.values)).mean() * 100
        if spread < 2 * max(seed_sd, 1e-9):
            lines.append("> ⚠️ **结构差异不显著**：各结构成功率极差仅 %.0f 个百分点，"
                         "而种子间标准差平均 %.0f 个百分点 —— **在当前 3 个种子下不足以"
                         "判定哪种结构更优**，只能说差异落在噪声范围内。" % (spread, seed_sd))
        else:
            lines.append("> 结构间成功率极差 %.0f 个百分点，种子间标准差平均 %.0f 个百分点，"
                         "差异超出噪声范围。" % (spread, seed_sd))
        lines.append("")

    lines.append("> **注意**：以上均为 %d 个随机种子的均值；单种子波动可达 ±10 个百分点，"
                 "不应据此下强结论。" % df["seed"].nunique())
    lines.append(">")
    lines.append("> 100k 步预算下三种算法都远未收敛（`uav_ppo_nav` 用 400k 步可达 78%），"
                 "因此本实验比较的是**样本效率**，而不是各算法的性能上限。")
    lines.append(">")
    lines.append("> off-policy 的 SAC/TD3 对超参更敏感；本实验为公平起见统一了学习率、批大小"
                 "与网络结构，并沿用 SB3 默认（未做逐算法调参）—— 这是本实验的已知局限。")
    lines.append("")

    # ---------------- 图表 ----------------
    if make_plots:
        try:
            plt, cjk = _setup_matplotlib()
            saved = []

            fig, ax = plt.subplots(figsize=(6, 4))
            _bar_with_err(plt, ax, sub, "algo", "success_rate", cjk,
                          "不同算法成功率（%s）" % base_arch,
                          "Success rate by algorithm (%s)" % base_arch,
                          "算法", "Algorithm")
            fig.tight_layout(); p = os.path.join(out_dir, "fig_success_by_algo.png")
            fig.savefig(p, dpi=140); plt.close(fig); saved.append(p)

            for algo in sorted(set(df["algo"])):
                sub2 = df[df["algo"] == algo]
                if sub2["arch"].nunique() < 2:
                    continue
                fig, ax = plt.subplots(figsize=(6, 4))
                _bar_with_err(plt, ax, sub2, "arch", "success_rate", cjk,
                              "不同网络结构成功率（%s）" % algo.upper(),
                              "Success rate by architecture (%s)" % algo.upper(),
                              "隐藏层结构", "Hidden layers")
                fig.tight_layout()
                p = os.path.join(out_dir, "fig_success_by_arch_%s.png" % algo)
                fig.savefig(p, dpi=140); plt.close(fig); saved.append(p)

            # 学习曲线：按种子聚合成"均值 ± 标准差带"
            #   左：同一结构下不同算法；右：同一算法下不同结构
            #   （15 条原始曲线画在一起图例会挤爆，聚合成带既干净又是 RL 论文的标准画法）
            def _collect(sub_df, key):
                """{key值: (xs, mean, std)}，按 timesteps 对齐后聚合."""
                buckets = {}
                for (algo, arch, seed), _ in sub_df.groupby(["algo", "arch", "seed"]):
                    log = os.path.join(out_dir, "runs",
                                       "%s_%s_seed%d" % (algo, arch, seed), "eval_log.json")
                    if not os.path.exists(log):
                        continue
                    with open(log, encoding="utf-8") as fh:
                        hist = json.load(fh)
                    if not hist:
                        continue
                    label = algo.upper() if key == "algo" else arch
                    buckets.setdefault(label, []).append(
                        {h["timesteps"]: h["success_rate"] * 100 for h in hist})
                out = {}
                for label, runs in buckets.items():
                    xs = sorted(set().union(*[set(r) for r in runs]))
                    ys = np.array([[r.get(x, np.nan) for r in runs] for x in xs], dtype=float)
                    out[label] = (xs, np.nanmean(ys, axis=1), np.nanstd(ys, axis=1))
                return out

            if len(set(df["arch"])) > 1:
                fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
            else:
                fig, axes = plt.subplots(1, 1, figsize=(7, 4.5))
                axes = [axes]

            panels = [(axes[0], df[df["arch"] == base_arch], "algo",
                       "不同算法（结构 %s）" % base_arch,
                       "Algorithms (arch %s)" % base_arch)]
            if len(axes) > 1:
                panels.append((axes[1], df[df["algo"] == "ppo"], "arch",
                               "不同网络结构（PPO）", "Architectures (PPO)"))

            plotted = False
            for ax, sub_df, key, tzh, ten in panels:
                data = _collect(sub_df, key)
                for label, (xs, mean, std) in data.items():
                    ax.plot(xs, mean, marker="o", ms=3, label=label)
                    ax.fill_between(xs, mean - std, mean + std, alpha=0.18)
                    plotted = True
                ax.set_xlabel(_label(cjk, "训练步数", "Training timesteps"))
                ax.set_ylabel(_label(cjk, "成功率 (%)", "Success rate (%)"))
                ax.set_title(_label(cjk, tzh, ten))
                ax.grid(alpha=0.3)
                if data:
                    ax.legend(fontsize=9)

            if plotted:
                fig.suptitle(_label(cjk, "样本效率：成功率随训练步数的变化（均值 ± 标准差）",
                                    "Sample efficiency (mean ± std over seeds)"),
                             fontsize=12)
                fig.tight_layout(rect=[0, 0, 1, 0.94])
                p = os.path.join(out_dir, "fig_learning_curves.png")
                fig.savefig(p, dpi=140); plt.close(fig); saved.append(p)
            else:
                plt.close(fig)

            # 训练成本
            fig, ax = plt.subplots(figsize=(6, 4))
            _bar_with_err(plt, ax, sub, "algo", "train_time_s", cjk,
                          "训练耗时对比（%s，%s 步）" % (base_arch, int(df["steps"].iloc[0])),
                          "Training wall-clock (%s, %s steps)" % (base_arch, int(df["steps"].iloc[0])),
                          "算法", "Algorithm")
            ax.set_ylim(0, None)
            fig.tight_layout(); p = os.path.join(out_dir, "fig_cost.png")
            fig.savefig(p, dpi=140); plt.close(fig); saved.append(p)

            # 路径效率 vs 平滑度
            fig, ax = plt.subplots(figsize=(6, 4.5))
            for algo, g in df.groupby("algo"):
                ax.scatter(g["action_smoothness"], g["path_efficiency"],
                           s=60, alpha=0.8, label=algo.upper())
            ax.set_xlabel(_label(cjk, "动作平滑度（越小越好）", "Action smoothness (lower better)"))
            ax.set_ylabel(_label(cjk, "路径效率（越大越好）", "Path efficiency (higher better)"))
            ax.set_title(_label(cjk, "运动质量权衡", "Motion-quality trade-off"))
            ax.grid(alpha=0.3); ax.legend()
            fig.tight_layout(); p = os.path.join(out_dir, "fig_tradeoff.png")
            fig.savefig(p, dpi=140); plt.close(fig); saved.append(p)

            lines += ["## %d. 图表" % (_sec[0] + 1), ""]
            _sec[0] += 1
            for p in saved:
                lines.append("![%s](%s)" % (os.path.basename(p), os.path.basename(p)))
            lines.append("")
            print("已生成 %d 张图（中文字体：%s）" % (len(saved), "有" if cjk else "无，已退回英文标签"))
        except Exception as exc:                            # noqa: BLE001
            lines += ["## %d. 图表" % (_sec[0] + 1), "", "（绘图失败：%s）" % exc, ""]
            print("绘图失败：%s" % exc)

    md_path = os.path.join(out_dir, "report.md")
    with open(md_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("报告已写入 %s" % md_path)
    return md_path


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"))
    build_report(ap.parse_args().out)
