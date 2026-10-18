#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评估：固定航线巡航策略的完整指标 + 轨迹图.

与训练脚本里的快速评估相比，这里额外输出：
  * 每个回合的轨迹（导出 PNG，直观看它怎么绕障碍）
  * 平均高度偏差、最大横向偏差（不只是均值，能看出偶发的大偏离）
  * 与「纯循迹」手工控制器的对照（证明避障能力确实来自学习）
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import policy as P
from route import make_route
from route_env import RouteEnv


def roll_policy(act_fn, episodes: int = 20, seed: int = 1000, **env_kw) -> dict:
    """跑若干回合并汇总指标。act_fn: obs -> action。"""
    env = RouteEnv(seed=seed, **env_kw)
    ok = crash = timeout = 0
    crosses, comps, heights, steps_all = [], [], [], []
    trajs = []
    for ep in range(int(episodes)):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        steps = 0
        info = {}
        cross_ep, z_ep = [], []
        pts = [(float(env.pos[0]), float(env.pos[1]))]
        while not done:
            a = act_fn(obs)
            obs, _r, term, trunc, info = env.step(a)
            _, _s, _, cross = env.route.project(env.pos[:2])
            cross_ep.append(abs(cross))
            z_ep.append(abs(float(env.pos[2] - env.route.points[0, 2])))
            pts.append((float(env.pos[0]), float(env.pos[1])))
            steps += 1
            done = bool(term or trunc)
        steps_all.append(steps)
        crosses.append(float(np.mean(cross_ep)))
        heights.append(float(np.mean(z_ep)))
        comps.append(env.route.completion(env.visited_s))
        trajs.append(np.asarray(pts))
        if info.get("success"):
            ok += 1
        elif info.get("crash_reason"):
            crash += 1
        else:
            timeout += 1
    n = float(max(episodes, 1))
    return {"episodes": int(episodes), "lap": ok, "crash": crash, "timeout": timeout,
            "lap_rate": ok / n, "crash_rate": crash / n, "timeout_rate": timeout / n,
            "mean_cross": float(np.mean(crosses)),
            "max_cross": float(np.max(crosses)),
            "mean_completion": float(np.mean(comps)),
            "mean_height_err": float(np.mean(heights)),
            "mean_steps": float(np.mean(steps_all)),
            "route_length": float(env.route.length),
            "route": env.route,
            "obstacles": list(env.obstacles_xy),
            "obstacle_radius": float(env.obstacle_radius),
            "trajectories": trajs}


def plot_runs(route, ppo: dict, baseline: dict, path: str) -> bool:
    """航线 + 两条典型轨迹 + 成功率柱状图。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        has_cjk = any("CJK" in f.name or "Hei" in f.name or "Song" in f.name
                      for f in matplotlib.font_manager.fontManager.ttflist)
        if has_cjk:
            plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei",
                                               "SimHei", "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
        L = (lambda zh, en: zh if has_cjk else en)

        fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
        ax = axes[0]
        # 障碍物：压在航线上的那些标红（正是需要绕行的），旁侧的标灰
        obstacles = ppo.get("obstacles") or []
        radius = float(ppo.get("obstacle_radius", 0.6))
        if obstacles:
            from matplotlib.patches import Circle
            for (ox, oy) in obstacles:
                on_route = abs(route.project((ox, oy))[3]) < radius
                ax.add_patch(Circle((ox, oy), radius,
                                    color="#E45756" if on_route else "#BBBBBB",
                                    alpha=0.55 if on_route else 0.35, zorder=1))
            ax.plot([], [], "o", color="#E45756", alpha=0.55,
                    label=L("压在航线上的障碍（必须绕行）", "blocking obstacle"))
            ax.plot([], [], "o", color="#BBBBBB", alpha=0.35, label=L("旁侧障碍", "side obstacle"))
        ax.plot(route.points[:, 0], route.points[:, 1], "-", lw=1.4,
                color="#999999", zorder=2, label=L("目标航线", "target route"))
        # 画**全部**回合的轨迹（半透明）：单拿某一条当示例不稳，
        # 因为随机障碍下某个回合可能开局就撞，看起来像"没飞"。
        if ppo["trajectories"]:
            for k, t in enumerate(ppo["trajectories"]):
                if len(t) < 2:
                    continue
                ax.plot(t[:, 0], t[:, 1], "-", lw=1.0, color="#4C78A8",
                        alpha=0.35, zorder=3,
                        label=L("PPO 策略（全部回合）", "PPO (all episodes)") if k == 0 else None)
        if baseline["trajectories"]:
            for k, t in enumerate(baseline["trajectories"]):
                if len(t) < 2:
                    continue
                ax.plot(t[:, 0], t[:, 1], "--", lw=0.9, color="#E45756",
                        alpha=0.45, zorder=3,
                        label=L("纯循迹（全部回合）", "pure pursuit (all)") if k == 0 else None)
            ends = np.array([t[-1] for t in baseline["trajectories"] if len(t)])
            if len(ends):
                ax.plot(ends[:, 0], ends[:, 1], "x", ms=7, mew=1.8, color="#E45756",
                        alpha=0.8, zorder=5, label=L("纯循迹碰撞点", "baseline crash"))
        ax.set_xlabel(L("x / m（东）", "x [m]"))
        ax.set_ylabel(L("y / m（北）", "y [m]"))
        ax.set_title(L("航行轨迹对比（全部 %d 个回合）" % len(ppo["trajectories"]),
                       "trajectories (%d episodes)" % len(ppo["trajectories"])))
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=7.0, loc="upper center", ncol=2)
        ax.grid(alpha=0.3)

        ax = axes[1]
        labels = [L("PPO 策略", "PPO"), L("纯循迹", "pure pursuit")]
        rates = [ppo["lap_rate"] * 100, baseline["lap_rate"] * 100]
        bars = ax.bar(labels, rates, color=["#4C78A8", "#E45756"])
        for b, r in zip(bars, rates):
            ax.text(b.get_x() + b.get_width() / 2, r + 2, "%.0f%%" % r,
                    ha="center", fontsize=10)
        ax.set_ylim(0, 108)
        ax.set_ylabel(L("完成一圈率 (%)", "lap completion (%)"))
        ax.set_title(L("同航线、同随机障碍", "same route, same obstacles"))
        ax.grid(axis="y", alpha=0.3)

        plt.tight_layout()
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        plt.savefig(path, dpi=130)
        plt.close()
        return True
    except Exception as exc:                              # noqa: BLE001
        print("  [提示] 画图失败（%s），跳过" % exc)
        return False


def pure_pursuit_factory(env, lookahead: float = 4.0):
    """纯循迹基线：只朝前方航点飞，**完全无视障碍**。"""
    def act(_obs):
        _, s, _, _ = env.route.project(env.pos[:2])
        wp = env.route.waypoint_ahead(env.pos[:2], s, lookahead)
        d = wp - env.pos[:2]
        n = np.linalg.norm(d) or 1.0
        a = np.zeros(4, dtype=np.float32)
        a[0] = d[0] / n
        a[1] = d[1] / n
        a[2] = float(np.clip(-(env.pos[2] - env.route.points[0, 2]) / 1.5, -1.0, 1.0))
        return a
    return act


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="评估无人机固定路线巡航策略")
    ap.add_argument("--model", default="models/route_ppo_s0.npz")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--route", default="rect", choices=["rect", "figure8"])
    ap.add_argument("--obstacles", type=int, default=14)
    ap.add_argument("--save-dir", default="results")
    args = ap.parse_args(argv)

    print("=" * 68)
    print("  无人机固定路线巡航 —— 评估")
    print("=" * 68)
    if not os.path.isfile(args.model):
        print("  [错误] 找不到模型: %s" % args.model)
        return 1
    pol = P.MlpPolicy.from_npz(args.model)
    kw = dict(route_name=args.route, num_obstacles=args.obstacles)

    r_ppo = roll_policy(lambda o: pol.forward(o), args.episodes, args.seed, **kw)
    route = r_ppo["route"]
    r_pp = roll_policy(pure_pursuit_factory(RouteEnv(seed=args.seed, **kw)),
                       args.episodes, args.seed, **kw)

    rows = [("完成一圈率(%)", "lap_rate", 100.0), ("碰撞率(%)", "crash_rate", 100.0),
            ("超时率(%)", "timeout_rate", 100.0),
            ("平均横向偏差(m)", "mean_cross", 1.0),
            ("最大横向偏差(m)", "max_cross", 1.0),
            ("平均完成度(%)", "mean_completion", 100.0),
            ("平均高度偏差(m)", "mean_height_err", 1.0),
            ("平均步数", "mean_steps", 1.0)]
    print()
    print("  %-20s %-16s %-16s" % ("指标", "PPO 策略", "纯循迹基线"))
    for label, key, scale in rows:
        print("  %-20s %-16.3f %-16.3f"
              % (label, r_ppo[key] * scale, r_pp[key] * scale))
    print("  %-20s %-16d %-16d" % ("回合数", r_ppo["episodes"], r_pp["episodes"]))
    print("  %-20s %.1f m" % ("航线总长", route.length))
    print("=" * 68)

    png = os.path.join(args.save_dir, "route_cruise_eval.png")
    if plot_runs(route, r_ppo, r_pp, png):
        print("  轨迹对比图:", png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
