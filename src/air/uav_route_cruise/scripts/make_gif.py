#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成运行效果 gif：同一场景下 PPO 策略 vs 纯跟踪基线，逐帧对比.

用途：作业要求文档里配 gif 动图（一般 <10MB）。这里不依赖 CARLA / ROS，
直接把质点环境里两个策略的飞行过程画成动画，能直观看出：
  * 纯跟踪贴着航线中线飞，遇到压线障碍立刻撞毁；
  * PPO 会主动偏离航线绕过去，再回到航线上。
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import policy as P
from route_env import RouteEnv


def rollout(pol, use_image: bool, seed: int, max_steps: int = 400,
            obstacles: int = 14, lookahead: float = 4.0):
    """跑一个回合，记录每帧的位置、横向偏差与终止原因。"""
    env = RouteEnv(seed=seed, num_obstacles=obstacles)
    env.reset(seed=seed)
    pts, crosses, steers = [], [], []
    done = False
    info = {}
    steps = 0
    while not done and steps < max_steps:
        if use_image:
            a = pol.forward(env._obs())
        else:
            # 纯跟踪：只朝前视点飞，完全无视障碍
            _, s, _, _ = env.route.project(env.pos[:2])
            wp = env.route.waypoint_ahead(env.pos[:2], s, lookahead)
            d = wp - env.pos[:2]
            n = np.linalg.norm(d) or 1.0
            a = np.zeros(4, dtype=np.float32)
            a[0] = d[0] / n
            a[1] = d[1] / n
            a[2] = float(np.clip(-(env.pos[2] - env.route.points[0, 2]) / 1.5, -1, 1))
        _o, _r, term, trunc, info = env.step(a)
        _, _s, _, c = env.route.project(env.pos[:2])
        pts.append((float(env.pos[0]), float(env.pos[1])))
        crosses.append(abs(c))
        steers.append(float(a[0]))
        done = bool(term or trunc)
        steps += 1
    return {"pts": np.asarray(pts), "crosses": crosses, "steers": steers,
            "success": bool(info.get("success")),
            "crash": info.get("crash_reason"), "steps": steps,
            "obstacles": list(env.obstacles_xy),
            "radius": float(env.obstacle_radius),
            "route": env.route}


def make_gif(model_path: str, out: str, seed: int = 1000, fps: int = 12,
             stride: int = 2, obstacles: int = 14) -> bool:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle

    has_cjk = any("CJK" in f.name or "Hei" in f.name or "Song" in f.name
                  for f in matplotlib.font_manager.fontManager.ttflist)
    if has_cjk:
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei",
                                           "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    L = (lambda zh, en: zh if has_cjk else en)

    pol = P.MlpPolicy.from_npz(model_path)
    r_ppo = rollout(pol, True, seed, obstacles=obstacles)
    r_pp = rollout(None, False, seed, obstacles=obstacles)
    route = r_ppo["route"]
    print("  PPO : %d 步，%s" % (r_ppo["steps"],
                                 "完成一圈" if r_ppo["success"] else (r_ppo["crash"] or "超时")))
    print("  基线: %d 步，%s" % (r_pp["steps"],
                                 "完成一圈" if r_pp["success"] else (r_pp["crash"] or "超时")))

    n_frames = max(len(r_ppo["pts"]), len(r_pp["pts"]))
    frames = list(range(0, n_frames, max(1, stride)))

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.4))
    ax, ax2 = axes

    def draw_scene(axis):
        axis.clear()
        for (ox, oy) in r_ppo["obstacles"]:
            on_route = abs(route.project((ox, oy))[3]) < r_ppo["radius"]
            axis.add_patch(Circle((ox, oy), r_ppo["radius"],
                                  color="#E45756" if on_route else "#BBBBBB",
                                  alpha=0.5 if on_route else 0.3, zorder=1))
        axis.plot(route.points[:, 0], route.points[:, 1], "-", lw=1.5,
                  color="#888888", zorder=2)
        axis.set_xlim(-13.5, 13.5)
        axis.set_ylim(-11.0, 10.0)
        axis.set_aspect("equal")
        axis.grid(alpha=0.25)
        axis.set_xlabel(L("x / m（东）", "x [m]"))
        axis.set_ylabel(L("y / m（北）", "y [m]"))

    def init():
        return []

    def update(k):
        draw_scene(ax)   # 右图是曲线图，不能用 draw_scene（它会设置等比例坐标轴）
        i1 = min(k, len(r_ppo["pts"]) - 1)
        i2 = min(k, len(r_pp["pts"]) - 1)
        # 左：轨迹对比
        ax.plot(r_ppo["pts"][:i1 + 1, 0], r_ppo["pts"][:i1 + 1, 1], "-",
                lw=2.2, color="#4C78A8", zorder=3)
        ax.plot(r_pp["pts"][:i2 + 1, 0], r_pp["pts"][:i2 + 1, 1], "--",
                lw=1.6, color="#E45756", zorder=3)
        ax.plot(r_ppo["pts"][i1, 0], r_ppo["pts"][i1, 1], "o", ms=7,
                color="#4C78A8", zorder=5)
        ax.plot(r_pp["pts"][i2, 0], r_pp["pts"][i2, 1], "o", ms=6,
                color="#E45756", zorder=5)
        if i2 == len(r_pp["pts"]) - 1:
            ax.plot(r_pp["pts"][-1, 0], r_pp["pts"][-1, 1], "x", ms=13, mew=2.6,
                    color="#E45756", zorder=6)
        ax.set_title(L("轨迹：PPO（蓝实线）vs 纯跟踪（红虚线）", "PPO (blue) vs pure pursuit (red)"))

        # 右：横向偏差随时间
        ax2.clear()
        ax2.plot(np.arange(len(r_ppo["crosses"]))[:i1 + 1], r_ppo["crosses"][:i1 + 1],
                 "-", lw=1.8, color="#4C78A8", label=L("PPO", "PPO"))
        ax2.plot(np.arange(len(r_pp["crosses"]))[:i2 + 1], r_pp["crosses"][:i2 + 1],
                 "--", lw=1.4, color="#E45756", label=L("纯跟踪", "pure pursuit"))
        ax2.set_xlim(0, n_frames)
        ax2.set_ylim(0, max(2.6, max(r_ppo["crosses"] + r_pp["crosses"]) * 1.1))
        ax2.set_xlabel(L("控制步", "step"))
        ax2.set_ylabel(L("横向偏差 / m", "cross-track error [m]"))
        ax2.set_title(L("离航线有多远（第 %d 步）" % k, "cross-track error (step %d)" % k))
        ax2.legend(fontsize=8, loc="upper right")
        ax2.grid(alpha=0.25)
        plt.tight_layout()
        return []

    anim = FuncAnimation(fig, update, frames=frames, init_func=init,
                         interval=1000.0 / max(fps, 1), blit=False)
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    size = os.path.getsize(out)
    print("  已生成 %s（%d 帧，%.1f MB）" % (out, len(frames), size / 1e6))
    if size > 10e6:
        print("  [提醒] 超过 10 MB，建议调大 --stride 或调低 --fps")
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="生成运行效果 gif")
    ap.add_argument("--model", default="models/route_ppo_s0.npz")
    ap.add_argument("--out", default="results/route_cruise_demo.gif")
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--obstacles", type=int, default=14)
    args = ap.parse_args(argv)
    if not os.path.isfile(args.model):
        print("  [错误] 找不到权重:", args.model)
        return 1
    make_gif(args.model, args.out, args.seed, args.fps, args.stride, args.obstacles)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
