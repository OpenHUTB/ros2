#!/usr/bin/env python3
"""入口：`python main.py --demo` 全流程，`--test` 跑单测，`--sweep` 只重跑实验矩阵。

本模块研究的是**控制链路延迟**对神经网络跟踪策略的影响：
同一份权重、同一条赛道，只改变「指令从算出来到落到车上有多少毫秒」，
看误差怎么变，以及三种补偿能把损失挽回多少。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from lat_nav import sweep, track                      # noqa: E402
from lat_nav.env import DT, LIDAR_BEAMS, LIDAR_RANGE, LatencyEnv  # noqa: E402
from lat_nav.nav import COLLIDE_CLEARANCE, run_episode  # noqa: E402
from lat_nav.net import LatencyNet                    # noqa: E402
from lat_nav.train import train                       # noqa: E402

RESULTS = ROOT / "results"
MODELS = ROOT / "models"


def fmt_row(r: dict) -> str:
    return (f"| {r['shape']} | {r['delay_ms']:.0f} | "
            f"{sweep.COMP_LABEL[r['comp']]} | {r['rms']:.3f} | {100 * r['success']:.0f}% | "
            f"{r['collide_frames']} | {r['min_clear']:.2f} |")


def table(rows: list[dict]) -> str:
    head = "| 赛道 | 延迟/ms | 补偿 | RMS/m | 完成率 | 碰撞帧 | 最小净空/m |\n|---|---|---|---|---|---|---|"
    return "\n".join([head] + [fmt_row(r) for r in rows])


def render_gif(policy, shape: str, delay_ms: float, comp: str, path: Path,
               every: int = 4, max_steps: int = 420) -> float:
    """俯视图动图：墙体 + 轨迹 + 车身。帧由 matplotlib 画，机器上没有 GL 也能出。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    env = LatencyEnv(shape, seed=0)
    res = run_episode(env, policy=policy, comp=comp, delay_ms=delay_ms, seed=0, record=True)
    frames = res["frames"][:max_steps]

    path_pts = track.centerline(shape)
    borders = [path_pts + o * track.normals(path_pts)
               for o in (track.LANE_HALF_WIDTH, -track.LANE_HALF_WIDTH)]
    lim = np.vstack([path_pts] + borders)
    lo, hi = lim.min(axis=0) - 0.8, lim.max(axis=0) + 0.8

    images = []
    for k in range(0, len(frames), every):
        fig, ax = plt.subplots(figsize=(5.2, 4.4))
        ax.plot(path_pts[:, 0], path_pts[:, 1], "k--", lw=0.8, alpha=0.6)
        for b in borders:
            ax.plot(b[:, 0], b[:, 1], color="0.55", lw=0.7)
        trail = np.asarray(frames[:k + 1])
        ax.plot(trail[:, 0], trail[:, 1], color="#2e86ab", lw=1.8)
        p = frames[k]
        ax.arrow(p[0], p[1], 0.28 * np.cos(p[2]), 0.28 * np.sin(p[2]),
                 head_width=0.16, head_length=0.14, color="#d1495b", length_includes_head=True)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_aspect("equal")
        ax.set_title(f"{shape} | delay {delay_ms:.0f} ms | {sweep.COMP_LABEL[comp]}\n"
                     f"t = {0.05 * k:4.1f} s", fontsize=10)
        ax.set_xticks([])
        ax.set_yticks([])
        fig.tight_layout()
        fig.canvas.draw()
        img = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
        images.append(img)
        plt.close(fig)

    images[0].save(path, save_all=True, append_images=images[1:], duration=80, loop=0,
                   optimize=True)
    return float(res["rms"])


def run_demo(epochs: int = 120) -> None:
    RESULTS.mkdir(exist_ok=True)
    MODELS.mkdir(exist_ok=True)
    t0 = time.time()

    print("== 1/6 训练（专家演示 + DAgger）==")
    net, info = train(episodes=3, dagger_rounds=1, epochs=epochs,
                      save_to=MODELS / "policy.npz")
    net.save(RESULTS / "policy.npz")
    print(f"   样本 {info['samples']}（专家 {info['expert_samples']} + DAgger "
          f"{info['dagger_samples']}×2）参数 {info['params']} 末轮损失 {info['final_loss']:.4f}")

    print("== 2/6 上界参考：真值纯追踪 ==")
    upper = []
    for sh in track.SHAPES:
        r = run_episode(LatencyEnv(sh, seed=0), policy=None, seed=0)
        upper.append({"shape": sh, "rms": r["rms"], "success": float(r["completed"])})
        print(f"   {sh:7s} rms={r['rms']:.3f} 完成={r['completed']} "
              f"最小净空={r['min_clear']:.2f}m")

    print("== 3/6 延迟扫描（20 Hz，3 赛道 × 2 种子）==")
    rows = sweep.run_matrix(net)
    print(table(rows))

    print("== 4/6 控制频率扫描（延迟固定 600 ms）==")
    rate_rows = sweep.run_rate_matrix(net, )
    print(table(rate_rows))

    print("== 5/6 画图 ==")
    sweep.plot_delay_sweep(rows, RESULTS / "delay_sweep.png")
    sweep.plot_rate_sweep(rate_rows, RESULTS / "rate_sweep.png")

    traces = {}
    env = LatencyEnv("kidney", seed=0)
    for c in sweep.COMPS:
        traces[c] = run_episode(env, policy=net, comp=c, delay_ms=600.0, seed=0,
                                record=True)["frames"]
    sweep.plot_traces(traces, RESULTS / "traces.png", "kidney")

    print("== 6/6 生成动图 ==")
    for c in ("none", "model"):
        p = RESULTS / f"track_{c}.gif"
        rms = render_gif(net, "kidney", 600.0, c, p)
        print(f"   {p.name}: rms={rms:.3f} 大小={p.stat().st_size / 1e6:.1f} MB")

    summary = {
        "env": {"shapes": list(track.SHAPES), "lane_half_width": track.LANE_HALF_WIDTH,
                "dt_ctrl_s": DT, "lidar_beams": LIDAR_BEAMS, "lidar_range": LIDAR_RANGE,
                "collide_clearance_m": COLLIDE_CLEARANCE},
        "net": {"params": info["params"], "samples": info["samples"],
                "expert_samples": info["expert_samples"], "dagger_samples": info["dagger_samples"],
                "final_loss": info["final_loss"]},
        "upper_bound": upper,
        "delay_rows": rows,
        "rate_rows": rate_rows,
        "wall_s": time.time() - t0,
    }
    (RESULTS / "metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\n完成，用时 {summary['wall_s']:.0f} s，产物在 {RESULTS}")


def rerender() -> None:
    """只重画图与 GIF（用 metrics.json 里的矩阵 + 已有权重），不动训练。"""
    RESULTS.mkdir(parents=True, exist_ok=True)
    data = json.loads((RESULTS / "metrics.json").read_text(encoding="utf-8"))
    sweep.plot_delay_sweep(data["delay_rows"], RESULTS / "delay_sweep.png")
    sweep.plot_rate_sweep(data["rate_rows"], RESULTS / "rate_sweep.png")
    net = LatencyNet().load(MODELS / "policy.npz")
    traces = {}
    env = LatencyEnv("kidney", seed=0)
    for c in sweep.COMPS:
        traces[c] = run_episode(env, policy=net, comp=c, delay_ms=600.0, seed=0,
                                record=True)["frames"]
    sweep.plot_traces(traces, RESULTS / "traces.png", "kidney")
    for c in ("none", "model"):
        p = RESULTS / f"track_{c}.gif"
        rms = render_gif(net, "kidney", 600.0, c, p)
        print(f"   {p.name}: rms={rms:.3f} 大小={p.stat().st_size / 1e6:.1f} MB")


def run_sweep() -> None:
    """用已有权重重跑矩阵。干净克隆里 results/ 不存在（.gitignore 挡住了），
    所以这里必须自己建目录——上一版忘了，矩阵算完存图时才抛 FileNotFoundError。"""
    RESULTS.mkdir(parents=True, exist_ok=True)
    net = LatencyNet().load(MODELS / "policy.npz")
    rows = sweep.run_matrix(net)
    print(table(rows))
    rate_rows = sweep.run_rate_matrix(net)
    print(table(rate_rows))
    sweep.plot_delay_sweep(rows, RESULTS / "delay_sweep.png")
    sweep.plot_rate_sweep(rate_rows, RESULTS / "rate_sweep.png")


def main() -> int:
    ap = argparse.ArgumentParser(description="神经网络跟踪策略的延迟鲁棒性实验")
    ap.add_argument("--demo", action="store_true", help="训练 + 实验 + 出图出 GIF")
    ap.add_argument("--sweep", action="store_true", help="只重跑实验矩阵（用已有权重）")
    ap.add_argument("--plots", action="store_true", help="只重画图与 GIF")
    ap.add_argument("--test", action="store_true", help="跑 pytest")
    ap.add_argument("--epochs", type=int, default=120)
    args = ap.parse_args()

    if args.test:
        import subprocess

        return subprocess.call([sys.executable, "-m", "pytest", str(ROOT / "test"), "-q"])
    if args.sweep:
        run_sweep()
        return 0
    if args.plots:
        rerender()
        return 0
    run_demo(epochs=args.epochs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
