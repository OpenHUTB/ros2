"""把迫降过程画成动图和静态图（figures for the docs）。

这里只依赖 matplotlib / Pillow；没有它们的时候调用方应该跳过出图，不要让整套
训练与评测流程因为画不出图而失败。
"""

from __future__ import annotations

import os
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np

from . import evaluate as E
from . import landing as L
from . import mapping as M


# --------------------------------------------------------------------------- #
# 小工具
# --------------------------------------------------------------------------- #
#: 中文字体候选：Ubuntu 上通常是 Noto Sans CJK，Windows 上是雅黑/黑体
_CJK_FONTS = ("Noto Sans CJK SC", "Noto Sans SC", "Microsoft YaHei", "SimHei",
              "WenQuanYi Zen Hei", "Source Han Sans SC", "Arial Unicode MS")


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import font_manager
    import matplotlib.pyplot as plt

    available = {f.name for f in font_manager.fontManager.ttflist}
    picked = [name for name in _CJK_FONTS if name in available]
    if picked:
        plt.rcParams["font.sans-serif"] = picked + ["DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
    return plt


def _fill(ax, color: str) -> None:
    ax.set_facecolor(color)


def _plot_altitudes(ax, runs: Dict[str, E.LandingResult], colors: Dict[str, str]) -> None:
    for mode, result in runs.items():
        if not result.times:
            continue
        ax.plot(result.times, result.altitudes, color=colors.get(mode, "#4C72B0"),
                lw=1.6, label=mode)
    ax.axhline(E.LAND_Z, color="#999999", ls=":", lw=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("altitude (m)")
    ax.set_title("altitude during forced landing", fontsize=9)
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(alpha=0.25)


def _plot_topdown(ax, score: np.ndarray, height_map: M.HeightMap,
                  runs: Dict[str, E.LandingResult], colors: Dict[str, str]) -> None:
    extent = [height_map.cfg.origin_xy[0],
              height_map.cfg.origin_xy[0] + height_map.cfg.size_m,
              height_map.cfg.origin_xy[1],
              height_map.cfg.origin_xy[1] + height_map.cfg.size_m]
    ax.imshow(score, origin="lower", extent=extent, cmap="viridis", vmin=0.0, vmax=1.0)
    ax.set_title("landing suitability (neural score)", fontsize=9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    for mode, result in runs.items():
        if result.times:
            # 轨迹用的是 record 里的时间序列，这里只标终点与落点
            if result.final_xy is not None:
                ax.plot(result.final_xy[0], result.final_xy[1], "o",
                        color=colors.get(mode, "#4C72B0"), ms=5, label=f"{mode} end")
        if result.target_xy is not None:
            ax.plot(result.target_xy[0], result.target_xy[1], "x",
                    color=colors.get(mode, "#4C72B0"), ms=7, mew=1.6)
    ax.legend(fontsize=7, loc="upper right")


MODE_COLORS = {"vertical": "#C44E52", "expert": "#4C72B0", "neural": "#55A868"}


# --------------------------------------------------------------------------- #
# 动图
# --------------------------------------------------------------------------- #
def make_landing_gif(out_path: str, plan_spec: Optional[dict] = None, scene: str = "urban_block",
                     seed: int = 0, modes: Sequence[str] = ("vertical", "expert"),
                     fps: int = 10, seconds: float = 40.0, score_net=None,
                     guidance_net=None, api=None, dpi: int = 80) -> str:
    """并排对比"直接垂直下降"与"选点迫降"的高度曲线，逐帧出一张 gif。"""
    plt = _pyplot()
    from . import _compat

    api = api or _compat.load_base()
    params, effectiveness = E.build_context(api)
    start_xy, hover_z = E.pick_obstacle_spot(api, scene, seed)
    plan_spec = plan_spec or dict(kind="degradation", rotor=2, severity=0.5)

    runs: Dict[str, E.LandingResult] = {}
    for mode in modes:
        runs[mode] = E.run_landing(api, mode, plan_spec, scene=scene, seed=seed,
                                   seconds=seconds, start_xy=start_xy, hover_z=hover_z,
                                   score_net=score_net, guidance_net=guidance_net,
                                   record=True, params=params, effectiveness=effectiveness)

    n = min((len(r.times) for r in runs.values() if r.times), default=0)
    if n == 0:
        raise RuntimeError("没有采到任何帧，检查 record 与 seconds")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.1), dpi=dpi)
    all_alt = [a for r in runs.values() for a in r.altitudes] or [0.0, hover_z]
    lo = min(-0.2, min(all_alt) - 0.2)
    hi = max(all_alt) + 0.4

    frames = []
    # 一次迫降有上千步，逐帧渲染出来动图太长也太重：按总帧数抽稀到 ~200 帧
    stride = max(1, int(round(n / 200.0)))
    indices = list(range(0, n, stride))
    if indices[-1] != n - 1:
        indices.append(n - 1)
    for i in indices:
        for ax in axes:
            ax.clear()
        t_now = runs[list(runs)[0]].times[i]
        for mode, result in runs.items():
            times = result.times[: i + 1]
            alts = result.altitudes[: i + 1]
            axes[0].plot(times, alts, color=MODE_COLORS.get(mode, "#4C72B0"), lw=1.8, label=mode)
        axes[0].axhline(E.LAND_Z, color="#999999", ls=":", lw=0.8)
        axes[0].set_ylim(lo, hi)
        axes[0].set_xlim(0.0, max(r.times[-1] for r in runs.values() if r.times))
        axes[0].set_xlabel("time (s)")
        axes[0].set_ylabel("altitude (m)")
        axes[0].set_title("altitude during forced landing", fontsize=9)
        axes[0].legend(fontsize=7, loc="upper right")
        axes[0].grid(alpha=0.25)

        status = "  ".join(f"{m}: {r.status}" for m, r in runs.items())
        axes[1].axis("off")
        axes[1].text(0.02, 0.92, f"t = {t_now:5.2f} s", fontsize=11, family="monospace",
                     transform=axes[1].transAxes)
        axes[1].text(0.02, 0.78, f"fault: {plan_spec['kind']} rotor{plan_spec['rotor']}",
                     fontsize=9, transform=axes[1].transAxes)
        for k, (mode, result) in enumerate(runs.items()):
            axes[1].text(0.02, 0.62 - 0.10 * k,
                         f"{mode:9s} {result.status}(最终)",
                         fontsize=9, transform=axes[1].transAxes,
                         color=MODE_COLORS.get(mode, "#333333"))
        fig.suptitle(f"forced landing on {scene} (fault at {runs[list(runs)[0]].times[0]:.1f}s)",
                     fontsize=9)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
    plt.close(fig)

    _save_gif(frames, out_path, fps=fps)
    return out_path


def _save_gif(frames: List[np.ndarray], out_path: str, fps: int = 10) -> None:
    from PIL import Image

    images = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=96)
              for f in frames]
    images[0].save(out_path, save_all=True, append_images=images[1:],
                   duration=int(round(1000.0 / max(fps, 1))), loop=0, optimize=True)


# --------------------------------------------------------------------------- #
# 静态图
# --------------------------------------------------------------------------- #
def make_map_png(height_map: M.HeightMap, score: np.ndarray, out_path: str,
                 dpi: int = 90) -> str:
    """左：建出来的 2.5D 高度图；右：落点评分图。"""
    plt = _pyplot()
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.6), dpi=dpi)
    extent = [height_map.cfg.origin_xy[0],
              height_map.cfg.origin_xy[0] + height_map.cfg.size_m,
              height_map.cfg.origin_xy[1],
              height_map.cfg.origin_xy[1] + height_map.cfg.size_m]
    im0 = axes[0].imshow(height_map.height, origin="lower", extent=extent,
                         cmap="terrain", vmin=0.0, vmax=max(height_map.cfg.max_height, 1.0))
    axes[0].set_title("height map from depth camera", fontsize=9)
    fig.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.03)
    im1 = axes[1].imshow(score, origin="lower", extent=extent, cmap="viridis",
                         vmin=0.0, vmax=1.0)
    axes[1].set_title("landing score (rule teacher)", fontsize=9)
    fig.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.03)
    for ax in axes:
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def make_summary_png(results: Iterable[E.LandingResult], out_path: str, dpi: int = 90) -> str:
    """三种模式在三个工况下的结果汇总：成功率 / 触地垂直速度。"""
    plt = _pyplot()
    rows = list(results)
    modes = []
    for r in rows:
        if r.mode not in modes:
            modes.append(r.mode)
    plans = []
    for r in rows:
        if r.plan not in plans:
            plans.append(r.plan)
    success = []
    vz = []
    n_safe = []
    for mode in modes:
        sel = [r for r in rows if r.mode == mode]
        # 用 LandingResult.safe 判"安全迫降"：包含触地速度与倾角包线，
        # 否则"以 9.8 m/s 砸在空地上"也算成功，看不出策略差别。
        ok = [1 if r.safe else 0 for r in sel]
        success.append(np.mean(ok) if ok else 0.0)
        # 只统计"真的安全落地"的那几次：越界工况会以 9.8 m/s 砸地，
        # 混进平均里会把两种策略之间 0.56 vs 0.05 的差别冲掉。
        good = [abs(r.touchdown_vz) for r in sel if r.safe]
        n_safe.append(len(good))
        vz.append(float(np.mean(good)) if good else 0.0)

    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.2), dpi=dpi)
    axes[0].bar(modes, success, color=[MODE_COLORS.get(m, "#4C72B0") for m in modes])
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_ylabel("safe landing rate")
    axes[0].set_title(f"safe landing rate ({len(plans)} cases per mode)", fontsize=9)
    axes[1].bar(modes, vz, color=[MODE_COLORS.get(m, "#4C72B0") for m in modes])
    axes[1].set_ylabel("touchdown |vz| (m/s)")
    axes[1].set_title("touchdown vertical speed (safe landings only)", fontsize=9)
    axes[1].set_ylim(0.0, max(vz + [0.1]) * 1.35)
    for ax, values in ((axes[0], success), (axes[1], vz)):
        for index, value in enumerate(values):
            label = f"{value:.2f}"
            if ax is axes[1]:
                label += f" (n={n_safe[index]})"
            ax.text(index, value, label, ha="center", va="bottom", fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.25, axis="y")
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path
