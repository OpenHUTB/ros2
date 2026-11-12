"""演示动图：把"故障注入 → 网络给出健康度估计"画成随时间推进的曲线。

两个子图：

* 左：四个桨的真实增益（实线）与网络估计（虚线），故障桨会被标出来；
* 右：飞行高度，用来解释"为什么必须在坠地之前就检测出来"。

图里的文字一律用 ASCII——Matplotlib 默认字体没有中文字形，写中文会变成方框。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .data import FAULT_CN, FaultPlan, simulate_episode
from .infer import estimate_sequence
from .metrics import DETECT_GAIN_THRESHOLD

#: 动图尺寸（作业要求 gif 一般 < 5 MB，这个尺寸下 80 帧约 0.3 MB）
FIG_WIDTH = 9.0
FIG_HEIGHT = 3.4
DPI = 80


def _prepare_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def make_detection_gif(
    checkpoint: str | Path,
    out_path: str | Path,
    kind: str = "loss",
    rotor: int = 0,
    onset: float = 2.0,
    severity: float = 1.0,
    scene: str = "open_ground",
    seed: int = 0,
    duration: float = 6.0,
    dt: float = 0.01,
    substeps: int = 4,
    frame_every: int = 10,
    fps: int = 10,
    quantize_colors: int = 96,
    api=None,
) -> Path:
    """跑一次带故障的飞行，边飞边估，输出 gif。"""
    from ._compat import load_base

    plt = _prepare_matplotlib()
    api = api or load_base()

    plan = FaultPlan(kind=kind, rotor=rotor, onset=onset, severity=severity)
    episode = simulate_episode(
        api,
        plan,
        scene=scene,
        seed=seed,
        duration=duration,
        dt=dt,
        substeps=substeps,
    )

    predictions, ends = estimate_sequence(checkpoint, episode.features, stride=frame_every)
    if predictions.shape[0] < 2:
        raise RuntimeError("估计样本太少，无法生成动图（把 duration 调长一点）")

    truth = episode.gains[ends]
    times = ends.astype(np.float64) * dt
    altitude = episode.altitude[ends]
    crashed = episode.crashed
    crash_time = float(episode.crash_time) if crashed else None

    colors = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd"]
    plt.rcParams["font.size"] = 9
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(FIG_WIDTH, FIG_HEIGHT), dpi=DPI)
    fig.subplots_adjust(left=0.06, right=0.985, top=0.86, bottom=0.14, wspace=0.18)

    # 曲线从第一个可用窗口开始（t≈0.99 s），横轴跟着裁剪，免得左边留一大片空白
    x_start = float(times[0]) if times[0] > 0 else 0.0
    x_end = float(times[-1]) if times[-1] > x_start else x_start + 1.0

    ax1.set_xlim(x_start, x_end)
    ax1.set_ylim(-0.08, 1.12)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("rotor health  g")
    ax1.grid(alpha=0.25)
    ax1.set_title("rotor health: truth (solid) vs estimate (dashed)", fontsize=9)

    truth_lines = []
    est_lines = []
    for i in range(4):
        line, = ax1.plot([], [], color=colors[i], lw=2.2 if i == rotor else 1.2,
                         label=f"r{i} truth")
        truth_lines.append(line)
    for i in range(4):
        line, = ax1.plot([], [], color=colors[i], lw=1.4, ls="--", alpha=0.9,
                         label=f"r{i} est")
        est_lines.append(line)

    onset_line = ax1.axvline(onset, color="k", ls=":", lw=1.2)
    onset_line.set_visible(False)
    detect_line = ax1.axvline(0.0, color="crimson", ls="-.", lw=1.4)
    detect_line.set_visible(False)

    status = ax1.text(0.015, 0.05, "", transform=ax1.transAxes, fontsize=9,
                      color="black", va="bottom",
                      bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.6", alpha=0.85))

    # 右图：高度曲线——说明"故障后留给检测器的时间有多短"
    ax2.set_xlim(x_start, x_end)
    ax2.set_ylim(0.0, max(2.0, float(np.nanmax(altitude)) * 1.15))
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("altitude (m)")
    ax2.grid(alpha=0.25)
    ax2.set_title(f"altitude  (crash at {crash_time:.2f}s)" if crashed else "altitude",
                  fontsize=9)
    alt_line, = ax2.plot([], [], color="#2c7fb8", lw=1.8)
    ax2.set_xlim(x_start, x_end)
    crash_line = ax2.axvline(crash_time, color="black", ls="--", lw=1.2) if crashed else None
    detect2 = ax2.axvline(0.0, color="crimson", ls="-.", lw=1.4)
    detect2.set_visible(False)
    if crashed:
        ax2.axhline(0.15, color="grey", lw=0.8, ls=":")

    def draw(frame: int):
        curve_time = times[: frame + 1]
        for i in range(4):
            truth_lines[i].set_data(curve_time, truth[: frame + 1, i])
            est_lines[i].set_data(curve_time, predictions[: frame + 1, i])

        alt_line.set_data(curve_time, altitude[: frame + 1])

        current_t = float(times[frame])
        onset_line.set_visible(current_t >= onset)

        detected_frame = None
        for i in range(frame + 1):
            if predictions[i, rotor] < DETECT_GAIN_THRESHOLD and times[i] >= onset:
                detected_frame = i
                break
        if detected_frame is not None:
            x = float(times[detected_frame])
            detect_line.set_visible(True)
            detect_line.set_xdata([x, x])
            detect2.set_visible(True)
            detect2.set_xdata([x, x])
            delay = x - onset
            status.set_text(f"t={current_t:4.1f}s   detected at +{delay:4.2f}s")
            status.set_color("crimson")
        else:
            detect_line.set_visible(False)
            detect2.set_visible(False)
            label = "healthy" if current_t < onset else "fault injected, watching..."
            status.set_text(f"t={current_t:4.1f}s   {label}")
            status.set_color("black")

        artists = truth_lines + est_lines + [alt_line, onset_line, detect_line, detect2, status]
        if crash_line is not None:
            artists.append(crash_line)
        return artists

    from matplotlib.animation import FuncAnimation

    animation = FuncAnimation(fig, draw, frames=len(times), interval=1000 // fps, blit=False)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = out_path.with_suffix(".tmp.gif")
    animation.save(str(tmp_path), writer="pillow", fps=fps)
    plt.close(fig)

    _shrink_gif(tmp_path, out_path, quantize_colors)
    if tmp_path.exists():
        tmp_path.unlink()
    return out_path


def _shrink_gif(src: Path, dst: Path, colors: int) -> None:
    """逐帧调色板量化，把 gif 体积压到作业要求的量级。"""
    try:
        from PIL import Image
    except ImportError:  # pragma: no cover
        src.replace(dst)
        return

    with Image.open(src) as image:
        frames = []
        durations = []
        for index in range(image.n_frames):
            image.seek(index)
            frame = image.convert("RGB")
            frames.append(frame.quantize(colors=colors, method=Image.MEDIANCUT))
            durations.append(image.info.get("duration", 100))

    frames[0].save(
        dst,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        optimize=True,
        disposal=2,
    )


def describe_plan(kind: str, rotor: int, onset: float, severity: float) -> str:
    """给终端打印一句人类可读的描述。"""
    name = FAULT_CN.get(kind, kind)
    return f"{name}(桨{rotor}) 在 {onset:.1f}s 注入，severity={severity:.2f}"
