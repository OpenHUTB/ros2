# -*- coding: utf-8 -*-
"""录制演示动图：把"健康悬停 vs 单桨失效"并排录成一张 gif。

作业对 gif 有硬约束（一般 <5 MB，上限 10 MB），所以这里做三件事控制体积：
一是默认 320×240 小分辨率、10 fps；二是交给 Pillow 做自适应调色板量化；
三是帧数按时长上限截断。实测 640×240、6 s、10 fps 的对比图约 1.5 MB。
"""

from __future__ import annotations

import os
from typing import List, Optional

import numpy as np

from .runner import RunConfig, run_case
from .scenario import make_case

#: 顶部标签条高度（像素）
BAR_HEIGHT = 18


def _annotate(frame: np.ndarray, text: str, rgb: tuple) -> np.ndarray:
    """在帧顶部画一条彩色标签条并写 ASCII 文字。"""
    from PIL import Image, ImageDraw, ImageFont

    img = Image.fromarray(frame).convert("RGB")
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, img.width, BAR_HEIGHT], fill=rgb)
    try:
        font = ImageFont.load_default()
    except OSError:  # pragma: no cover - 极简 Pillow 构建
        font = None
    draw.text((4, 4), text, fill=(255, 255, 255), font=font)
    return np.asarray(img)


def make_comparison_gif(
    out_path: str,
    scene: str = "open_ground",
    seed: int = 0,
    onset: float = 2.0,
    rotor: int = 0,
    seconds: float = 6.0,
    width: int = 320,
    height: int = 240,
    fps: int = 10,
    quality: str = "full",
    camera: str = "track",
) -> str:
    """录一张"健康 / 单桨失效"并排对比 gif，返回文件路径。

    Parameters
    ----------
    out_path:
        输出 ``.gif`` 路径，父目录不存在会自动创建。
    seconds:
        录制时长，秒。超过 8 s 意义不大（坠机发生在 0.6 s 内）。
    quality:
        ``full`` 开阴影反射（好看、慢），``fast`` 关掉（快 6 倍）。
    camera:
        ``track`` 外部跟拍（默认，能看见姿态与坠地），``fpv`` 机载前视。
    """
    from PIL import Image

    dt = 0.01
    frame_every = max(1, int(round(1.0 / (fps * dt))))
    cfg = RunConfig(
        duration=seconds,
        dt=dt,
        record_frames=True,
        frame_every=frame_every,
        frame_width=width,
        frame_height=height,
        quality=quality,
        camera=camera,
    )

    healthy = run_case(make_case(scene, "healthy", seed=seed, rotor=rotor, onset=onset), cfg=cfg)
    faulty = run_case(make_case(scene, "single_loss", seed=seed, rotor=rotor, onset=onset), cfg=cfg)

    n = min(len(healthy.frames), len(faulty.frames))
    if n == 0:
        raise RuntimeError("没有采到任何帧，检查 record_frames 与场景设置")

    # 故障侧坠地后再留 0.6 s 就收尾：再往后机身已经穿到地面以下，
    # 画面只会变成一片黑，留着反而降低可读性。
    if faulty.crash_delay is not None and faulty.onset is not None:
        keep = faulty.onset + faulty.crash_delay + 0.6
        n = min(n, max(1, int(round(keep / (frame_every * dt)))))

    def _hud(frame, label, rgb, t, alt):
        # 顶部条同时给时间与高度：坠机时无人机穿出画面，读数仍在说话
        return _annotate(frame, "%s  t=%4.1fs  alt=%+7.2fm" % (label, t, alt), rgb)

    left, right = [], []
    for i in range(n):
        k = i * frame_every
        t = k * dt
        left.append(_hud(healthy.frames[i], "HEALTHY", (30, 120, 60),
                         t, float(healthy.pos[k, 2])))
        right.append(_hud(faulty.frames[i], "ROTOR %d LOST %.1fs" % (rotor, onset),
                          (160, 40, 40), t, float(faulty.pos[k, 2])))

    panels = [np.concatenate([a, b], axis=1) for a, b in zip(left, right)]

    parent = os.path.dirname(os.path.abspath(out_path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    images = [Image.fromarray(p) for p in panels]
    # 用自适应调色板量化，体积能压到真彩的 1/5 左右
    images[0].save(
        out_path,
        save_all=True,
        append_images=images[1:],
        duration=int(round(1000.0 / fps)),
        loop=0,
        optimize=True,
    )
    return out_path


def make_trace_png(out_path: str, results, title: str = "Altitude vs time") -> Optional[str]:
    """画一张高度-时间曲线（可选，需要 matplotlib）。"""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    parent = os.path.dirname(os.path.abspath(out_path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=160)
    for r in results:
        label = r.scenario + ("" if r.onset is None else f" (t0={r.onset:.1f}s)")
        ax.plot(r.t, r.pos[:, 2], linewidth=1.4, label=label)
    ax.axhline(0.0, color="0.3", linewidth=0.8, linestyle="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("altitude (m)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return out_path
