#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按轨迹帧渲染对比动图与指标图（离屏渲染，无需显示器）。"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Sequence

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .env import build_world
from .sim import clearance

PANEL_W, PANEL_H = 288, 216
FONT_CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_CJK_INDEX = 2   # NotoSansCJK 合集里第 2 个是简体中文
FONT_FALLBACK = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

MODE_COLOR = {
    "NORMAL": (54, 168, 92),
    "LINK": (54, 168, 92),
    "DEGRADED": (232, 162, 40),
    "RETURN": (206, 74, 58),
    "LAND": (58, 120, 200),
    "HOLD": (128, 128, 132),
    "HOVER": (128, 128, 132),
    "CONTACT": (200, 40, 40),
}
MODE_CN = {
    "NORMAL": "遥控掌舵",
    "LINK": "链路正常",
    "DEGRADED": "降级慢飞",
    "RETURN": "失联返航",
    "LAND": "返航点下降",
    "HOLD": "沿用最后指令",
    "HOVER": "超时悬停",
    "CONTACT": "接触障碍",
}


def _font(size: int) -> ImageFont.FreeTypeFont:
    try:
        return ImageFont.truetype(FONT_CJK, size, index=FONT_CJK_INDEX)
    except Exception:  # noqa: BLE001
        pass
    try:
        return ImageFont.truetype(FONT_FALLBACK, size)
    except Exception:  # noqa: BLE001
        return ImageFont.load_default()


def _camera(model, pos: np.ndarray, distance: float = 1.7) -> mujoco.MjvCamera:
    """机尾后上方跟拍：沿 +x 看走廊纵深。"""
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [pos[0] + 0.4, pos[1], pos[2] + 0.05]
    cam.distance = distance
    cam.elevation = -12.0
    cam.azimuth = 0.0
    return cam


def render_trace(scene, npz: Path) -> List[Image.Image]:
    """把一条轨迹的采样点逐帧渲染成图（直接放置机体，不重复积分）。"""
    data = np.load(npz, allow_pickle=False)
    pos = data["pos"]
    modes = [str(m) for m in data["mode"]]
    ages = data["age"]

    world = build_world()
    model, d = world.quad.model, world.quad.data
    renderer = mujoco.Renderer(model, PANEL_H, PANEL_W)
    qadr = model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    ]

    frames: List[Image.Image] = []
    frozen = False
    last = pos[0]
    for i, p in enumerate(pos):
        if frozen:
            p = last
        else:
            last = p
            if clearance(p, scene) < 0.10:
                frozen = True
        d.qpos[qadr + 0:qadr + 3] = p
        d.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, d)
        renderer.update_scene(d, camera=_camera(model, p))
        frames.append(Image.fromarray(renderer.render()))
    renderer.close()
    # 附带每帧元信息
    for f, m, a in zip(frames, modes, ages):
        f.info["mode"] = m
        f.info["age"] = float(a)
    return frames


def compose_panel(img: Image.Image, title: str, mode: str, age: float,
                  clr: float, t: float, subtitle: str = "") -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H + 54), (26, 28, 32))
    panel.paste(img, (0, 26))
    draw = ImageDraw.Draw(panel)
    f_title, f_small = _font(15), _font(12)
    draw.text((8, 5), title, font=f_title, fill=(236, 238, 242))
    draw.text((PANEL_W - 8 - draw.textlength(f"t={t:5.2f}s", font=f_small), 7),
              f"t={t:5.2f}s", font=f_small, fill=(200, 204, 212))
    color = MODE_COLOR.get(mode, (150, 150, 150))
    draw.rectangle([0, PANEL_H + 26, PANEL_W, PANEL_H + 26 + 26], fill=color)
    label = f"{mode} · {MODE_CN.get(mode, mode)}"
    draw.text((8, PANEL_H + 31), label, font=f_title, fill=(255, 255, 255))
    draw.text((8, PANEL_H + 26 + 28), f"命令年龄 {age:4.2f}s   净空 {clr:4.2f}m  {subtitle}",
              font=f_small, fill=(180, 186, 196))
    return panel


def build_comparison_gif(profile: str, npz_a: Path, npz_b: Path, label_a: str,
                         label_b: str, out: Path, fps: float = 12.5,
                         max_seconds: float = 16.0) -> Path:
    """左右并排：对照策略 vs 分级降级。"""
    world = build_world()
    scene = world.scene
    fa = render_trace(scene, npz_a)
    fb = render_trace(scene, npz_b)
    n = min(max(len(fa), len(fb)), int(max_seconds * 20))     # 记录频率 20 Hz

    da = np.load(npz_a, allow_pickle=False)["clearance"]
    db = np.load(npz_b, allow_pickle=False)["clearance"]
    keep = max(1, int(round(20.0 / fps)))
    frames: List[Image.Image] = []
    for i in range(0, n, keep):
        ia, ib = min(i, len(fa) - 1), min(i, len(fb) - 1)
        pa, pb = fa[ia], fb[ib]
        t = i / 20.0
        left = compose_panel(pa, label_a, pa.info["mode"], pa.info["age"],
                             float(da[min(ia, len(da) - 1)]), t)
        right = compose_panel(pb, label_b, pb.info["mode"], pb.info["age"],
                              float(db[min(ib, len(db) - 1)]), t)
        frame = Image.new("RGB", (PANEL_W * 2 + 6, PANEL_H + 54), (12, 13, 15))
        frame.paste(left, (0, 0))
        frame.paste(right, (PANEL_W + 6, 0))
        frames.append(frame.convert("P", palette=Image.ADAPTIVE, colors=128))

    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    return out


def build_metrics_figure(results: Dict[str, object], profile: str, out: Path) -> Path:
    """链路场景下三种策略的命令年龄时间线 + 典型指标柱状图。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.6), dpi=150)
    ax = axes[0]
    colors = {"hold_last": "#6b7280", "hover": "#2f6fd0", "supervised": "#d97706"}
    for policy in ("hold_last", "hover", "supervised"):
        npz = Path(results["outdir"]) / f"trace_{profile}_{policy}_s0.npz"
        if not npz.exists():
            continue
        d = np.load(npz, allow_pickle=False)
        ax.plot(d["t"], np.clip(d["age"], 0, 3.0), lw=1.1, color=colors[policy],
                label={"hold_last": "沿用最后指令", "hover": "超时悬停",
                       "supervised": "分级降级"}[policy])
    ax.axhline(0.35, ls="--", lw=0.8, color="#b45309")
    ax.axhline(1.20, ls=":", lw=0.8, color="#b91c1c")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("命令年龄 / s")
    ax.set_title("指令年龄（虚线=降级阈值，点线=失联阈值）")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(alpha=0.25)

    ax = axes[1]
    names = {"hold_last": "沿用最后指令", "hover": "超时悬停", "supervised": "分级降级"}
    keys = ["hold_last", "hover", "supervised"]
    vals = []
    for k in keys:
        row = results["aggregate"].get(f"{profile}|{k}", {})
        vals.append(float(row.get("min_clearance_m", 0.0)))
    bars = ax.bar([names[k] for k in keys], vals,
                  color=[colors[k] for k in keys], width=0.62)
    ax.axhline(0.10, ls="--", lw=0.8, color="#b91c1c")
    ax.set_ylabel("最小净空 / m")
    ax.set_title("全程最小障碍净空（虚线=接触判据 0.10 m）")
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.01, f"{v:.2f}",
                ha="center", fontsize=8)
    ax.set_ylim(0, max(vals + [0.2]) * 1.2)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return out
