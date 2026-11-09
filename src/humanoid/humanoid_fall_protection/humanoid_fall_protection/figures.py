# -*- coding: utf-8 -*-
"""figures.py —— 曲线、柱状图与关键帧渲染

图表里的文字统一用英文：仓库的文档构建环境不一定装了中文字体，
matplotlib 缺字体会把中文画成方块，反而更难读。
"""

import os

import numpy as np

#: 三种策略的绘图配色（与文档中的图例一致）
COLORS = {
    "rigid": "#4C78A8",
    "limp": "#E45756",
    "protect": "#54A24B",
}

LABELS = {
    "rigid": "rigid (frozen pose)",
    "limp": "limp (zero torque)",
    "protect": "protect (crouch)",
}


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_head_height(recorders, path):
    """头部球心高度随时间变化，三种策略叠在一张图里。"""
    plt = _plt()
    fig, ax = plt.subplots(figsize=(7.2, 3.6), dpi=140)
    for strategy, recorder in recorders.items():
        rows = np.array(recorder.rows, dtype=float)
        ax.plot(rows[:, 0], rows[:, 3], color=COLORS.get(strategy, "k"), lw=1.6,
                label=LABELS.get(strategy, strategy))
    ax.axhline(0.09, color="#888", ls="--", lw=1.0)
    ax.text(3.45, 0.115, "head radius 0.09 m", fontsize=8, color="#666", ha="right")
    ax.axvspan(1.0, 1.08, color="#f2c14e", alpha=0.25, lw=0)
    ax.text(1.0, 1.45, "push 300 N", fontsize=8, color="#8a6d1f")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("head centre height (m)")
    ax.set_xlim(0, 3.5)
    ax.set_ylim(0, 1.75)
    ax.grid(alpha=0.25, lw=0.6)
    ax.legend(fontsize=8, loc="upper right", frameon=False)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_impacts(summaries, path):
    """三个部位的峰值接触力分组柱状图。"""
    plt = _plt()
    parts = [("head_peak_force", "head"), ("torso_peak_force", "torso"),
             ("knee_peak_force", "knee")]
    strategies = [s["strategy"] for s in summaries]
    x = np.arange(len(parts))
    width = 0.26

    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=140)
    for i, summary in enumerate(summaries):
        values = [summary[key] for key, _ in parts]
        ax.bar(x + (i - 1) * width, values, width,
               color=COLORS.get(summary["strategy"], "k"),
               label=LABELS.get(summary["strategy"], summary["strategy"]))
    ax.set_xticks(x)
    ax.set_xticklabels([name for _, name in parts])
    ax.set_ylabel("peak contact force (N)")
    ax.set_yscale("symlog", linthresh=10)
    ax.grid(axis="y", alpha=0.25, lw=0.6)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    for i, summary in enumerate(summaries):
        for j, (key, _) in enumerate(parts):
            value = summary[key]
            ax.text(j + (i - 1) * width, max(value, 3) * 1.25, "{:.0f}".format(value),
                    ha="center", fontsize=7, color="#333")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def _camera():
    import mujoco
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.42, 0.0, 0.38]
    cam.distance = 2.55
    cam.azimuth = 92
    cam.elevation = -6
    return cam


def render_frames(strategy, times, out_dir, model_path=None, width=640, height=420):
    """在指定时刻渲染关键帧，返回图片路径列表。"""
    import mujoco
    from PIL import Image

    from .protection import FallController
    from .sim_core import FallSim

    os.makedirs(out_dir, exist_ok=True)
    sim = FallSim(model_path)
    sim.reset()
    controller = FallController()
    controller.reset(strategy)

    renderer = mujoco.Renderer(sim.model, height, width)
    cam = _camera()
    shots = []
    targets = sorted(times)
    index = 0
    total = int(max(targets) / sim.dt) + 5
    for i in range(total):
        t = i * sim.dt
        sim.apply_push(300.0, 1.0 <= t < 1.08)
        controller.step(sim, t)
        sim.step()
        if index < len(targets) and t >= targets[index]:
            renderer.update_scene(sim.data, cam)
            frame = renderer.render()
            path = os.path.join(out_dir, "{}_t{:.2f}.png".format(strategy, targets[index]))
            Image.fromarray(frame).save(path)
            shots.append(path)
            index += 1
    renderer.close()
    return shots


def render_gif(strategy, out_path, duration=3.2, fps=25, model_path=None,
               width=520, height=340, every=1):
    """整段倒地过程渲染成 GIF。"""
    import mujoco
    from PIL import Image

    from .protection import FallController
    from .sim_core import FallSim

    sim = FallSim(model_path)
    sim.reset()
    controller = FallController()
    controller.reset(strategy)

    renderer = mujoco.Renderer(sim.model, height, width)
    cam = _camera()
    frames = []
    steps = int(duration / sim.dt)
    interval = int(round(1.0 / (fps * sim.dt)))
    for i in range(steps):
        t = i * sim.dt
        sim.apply_push(300.0, 1.0 <= t < 1.08)
        controller.step(sim, t)
        sim.step()
        if i % interval == 0:
            renderer.update_scene(sim.data, cam)
            frames.append(Image.fromarray(renderer.render()))
    renderer.close()

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    # 调色板量化：MuJoCo 渲染是平滑渐变，直接存 24 位会让 GIF 超过 10 MB
    frames = [f.convert("P", palette=Image.ADAPTIVE, colors=64) for f in frames]
    frames[0].save(out_path, save_all=True, append_images=frames[1:], duration=int(1000 / fps),
                   loop=0, optimize=True)
    return out_path, len(frames)


def montage(paths, out_path, cols=3):
    """把多张关键帧拼成一张对比图。"""
    from PIL import Image

    images = [Image.open(p) for p in paths]
    if not images:
        return None
    width, height = images[0].size
    rows = (len(images) + cols - 1) // cols
    canvas = Image.new("RGB", (width * cols, height * rows), (255, 255, 255))
    for i, image in enumerate(images):
        canvas.paste(image, ((i % cols) * width, (i // cols) * height))
    canvas.save(out_path)
    return out_path
