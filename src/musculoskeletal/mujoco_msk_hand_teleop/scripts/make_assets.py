#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成项目素材：3 个场景演示 GIF + 任务性能图表（无 ROS，无头渲染）。

输出（源码树写入 docs/img/，安装空间回退 assets/）：
  msk_hand_demo_button.gif / msk_hand_demo_ball.gif / msk_hand_demo_free.gif
  msk_hand_task_metrics.png   按键深度-时间 / 球高-时间 / NN 到达误差-时间
  nn_training.png    由 msk_hand_nn.py 生成（若缺失则跳过）
"""
import os
import sys

import numpy as np
import imageio
import mujoco
from PIL import Image, ImageDraw

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from msk_hand_core import MskHandSim, N_MUSCLE, PKG_DIR  # noqa: E402
from msk_hand_keyboard import KeyboardPolicy    # noqa: E402

ASSET_DIR = os.path.join(PKG_DIR, "assets")
# 文档图片写入 docs/img/（与 mkdocs 引用一致）；安装空间无 docs/ 时回退 assets/
DOCS_IMG_DIR = os.path.normpath(
    os.path.join(PKG_DIR, os.pardir, os.pardir, os.pardir, "docs", "img"))
OUT_DIR = DOCS_IMG_DIR if os.path.isdir(DOCS_IMG_DIR) else ASSET_DIR
PREFIX = "msk_hand_" if OUT_DIR == DOCS_IMG_DIR else ""
W, H, FPS = 480, 360, 20

# 各场景相机参数（方位角/距离/仰角/注视点，取自离线调参）
CAMS = {
    "button": dict(az=130, dist=0.38, elev=-8, lookat=(0.017, 0.15, 0.10)),
    "ball":   dict(az=100, dist=0.40, elev=-10, lookat=(0.017, 0.15, 0.08)),
    "free":   dict(az=130, dist=0.50, elev=-12, lookat=(0.010, 0.20, 0.12)),
}
# GIF 叠加用英文标签（PIL 默认字体无 CJK 字形）
LABELS = {"button": "BUTTON", "ball": "BALL", "free": "NN-REACH"}


def run_keys(sim, script, capture_every=2):
    """按 KeyboardPolicy 脚本驱动仿真，逐步产出 (t, 帧数据, 指标)。"""
    policy = KeyboardPolicy()
    dt = 0.01
    for key, hold in script:
        if key:
            policy.toggle(key)
        for _ in range(max(1, int(hold / dt))):
            cmd = policy.update(dt)
            sim.step(cmd)
            yield sim.time, cmd


def run_nn(sim, targets, net, x_mean, x_std, hold=1.2, settle_hint=0.4,
           capture_every=2):
    """NN 逆模型依次到达多个目标点。"""
    dt = 0.01
    for tgt in targets:
        sim.set_target_marker(tgt)
        x = (np.asarray(tgt)[None] - x_mean) / x_std
        a, _ = net.forward(x)
        a = np.clip(a[0], 0.0, 1.0)
        for _ in range(int(hold / dt)):
            sim.step(a)
            yield sim.time, a


def make_camera(sim, cfg):
    cam = mujoco.MjvCamera()
    cam.azimuth = cfg["az"]
    cam.distance = cfg["dist"]
    cam.elevation = cfg["elev"]
    cam.lookat[:] = cfg["lookat"]
    return cam


def render_gif(sim, scene, frames_iter, metric_fn, path, capture_every=2):
    """边仿真边渲染帧并叠加文字，输出 GIF；同时返回时间序列。"""
    renderer = mujoco.Renderer(sim.model, height=H, width=W)
    cam = make_camera(sim, CAMS[scene])
    ts, metrics, imgs = [], [], []
    for step, (t, cmd) in enumerate(frames_iter):
        if step % capture_every == 0:
            renderer.update_scene(sim.data, camera=cam)
            img = renderer.render()
            overlay = "%s  t=%.2fs  %s" % (LABELS[scene], t, metric_fn())
            imgs.append(_draw_text(img, overlay))
            ts.append(t)
            metrics.append(metric_fn(raw=True))
    renderer.close()
    imageio.mimsave(path, imgs, fps=FPS, loop=0)
    size_mb = os.path.getsize(path) / 1e6
    print("GIF %s: %d 帧, %.2f MB" % (os.path.basename(path), len(imgs), size_mb))
    return np.array(ts), np.array(metrics)


def _draw_text(img, text):
    pil = Image.fromarray(img)
    d = ImageDraw.Draw(pil)
    d.rectangle([4, 4, 4 + 7 * len(text) + 8, 20], fill=(0, 0, 0))
    d.text((8, 7), text, fill=(255, 255, 255))
    return np.asarray(pil)


# ------------------------------------------------------------------ #
def scene_button():
    sim = MskHandSim("button")
    sim.reset()

    def metric(raw=False):
        d = sim.button_depth()
        return ("depth=%.1fmm" % (d * 1e3) if not raw else d)

    script = [("u", 0.5), ("a", 1.4), ("a", 0.0), ("j", 0.7), ("", 0.3)]
    ts, ms = render_gif(sim, "button", run_keys(sim, script), metric,
                        os.path.join(OUT_DIR, PREFIX + "demo_button.gif"))
    ok = ms.max() >= 0.005
    print("  按键最大深度 %.1fmm，行程达标: %s" % (ms.max() * 1e3, ok))
    return ts, ms


def scene_ball():
    sim = MskHandSim("ball")
    sim.reset()

    def metric(raw=False):
        z = float(sim.ball_pos()[2])
        return ("ball z=%.1fmm" % (z * 1e3) if not raw else z)

    script = [("u", 0.5), ("a", 1.4), ("a", 0.0), ("j", 0.7), ("", 0.4)]
    ts, ms = render_gif(sim, "ball", run_keys(sim, script), metric,
                        os.path.join(OUT_DIR, PREFIX + "demo_ball.gif"))
    off = ms.min() < 0.030
    print("  球最终 z=%.1fmm，离台成功: %s" % (ms.min() * 1e3, off))
    return ts, ms


def scene_free():
    from msk_hand_nn import MLP, collect
    sim = MskHandSim("free")
    sim.reset()
    net, x_mean, x_std = MLP.load(os.path.join(ASSET_DIR, "nn_weights.npz"))
    # 从测试集挑 3 个分散目标（与训练评估一致的数据流）
    X_te, _ = collect(80, seed=2)
    picks = [int(i * (len(X_te) - 1) / 2.999) for i in range(3)]
    targets = X_te[picks]

    state = {"tgt": targets[0], "i": 0}

    def metric(raw=False):
        e = float(np.linalg.norm(sim.index_tip() - state["tgt"]))
        return ("err=%.1fmm" % (e * 1e3) if not raw else e)

    def frames():
        dt = 0.01
        for i, tgt in enumerate(targets):
            # 与评估协议一致：每次到达从静息位姿 reset 开始
            # （连续切换时屈曲手指可能被接触卡住，存在迟滞）
            sim.reset()
            state["tgt"], state["i"] = tgt, i
            sim.set_target_marker(tgt)
            x = (np.asarray(tgt)[None] - x_mean) / x_std
            a, _ = net.forward(x)
            a = np.clip(a[0], 0.0, 1.0)
            for _ in range(int(1.2 / dt)):
                sim.step(a)
                yield sim.time, a

    ts, ms = render_gif(sim, "free", frames(), metric,
                        os.path.join(OUT_DIR, PREFIX + "demo_free.gif"))
    print("  稳态到达误差 %.1fmm" % (ms[-20:].mean() * 1e3))
    return ts, ms


def make_chart(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.2))
    ts, ms = res[0]
    axes[0].plot(ts, ms * 1e3)
    axes[0].axhline(6.0, ls="--", c="r", label="travel 6mm")
    axes[0].set(title="Button depth", xlabel="t (s)", ylabel="depth (mm)")
    axes[0].legend()
    ts, ms = res[1]
    axes[1].plot(ts, ms * 1e3)
    axes[1].axhline(40.0, ls="--", c="g", label="pedestal top")
    axes[1].set(title="Ball height", xlabel="t (s)", ylabel="z (mm)")
    axes[1].legend()
    ts, ms = res[2]
    axes[2].plot(ts, ms * 1e3)
    axes[2].set(title="NN reach error", xlabel="t (s)", ylabel="err (mm)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, PREFIX + "task_metrics.png"), dpi=130)
    print("图表已保存 task_metrics.png")


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    res = [scene_button(), scene_ball(), scene_free()]
    make_chart(res)
    print("完成")
