"""实飞演示：转弯飞行中由 IMU 估计 roll/pitch，并与真值对比。"""
from __future__ import annotations

import os
import time

import numpy as np
from PIL import Image

from .env import ImuAttEnv
from .mlp import MLP


def _write_gif(frames, path, w=320, h=240, duration=70):
    imgs = []
    for f in frames:
        im = Image.fromarray(f)
        if im.size != (w, h):
            im = im.resize((w, h), Image.LANCZOS)
        imgs.append(im.convert("P", palette=Image.ADAPTIVE, colors=128))
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=duration, loop=0, optimize=True)
    return os.path.getsize(path)


def to_gif(frames, path):
    frames = list(frames)
    size = _write_gif(frames, path)
    step = 1
    while size > 10 * 1024 * 1024 and len(frames) > 8:
        step += 1
        frames = frames[::step]
        size = _write_gif(frames, path, duration=70 * step)
    return size


def run_demo(weights="models_trained/attitude.npz", out_gif="demo.gif",
             steps=150, verbose=True):
    w = np.load(weights)
    net = MLP(n_in=30, hidden=(32,), n_out=2, seed=0)
    net.load_state({k: w[k] for k in ("W0", "b0", "W1", "b1")})
    x_mean, x_std = w["x_mean"], w["x_std"]
    y_mean, y_std = w["y_mean"], w["y_std"]

    env = ImuAttEnv()
    env.reset(0.0, 0.0, -5.0, 0.0)

    frames, errs = [], []
    try:
        for t in range(steps):
            # 螺旋航线：偏航速率随时间正弦变化，机体随之侧倾
            yaw_rate = float(0.7 * np.sin(2.0 * np.pi * t / 60.0))
            speed = 1.6
            fut = env.fly(speed, yaw_rate, -5.0, duration=0.25, wait=False)
            time.sleep(0.14)
            x = env.imu_window()
            pred = (net.forward(((x - x_mean) / x_std).astype(np.float32)) * y_std) + y_mean
            truth = np.array(env.attitude(), dtype=np.float32)
            err = np.abs(pred - truth)
            errs.append((err[0], err[1]))
            frames.append(env.scene_rgb())
            if verbose and t % 25 == 0:
                print("step %3d  真值 roll %.3f pitch %.3f | 估计 roll %.3f pitch %.3f"
                      % (t, truth[0], truth[1], pred[0], pred[1]), flush=True)
            time.sleep(0.11)
            fut.join()

        env.land()
        for _ in range(10):
            frames.append(env.scene_rgb())
    finally:
        env.close()

    size = to_gif(frames, out_gif)
    e = np.array(errs)
    stats = {"steps": len(frames) - 10, "gif_kb": size // 1024,
             "mae_roll_rad": float(e[:, 0].mean()),
             "mae_pitch_rad": float(e[:, 1].mean()),
             "mae_deg": float(np.rad2deg(e).mean())}
    if verbose:
        print("gif 写入 %s（%d KB）" % (out_gif, stats["gif_kb"]))
        print("实飞 MAE：roll %.3f rad、pitch %.3f rad（平均 %.1f 度）"
              % (stats["mae_roll_rad"], stats["mae_pitch_rad"], stats["mae_deg"]))
    return stats
