"""实飞演示：注入风，网络补偿，录真实画面 gif，并与无补偿对照。"""
from __future__ import annotations

import os
import time

import numpy as np
from PIL import Image

from .env import CMD_SCALE, WindEnv
from .mlp import MLP

TARGET = np.array([0.0, 0.0, -5.0])


def _write_gif(frames, path, w=320, h=240, duration=60):
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
        size = _write_gif(frames, path, duration=60 * step)
    return size


def _load(weights):
    net = MLP(n_in=32, hidden=(32,), n_out=2)
    net.load(weights)
    d = np.load(os.path.join(os.path.dirname(weights), "wind_data.npz"))
    return net, d["x_mean"], d["x_std"]


def run_demo(weights="models_trained/wind.npz", out_gif="demo.gif", steps=160,
             wind=(6.0, 0.0), verbose=True):
    net, mean, std = _load(weights)
    env = WindEnv()
    env.reset(0.0, 0.0, -5.0, 0.0)
    env.set_wind(wind[0], wind[1], 0.0)

    frames, errs, devs = [], [], []
    p_prev, _ = env.pose()
    try:
        for t in range(steps):
            # 专家只用于对比记录，实际控制完全来自网络
            p_now, _ = env.pose()
            err = TARGET[:2] - p_now[:2]
            x = np.concatenate([env.imu_window(), err.astype(np.float32)])
            x = ((x - mean) / std).astype(np.float32)
            act = net.forward(x) * CMD_SCALE

            env.apply_velocity(act[0], act[1], TARGET[2], duration=0.25, wait=False)
            time.sleep(0.12)
            p_after, _ = env.pose()
            errs.append(float(np.linalg.norm(p_after[:2] - TARGET[:2])))
            devs.append((p_after[0], p_after[1]))
            frames.append(env.scene_rgb())
            if verbose and t % 30 == 0:
                print("demo step %3d  补偿 (%.2f, %.2f)  偏移 %.2f m  位置 (%.2f %.2f %.2f)"
                      % (t, act[0], act[1], errs[-1], p_after[0], p_after[1], p_after[2]),
                      flush=True)
            time.sleep(0.13)
        env.land()
    finally:
        env.close()

    size = to_gif(frames, out_gif)
    e = np.array(errs)
    stats = {
        "steps": len(frames),
        "mean_dev": float(e.mean()),
        "max_dev": float(e.max()),
        "final_dev": float(e[-20:].mean()),
        "gif_kb": size // 1024,
        "wind": wind,
    }
    if verbose:
        print("gif 写入 %s（%d KB）" % (out_gif, stats["gif_kb"]))
        print("风速 %s m/s，平均偏移 %.2f m，最大 %.2f m，末段 %.2f m"
              % (wind, stats["mean_dev"], stats["max_dev"], stats["final_dev"]))
    return stats


def run_baseline(steps=160, wind=(6.0, 0.0), verbose=True):
    """不开网络、只靠 AirSim 自身速度环的对照。"""
    env = WindEnv()
    env.reset(0.0, 0.0, -5.0, 0.0)
    env.set_wind(wind[0], wind[1], 0.0)
    devs = []
    try:
        for t in range(steps):
            env.apply_velocity(0.0, 0.0, TARGET[2], duration=0.25, wait=False)
            time.sleep(0.25)
            p, _ = env.pose()
            devs.append(float(np.linalg.norm(p[:2] - TARGET[:2])))
        env.land()
    finally:
        env.close()
    d = np.array(devs)
    stats = {"mean_dev": float(d.mean()), "max_dev": float(d.max())}
    if verbose:
        print("对照组（无网络补偿）：平均偏移 %.2f m，最大 %.2f m"
              % (stats["mean_dev"], stats["max_dev"]))
    return stats
