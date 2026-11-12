"""实飞演示：边飞边用网络估速度大小，录真实画面 gif，并统计估计误差。"""
from __future__ import annotations

import os
import time

import numpy as np
from PIL import Image

from .env import VEL_SCALE, ImuVelEnv
from .mlp import MLP


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


def _load_model(weights):
    net = MLP(n_in=30, hidden=(32,), n_out=1)
    net.load(weights)
    d = np.load(os.path.join(os.path.dirname(weights), "imu_vel_data.npz"))
    return net, d["x_mean"], d["x_std"]


def run_demo(weights="models_trained/imu_vel.npz", out_gif="demo.gif",
             steps=180, z0=-5.0, verbose=True):
    net, mean, std = _load_model(weights)

    env = ImuVelEnv()
    env.reset(-24.0, 0.0, z0, 0.0)

    frames, ests, truths, z_ref = [], [], [], z0
    rng = np.random.RandomState(99)
    forward, yaw_rate = 2.0, 0.0
    try:
        for t in range(steps):
            # 与采集阶段同样的随机指令分布，保证 demo 不出训练分布
            if t % 8 == 0:
                forward = float(rng.uniform(0.3, 3.2))
                yaw_rate = float(rng.uniform(-0.8, 0.8))

            env.fly(forward, yaw_rate, z_ref, duration=0.25, wait=False)
            time.sleep(0.12)

            x = ((env.imu_window() - mean) / std).astype(np.float32)
            pred = float(net.forward(x)[0]) * VEL_SCALE
            truth = float(np.linalg.norm(env.velocity()))
            ests.append(pred)
            truths.append(truth)
            frames.append(env.scene_rgb())

            if verbose and t % 30 == 0:
                p, _ = env.pose()
                print("demo step %3d  估计 %.2f m/s  真值 %.2f m/s  位置 (%.1f %.1f %.1f)"
                      % (t, pred, truth, p[0], p[1], p[2]), flush=True)
            time.sleep(0.13)
        env.land()
    finally:
        env.close()

    size = to_gif(frames, out_gif)
    e = np.array(ests)
    g = np.array(truths)
    stats = {
        "steps": len(frames),
        "rmse_ms": float(np.sqrt(np.mean((e - g) ** 2))),
        "mae_ms": float(np.mean(np.abs(e - g))),
        "gif_kb": size // 1024,
    }
    if verbose:
        print("gif 写入 %s（%d KB）" % (out_gif, stats["gif_kb"]))
        print("速度大小估计 RMSE %.3f m/s，MAE %.3f m/s"
              % (stats["rmse_ms"], stats["mae_ms"]))
    return stats, e, g


def run_eval(weights="models_trained/imu_vel.npz", steps=150, verbose=True):
    net, mean, std = _load_model(weights)
    env = ImuVelEnv()
    env.reset(-24.0, 0.0, -5.0, 0.0)

    rng = np.random.RandomState(7)
    errs = []
    z_ref = -5.0
    try:
        for t in range(steps):
            forward = float(rng.uniform(0.3, 3.0)) if t % 10 == 0 else 2.0
            yaw_rate = float(rng.uniform(-0.7, 0.7)) if t % 10 == 0 else 0.0
            env.fly(forward, yaw_rate, z_ref, duration=0.25, wait=False)
            time.sleep(0.12)
            x = ((env.imu_window() - mean) / std).astype(np.float32)
            pred = float(net.forward(x)[0]) * VEL_SCALE
            truth = float(np.linalg.norm(env.velocity()))
            errs.append(abs(pred - truth))
            time.sleep(0.13)
        env.land()
    finally:
        env.close()

    mae = float(np.mean(errs))
    if verbose:
        print("评估：速度大小 MAE = %.3f m/s（%d 帧）" % (mae, len(errs)))
    return {"mae_ms": mae, "frames": len(errs)}
