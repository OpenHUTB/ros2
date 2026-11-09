"""实飞演示：飞向墙，网络预测 TTC，gif 记录真实画面。"""
from __future__ import annotations

import os
import time

import numpy as np
from PIL import Image

from .cnn import DepthPolicyNet
from .env import TTC_MAX, TtcEnv


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


def run_demo(weights="models_trained/ttc.npz", out_gif="demo.gif", steps=120,
             speed=1.6, verbose=True):
    net = DepthPolicyNet.load(weights)

    env = TtcEnv()
    env.reset(-14.0, 0.0, -4.0, 0.0)
    frames, preds, truths = [], [], []
    p_prev, _ = env.pose()
    stopped = False

    try:
        for t in range(steps):
            d = env.depth()
            raw = env.depth_raw()
            min_d = float(np.min(raw))
            p_now, _ = env.pose()
            vel = (p_now - p_prev) / 0.25
            p_prev = p_now
            v_forward = max(0.35, float(np.hypot(vel[0], vel[1])))
            true_ttc = min(min_d / v_forward, TTC_MAX)

            pred_ttc = float(net.forward(d)[0]) * TTC_MAX
            preds.append(pred_ttc)
            truths.append(true_ttc)
            frames.append(env.scene_rgb())

            if verbose and t % 20 == 0:
                print("step %3d  最近距离 %.2f m  速度 %.2f m/s  真值 TTC %.1f s  预测 %.1f s"
                      % (t, min_d, v_forward, true_ttc, pred_ttc), flush=True)

            # 安全策略：预测 TTC 低于 2.5 s 或真实距离低于 1.2 m 就停
            if pred_ttc < 1.8 or min_d < 1.0:
                stopped = True
                if verbose:
                    print("触发安全停：预测 TTC %.1f s，距离 %.2f m" % (pred_ttc, min_d), flush=True)
                for _ in range(8):
                    frames.append(env.scene_rgb())
                break

            env.fly_forward(speed, -4.0, duration=0.25, wait=False)
            time.sleep(0.17)

        env.land()
    finally:
        env.close()

    size = to_gif(frames, out_gif)
    p = np.array(preds)
    gt = np.array(truths)
    stats = {
        "steps": len(preds),
        "gif_kb": size // 1024,
        "mae_s": float(np.mean(np.abs(p - gt))),
        "min_true_ttc": float(gt.min()),
        "stopped": stopped,
    }
    if verbose:
        print("gif 写入 %s（%d KB）" % (out_gif, stats["gif_kb"]))
        print("预测 MAE %.2f s，最小真值 TTC %.1f s，安全停触发 %s"
              % (stats["mae_s"], stats["min_true_ttc"], stopped))
    return stats
