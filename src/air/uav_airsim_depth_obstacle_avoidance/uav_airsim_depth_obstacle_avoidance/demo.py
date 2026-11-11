"""演示与评估：让训练好的网络在 AirSim 里实飞，并把真实画面录成 gif。

画面全部来自 AirSim 的 Scene 相机（``simGetImages``），不是渲染示意图。
"""
from __future__ import annotations

import os

import numpy as np
from PIL import Image

from .cnn import DepthPolicyNet
from .env import DepthAvoidEnv
from .policy import NeuralPolicy


def _save_gif(frames, path, target_w=320, target_h=240, min_interval=100):
    """把帧序列写成 gif，并压到 10 MB 以内。"""
    imgs = []
    for f in frames:
        im = Image.fromarray(f)
        if im.size != (target_w, target_h):
            im = im.resize((target_w, target_h), Image.LANCZOS)
        imgs.append(im.convert("P", palette=Image.ADAPTIVE, colors=128))
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=min_interval, loop=0, optimize=True)
    return os.path.getsize(path)


def _to_gif(frames, path, target_w=320, target_h=240):
    size = _save_gif(frames, path, target_w, target_h, 60)
    # 超过 10 MB 就逐级降帧、降色，直到达标
    scale = 1
    while size > 10 * 1024 * 1024 and len(frames) > 8:
        scale += 1
        frames = frames[::scale]
        size = _save_gif(frames, path, target_w, target_h, 60 * scale)
    return size


def run_demo(weights="models_trained/depth_avoid.npz", out_gif="demo.gif",
             steps=160, z0=-5.0, start=(-24.0, 0.0), verbose=True):
    """让网络自己飞一遍，录 gif，并返回统计。"""
    net = DepthPolicyNet()
    net.load(weights)
    policy = NeuralPolicy(net, z_ref=z0)

    env = DepthAvoidEnv()
    env.reset(start[0], start[1], z0, 0.0)

    frames, dmins, actions, zs = [], [], [], []
    z_ref = z0
    try:
        for t in range(steps):
            depth = env.depth()
            yaw_rate, z_target = policy.act(depth)
            frames.append(env.scene_rgb())
            dmins.append(float(depth.min()))
            actions.append((yaw_rate, z_target))
            env.step(yaw_rate, z_target, vx=2.5, duration=0.25)
            z_ref = z_target
            p, _ = env.pose()
            zs.append(p[2])
            if verbose and t % 20 == 0:
                print("demo step %3d  位置 (%.1f %.1f %.1f)  最小深度 %.2f"
                      % (t, p[0], p[1], p[2], depth.min()), flush=True)
        env.land()
    finally:
        env.close()

    gif_size = _to_gif(frames, out_gif)
    stats = {
        "steps": len(frames),
        "min_depth": float(np.min(dmins)),
        "mean_min_depth": float(np.mean(dmins)),
        "z_min": float(np.min(zs)),
        "z_max": float(np.max(zs)),
        "gif_kb": gif_size // 1024,
    }
    if verbose:
        print("gif 写入 %s（%d KB）" % (out_gif, stats["gif_kb"]))
        print("最小深度 %.2f，平均每帧最小深度 %.2f，高度区间 %.2f ~ %.2f"
              % (stats["min_depth"], stats["mean_min_depth"], stats["z_min"], stats["z_max"]))
    return stats


def run_eval(weights="models_trained/depth_avoid.npz", episodes=3, steps=80, verbose=True):
    """跑几回合，统计网络与专家在动作上的偏差。"""
    from .policy import ExpertPolicy

    net = DepthPolicyNet()
    net.load(weights)

    env = DepthAvoidEnv()
    rng = np.random.RandomState(7)
    diffs = []
    for ep in range(episodes):
        x0 = rng.uniform(-26.0, -16.0)
        y0 = rng.uniform(-5.0, 5.0)
        z0 = rng.uniform(-7.0, -4.0)
        env.reset(x0, y0, z0, 0.0)
        expert = ExpertPolicy()
        z_ref = z0
        for t in range(steps):
            depth = env.depth()
            expert_a = expert.act(depth)
            yaw_norm, alt_norm = net.predict(depth)
            diffs.append([abs(yaw_norm - expert_a[0]), abs(alt_norm - expert_a[1])])
            yaw_rate = float(expert_a[0]) * 1.0
            z_target = float(np.clip(z_ref - float(expert_a[1]) * 0.5, -9.0, -3.0))
            env.step(yaw_rate, z_target, vx=2.5, duration=0.25)
            z_ref = z_target
    env.land()
    env.close()

    d = np.array(diffs)
    stats = {"yaw_mae": float(d[:, 0].mean()), "alt_mae": float(d[:, 1].mean())}
    if verbose:
        print("评估：偏航 MAE %.3f，高度 MAE %.3f（各 %d 帧）"
              % (stats["yaw_mae"], stats["alt_mae"], len(d)))
    return stats
