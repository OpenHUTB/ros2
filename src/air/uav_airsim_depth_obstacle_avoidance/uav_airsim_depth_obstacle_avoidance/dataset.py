"""采集专家样本：让无人机在 Blocks 场景里朝方块墙飞，记录 (深度图, 专家动作)。"""
from __future__ import annotations

import os

import numpy as np

from .env import DepthAvoidEnv
from .policy import ALT_DELTA_MAX, YAW_RATE_MAX, ExpertPolicy


def collect(out_path="models_trained/depth_avoid.npz", episodes=6, steps=120,
            seed=0, verbose=True):
    rng = np.random.RandomState(seed)
    env = DepthAvoidEnv()
    xs, ys = [], []

    for ep in range(episodes):
        x0 = rng.uniform(-28.0, -14.0)
        y0 = rng.uniform(-6.0, 6.0)
        z0 = rng.uniform(-7.0, -4.0)
        yaw0 = rng.uniform(-20.0, 20.0)
        env.reset(x0, y0, z0, yaw0)
        z_ref = z0
        expert = ExpertPolicy()

        for t in range(steps):
            depth = env.depth()
            action = expert.act(depth)
            xs.append(depth)
            ys.append(action)

            yaw_rate = float(action[0]) * YAW_RATE_MAX
            z_target = z_ref - float(action[1]) * ALT_DELTA_MAX
            z_target = float(np.clip(z_target, -9.0, -3.0))
            env.step(yaw_rate, z_target, vx=2.5, duration=0.25)
            z_ref = z_target

            if t % 20 == 0 and verbose:
                p, _ = env.pose()
                print("  ep%d step%3d  位置 (%.1f, %.1f, %.1f)  最小深度 %.2f"
                      % (ep, t, p[0], p[1], p[2], float(depth.min())), flush=True)

        env.reset(0, 0, -5, 0)
        if verbose:
            print("  回合 %d 结束，累计样本 %d" % (ep, len(xs)), flush=True)

    env.close()
    X = np.asarray(xs, dtype=np.float32)
    Y = np.asarray(ys, dtype=np.float32)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez_compressed(out_path, X=X, Y=Y)
    if verbose:
        print("样本保存到 %s，形状 X=%s Y=%s" % (out_path, X.shape, Y.shape))
        print("深度均值 %.3f，动作均值 %s" % (float(X.mean()), Y.mean(axis=0)))
    return X, Y


def load(path="models_trained/depth_avoid.npz"):
    data = np.load(path)
    return data["X"], data["Y"]
