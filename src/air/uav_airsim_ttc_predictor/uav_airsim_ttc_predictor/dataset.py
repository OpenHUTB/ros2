"""采集 TTC 样本：深度图 -> TTC 真值。

真值 = 最近障碍距离 / 当前前进速度，超过 TTC_MAX 视为无穷（截断）。
"""
from __future__ import annotations

import os
import time

import numpy as np

from .env import TTC_MAX, TtcEnv


def collect(out_path="models_trained/ttc_data.npz", episodes=6, steps=80,
            seed=0, verbose=True):
    rng = np.random.RandomState(seed)
    env = TtcEnv()
    X, Y = [], []

    for ep in range(episodes):
        x0 = rng.uniform(-16.0, -8.0)
        y0 = rng.uniform(-2.0, 2.0)
        env.reset(x0, y0, -4.0, rng.uniform(-8, 8))
        speed = rng.uniform(0.6, 2.5)
        if verbose:
            print("回合 %d 起点 (%.1f %.1f) 速度 %.1f m/s" % (ep, x0, y0, speed), flush=True)

        p_prev, _ = env.pose()
        for t in range(steps):
            d = env.depth()
            raw = env.depth_raw()
            min_d = float(np.min(raw))
            if min_d < 0.9:
                break

            p_now, _ = env.pose()
            vel = (p_now - p_prev) / 0.25
            p_prev = p_now
            # 位置差分噪声太大，改用采集时设定的前进速度作为基准，
            # 再用差分速度做上下限约束，避免标签被噪声主导。
            v_diff = float(np.hypot(vel[0], vel[1]))
            v_forward = float(np.clip(0.5 * speed + 0.5 * v_diff, 0.5, 3.0))

            ttc = min(min_d / v_forward, TTC_MAX)
            X.append(d)
            Y.append(np.array([ttc / TTC_MAX], dtype=np.float32))

            env.fly_forward(speed, -4.0, duration=0.25, wait=False)
            time.sleep(0.16)

    env.land()
    env.close()

    X = np.array(X, dtype=np.float32)
    Y = np.array(Y, dtype=np.float32)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez_compressed(out_path, X=X, Y=Y)
    if verbose:
        print("样本 %d 条，TTC 均值 %.1f s，最小 %.1f s"
              % (len(X), float(Y.mean()) * TTC_MAX, float(Y.min()) * TTC_MAX))
    return X, Y


def load(path="models_trained/ttc_data.npz"):
    d = np.load(path)
    return d["X"], d["Y"]
