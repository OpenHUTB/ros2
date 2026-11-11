"""采集抗风样本：IMU 窗口 + 位置误差 -> 专家补偿速度。"""
from __future__ import annotations

import os
import time

import numpy as np

from .env import CMD_SCALE, WindEnv
from .policy import encode, expert_action

TARGET = np.array([0.0, 0.0, -5.0])


def collect(out_path="models_trained/wind_data.npz", episodes=6, steps=100,
            seed=0, verbose=True):
    rng = np.random.RandomState(seed)
    env = WindEnv()
    xs, ys = [], []

    for ep in range(episodes):
        env.reset(0.0, 0.0, -5.0, 0.0)
        # 每个回合随机一组风：2~8 m/s，随机方向，含少量垂直分量
        speed = rng.uniform(8.0, 18.0)
        ang = rng.uniform(0, 2 * np.pi)
        wind = np.array([speed * np.cos(ang), speed * np.sin(ang), rng.uniform(-2.0, 2.0)])
        env.set_wind(wind[0], wind[1], wind[2])
        if verbose:
            print("回合 %d 风 (%.1f %.1f %.1f) m/s" % (ep, wind[0], wind[1], wind[2]), flush=True)

        p_prev, _ = env.pose()
        for t in range(steps):
            env.apply_velocity(0.0, 0.0, TARGET[2], duration=0.25, wait=False)
            time.sleep(0.12)

            p_now, _ = env.pose()
            vel = (p_now - p_prev) / 0.25
            p_prev = p_now

            err = TARGET[:2] - p_now[:2]
            act = expert_action(err, vel[:2], wind[:2])
            x = np.concatenate([env.imu_window(), err.astype(np.float32)])
            xs.append(x)
            ys.append(encode(act))

            env.apply_velocity(act[0], act[1], TARGET[2], duration=0.25, wait=False)
            time.sleep(0.13)

        if verbose:
            p, _ = env.pose()
            print("  回合结束 位置 (%.2f %.2f %.2f)" % (p[0], p[1], p[2]), flush=True)

    env.land()
    env.close()

    X = np.array(xs, dtype=np.float32)
    Y = np.array(ys, dtype=np.float32)
    x_mean = X.mean(axis=0)
    x_std = X.std(axis=0) + 1e-6
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez_compressed(out_path, X=(X - x_mean) / x_std, Y=Y, x_mean=x_mean, x_std=x_std)
    if verbose:
        print("样本 %d 条，保存到 %s" % (len(X), out_path))
        print("补偿速度均值 %s m/s，绝对均值 %s"
              % (np.round(Y.mean(axis=0) * CMD_SCALE, 3),
                 np.round(np.abs(Y).mean(axis=0) * CMD_SCALE, 3)))
    return (X - x_mean) / x_std, Y


def load(path="models_trained/wind_data.npz"):
    d = np.load(path)
    return d["X"], d["Y"], d["x_mean"], d["x_std"]
