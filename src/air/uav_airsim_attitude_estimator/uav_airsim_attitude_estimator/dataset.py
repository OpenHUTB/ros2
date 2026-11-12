"""采集 IMU 时序窗与真值姿态角。

在飞行中采样：moveByVelocityAsync 让机体前倾/侧倾，读到的姿态才不是全零。
"""
from __future__ import annotations

import time

import numpy as np

from .env import WINDOW, ImuAttEnv


def collect(out_path="models_trained/attitude_data.npz", episodes=6, steps=110, seed=0):
    rng = np.random.RandomState(seed)
    env = ImuAttEnv()
    env.reset(0.0, 0.0, -5.0, 0.0)

    X, Y = [], []
    try:
        for ep in range(episodes):
            env.reset(-6.0 + rng.uniform(-2, 2), rng.uniform(-2, 2), -5.0,
                      rng.uniform(-40, 40))
            for t in range(steps):
                yaw_rate = float(rng.uniform(-0.8, 0.8))
                speed = float(rng.uniform(0.4, 2.6))
                fut = env.fly(speed, yaw_rate, -5.0, duration=0.25, wait=False)
                time.sleep(0.14)
                w = env.imu_window()
                roll, pitch = env.attitude()
                X.append(w)
                Y.append([roll, pitch])
                time.sleep(0.11)
                fut.join()
            print("回合 %d 完成，累计 %d 条" % (ep, len(Y)), flush=True)
    finally:
        env.land()
        env.close()

    X = np.array(X, dtype=np.float32)
    Y = np.array(Y, dtype=np.float32)
    np.savez_compressed(out_path, X=X, Y=Y)
    print("样本 %d 条，roll 范围 %.3f~%.3f rad，pitch 范围 %.3f~%.3f rad"
          % (len(X), Y[:, 0].min(), Y[:, 0].max(), Y[:, 1].min(), Y[:, 1].max()),
          flush=True)


def load(path="models_trained/attitude_data.npz"):
    d = np.load(path)
    return d["X"].astype(np.float32), d["Y"].astype(np.float32)
