"""采集 IMU-速度样本：飞行中同步记录 IMU 时序窗与真值速度。"""
from __future__ import annotations

import os
import time

import numpy as np

from .env import VEL_SCALE, ImuVelEnv


def collect(out_path="models_trained/imu_vel_data.npz", episodes=6, steps=120,
            seed=0, verbose=True):
    rng = np.random.RandomState(seed)
    env = ImuVelEnv()
    xs, ys = [], []

    for ep in range(episodes):
        x0 = rng.uniform(-28.0, -16.0)
        y0 = rng.uniform(-4.0, 4.0)
        z0 = rng.uniform(-7.0, -4.5)
        env.reset(x0, y0, z0, rng.uniform(-30.0, 30.0))

        z_ref = z0
        forward = 2.0
        yaw_rate = 0.0
        for t in range(steps):
            # 每隔几步换一次指令，让速度有丰富的变化
            if t % 8 == 0:
                forward = float(rng.uniform(0.3, 3.2))
                yaw_rate = float(rng.uniform(-0.8, 0.8))
                z_ref = float(np.clip(z_ref + rng.uniform(-0.8, 0.8), -8.5, -3.5))

            # 发速度指令但不阻塞，在指令执行中段采样 IMU 和真实速度
            env.fly(forward, yaw_rate, z_ref, duration=0.25, wait=False)
            time.sleep(0.14)
            xs.append(env.imu_window())
            v = env.velocity()
            # 只回归速度大小：IMU 的振动强度与速率强相关，比三轴分量稳定得多
            ys.append(np.array([np.linalg.norm(v) / VEL_SCALE], dtype=np.float32))
            time.sleep(0.11)

        if verbose:
            p, _ = env.pose()
            print("回合 %d 结束，位置 (%.1f %.1f %.1f)，累计样本 %d"
                  % (ep, p[0], p[1], p[2], len(xs)), flush=True)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    X = np.array(xs, dtype=np.float32)
    Y = np.array(ys, dtype=np.float32)
    # IMU 各通道尺度差一个数量级，标准化后再喂网络，否则 tanh 直接饱和
    x_mean = X.mean(axis=0)
    x_std = X.std(axis=0) + 1e-6
    np.savez_compressed(out_path, X=(X - x_mean) / x_std, Y=Y, x_mean=x_mean, x_std=x_std)
    if verbose:
        print("样本保存到 %s，形状 X=%s Y=%s" % (out_path, X.shape, Y.shape))
        print("标准化前加速度 RMS %.3f，速度均值 %s（归一化）"
              % (float(np.sqrt((X[:, :3] ** 2).mean())), Y.mean(axis=0)))
    env.close()
    return (X - x_mean) / x_std, Y


def load(path="models_trained/imu_vel_data.npz"):
    d = np.load(path)
    return d["X"], d["Y"], d["x_mean"], d["x_std"]
