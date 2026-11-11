# -*- coding: utf-8 -*-
"""
collect_demo.py
用 逆运动学(IK) + PD 控制器 生成监督学习演示数据。
对每个 episode: 随机初始关节角 + 随机目标, PD 控制跟踪至收敛,
记录 (感知状态, PD控制动作) 数据对, 供 MLP 监督学习逼近该控制律。
"""
import os

import numpy as np

try:                       # 作为包导入（python -m / pytest）
    from .reacher_env import ReacherEnv
except ImportError:        # 作为脚本直接运行（main.py / launch 的用法）
    from reacher_env import ReacherEnv


def main(n_eps=400, max_steps=250, seed=0):
    # 环境变量只用于「快速模式」（CI / 冒烟验证），不改变默认实验规模
    n_eps = int(os.environ.get('RT_DEMO_EPISODES', n_eps))
    max_steps = int(os.environ.get('RT_MAX_STEPS', max_steps))
    np.random.seed(seed)
    env = ReacherEnv("models/reacher.xml")
    X, Y = [], []
    final_dists = []
    n_ep, n_step = 0, 0
    for ep in range(n_eps):
        q1 = np.random.uniform(-1.5, 1.5)
        q2 = np.random.uniform(-1.5, 1.5)
        target = env.reset(init_q=(q1, q2))
        ep_steps = 0
        for _ in range(max_steps):
            s = env.state()
            u = env.ik_pd_ctrl(target)
            X.append(s)
            Y.append(u)
            env.step(u)
            n_step += 1
            ep_steps += 1
            if env.distance() < 0.02:
                break
        final_dists.append(env.distance())
        n_ep += 1
        if (ep + 1) % 50 == 0:
            print(f"ep {ep+1}/{n_eps} done, last final_dist={env.distance():.4f}")

    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    out = "demo.npz"
    np.savez(out, X=X, Y=Y)
    fd = np.array(final_dists)
    print("=" * 40)
    print("samples:", X.shape, "Y:", Y.shape)
    print("episodes:", n_ep, "total_steps:", n_step)
    print("final_dist  mean=%.4f  median=%.4f  success(<0.05)=%.2f%%"
          % (fd.mean(), np.median(fd), (fd < 0.05).mean() * 100))
    print("saved ->", out)


if __name__ == "__main__":
    main()
