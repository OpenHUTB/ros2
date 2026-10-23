# -*- coding: utf-8 -*-
"""
dagger_iter.py
DAgger(Dataset Aggregation) 迭代训练: 解决监督学习闭环的"分布偏移"问题。

流程:
  1. 用当前 MLP 部署, 收集其自身轨迹中的状态(覆盖真实部署分布)
  2. 用 PD 专家控制器对这些状态标注正确动作
  3. 与原始演示数据合并, 重新训练 MLP
  4. 重复 2 轮, 最终 MLP 在部署分布上也准确 -> 闭环稳定
"""
import numpy as np

from reacher_env import ReacherEnv, inverse_kinematics
from nn import MLP, Adam


def make_policy(net, Xstat, Ystat):
    Xm, Xs = Xstat[0], Xstat[1]
    Ym, Ys = Ystat[0], Ystat[1]

    def p(s):
        xs = (np.asarray(s, dtype=np.float64) - Xm) / Xs
        return net.forward(xs.reshape(1, -1))[0] * Ys + Ym

    return p


def collect_states(env, pol, n_eps=60, max_steps=150):
    """用当前策略部署, 收集其轨迹状态(及对应目标)。"""
    out = []
    for _ in range(n_eps):
        q1 = np.random.uniform(-1.5, 1.5)
        q2 = np.random.uniform(-1.5, 1.5)
        t = env.reset(init_q=(q1, q2))
        for _ in range(max_steps):
            s = env.state()
            out.append((s, t))
            env.step(pol(s))
            if env.distance() < 0.02:
                break
    return out


def label_pd(states, kp=15.0):
    """用 PD 专家对状态标注正确动作。"""
    X, Y = [], []
    for s, t in states:
        q1 = np.arctan2(s[0], s[1])
        q2 = np.arctan2(s[2], s[3])
        qd = inverse_kinematics(t[0], t[1])
        X.append(s)
        Y.append([kp * (qd[0] - q1), kp * (qd[1] - q2)])
    return np.asarray(X), np.asarray(Y)


def train_mlp(X, Y, dims, epochs=400, lr=3e-3, seed=0):
    Xmean, Xstd = X.mean(0), X.std(0) + 1e-8
    Ymean, Ystd = Y.mean(0), Y.std(0) + 1e-8
    Xs = (X - Xmean) / Xstd
    Ys = (Y - Ymean) / Ystd
    net = MLP(dims, seed=seed)
    opt = Adam(lr=lr)
    n = len(X)
    for ep in range(epochs):
        idx = np.random.permutation(n)
        for s in range(0, n, 64):
            b = idx[s:s + 64]
            pred = net.forward(Xs[b])
            g = net.mse_grad(Ys[b])
            gw, gb = net.backward(g)
            opt.step(*net.params(), gw, gb)
        if (ep + 1) % 100 == 0:
            print(f"  epoch {ep+1}/{epochs} done")
    return net, Xmean, Xstd, Ymean, Ystd


def main(iter_n=2, n_eps=60, max_steps=150, epochs=400, lr=3e-3):
    np.random.seed(0)
    env = ReacherEnv("models/reacher.xml")
    net = MLP([6, 128, 128, 2])
    net.load("mlp.npz")
    Xstat = np.load("Xstat.npy")
    Ystat = np.load("Ystat.npy")
    pol = make_policy(net, Xstat, Ystat)
    d = np.load("demo.npz")
    X0, Y0 = d["X"], d["Y"]

    for it in range(iter_n):
        states = collect_states(env, pol, n_eps, max_steps)
        Xn, Yn = label_pd(states)
        X = np.vstack([X0, Xn])
        Y = np.vstack([Y0, Yn])
        print(f"== iter {it}: total {len(X)} (new {len(Xn)}) ==")
        net, Xm, Xs, Ym, Ys = train_mlp(X, Y, [6, 128, 128, 2], epochs, lr)
        net.save("mlp.npz")
        np.save("Xstat.npy", np.stack([Xm, Xs]))
        np.save("Ystat.npy", np.stack([Ym, Ys]))
        Xstat = np.load("Xstat.npy")
        Ystat = np.load("Ystat.npy")
        pol = make_policy(net, Xstat, Ystat)
    print("DAgger done, saved mlp.npz")


if __name__ == "__main__":
    main()
