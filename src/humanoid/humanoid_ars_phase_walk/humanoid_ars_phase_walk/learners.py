# -*- coding: utf-8 -*-
"""学习器：ARS（无梯度）与行为克隆 / DAgger（有监督）。纯 numpy。"""

import numpy as np

import humanoid_ars_phase_walk.config as config
import humanoid_ars_phase_walk.env as env


class LinearPolicy:
    """线性策略 theta @ obs，用作 ARS 的基线。"""

    def __init__(self, obs_dim=env.OBS_DIM, act_dim=env.ACT_DIM, seed=config.SEED):
        self.theta = np.zeros((act_dim, obs_dim))
        self.rng = np.random.default_rng(seed)

    def __call__(self, ph, obs):
        return self.theta @ obs

    def commit(self):
        return lambda ph, obs: self.theta @ obs

    def train(self, iters=200, n_dir=config.ARS_DIRS, sigma=0.04, alpha=0.02):
        for _ in range(iters):
            dd = [self.rng.normal(size=self.theta.shape) for _ in range(n_dir)]
            rew = np.zeros(2 * n_dir)
            for i, delta in enumerate(dd):
                rew[i] = env.rollout(env.make_env(0),
                                     lambda ph, o, t=self.theta + sigma * delta: t @ o)["reward"]
                rew[n_dir + i] = env.rollout(env.make_env(0),
                                             lambda ph, o, t=self.theta - sigma * delta: t @ o)["reward"]
            step = np.zeros(self.theta.shape)
            for idx in np.argsort(-rew)[:n_dir]:
                i = idx % n_dir
                sgn = 1.0 if idx < n_dir else -1.0
                step += (rew[idx] - rew.mean()) * sgn * dd[i]
            self.theta += alpha / (n_dir * sigma) * step
        return self


class MLPPolicy:
    """单隐层 tanh MLP，用于行为克隆 / DAgger。"""

    def __init__(self, n_hidden=config.BC_HIDDEN, seed=config.SEED):
        rng = np.random.default_rng(seed)
        self.W1 = rng.normal(0, np.sqrt(2 / env.OBS_DIM), (env.OBS_DIM, n_hidden))
        self.b1 = np.zeros(n_hidden)
        self.W2 = rng.normal(0, np.sqrt(2 / n_hidden), (n_hidden, env.ACT_DIM))
        self.b2 = np.zeros(env.ACT_DIM)
        self.mu = None
        self.sd = None

    def predict(self, X):
        Z = (np.atleast_2d(X) - self.mu) / self.sd
        return np.tanh(Z @ self.W1 + self.b1) @ self.W2 + self.b2

    def __call__(self, ph, obs):
        return self.predict(obs)[0]

    def fit(self, X, Y, epochs=config.BC_EPOCHS, lr=0.01, batch=256):
        self.mu = X.mean(axis=0)
        self.sd = X.std(axis=0) + 1e-6
        rng = np.random.default_rng(config.SEED)
        n = len(X)
        for _ in range(epochs):
            order = rng.permutation(n)
            for s0 in range(0, n, batch):
                idx = order[s0:s0 + batch]
                Z = (X[idx] - self.mu) / self.sd
                H = np.tanh(Z @ self.W1 + self.b1)
                err = H @ self.W2 + self.b2 - Y[idx]
                d2 = err * (2.0 / len(idx))
                gW2 = H.T @ d2
                gb2 = d2.sum(axis=0)
                d1 = (d2 @ self.W2.T) * (1 - H ** 2)
                gW1 = Z.T @ d1
                gb1 = d1.sum(axis=0)
                self.W2 -= lr * gW2; self.b2 -= lr * gb2
                self.W1 -= lr * gW1; self.b1 -= lr * gb1
        return self


def collect_cpg(n_episodes=4):
    """用 CPG 跑几个回合，收集 (obs, cpg 动作)。"""
    X, Y = [], []
    for _ in range(n_episodes):
        r = env.rollout(env.make_env(0), record=True)
        X.append(r["X"]); Y.append(r["Y"])
    return np.vstack(X), np.vstack(Y)


def dagger(rounds=config.DAGGER_ROUNDS):
    """DAgger：用当前策略跑、再用 CPG 给访问到的状态重新标注，迭代。"""
    X, Y = collect_cpg()
    net = MLPPolicy().fit(X, Y)
    hist = []
    for rd in range(rounds):
        r = env.rollout(env.make_env(0), net, record=True)
        X = np.vstack([X, r["X"]]); Y = np.vstack([Y, r["Y"]])
        net = MLPPolicy(seed=rd + 1).fit(X, Y, epochs=300)
        ev = env.rollout(env.make_env(0), net)
        hist.append(ev)
    return net, hist