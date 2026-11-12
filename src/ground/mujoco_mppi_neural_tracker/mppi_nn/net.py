"""纯 numpy 的 12→64→32→2 MLP：把 MPPI 的策略蒸馏进网络。

输入 12 维都来自**可测量**：雷达估的横向误差、航向误差、编码器速度、上一控制量、
以及参考线在前方的三个横向误差（由已知地图 + 当前位姿估计给出）。
输出 (δ, v)。隐层 tanh，输出用 tanh / sigmoid 限制在物理范围内。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

IN_DIM = 12
DELTA_MAX = 0.6
V_MIN, V_MAX = 0.35, 1.6


class TrackerNet:
    def __init__(self, in_dim: int = IN_DIM, h1: int = 64, h2: int = 32, seed: int = 0):
        rng = np.random.default_rng(seed)
        self.p = {
            "w1": rng.normal(0, np.sqrt(2.0 / in_dim), (in_dim, h1)).astype(np.float32),
            "b1": np.zeros(h1, np.float32),
            "w2": rng.normal(0, np.sqrt(2.0 / h1), (h1, h2)).astype(np.float32),
            "b2": np.zeros(h2, np.float32),
            "w3": rng.normal(0, np.sqrt(2.0 / h2), (h2, 2)).astype(np.float32),
            "b3": np.zeros(2, np.float32),
        }
        self.m = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.v = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.t = 0

    # ------------------------------------------------------------------
    def forward(self, x):
        p = self.p
        a1 = np.tanh(x @ p["w1"] + p["b1"])
        a2 = np.tanh(a1 @ p["w2"] + p["b2"])
        z3 = a2 @ p["w3"] + p["b3"]
        d = np.tanh(z3[:, 0])
        s = 1.0 / (1.0 + np.exp(-z3[:, 1]))
        return np.stack([d, s], axis=1), (x, a1, a2, d, s)

    def backward(self, cache, y):
        x, a1, a2, d, s = cache
        n = x.shape[0]
        pred = np.stack([d, s], axis=1)
        g_out = 2.0 * (pred - y) / n
        dz = np.stack([g_out[:, 0] * (1 - d ** 2), g_out[:, 1] * s * (1 - s)], axis=1)
        g = {"w3": a2.T @ dz, "b3": dz.sum(axis=0)}
        da2 = (dz @ self.p["w3"].T) * (1 - a2 ** 2)
        g["w2"] = a1.T @ da2
        g["b2"] = da2.sum(axis=0)
        da1 = (da2 @ self.p["w2"].T) * (1 - a1 ** 2)
        g["w1"] = x.T @ da1
        g["b1"] = da1.sum(axis=0)
        return g

    def step(self, grads, lr: float = 2e-3):
        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for k in self.p:
            self.m[k] = b1 * self.m[k] + (1 - b1) * grads[k]
            self.v[k] = b2 * self.v[k] + (1 - b2) * grads[k] ** 2
            mh = self.m[k] / (1 - b1 ** self.t)
            vh = self.v[k] / (1 - b2 ** self.t)
            self.p[k] -= (lr * mh / (np.sqrt(vh) + eps)).astype(np.float32)

    # ------------------------------------------------------------------
    def act(self, feats) -> tuple[float, float]:
        out = self.forward(np.asarray(feats, dtype=np.float32)[None, :])[0][0]
        delta = float(out[0]) * DELTA_MAX
        v = V_MIN + float(out[1]) * (V_MAX - V_MIN)
        return v, delta

    def save(self, path) -> str:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        np.savez(p, **{f"p_{k}": v for k, v in self.p.items()})
        return str(p)

    def load(self, path) -> "TrackerNet":
        with np.load(path) as z:
            for k in self.p:
                arr = z[f"p_{k}"]
                if arr.shape != self.p[k].shape:
                    raise ValueError(f"权重 {k} 形状 {arr.shape} != {self.p[k].shape}")
                self.p[k] = arr.astype(np.float32)
        return self

    def config(self) -> dict:
        return {"in_dim": self.p["w1"].shape[0],
                "params": int(sum(v.size for v in self.p.values()))}


def build_features(e, psi, v, delta_prev, v_prev, path, idx, pose,
                   v_ref: float = 0.9) -> np.ndarray:
    """把可观测量拼成网络输入；训练与部署共用。

    ``e/psi`` 是当前横向/航向误差；另外三个横向误差是**前方 1/2/3 m 参考点**
    相对当前位姿的法向偏差，等价于"提前看到弯"。
    """
    n = len(path)
    look_e, look_psi = [], []
    for k in (20, 40, 60):
        j = (idx + k) % n
        p, nxt = path[j], path[(j + 1) % n]
        t = nxt - p
        t = t / (np.linalg.norm(t) + 1e-12)
        nrm = np.array([-t[1], t[0]])
        look_e.append(float(np.dot(np.asarray(pose[:2]) - p, nrm)))
        heading = np.array([np.cos(pose[2]), np.sin(pose[2])])
        look_psi.append(float(np.arctan2(t[0] * heading[1] - t[1] * heading[0],
                                         float(t @ heading))))
    feats = [np.clip(e / 2.0, -2, 2), np.clip(psi / 1.0, -2, 2), v / V_MAX,
             delta_prev / DELTA_MAX, v_prev / V_MAX,
             np.clip(look_e[0] / 2.0, -2, 2), np.clip(look_e[1] / 2.0, -2, 2),
             np.clip(look_e[2] / 2.0, -2, 2),
             np.clip(look_psi[0], -2, 2), np.clip(look_psi[1], -2, 2),
             np.clip((v_ref - v) / V_MAX, -2, 2), 0.0]
    return np.array(feats, dtype=np.float32)
