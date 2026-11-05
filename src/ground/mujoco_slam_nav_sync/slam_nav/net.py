"""纯 numpy 的 MLP 局部规划器：雷达扇区 + 目标方向 -> (v, w)。

输入 37 维：120 条雷达压成 30 个扇区（每 4 条取最小距离，归一化），
再加机器人系下 3 个前视点 (x, y) 与到终点的距离。三个前视点让网络"提前看到弯"。
输出 2 维：前进速度 sigmoid（× v_max）与角速度 tanh（× w_max）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .env import LIDAR_BEAMS, LIDAR_RANGE

SECTORS = 30
GROUP = LIDAR_BEAMS // SECTORS      # 120 / 30 = 4
IN_DIM = SECTORS + 3 * 2 + 1


def features(scan: np.ndarray, pose, targets, goal=None, v_max: float = 0.7) -> np.ndarray:
    """雷达 + 前视点 -> 网络输入（训练、部署共用同一函数）。

    targets：1~3 个世界坐标前视点；不足 3 个时用最后一个点补齐。
    """
    s = np.asarray(scan, dtype=np.float32).reshape(SECTORS, GROUP).min(axis=1)
    feats = list(s / LIDAR_RANGE)
    if not isinstance(targets, (list, tuple)):
        targets = [targets]
    targets = list(targets)[:3]
    while len(targets) < 3:
        targets.append(targets[-1] if targets else (pose[0], pose[1]))
    x, y, yaw = pose
    c, sn = np.cos(-yaw), np.sin(-yaw)
    for tx, ty in targets:
        dx, dy = tx - x, ty - y
        feats += [(c * dx - sn * dy) / 6.0, (sn * dx + c * dy) / 6.0]
    gx, gy = goal if goal is not None else targets[-1]
    feats.append(float(np.hypot(gx - x, gy - y)) / 12.0)
    return np.array(feats, dtype=np.float32)


class LocalPlannerNet:
    """33 -> 64 -> 32 -> 2 的 MLP（tanh 隐层，输出 v∈[0,1]、w∈[-1,1]）。"""

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

    def forward(self, x):
        p = self.p
        z1 = x @ p["w1"] + p["b1"]
        a1 = np.tanh(z1)
        z2 = a1 @ p["w2"] + p["b2"]
        a2 = np.tanh(z2)
        z3 = a2 @ p["w3"] + p["b3"]
        v = 1.0 / (1.0 + np.exp(-z3[:, 0]))
        w = np.tanh(z3[:, 1])
        return np.stack([v, w], axis=1), (x, a1, a2, z3, v, w)

    def backward(self, cache, y_true):
        x, a1, a2, z3, v, w = cache
        n = x.shape[0]
        pred = np.stack([v, w], axis=1)
        d = 2.0 * (pred - y_true) / n
        dz3 = np.stack([d[:, 0] * v * (1 - v), d[:, 1] * (1 - w ** 2)], axis=1)
        g = {"w3": a2.T @ dz3, "b3": dz3.sum(axis=0)}
        da2 = (dz3 @ self.p["w3"].T) * (1 - a2 ** 2)
        g["w2"] = a1.T @ da2
        g["b2"] = da2.sum(axis=0)
        da1 = (da2 @ self.p["w2"].T) * (1 - a1 ** 2)
        g["w1"] = x.T @ da1
        g["b1"] = da1.sum(axis=0)
        return g

    def step(self, grads, lr: float = 3e-3):
        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for k in self.p:
            self.m[k] = b1 * self.m[k] + (1 - b1) * grads[k]
            self.v[k] = b2 * self.v[k] + (1 - b2) * grads[k] ** 2
            mh = self.m[k] / (1 - b1 ** self.t)
            vh = self.v[k] / (1 - b2 ** self.t)
            self.p[k] -= (lr * mh / (np.sqrt(vh) + eps)).astype(np.float32)

    def predict(self, x):
        return self.forward(x[None, :])[0][0] if x.ndim == 1 else self.forward(x)[0]

    def act(self, scan, pose, targets, v_max: float = 0.7, w_max: float = 1.5, goal=None):
        f = features(scan, pose, targets, goal, v_max)
        out = self.forward(f[None, :])[0][0]
        return float(out[0] * v_max), float(out[1] * w_max)

    def save(self, path) -> str:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        np.savez(p, **{f"p_{k}": v for k, v in self.p.items()})
        return str(p)

    def load(self, path) -> "LocalPlannerNet":
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
