#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""残差数据采集：飞一条轨迹、记录 (状态, 指令) → 实测 Δa。

采集用**纯几何控制器**（无残差）而不是最优控制器：只有控制器按照标称模型
行动，实测加速度与标称预测之差才是纯粹的模型误差；若用带残差的控制器采，
网络学到的是"自己上一步的修正"，会自洽地收敛到一个错误模型。

激励是必需的：沿参考轨迹飞时状态几乎落在一条一维流形上，网络在流形外
没有任何约束（实测：无激励时训练 RMSE 0.03 m/s²，但换一条轨迹立刻退化到
0.44 m/s²）。叠加带限随机加速度把状态铺开到整个包线。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .controllers import GeometricTracker
from .plant import ResidualPlant
from .residual import FEATURE_DIM
from .trajectories import Trajectory, make_trajectory

__all__ = ["Dataset", "EpisodeCollector", "excitation_series"]


def excitation_series(n: int, dt: float, std: float, tau: float = 0.45,
                      seed: int = 0) -> np.ndarray:
    """一阶低通白噪声 (n, 3)：带限是为了让无人机跟得上，全带白噪声会被
    姿态环滤掉，只剩推力抖动，状态反而铺不开。"""
    if std <= 0.0:
        return np.zeros((n, 3))
    rng = np.random.default_rng(seed)
    a = float(np.exp(-dt / tau))
    s = std * float(np.sqrt(1.0 - a * a))
    out = np.zeros((n, 3))
    x = np.zeros(3)
    for i in range(n):
        x = a * x + s * rng.standard_normal(3)
        out[i] = x
    return out


@dataclass
class Dataset:
    """监督学习数据集：X 为网络输入特征，Y 为**机体系**残差加速度。"""

    X: np.ndarray
    Y: np.ndarray
    meta: List[Dict[str, object]]

    def __len__(self) -> int:
        return int(len(self.X))

    def split(self, val_frac: float = 0.2, seed: int = 0) -> Tuple["Dataset", "Dataset"]:
        idx = np.random.default_rng(seed).permutation(len(self.X))
        n_val = int(len(self.X) * val_frac)
        val, tr = idx[:n_val], idx[n_val:]
        return (Dataset(self.X[tr], self.Y[tr], [self.meta[i] for i in tr]),
                Dataset(self.X[val], self.Y[val], [self.meta[i] for i in val]))

    def save(self, path: str) -> None:
        np.savez_compressed(path, X=self.X, Y=self.Y,
                            meta=np.asarray([str(m) for m in self.meta]))

    @classmethod
    def load(cls, path: str) -> "Dataset":
        import ast
        with np.load(path, allow_pickle=False) as d:
            meta = [ast.literal_eval(str(s)) for s in d["meta"]]
            return cls(np.array(d["X"], float), np.array(d["Y"], float), meta)


class EpisodeCollector:
    """跑一条轨迹并记录残差样本。"""

    def __init__(self, plant: ResidualPlant, excite_std: float = 1.2,
                 excite_tau: float = 0.45, seed: int = 0):
        self.plant = plant
        self.excite_std = float(excite_std)
        self.excite_tau = float(excite_tau)
        self.seed = int(seed)

    def run(self, traj: Trajectory, warmup: float = 1.5,
            max_residual: float = 25.0) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (X, Y)。

        ``warmup`` 秒内的样本丢弃：起飞段贴地，地面效应主导且高度分布极窄，
        混进训练集会让网络把"近地增益"错当成常数偏置。

        起点取在轨迹上（含初始偏航）：从别处起飞会让偏航误差一开始就大到
        混控饱和——偏航力矩受混控预算约束，误差超过约 13° 就会挤占滚转/俯仰
        的权限，无人机翻掉，整段数据变成废样本（实测 |Δa| 出现 207 m/s²）。
        """
        dt = self.plant.dt
        n = int(round(traj.duration / dt))
        exc = excitation_series(n, dt, self.excite_std, self.excite_tau, self.seed)
        tracker = GeometricTracker(self.plant, traj)
        p0, _, _, yaw0, _ = traj.eval(0.0)
        self.plant.reset(pos=tuple(p0), yaw=float(yaw0))

        X = np.empty((n, FEATURE_DIM))
        Y = np.empty((n, 3))
        for k in range(n):
            t = k * dt
            tracker.excitation = exc[k]
            u = tracker.act(t)
            self.plant.step_with_control(u)
            tracker.observe()
            X[k] = self._features(tracker, u)
            Y[k] = tracker.last_delta

        skip = int(warmup / dt)
        X, Y = X[skip:], Y[skip:]
        keep = np.linalg.norm(Y, axis=1) < max_residual
        return X[keep], Y[keep]

    @staticmethod
    def _features(tracker: GeometricTracker, u: np.ndarray) -> np.ndarray:
        st = tracker.plant.state()
        R = tracker._R
        x = np.empty(FEATURE_DIM)
        x[0:3] = R.T @ st.vel
        s_h = float(np.hypot(x[0], x[1]))
        x[3] = x[0] * s_h
        x[4] = x[1] * s_h
        x[5] = x[2] * abs(x[2])
        x[6:9] = R.T @ tracker.last_a_cmd
        x[9:12] = st.omega
        x[12] = float(st.pos[2])
        x[13] = float(np.sum(u)) / tracker.params.mass
        return x


def collect_dataset(specs: Sequence[Tuple[str, str, str, int]],
                    excite_std: float = 1.2, seconds_per_episode: Optional[float] = None,
                    verbose: bool = False) -> Dataset:
    """按 ``(payload, wind, traj, seed)`` 列表采集并拼接。

    每个组合换一次风种子：同一个 OU 实现重复出现会让网络把某条特定风速曲线
    当成特征，泛化到别的种子时残差预测出现明显偏差。
    """
    from .disturbances import make_plant

    Xs, Ys, meta = [], [], []
    for payload, wind, traj_name, seed in specs:
        traj = make_trajectory(traj_name)
        if seconds_per_episode is not None:
            traj.duration = float(seconds_per_episode)
        plant = ResidualPlant(plant=make_plant(payload, wind, seed=seed),
                              scene="open_ground", seed=seed)
        x, y = EpisodeCollector(plant, excite_std=excite_std, seed=seed).run(traj)
        Xs.append(x)
        Ys.append(y)
        meta.extend([{"payload": payload, "wind": wind, "traj": traj_name,
                      "seed": seed}] * len(x))
        if verbose:
            print("  采集 %-6s %-7s %-8s seed=%d  %5d 样本   |Δa|均值=%.3f m/s²"
                  % (payload, wind, traj_name, seed, len(x),
                     float(np.mean(np.linalg.norm(y, axis=1)))))
    return Dataset(np.vstack(Xs), np.vstack(Ys), meta)
