# -*- coding: utf-8 -*-
"""肌肉协同（muscle synergy）降维：需要几个协同才能重建 EMG？

问题
----
运动控制里常说"人体用少数几个肌肉协同完成动作"。这句结论怎么验证？
把 n 块肌肉的激活矩阵 E（n × T）做非负矩阵分解

    E ≈ W · H,    W ∈ R^{n×k}_+,  H ∈ R^{k×T}_+

k 就是协同数。k 越大重建误差越小，但"重建得像"不等于"协同是真的"——
本模块同时给出两个指标，专门用来戳破这个差别：

1. **VAF**（variance accounted for）= 1 − ‖E−WH‖²/‖E‖²，衡量重建质量；
2. **子空间夹角**：把估计出的 W 与"真值" W₀ 的列空间做主角分析，
   夹角越小说明恢复的协同方向越准。

因为仓库里没有真实 EMG 数据，本模块用**已知协同合成数据**做验证：
先设定 W₀、H₀ 生成 E，再加噪声，最后看 NMF 能否把 W₀ 找回来。
这样结论是可验证的（有真值可比），也避免了"用假数据编结论"。

单位：激活值无量纲（0~1）。
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

EPS = 1e-12


@dataclass
class SynergyData:
    """一块 EMG 激活矩阵及其真值（真值仅合成数据有）。"""

    E: np.ndarray                 # n_muscles × n_samples
    W_true: Optional[np.ndarray] = None


def synth_data(n_muscles: int = 8, n_synergies: int = 3, n_samples: int = 200,
               noise: float = 0.03, seed: int = 0) -> SynergyData:
    """按已知协同生成合成 EMG。

    W₀ 每列只激活一块"肌肉簇"（稀疏、非负），H₀ 用平滑的升余弦时间包络，
    最后按 `noise` 的相对强度叠加均匀噪声并截断到非负。
    """
    rng = np.random.default_rng(seed)

    # 真值协同：把肌肉分成 k 簇，每簇一列（外加少量交叉激活）
    W0 = np.zeros((n_muscles, n_synergies))
    members = np.array_split(np.arange(n_muscles), n_synergies)
    for j, grp in enumerate(members):
        W0[grp, j] = 0.6 + 0.4 * rng.random(len(grp))
    W0 += 0.05 * rng.random(W0.shape)

    # 时间包络：几个不同相位的升余弦
    t = np.linspace(0.0, 1.0, n_samples)
    H0 = np.zeros((n_synergies, n_samples))
    for j in range(n_synergies):
        phase = 2.0 * math.pi * j / n_synergies
        H0[j] = 0.5 * (1.0 - np.cos(2.0 * math.pi * t + phase)) + 0.05

    E = W0 @ H0
    if noise > 0:
        E = E + noise * E.max() * rng.random(E.shape)
    E = np.clip(E, 0.0, None)
    return SynergyData(E=E, W_true=W0)


def nmf(E: np.ndarray, k: int, iters: int = 300, seed: int = 0,
        init: Optional[Tuple[np.ndarray, np.ndarray]] = None) -> Tuple[np.ndarray, np.ndarray]:
    """非负矩阵分解（Lee & Seung 乘性更新），固定 seed 保证可复现。"""
    E = np.asarray(E, dtype=float)
    if E.ndim != 2:
        raise ValueError("E 必须是二维矩阵")
    n, T = E.shape
    if not 1 <= k <= n:
        raise ValueError("协同数 k 必须在 1..n_muscles 之间")

    if init is not None:
        W = np.asarray(init[0], dtype=float).copy()
        H = np.asarray(init[1], dtype=float).copy()
        if W.shape != (n, k) or H.shape != (k, T):
            raise ValueError("初值形状与 (n, k) / (k, T) 不一致")
    else:
        rng = np.random.default_rng(seed)
        W = rng.random((n, k)) * math.sqrt(E.mean() / k) + EPS
        H = rng.random((k, T)) * math.sqrt(E.mean() / k) + EPS

    for _ in range(iters):
        H *= (W.T @ E) / (W.T @ W @ H + EPS)
        W *= (E @ H.T) / (W @ H @ H.T + EPS)
    return W, H


def vaf(E: np.ndarray, W: np.ndarray, H: np.ndarray) -> float:
    """重建方差解释率，1.0 表示完全重建。"""
    E = np.asarray(E, dtype=float)
    denom = float(np.sum(E ** 2))
    if denom <= 0:
        return 1.0
    recon = float(np.sum((E - W @ H) ** 2))
    return 1.0 - recon / denom


def subspace_angle_deg(W_a: np.ndarray, W_b: np.ndarray) -> float:
    """两个列空间的最大主角（度）。0° 表示子空间相同，90° 表示正交。

    只比较方向，不比较列顺序与幅值——NMF 的列本来就可以换序、缩放。

    实现注意（review 指出过的坑）：**不能**把两边的正交基按 min(k1, k2) 截断。
    QR 的列顺序本身没有意义，`Qa[:, :k].T @ Qb[:, :k]` 取的是"前 k 个基向量"，
    换个列序结果就变（同一子空间能读出 37.6° / 48.2° / 84.8° 三个值）。
    直接算 `Qa.T @ Qb` 即可：它是 k1×k2 矩阵，奇异值个数自然就是 min(k1, k2)。

    语义提醒：这个量度量的是**子空间的包含/夹角关系**，不是"协同数选对了没有"。
    当估计子空间落在真值子空间内部（k ≤ k0）或包含它（k ≥ k0）时，主角一律接近 0°。
    所以它只适合验证"因素有没有跑到真值结构之外"，**不能**用来选协同数。
    """
    def _basis(M):
        M = np.asarray(M, dtype=float)
        if M.ndim != 2:
            raise ValueError("矩阵必须是二维")
        q, _ = np.linalg.qr(M)
        return q

    Qa, Qb = _basis(W_a), _basis(W_b)
    s = np.linalg.svd(Qa.T @ Qb, compute_uv=False)
    s = np.clip(s, -1.0, 1.0)
    return float(np.degrees(np.arccos(s.min())))


def scan_k(E: np.ndarray, k_max: Optional[int] = None, iters: int = 300,
           seed: int = 0, warm_start: bool = True) -> List[Dict[str, float]]:
    """对 k = 1..k_max 各做一次分解，返回 VAF 与重建误差。

    NMF 是非凸的，独立地对每个 k 求解时 VAF 可能出现轻微回落；
    默认用 warm start（把 k−1 的解加一列作为 k 的初值）来消除这种抖动。
    """
    E = np.asarray(E, dtype=float)
    n, T = E.shape
    k_max = k_max or n
    rng = np.random.default_rng(seed)
    rows = []
    prev: Optional[Tuple[np.ndarray, np.ndarray]] = None
    for k in range(1, k_max + 1):
        init = None
        if warm_start and prev is not None:
            Wp, Hp = prev
            init = (np.hstack([Wp, 0.5 * Wp[:, :1] + EPS]),
                    np.vstack([Hp, 0.5 * Hp[:1] + EPS]))
        W, H = nmf(E, k, iters=iters, seed=seed, init=init)
        prev = (W, H)
        rows.append({"k": k, "vaf": vaf(E, W, H),
                     "sse": float(np.sum((E - W @ H) ** 2))})
    return rows


def k_for_vaf(rows: Sequence[Dict[str, float]], threshold: float = 0.95) -> Optional[int]:
    """达到给定 VAF 阈值所需的最少协同数；达不到返回 None。"""
    for r in rows:
        if r["vaf"] >= threshold:
            return int(r["k"])
    return None


def recovery_check(E: np.ndarray, W_true: np.ndarray, k: int,
                   iters: int = 300, seed: int = 0) -> Dict[str, float]:
    """在已知真值的合成数据上评估恢复质量：VAF + 子空间夹角。"""
    W, H = nmf(E, k, iters=iters, seed=seed)
    return {
        "k": k,
        "vaf": vaf(E, W, H),
        "angle_deg": subspace_angle_deg(W, W_true),
    }
