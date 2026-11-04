"""安全辅助：传感器超时跟踪、噪声参数校验与协方差数值修复。

与 EKF 核心一样不依赖 rclpy，便于在无 ROS 环境下被单元测试覆盖。

三类问题由它兜底：

1. 话题断流：滤波器若继续沿用上一帧里程计/观测，等于在错误的时间戳上
   递推，这里按数据源记录最后到达时刻，超时后调用方应停止预测或停止更新。
2. 噪声参数非法：sigma 为 0 会让滤波器过度自信（协方差迅速塌缩，
   之后任何观测都拉不动状态），为负则直接让协方差失去正定性。
3. 协方差数值异常：连续递推后 P 可能因浮点误差失去对称性甚至出现负
   特征值，导致 inv(S) 得到错误结果，这里统一修复。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class FreshnessTracker:
    """跟踪若干数据源的最后到达时刻，判定其是否已过期（时间单位为秒）。"""

    timeouts: dict[str, float] = field(default_factory=dict)
    last_seen: dict[str, float | None] = field(default_factory=dict)

    def register(self, source: str, timeout: float) -> None:
        self.timeouts[source] = float(timeout)
        self.last_seen.setdefault(source, None)

    def mark(self, source: str, now: float) -> None:
        self.last_seen[source] = float(now)

    def age(self, source: str, now: float) -> float:
        """距上一帧过去了多久；从未收到过返回 inf。"""
        last = self.last_seen.get(source)
        if last is None:
            return math.inf
        return max(float(now) - last, 0.0)

    def is_stale(self, source: str, now: float) -> bool:
        return self.age(source, now) > self.timeouts.get(source, math.inf)

    def is_fresh(self, source: str, now: float) -> bool:
        return not self.is_stale(source, now)


def sanitize_sigma(
    value: float,
    default: float,
    minimum: float = 1e-6,
    maximum: float = 1e3,
) -> tuple[float, str | None]:
    """校验噪声标准差，返回 (可用值, 警告信息或 None)。

    非有限值、小于等于 0 或超过上限的都回退到 default。
    sigma 取 0 是一个典型错误：滤波器会把自己的预测当成绝对真理。
    """
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(default), f"噪声参数无法解析为数值: {value!r}，已回退 {default}"
    if not math.isfinite(v):
        return float(default), f"噪声参数非法 ({value!r})，已回退 {default}"
    if v <= minimum:
        return float(default), f"噪声参数 {v} 过小（会导致滤波器过自信），已回退 {default}"
    if v > maximum:
        return float(default), f"噪声参数 {v} 超过上限 {maximum}，已回退 {default}"
    return v, None


def repair_covariance(
    P: np.ndarray, min_diag: float = 1e-9, max_diag: float = 1e6
) -> tuple[np.ndarray, bool]:
    """修复协方差矩阵，返回 (修复后的 P, 是否发生过修复)。

    - 非有限值 -> 重置为单位阵
    - 强制对称：(P + P^T) / 2，消除浮点乘法累积出的非对称部分
    - 对角线夹紧到 [min_diag, max_diag]，避免过自信与数值爆炸
    """
    P = np.asarray(P, dtype=float)
    repaired = False

    if not np.all(np.isfinite(P)) or P.shape != (3, 3):
        return np.eye(3) * max(min_diag, 1e-3), True

    # 对称化总是执行（无损），但只有在非对称量确实超出浮点误差时才计入 repaired，
    # 否则协方差修复计数会在每次迭代都被误触发，失去诊断意义。
    asym = float(np.max(np.abs(P - P.T)))
    scale = max(float(np.max(np.abs(P))), 1.0)
    P = 0.5 * (P + P.T)
    if asym > 1e-10 * scale:
        repaired = True

    diag = np.diag(P).copy()
    if np.any(diag < min_diag) or np.any(diag > max_diag):
        P = P.copy()
        np.fill_diagonal(P, np.clip(diag, min_diag, max_diag))
        repaired = True

    return P, repaired
