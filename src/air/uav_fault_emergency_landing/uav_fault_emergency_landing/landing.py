"""迫降制导律（专家）、特征构造与落点选择。

分三层：

- **选落点**：:func:`select_target` 从评分图里挑格子。评分图可以来自规则老师
  （:mod:`mapping`）或神经网络（:mod:`model`），接口一样。
- **专家律**：:func:`expert_velocity` 是手写的迫降速度指令。它只依赖相对落点的位置、
  当前速度、高度与四桨推力权限，所以可以直接当神经网络的回归标签（行为克隆）。
- **特征**：:func:`guidance_features` 把上面这些量拼成固定顺序的 13 维向量。

为什么让网络输出**速度指令**而不是推力或加速度：底层几何控制器已经负责姿态与角速率
内环，网络只接管"往哪飞、多快下降"这一层。这样即使网络在训练分布之外给出离谱的输出，
最多是飞得难看，不会因为错误的力矩指令把机体翻过来——迫降场景里这一点比精度更重要。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

#: 制导网络输入特征（顺序固定，训练/部署都从这里取）
GUIDANCE_FEATURES = (
    "rel_x", "rel_y", "rel_z",      # 落点相对位置（m，z 为 0 时表示已在地面高度）
    "vx", "vy", "vz",               # 世界系速度（m/s）
    "g0", "g1", "g2", "g3",         # 四桨推力权限
    "altitude",                     # 离地高度（m）
    "horiz_dist",                   # 水平距离（m）
    "authority",                    # sum(g)/4，剩余推力权限占比
)
GUIDANCE_DIM = len(GUIDANCE_FEATURES)


@dataclass(frozen=True)
class LandingConfig:
    """迫降参数（专家律与安全判据共用）。"""

    #: 目标触地高度（m）：判定"落地"的高度
    touch_z: float = 0.12
    #: 最高水平接近速度（m/s）
    max_hspeed: float = 2.5
    #: 最高下降速度（m/s）
    max_descent: float = 2.2
    #: 末端（低空）下降速度（m/s）
    touch_descent: float = 0.55
    #: 低空减速高度（m）：低于它开始收敛水平速度与下降速度
    flare_height: float = 1.0
    #: 水平接近增益（1/s）
    k_xy: float = 0.9
    #: 认为"还在飞"的最低推力权限占比，低于它就别指望悬停了
    min_authority: float = 0.55


def guidance_features(rel: np.ndarray, vel: np.ndarray, gains: np.ndarray,
                      altitude: float) -> np.ndarray:
    """构造 13 维制导特征。``rel`` 是落点相对位置（落点 − 机体）。"""
    rel = np.asarray(rel, dtype=float).reshape(3)
    vel = np.asarray(vel, dtype=float).reshape(3)
    gains = np.asarray(gains, dtype=float).reshape(4)
    altitude = float(max(altitude, 0.0))
    feat = np.empty(GUIDANCE_DIM, dtype=np.float32)
    feat[0:3] = rel
    feat[3:6] = vel
    feat[6:10] = gains
    feat[10] = altitude
    feat[11] = float(np.hypot(rel[0], rel[1]))
    feat[12] = float(np.clip(gains.sum() / 4.0, 0.0, 1.0))
    return feat


def expert_velocity(rel: np.ndarray, vel: np.ndarray, gains: np.ndarray,
                    altitude: float, cfg: Optional[LandingConfig] = None) -> np.ndarray:
    """专家迫降律：给定期望速度 (vx, vy, vz)。

    三段式，参数都随"剩余推力权限"缩放——权限越低越不能悬停，只能更早、更快地落下去：

    1. **水平接近**：``v_xy = k_xy · rel_xy``，上皮 ``max_hspeed · (0.4 + 0.6 authority)``；
    2. **下降**：高度越高降得越快，``v_z = -(touch_descent + (max_descent-touch_descent)·alt/3)``，
       权限不足时额外加快；
    3. **末端拉平（flare）**：高度低于 ``flare_height`` 时水平速度向 0 收敛、下降速度收敛到
       ``touch_descent``，让触地速度落在安全包线内。
    """
    cfg = cfg or LandingConfig()
    rel = np.asarray(rel, dtype=float).reshape(3)
    vel = np.asarray(vel, dtype=float).reshape(3)
    gains = np.asarray(gains, dtype=float).reshape(4)
    rel[2] = -abs(rel[2]) if rel[2] != 0 else -float(altitude)
    del vel  # 当前速度只用于观测特征；专家律本身不含速度反馈，避免和底层内环打架

    authority = float(np.clip(gains.sum() / 4.0, 0.0, 1.0))
    alt = max(float(altitude), 0.0)

    # 1) 水平接近
    v_xy = cfg.k_xy * rel[:2]
    hspeed_cap = cfg.max_hspeed * (0.4 + 0.6 * authority)
    hnorm = float(np.hypot(v_xy[0], v_xy[1]))
    if hnorm > hspeed_cap > 0:
        v_xy *= hspeed_cap / hnorm

    # 2) 下降
    frac = float(np.clip(alt / 3.0, 0.0, 1.0))
    v_z = -(cfg.touch_descent + (cfg.max_descent - cfg.touch_descent) * frac)
    if authority < cfg.min_authority:
        # 推力不够悬停时必须更快落地，否则会以更大的速度撞下来
        shortfall = (cfg.min_authority - authority) / cfg.min_authority
        v_z -= 0.6 * shortfall * cfg.max_descent

    # 3) 末端拉平
    if alt < cfg.flare_height:
        scale = float(np.clip(alt / max(cfg.flare_height, 1e-6), 0.0, 1.0))
        v_xy = v_xy * scale
        v_z = -(cfg.touch_descent + (abs(v_z) - cfg.touch_descent) * scale)

    return np.array([v_xy[0], v_xy[1], v_z], dtype=float)


def select_target(score: np.ndarray, height_map, position_xy: Optional[np.ndarray] = None,
                  min_score: float = 0.15,
                  max_distance: float = 12.0) -> Optional[Tuple[int, int, float]]:
    """在评分图上挑落点，返回 ``(row, col, score)``；实在没有可降落处返回 None。

    除了分数，还按"离当前位置的距离"做一个温和的惩罚项：迫降时间有限，
    远处的满分落点不一定比近处的次优落点划算。
    """
    from . import mapping as M

    if score is None or score.size == 0:
        return None
    cfg = height_map.cfg
    work = np.array(score, dtype=float, copy=True)
    if position_xy is not None:
        R, C = work.shape
        rows, cols = np.meshgrid(np.arange(R), np.arange(C), indexing="ij")
        cx = cfg.origin_xy[0] + (rows + 0.5) * cfg.cell_m
        cy = cfg.origin_xy[1] + (cols + 0.5) * cfg.cell_m
        d = np.hypot(cx - float(position_xy[0]), cy - float(position_xy[1]))
        work = work * np.clip(1.0 - d / max(max_distance, 1e-6), 0.0, 1.0)
    idx = int(np.argmax(work))
    row, col = np.unravel_index(idx, work.shape)
    if float(score[row, col]) < min_score:
        return None
    return int(row), int(col), float(score[row, col])


def touchdown_ok(vertical_speed: float, horizontal_speed: float, tilt_deg: float,
                 on_ground: bool, cfg: Optional[LandingConfig] = None) -> bool:
    """触地是否算"安全迫降"。"""
    cfg = cfg or LandingConfig()
    return bool(on_ground and vertical_speed <= 1.2 and horizontal_speed <= 1.0
                and tilt_deg <= 35.0)
