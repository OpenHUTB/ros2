"""观测构造：前视深度图 + 机体系状态 + 桨健康度。

观测刻意做成"策略在真实故障机上也能拿到"的量：

* 深度图：前视相机（机体系 -z 以外的 +x 方向），按 4 倍下采样成 32×32；
* 机体系速度：世界系速度左乘姿态矩阵的转置（不要世界系速度，否则策略等于偷看 GPS）；
* 姿态：只给 roll/pitch 的正弦余弦与偏航角速率，不给绝对偏航角；
* 健康度：4 个桨的剩余推力权限（来自故障检测或故障脚本，是端到端控制唯一的"故障输入"）；
* 上一步指令：4 维，让策略知道自己在干什么，避免抖动。

没有世界系位置、没有目标点坐标 —— 任务方向由走廊几何自己给（相机能看到墙）。
"""

from __future__ import annotations

import numpy as np

DEPTH_SIZE = 32                 # 观测深度图边长（64 像素下采样 2 倍）
DEPTH_CHANNELS = 1
PROPRIO_DIM = 16                # 3 速度 + 2 倾角 + 3 角速率 + 4 健康度 + 4 上一步指令
DEPTH_DIM = DEPTH_SIZE * DEPTH_SIZE * DEPTH_CHANNELS
OBS_DIM = DEPTH_DIM + PROPRIO_DIM
MAX_DEPTH = 10.0                # 深度截断（米），超过按远处理
MAX_RATE = 4.0                  # 角速率归一化尺度（rad/s）


def quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    """四元数 (w, x, y, z) → 旋转矩阵（机体→世界）。"""
    w, x, y, z = np.asarray(quat, dtype=float)
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array(
        [
            [1.0 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
            [s * (x * y + w * z), 1.0 - s * (x * x + z * z), s * (y * z - w * x)],
            [s * (x * z - w * y), s * (y * z + w * x), 1.0 - s * (x * x + y * y)],
        ]
    )


def crop_depth(depth: np.ndarray, size: int = DEPTH_SIZE) -> np.ndarray:
    """把相机深度图按 2×2 均值池化到 size×size，并归一化到 [0, 1]。"""
    d = np.asarray(depth, dtype=float)
    if d.ndim != 2:
        raise ValueError(f"深度图应是二维，收到 {d.shape}")
    if d.shape == (size, size):
        pooled = d                      # 已经是目标分辨率，不用池化
    elif d.shape == (size * 2, size * 2):
        pooled = d.reshape(size, 2, size, 2).mean(axis=(1, 3))
    else:
        # 相机分辨率改了也能用：先裁成 size 的整数倍再池化
        h, w = d.shape
        h2, w2 = (h // (2 * size)) * (2 * size), (w // (2 * size)) * (2 * size)
        if h2 >= size and w2 >= size:
            d = d[:h2, :w2]
            pooled = d.reshape(size, h2 // size, size, w2 // size).mean(axis=(1, 3))
        else:
            # 比目标分辨率还小：最近邻复制到 size×size
            idx = (np.arange(size) * d.shape[0] // size).clip(0, d.shape[0] - 1)
            idy = (np.arange(size) * d.shape[1] // size).clip(0, d.shape[1] - 1)
            pooled = d[np.ix_(idx, idy)]
    pooled = np.clip(np.nan_to_num(pooled, nan=MAX_DEPTH, posinf=MAX_DEPTH), 0.0, MAX_DEPTH)
    return (pooled / MAX_DEPTH).astype(np.float32)


def body_velocity(vel: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """世界系速度 → 机体系速度（``R^T v``）。"""
    return quat_to_matrix(quat).T @ np.asarray(vel, dtype=float)


def proprio_features(state, gains: np.ndarray,
                     last_commands: np.ndarray | None = None,
                     max_thrust: float = 6.0) -> np.ndarray:
    """16 维本体感知特征；顺序必须与 ``PROPRIO_NAMES`` 一致。

    三轴角速率（机体系 p/q/r）是必须给的：专家策略的姿态内环就是 ``kw·e_ω``，
    只给倾角不给角速率时网络学不到阻尼，闭环一定发散（实测只给偏航角速率时，
    纯 BC 与 DAgger 都撑不过 2 s）。
    """
    v_b = body_velocity(state.vel, state.quat)
    roll = float(getattr(state, "roll", 0.0))
    pitch = float(getattr(state, "pitch", 0.0))
    omega = np.asarray(state.omega, dtype=float).reshape(3)
    g = np.asarray(gains, dtype=float).reshape(4)
    if last_commands is None:
        cmd = np.full(4, 0.35, dtype=float)
    else:
        cmd = np.asarray(last_commands, dtype=float).reshape(4) / max(max_thrust, 1e-6)
    return np.concatenate(
        [
            np.clip(v_b / 4.0, -1.0, 1.0),
            [np.sin(roll), np.sin(pitch)],
            np.clip(omega / MAX_RATE, -1.0, 1.0),
            np.clip(g, 0.0, 1.0),
            np.clip(cmd, 0.0, 1.0),
        ]
    ).astype(np.float32)


PROPRIO_NAMES = (
    "vel_x_b", "vel_y_b", "vel_z_b",
    "sin_roll", "sin_pitch",
    "rate_x", "rate_y", "rate_z",
    "gain_0", "gain_1", "gain_2", "gain_3",
    "cmd_0", "cmd_1", "cmd_2", "cmd_3",
)


def build_observation(depth: np.ndarray, proprio: np.ndarray) -> dict:
    """打包成策略输入：``depth (1, S, S)`` + ``proprio (14,)``。"""
    d = crop_depth(depth)
    p = np.asarray(proprio, dtype=np.float32).reshape(PROPRIO_DIM)
    return {"depth": d[None, :, :], "proprio": p}


def flatten_observation(obs: dict) -> np.ndarray:
    """给 MLP 基线用的展平观测。"""
    return np.concatenate([obs["depth"].reshape(-1), obs["proprio"]]).astype(np.float32)
