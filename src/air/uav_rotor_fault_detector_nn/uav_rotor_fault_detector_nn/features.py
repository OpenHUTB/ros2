"""特征构造与滑窗切分。

检测器不直接吃原始图像/激光，而是吃一条**低维时序信号**：机身 IMU 的比力与角速度、
世界系速度、以及控制器下发的四个桨推力。之所以把"指令推力"也放进去，是因为桨失效后
指令与实际推力会分叉——这个分叉本身就是最直接的故障证据。

所有特征都做了物理量归一化（见 ``FEATURE_SCALE``），让网络输入落在 O(1) 附近，
避免训练时靠调学习率去补偿量纲。
"""

from __future__ import annotations

import numpy as np

#: 每个时间步的特征维度与含义
FEATURE_NAMES = (
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "vel_x", "vel_y", "vel_z",
    "cmd_r0", "cmd_r1", "cmd_r2", "cmd_r3",
)

#: 粗归一化系数（比力 m/s²、角速度 rad/s、速度 m/s、推力已按最大推力归一）
FEATURE_SCALE = np.array(
    [12.0, 12.0, 20.0, 6.0, 6.0, 6.0, 4.0, 4.0, 3.0, 1.0, 1.0, 1.0, 1.0],
    dtype=np.float64,
)

FEATURE_DIM = len(FEATURE_NAMES)
ROTOR_COUNT = 4
GRAVITY = 9.80665


def quat_to_matrix(quat) -> np.ndarray:
    """``(w, x, y, z)`` 机体->世界 的四元数转旋转矩阵。

    与底座 ``core.quadrotor.quat_to_mat`` 同约定；这里自带一份实现是为了让
    数据集/推理侧不依赖底座内部函数（数据集可以在没有 MuJoCo 的机器上使用）。
    """
    w, x, y, z = (float(v) for v in np.asarray(quat, dtype=np.float64).reshape(4))
    norm = w * w + x * x + y * y + z * z
    if norm <= 0.0:
        return np.eye(3)
    s = 2.0 / norm
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def specific_force(vel, prev_vel, dt: float, quat) -> np.ndarray:
    """由速度差分反推机身比力（IMU 加速度计读数）。

    ``a_world = dv/dt``；加速度计测的是"除重力外的比力"，所以在世界系里加上
    ``[0, 0, g]`` 后再旋到机体系：``f_body = R^T (a_world + g·ẑ)``。
    """
    a_world = (np.asarray(vel, dtype=np.float64) - np.asarray(prev_vel, dtype=np.float64)) / dt
    f_world = a_world + np.array([0.0, 0.0, GRAVITY])
    return quat_to_matrix(quat).T @ f_world


def build_step_features(acc_body, gyro, vel, cmd_thrusts, max_thrust: float) -> np.ndarray:
    """把一步的物理量拼成归一化特征向量。

    ``cmd_thrusts`` 是**控制器下发**的四个桨推力（故障前），单位 N；
    除以 ``max_thrust`` 后落在 [0, 1]。
    """
    cmd = np.asarray(cmd_thrusts, dtype=np.float64).reshape(ROTOR_COUNT) / max(max_thrust, 1e-9)
    raw = np.concatenate(
        [
            np.asarray(acc_body, dtype=np.float64).reshape(3),
            np.asarray(gyro, dtype=np.float64).reshape(3),
            np.asarray(vel, dtype=np.float64).reshape(3),
            np.clip(cmd, 0.0, 1.5),
        ]
    )
    return raw / FEATURE_SCALE


def make_windows(features: np.ndarray, labels: np.ndarray, window: int, stride: int):
    """按时间切片成滑窗样本。

    返回 ``(X, Y, end_index)``：

    * ``X``: ``(n, window, FEATURE_DIM)``
    * ``Y``: ``(n, ROTOR_COUNT)`` —— 取窗口**最后一帧**的真值作为标签，
      因为检测任务关心"现在这一刻四桨健康度是多少"
    * ``end_index``: ``(n,)`` 每样本在原始序列里的结束下标（求检测延时要用）
    """
    if window < 1:
        raise ValueError(f"window 必须 >= 1，收到 {window}")
    if stride < 1:
        raise ValueError(f"stride 必须 >= 1，收到 {stride}")
    n_steps = features.shape[0]
    if n_steps < window:
        return (
            np.zeros((0, window, FEATURE_DIM), dtype=np.float32),
            np.zeros((0, ROTOR_COUNT), dtype=np.float32),
            np.zeros((0,), dtype=np.int64),
        )

    ends = np.arange(window - 1, n_steps, stride, dtype=np.int64)
    windows = np.stack([features[end - window + 1 : end + 1] for end in ends])
    targets = labels[ends]
    return (
        windows.astype(np.float32),
        targets.astype(np.float32),
        ends,
    )
