"""神经网络残差分配器的输入特征与标签定义。

输入只允许用**在线可得**的量：几何控制器下发的期望 (T, τ)、健康度估计
``ĝ``、以及近似分配 ``u_ls``。标签是"精确箱约束 QP 的解与 ``u_ls`` 之差"，
所以网络学的是**近似分配在饱和工况下的修正量**，而不是从零学控制。

.. math::

    \\mathbf x = \\big[\\tfrac{T}{4T_{max}},\\ \\tfrac{\\tau}{4T_{max}a},\\ \\mathbf g,\\ \\tfrac{\\mathbf u_{ls}}{T_{max}}\\big] \\in \\mathbb R^{12}

    \\mathbf y = \\frac{\\mathbf u^{\\star} - \\mathbf u_{ls}}{T_{max}} \\in \\mathbb R^{4}
"""

from __future__ import annotations

import numpy as np

#: 特征顺序（写进检查点，推理时按名字对齐）
FEATURE_NAMES = (
    "thrust_norm",
    "tau_x_norm", "tau_y_norm", "tau_z_norm",
    "gain_0", "gain_1", "gain_2", "gain_3",
    "u_ls_0", "u_ls_1", "u_ls_2", "u_ls_3",
)
FEATURE_DIM = len(FEATURE_NAMES)
ROTOR_COUNT = 4


def build_features(desired, gains, u_ls, params) -> np.ndarray:
    """把一个时刻的状态压成 12 维特征向量。"""
    t_max = float(params.max_thrust)
    arm = float(params.arm)
    d = np.asarray(desired, dtype=float)
    g = np.asarray(gains, dtype=float)
    u = np.asarray(u_ls, dtype=float)

    scale_thrust = 4.0 * t_max
    scale_torque = max(4.0 * t_max * arm, 1e-9)

    features = np.empty(FEATURE_DIM, dtype=np.float32)
    features[0] = d[0] / scale_thrust
    features[1:4] = d[1:4] / scale_torque
    features[4:8] = g
    features[8:12] = u / max(t_max, 1e-9)
    return features


def build_target(u_star, u_ls, params) -> np.ndarray:
    """标签：精确解相对近似解的残差，按单桨最大推力归一化。"""
    t_max = float(params.max_thrust)
    return ((np.asarray(u_star, dtype=float) - np.asarray(u_ls, dtype=float))
            / max(t_max, 1e-9)).astype(np.float32)


def apply_residual(u_ls, residual, params) -> np.ndarray:
    """推理：近似解 + 网络残差（乘回物理单位）。"""
    t_max = float(params.max_thrust)
    return np.asarray(u_ls, dtype=float) + np.asarray(residual, dtype=float) * t_max
