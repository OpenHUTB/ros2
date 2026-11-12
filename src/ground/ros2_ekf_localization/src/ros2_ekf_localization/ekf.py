"""扩展卡尔曼滤波（EKF）定位：轮式里程计 + IMU 航向 + 位置观测。

状态量与模型
------------
状态向量取平面位姿

    x = [px, py, theta]^T

控制量取轮式里程计给出的 (v, omega)，差速运动学做一阶欧拉离散：

    px'    = px + v cos(theta) dt
    py'    = py + v sin(theta) dt
    theta' = theta + omega dt

预测步：

    x_hat = f(x, u)
    P_hat = F P F^T + Q,    F = df/dx,  Q = G N G^T,  G = df/du

把过程噪声写成 G N G^T（而不是拍一个常数对角阵）是有意义的：控制噪声
（轮径标定误差、打滑）本来作用在 (v, omega) 上，经 G 映射到状态空间后，
直线行驶时主要污染位置、原地转向时主要污染航向，与物理直觉一致。

更新步支持两类观测：

    IMU 航向   z = theta,          H = [0 0 1]
    位置观测   z = [px, py]^T,     H = [[1 0 0], [0 1 0]]

（GPS / UWB / 视觉路标等给出绝对位置的传感器都归入第二类。）

两个容易被忽略但必须处理的细节：

1. **角度残差归一化**：航向残差必须折算到 (-pi, pi]，否则滤波器会在
   ±pi 边界处被"绕远路"的巨大残差带偏；
2. **协方差数值健康**：连续上千次乘法后 P 可能因浮点误差失去对称性甚至
   出现负特征值，这里每次更新后都做对称化与对角下限保护（safety.py）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .safety import repair_covariance

STATE_DIM = 3


def normalize_angle(angle: float) -> float:
    """把角度折算到 (-pi, pi]（±pi 端点两种符号均为合法表示）。"""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class EKFConfig:
    """滤波器噪声参数（均为 1 sigma）。"""

    # 过程噪声取得比传感器标称白噪声（0.03 m/s）更保守：里程计还存在
    # 轮径标定误差这类**系统性偏差**，滤波器无法建模，只能靠放大过程噪声
    # 维持一致性（NEES 均值接近自由度 3）。见文档第 6 节的调参与诊断实验。
    sigma_v: float = 0.15        # 里程计线速度噪声 (m/s)
    sigma_w: float = 0.15        # 里程计角速度噪声 (rad/s)
    sigma_imu_yaw: float = 0.05  # IMU 航向噪声 (rad)
    sigma_pos: float = 0.15      # 位置观测噪声 (m)
    min_cov: float = 1e-9        # 协方差对角下限，防止过自信
    max_cov: float = 1e6         # 协方差对角上限，防止发散后数值爆炸


class EKFLocalizer:
    """三状态平面位姿的扩展卡尔曼滤波器。"""

    def __init__(
        self,
        config: EKFConfig | None = None,
        x0: np.ndarray | None = None,
        P0: np.ndarray | None = None,
    ) -> None:
        self.config = config or EKFConfig()
        self.x = np.zeros(STATE_DIM, dtype=float) if x0 is None else np.asarray(x0, dtype=float).copy()
        self.P = np.eye(STATE_DIM) * 1e-3 if P0 is None else np.asarray(P0, dtype=float).copy()
        self._P0 = self.P.copy()
        #: 因数值异常而重置协方差的次数，正常运行时应保持为 0
        self.cov_repairs = 0
        #: 被拒绝的观测次数（NaN / 奇异矩阵）
        self.rejected_updates = 0

    # ------------------------------------------------------------------ 预测

    def predict(self, v: float, w: float, dt: float) -> bool:
        """用里程计 (v, w) 做一次状态预测。

        非法输入（非有限值、dt <= 0）直接拒绝，保持状态不变。
        """
        if not (math.isfinite(v) and math.isfinite(w) and math.isfinite(dt)) or dt <= 0.0:
            return False

        cfg = self.config
        th = self.x[2]
        c, s = math.cos(th), math.sin(th)

        # df/dx
        F = np.eye(STATE_DIM)
        F[0, 2] = -v * s * dt
        F[1, 2] = v * c * dt

        # df/du 与过程噪声
        G = np.array([[c * dt, 0.0], [s * dt, 0.0], [0.0, dt]], dtype=float)
        N = np.diag([cfg.sigma_v**2, cfg.sigma_w**2])
        Q = G @ N @ G.T

        self.x[0] += v * c * dt
        self.x[1] += v * s * dt
        self.x[2] = normalize_angle(self.x[2] + w * dt)
        self.P = F @ self.P @ F.T + Q
        self._sanitize()
        return True

    # ------------------------------------------------------------------ 更新

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray, angle_row: int | None = None) -> bool:
        """通用更新：z = H x + v, v ~ N(0, R)。"""
        z = np.asarray(z, dtype=float)
        if not np.all(np.isfinite(z)):
            self.rejected_updates += 1
            return False

        innovation = z - H @ self.x
        if angle_row is not None:
            innovation[angle_row] = normalize_angle(innovation[angle_row])

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.rejected_updates += 1
            return False

        self.x = self.x + K @ innovation
        self.x[2] = normalize_angle(self.x[2])

        # Joseph 形式：比 (I-KH)P 数值更稳定，能保持对称正定性
        I = np.eye(STATE_DIM)
        A = I - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self._sanitize()
        return True

    def update_yaw(self, yaw: float, r: float | None = None) -> bool:
        """用 IMU 航向观测做一次更新。"""
        cfg = self.config
        R = np.array([[cfg.sigma_imu_yaw**2 if r is None else r**2]])
        return self._update(
            np.array([yaw]), np.array([[0.0, 0.0, 1.0]]), R, angle_row=0
        )

    def update_position(self, px: float, py: float, r: float | None = None) -> bool:
        """用绝对位置观测（GPS / UWB / 路标）做一次更新。"""
        cfg = self.config
        s2 = (cfg.sigma_pos if r is None else r) ** 2
        R = np.eye(2) * s2
        return self._update(
            np.array([px, py]), np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]), R
        )

    # ------------------------------------------------------------------ 诊断

    def nees(self, x_true: np.ndarray) -> float:
        """归一化估计误差平方（3 自由度，理想均值约为 3）。"""
        d = np.asarray(x_true, dtype=float) - self.x
        d[2] = normalize_angle(d[2])
        return float(d @ np.linalg.solve(self.P, d))

    def position_error(self, x_true: np.ndarray) -> float:
        """位置分量的欧氏误差 (m)。"""
        d = np.asarray(x_true, dtype=float) - self.x
        return float(math.hypot(d[0], d[1]))

    def heading_error(self, x_true: np.ndarray) -> float:
        """航向误差的绝对值 (rad)。"""
        return abs(normalize_angle(float(np.asarray(x_true)[2]) - self.x[2]))

    def trace_cov(self) -> float:
        """协方差迹，用于观察滤波器置信度变化。"""
        return float(np.trace(self.P))

    # ------------------------------------------------------------------ 内部

    def _sanitize(self) -> None:
        """保证状态有限、协方差对称且有合理量级。"""
        if not np.all(np.isfinite(self.x)):
            self.x = np.zeros(STATE_DIM)
            self.P = self._P0.copy()
            self.cov_repairs += 1
            return

        P, repaired = repair_covariance(self.P, self.config.min_cov, self.config.max_cov)
        self.P = P
        if repaired:
            self.cov_repairs += 1
