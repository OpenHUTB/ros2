"""
水下机器人多源融合导航：误差状态卡尔曼滤波 (ESKF)
===================================================
状态 (15 维误差状态):
    δp(3) 位置, δv(3) 速度, δθ(3) 姿态, δba(3) 加速度计零偏, δbg(3) 陀螺零偏
标称状态:
    p(3), v(3), q(4, wxyz), ba(3), bg(3)

融合来源:
    - IMU (加速度/角速度)  : 预测
    - DVL (机体速度)       : 更新
    - 深度计               : 更新
    - 姿态 (AHRS/磁罗经)   : 更新

只依赖 numpy，方便单元测试与离线回放。
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY_W = np.array([0.0, 0.0, -9.81])

# 误差状态分块索引
IP = slice(0, 3)     # δp
IV = slice(3, 6)     # δv
ITH = slice(6, 9)    # δθ
IBA = slice(9, 12)   # δba
IBG = slice(12, 15)  # δbg


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_mul(a, b):
    """Hamilton 乘积 (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_rotvec(phi):
    """旋转向量 -> 四元数 (小角度稳定)."""
    theta = float(np.linalg.norm(phi))
    if theta < 1e-9:
        return np.array([1.0, 0.5 * phi[0], 0.5 * phi[1], 0.5 * phi[2]])
    axis = phi / theta
    s = math.sin(theta / 2.0)
    return np.array([math.cos(theta / 2.0), s * axis[0], s * axis[1], s * axis[2]])


def quat_to_rot(q):
    """四元数 -> 旋转矩阵 (机体系 -> 世界系)."""
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class ROVEKF:
    """15 状态 ESKF。"""

    def __init__(self, p0, v0, q0, P0_diag=None,
                 sigma_acc=0.05, sigma_gyro=0.01,
                 sigma_ba=1e-4, sigma_bg=1e-5):
        self.p = np.array(p0, dtype=float)
        self.v = np.array(v0, dtype=float)
        self.q = quat_normalize(np.array(q0, dtype=float))
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        if P0_diag is None:
            P0_diag = ([1e-4] * 3 + [1e-2] * 3 + [1e-4] * 3 + [1e-2] * 3 + [1e-4] * 3)
        self.P = np.diag(np.asarray(P0_diag, dtype=float))
        # 连续时间噪声谱密度
        self.sigma_acc = sigma_acc
        self.sigma_gyro = sigma_gyro
        self.sigma_ba = sigma_ba
        self.sigma_bg = sigma_bg
        self.initialized = True

    # ---------------- 预测 ----------------
    def predict(self, a_m, w_m, dt):
        a_m = np.asarray(a_m, dtype=float)
        w_m = np.asarray(w_m, dtype=float)
        R = quat_to_rot(self.q)

        # 标称状态
        a_w = R @ (a_m - self.ba) + GRAVITY_W
        self.p = self.p + self.v * dt + 0.5 * a_w * dt * dt
        self.v = self.v + a_w * dt
        self.q = quat_normalize(quat_mul(self.q, quat_from_rotvec((w_m - self.bg) * dt)))

        # 误差状态雅可比
        F = np.eye(15)
        F[IP, IV] = np.eye(3) * dt
        F[IV, ITH] = -R @ skew(a_m - self.ba) * dt
        F[IV, IBA] = -R * dt
        F[ITH, ITH] = np.eye(3) - skew((w_m - self.bg) * dt)
        F[ITH, IBG] = -np.eye(3) * dt

        Q = np.zeros((15, 15))
        Q[IV, IV] = np.eye(3) * (self.sigma_acc ** 2) * dt * dt
        Q[ITH, ITH] = np.eye(3) * (self.sigma_gyro ** 2) * dt * dt
        Q[IBA, IBA] = np.eye(3) * (self.sigma_ba ** 2) * dt
        Q[IBG, IBG] = np.eye(3) * (self.sigma_bg ** 2) * dt

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    # ---------------- 更新 ----------------
    def _update(self, H, residual, R_meas):
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ residual
        self._inject(dx)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return float(residual @ np.linalg.solve(S, residual))

    def update_dvl(self, v_body_meas, sigma=0.01):
        R = quat_to_rot(self.q)
        h = R.T @ self.v
        H = np.zeros((3, 15))
        H[:, IV] = R.T
        H[:, ITH] = skew(h)
        return self._update(H, np.asarray(v_body_meas) - h, np.eye(3) * sigma * sigma)

    def update_depth(self, depth_meas, sigma=0.02):
        """depth_meas: 向下为正 (m)。"""
        h = -self.p[2]
        H = np.zeros((1, 15))
        H[0, 2] = -1.0
        return self._update(H, np.array([depth_meas - h]), np.array([[sigma * sigma]]))

    def update_attitude(self, q_meas, sigma=0.005):
        q_err = quat_mul(np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]]),
                         quat_normalize(np.asarray(q_meas, dtype=float)))
        if q_err[0] < 0:          # 保持最短路径
            q_err = -q_err
        residual = 2.0 * q_err[1:4]
        H = np.zeros((3, 15))
        H[:, ITH] = np.eye(3)
        return self._update(H, residual, np.eye(3) * sigma * sigma)

    def _inject(self, dx):
        self.p = self.p + dx[IP]
        self.v = self.v + dx[IV]
        self.q = quat_normalize(quat_mul(self.q, quat_from_rotvec(dx[ITH])))
        self.ba = self.ba + dx[IBA]
        self.bg = self.bg + dx[IBG]

    # ---------------- 输出 ----------------
    def position_cov(self):
        return self.P[IP, IP]


class SpaceState:
    """一个只做 IMU 积分的航位推算器，用作对比基线。"""

    def __init__(self, p0, v0, q0):
        self.p = np.array(p0, dtype=float)
        self.v = np.array(v0, dtype=float)
        self.q = quat_normalize(np.array(q0, dtype=float))

    def predict(self, a_m, w_m, dt):
        R = quat_to_rot(self.q)
        a_w = R @ np.asarray(a_m, dtype=float) + GRAVITY_W
        self.p = self.p + self.v * dt + 0.5 * a_w * dt * dt
        self.v = self.v + a_w * dt
        self.q = quat_normalize(quat_mul(self.q, quat_from_rotvec(np.asarray(w_m) * dt)))


class SensorModel:
    """传感器噪声/零偏/丢包模型（仿真节点与测试脚本共用）。"""

    def __init__(self, seed=7,
                 sigma_acc=0.05, sigma_gyro=0.005,
                 bias_acc=0.05, bias_gyro=0.003,
                 sigma_dvl=0.01, bias_dvl=0.005,
                 sigma_depth=0.02, sigma_att=0.01,
                 dvl_dropout=0.10):
        self.rng = np.random.default_rng(seed)
        self.sigma_acc = sigma_acc
        self.sigma_gyro = sigma_gyro
        self.sigma_dvl = sigma_dvl
        self.sigma_depth = sigma_depth
        self.sigma_att = sigma_att
        self.dvl_dropout = dvl_dropout
        self.ba = self.rng.normal(0.0, bias_acc, 3)
        self.bg = self.rng.normal(0.0, bias_gyro, 3)
        self.b_dvl = self.rng.normal(0.0, bias_dvl, 3)

    def _walk(self, dt):
        self.ba += self.rng.normal(0.0, 1e-4, 3) * math.sqrt(dt)
        self.bg += self.rng.normal(0.0, 1e-5, 3) * math.sqrt(dt)

    def imu(self, R, a_world, omega_body, dt):
        """返回 (a_m, w_m)：比力 + 机体角速度，含零偏与噪声。"""
        self._walk(dt)
        a_body = R.T @ (np.asarray(a_world) - GRAVITY_W)
        a_m = a_body + self.ba + self.rng.normal(0.0, self.sigma_acc, 3)
        w_m = np.asarray(omega_body) + self.bg + self.rng.normal(0.0, self.sigma_gyro, 3)
        return a_m, w_m

    def dvl(self, R, v_world):
        """机体坐标系速度；按 dropout 概率返回 None（丢包）。"""
        if self.rng.random() < self.dvl_dropout:
            return None
        v_body = R.T @ np.asarray(v_world) + self.b_dvl
        return v_body + self.rng.normal(0.0, self.sigma_dvl, 3)

    def depth(self, p):
        return -float(p[2]) + float(self.rng.normal(0.0, self.sigma_depth))

    def attitude(self, q):
        phi = self.rng.normal(0.0, self.sigma_att, 3)
        return quat_normalize(quat_mul(np.asarray(q, dtype=float), quat_from_rotvec(phi)))
