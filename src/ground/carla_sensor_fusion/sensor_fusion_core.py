"""
sensor_fusion_core.py —— 多传感器真融合共享核心
=====================================================
同一套「标定 + 时间同步 + 扩展卡尔曼滤波(EKF)融合」逻辑被两个平台复用：
  - carla_sensor_fusion.py : CARLA 渲染版（本机 CARLA 挂 camera+lidar+imu）
  - sim_2d_fusion.py       : 轻量 2D 合成传感器仿真（可直接跑，生成效果动图）

目标：对单个移动目标做融合跟踪。
  - 雷达(Radar, ~10Hz)：测距 r + 方位 θ_body（体坐标系，测距较准）
  - 相机(Camera, ~30Hz)：测方位 θ_body（仅方位，测距弱，噪声大）
  - IMU(~100Hz)：估计载体姿态角 φ̂（含偏置+噪声），把各传感器体坐标观测
    统一转到世界系（载体运动补偿），再送入 EKF
  - EKF(20Hz)：世界系 4 态 [px,py,vx,vy]，匀速预测，分别用雷达/相机观测更新

只依赖 numpy。
"""

import numpy as np


# ================================================================ 工具
def rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ================================================================ 外参标定
def estimate_extrinsic_2d(pts_cam, pts_radar):
    """最小二乘估计相机→雷达的 2D 外参 [Δθ, tx, ty]。
    p_radar ≈ R(Δθ) @ p_cam + t
    用于把相机坐标系观测对齐到雷达坐标系（标定流程）。
    返回 (theta_deg, t_x, t_y)，以及拟合残差。"""
    A = np.hstack([np.column_stack([pts_cam[:, 0], pts_cam[:, 1]]),
                   np.ones((len(pts_cam), 1))])     # (N,3)
    # 逐点方程：p_radar = R p_cam + t → 线性化于 Δθ≈0 附近迭代
    theta = 0.0
    for _ in range(10):                              # 高斯-牛顿
        R = rot2(theta)
        Rp = (R @ pts_cam.T).T                      # (N,2)
        # 残差 e = p_radar - (Rp + t)；对 theta 的雅可比 = d(R p)/dθ
        J_theta = (-np.sin(theta) * pts_cam[:, 0] - np.cos(theta) * pts_cam[:, 1],
                   np.cos(theta) * pts_cam[:, 0] - np.sin(theta) * pts_cam[:, 1])
        J_theta = np.column_stack(J_theta)          # (N,2)
        M = np.hstack([J_theta, np.ones((len(pts_cam), 1)) * [0, 0]])  # 占位
        # 简化：直接构造 [J_theta | I2]
        M = np.hstack([J_theta.reshape(-1, 1),
                       np.kron(np.ones((len(pts_cam), 1)), np.eye(2))])
        r = (pts_radar - Rp).reshape(-1)            # 2N
        # 求解 3 参数 [Δθ, tx, ty]
        delta = np.linalg.lstsq(M, r, rcond=None)[0]
        theta += delta[0]
        t = delta[1:3]
    R = rot2(theta)
    res = pts_radar - ((R @ pts_cam.T).T + t)
    return np.degrees(theta), t[0], t[1], np.mean(np.linalg.norm(res, axis=1))


# ================================================================ 时间同步
class SyncBuffer:
    """多传感器时间同步：每个传感器按自身采样率写入 (t, obs)，
    融合网格上按『最近时间戳』取各传感器观测，超时阈值内视为同步。"""

    def __init__(self, tol=0.05):
        self.tol = tol
        self.hist = {}            # name -> list[(t, obs)]

    def push(self, name, t, obs):
        self.hist.setdefault(name, []).append((t, obs))

    def get(self, name, t_query):
        """返回 (t, obs)，取时间戳距 t_query 最近且在 tol 内的观测；否则 None。"""
        arr = self.hist.get(name, [])
        if not arr:
            return None
        ts = np.array([a[0] for a in arr])
        idx = int(np.argmin(np.abs(ts - t_query)))
        if abs(ts[idx] - t_query) > self.tol:
            return None
        return arr[idx]

    def age(self, name, t_query):
        """该传感器最近观测的时间戳距今差（秒）；无则 inf。"""
        arr = self.hist.get(name, [])
        if not arr:
            return float("inf")
        return abs(arr[-1][0] - t_query)


# ================================================================ EKF 融合
class EKF2D:
    """世界系 4 态目标跟踪滤波器 [px, py, vx, vy]。
    观测都先经载体姿态补偿转到世界系：θ_world = θ_body + phi_est。"""

    def __init__(self, x0, y0, q_pos=0.3, q_vel=0.6,
                 sig_r=0.5, sig_theta_radar=0.10, sig_theta_cam=0.04):
        self.x = np.array([x0, y0, 0.0, 0.0], dtype=float)
        self.P = np.diag([4.0, 4.0, 2.0, 2.0])
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])
        self.sig_r = sig_r
        self.sig_theta_radar = sig_theta_radar
        self.sig_theta_cam = sig_theta_cam

    def predict(self, dt):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def _theta_h(self):
        px, py = self.x[0], self.x[1]
        r2 = px * px + py * py + 1e-9
        return np.arctan2(py, px), r2, px, py

    def update_radar(self, r, theta_world):
        # 观测 [r, θ_world]（雷达原始是体坐标 θ_body，已由 IMU 转世界）
        px, py = self.x[0], self.x[1]
        r2 = px * px + py * py + 1e-9
        r_pred = np.sqrt(r2)
        theta_pred = np.arctan2(py, px)
        z = np.array([r, theta_world])
        h = np.array([r_pred, theta_pred])
        H = np.array([[px / r_pred, py / r_pred, 0, 0],
                      [-py / r2, px / r2, 0, 0]])
        R = np.diag([self.sig_r ** 2, self.sig_theta_radar ** 2])
        innov = np.array([z[0] - h[0], wrap_angle(z[1] - h[1])])  # 方位需回绕
        self._apply(z, h, H, R, innov)

    def update_camera(self, theta_world):
        px, py = self.x[0], self.x[1]
        r2 = px * px + py * py + 1e-9
        theta_pred = np.arctan2(py, px)
        z = np.array([theta_world])
        h = np.array([theta_pred])
        H = np.array([[-py / r2, px / r2, 0, 0]])
        R = np.diag([self.sig_theta_cam ** 2])
        innov = np.array([wrap_angle(z[0] - h[0])])
        self._apply(z, h, H, R, innov)

    def _apply(self, z, h, H, R, innov):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(4) - K @ H) @ self.P

    def state(self):
        return self.x.copy()

    def covariance(self):
        return self.P.copy()
