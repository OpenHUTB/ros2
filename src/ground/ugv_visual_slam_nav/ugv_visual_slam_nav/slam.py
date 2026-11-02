"""EKF-SLAM：状态 = 车辆位姿 + 全部已观测路标坐标。

预测步用轮式里程计，更新步用视觉观测（距离 + 方位）。
路标首次观测时按标准 EKF-SLAM 做状态增广（保留与位姿的相关性），
不用"新路标与位姿独立"的简化近似。
"""

import math

import numpy as np

from . import params as P
from .kinematics import integrate, wrap

SIGMA_R_BASE = 0.05      # 测距基础噪声（米）
SIGMA_R_GAIN = 0.02      # 测距随距离增长项
SIGMA_B = 0.020          # 方位噪声（弧度）

# 过程噪声是一个经过权衡的取值：里程计打滑是"一阶随机偏置"，误差是系统性
# 漂移而非白噪声，Q 大可以让视觉观测更快拉回位姿（夜间角落个例的 0.65 m
# 漂移能修掉）；但 Q 大同时让滤波器过度信任含噪观测，全部 12 个 episode 的
# 末端定位反而普遍劣化到 0.5~2.0 m。实测 0.004² 全局最稳（见文档"结果"），
# 个别角落的残余漂移作为已知局限如实报告，而不是为了个例毁掉整体。
ODOM_Q = np.diag([0.004 ** 2, 0.004 ** 2, 0.012 ** 2])


class EKFSLAM:
    def __init__(self, pose, sigma=1e-2):
        self.mu = np.asarray(pose, dtype=np.float64).reshape(3).copy()
        # sigma 是初始位姿标准差：起点已知就给小值，靠观测收敛就给大值
        self.S = np.eye(3) * float(sigma) ** 2
        self.ids = []            # 状态中第 i 个路标 → 颜色 id
        self.n_updates = 0
        self.n_init = 0
        self.reject = {}         # 路标 id → 连续被拒次数（诊断用）

    # ------------------------------------------------------------ 索引
    def landmark_index(self, lid):
        try:
            return self.ids.index(lid)
        except ValueError:
            return -1

    def landmark_xy(self):
        """已初始化路标的 (x, y, id) 列表。"""
        out = []
        for i, lid in enumerate(self.ids):
            k = 3 + 2 * i
            out.append((self.mu[k], self.mu[k + 1], lid))
        return out

    @property
    def pose(self):
        return self.mu[:3].copy()

    # ------------------------------------------------------------ 预测
    def predict(self, dx_l, dy_l, dth):
        """用里程计的车体系增量做预测步（离线与 ROS 走同一条路径）。"""
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        n = len(self.mu)

        self.mu[0] = x + c * dx_l - s * dy_l
        self.mu[1] = y + s * dx_l + c * dy_l
        self.mu[2] = wrap(th + dth)

        F = np.eye(3)
        F[0, 2] = -s * dx_l - c * dy_l
        F[1, 2] = c * dx_l - s * dy_l

        if n > 3:
            Fa = np.eye(n)
            Fa[:3, :3] = F
            self.S = Fa @ self.S @ Fa.T
        else:
            self.S[:3, :3] = F @ self.S[:3, :3] @ F.T
        self.S[:3, :3] += ODOM_Q

    def predict_twist(self, v, w, dt):
        self.predict(v * dt, 0.0, w * dt)

    # ------------------------------------------------------------ 更新
    def update(self, observations):
        """observations: [(id, range, bearing)]，bearing 为车体坐标系。"""
        for lid, rng, brg in observations:
            k = self.landmark_index(lid)
            if k < 0:
                self._init_landmark(lid, rng, brg)
            else:
                self._correct(k, rng, brg)
        self.S = 0.5 * (self.S + self.S.T)

    def _init_landmark(self, lid, rng, brg):
        x, y, th = self.mu[:3]
        a = th + brg
        mx = x + rng * math.cos(a)
        my = y + rng * math.sin(a)

        Jx = np.array([[1.0, 0.0, -rng * math.sin(a)],
                       [0.0, 1.0, rng * math.cos(a)]])
        Jz = np.array([[math.cos(a), -rng * math.sin(a)],
                       [math.sin(a), rng * math.cos(a)]])

        n = len(self.mu)
        Sxx = self.S[:3, :3]
        Sxm = self.S[:n, :3] @ Jx.T
        R = np.diag([_sig_r(rng) ** 2, SIGMA_B ** 2])
        Smm = Jx @ Sxx @ Jx.T + Jz @ R @ Jz.T

        S2 = np.zeros((n + 2, n + 2))
        S2[:n, :n] = self.S
        S2[:n, n:] = Sxm
        S2[n:, :n] = Sxm.T
        S2[n:, n:] = Smm

        self.mu = np.concatenate([self.mu, [mx, my]])
        self.S = S2
        self.ids.append(lid)
        self.n_init += 1

    def _correct(self, i, rng, brg):
        k = 3 + 2 * i
        x, y, th = self.mu[:3]
        mx, my = self.mu[k], self.mu[k + 1]
        dx, dy = mx - x, my - y
        q = dx * dx + dy * dy
        if q < 1e-6:
            return
        sq = math.sqrt(q)

        z_hat = np.array([sq, wrap(math.atan2(dy, dx) - th)])
        z = np.array([rng, wrap(brg)])

        H = np.zeros((2, len(self.mu)))
        H[0, 0] = -dx / sq
        H[0, 1] = -dy / sq
        H[0, k] = dx / sq
        H[0, k + 1] = dy / sq
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[1, k] = -dy / q
        H[1, k + 1] = dx / q

        R = np.diag([_sig_r(rng) ** 2, SIGMA_B ** 2])
        S = H @ self.S @ H.T + R
        try:
            K = self.S @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        innov = z - z_hat
        innov[1] = wrap(innov[1])
        # 门限：马氏距离超过 3σ（2 自由度，卡方 99.9% 分位 13.8）视为误关联
        try:
            d2 = float(innov @ np.linalg.solve(S, innov))
        except np.linalg.LinAlgError:
            return
        if d2 > 13.8:
            # 被拒的观测不直接丢弃，而是膨胀这个路标的协方差（标准做法）。
            # 里程计航向漂移会让子地图整体"带弯"：路标估计偏 1 m，但协方差
            # 已经收敛得很小，新息恒超门限，EKF 永远改不动 —— 除非有人先
            # 承认"我可能不准了"。膨胀正是这个自我怀疑，几次之后观测重新
            # 被接受，漂移被拉回来。
            self.S[k:k + 2, k:k + 2] *= 1.8
            self.reject[i] = self.reject.get(i, 0) + 1
            return

        self.reject[i] = 0
        self.mu = self.mu + K @ innov
        self.mu[2] = wrap(self.mu[2])
        I_KH = np.eye(len(self.mu)) - K @ H
        self.S = I_KH @ self.S @ I_KH.T + K @ R @ K.T
        self.n_updates += 1


def _sig_r(r):
    return SIGMA_R_BASE + SIGMA_R_GAIN * r
