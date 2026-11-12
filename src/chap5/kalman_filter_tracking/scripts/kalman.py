"""一维匀速目标跟踪的卡尔曼滤波（纯标准库，手写 2x2 矩阵运算）。

问题设定
--------
目标沿 x 轴匀速运动，但我们**只能测到位置**（例如激光/视觉测距），且测量含
高斯噪声。卡尔曼滤波的价值在于：即便速度不可直接测量，也能从带噪位置序列中
把速度"估"出来，并给出估计的不确定度。

状态量 x = [p, v]^T

    预测:  x⁻ = F x,        P⁻ = F P Fᵀ + Q
    更新:  K  = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
           x  = x⁻ + K (z - H x⁻)
           P  = (I - K H) P⁻

其中 F = [[1, dt], [0, 1]]，H = [1, 0]，Q 由过程噪声（加速度扰动）决定，
R 为测量噪声方差。

本文件不依赖 numpy：2x2 矩阵乘、转置、求逆都显式写出来，便于逐行对照公式。
"""

import math

# ---------------------------------------------------------------------------
# 确定性伪随机数（Box-Muller 生成高斯），保证每台机器结果一致
# ---------------------------------------------------------------------------


class Rng:
    """线性同余发生器 + Box-Muller 正态采样。"""

    def __init__(self, seed=12345):
        self.state = seed & 0x7FFFFFFF
        self._spare = None

    def uniform(self):
        # Numerical Recipes LCG 参数
        self.state = (1103515245 * self.state + 12345) & 0x7FFFFFFF
        return (self.state + 0.5) / 2147483648.0

    def normal(self, sigma=1.0):
        if self._spare is not None:
            v = self._spare
            self._spare = None
            return v * sigma
        u1 = max(self.uniform(), 1e-12)
        u2 = self.uniform()
        r = math.sqrt(-2.0 * math.log(u1))
        self._spare = r * math.sin(2.0 * math.pi * u2)
        return r * math.cos(2.0 * math.pi * u2) * sigma


# ---------------------------------------------------------------------------
# 2x2 矩阵工具
# ---------------------------------------------------------------------------


def mat_mul(A, B):
    n, m, p = len(A), len(B), len(B[0])
    return [[sum(A[i][k] * B[k][j] for k in range(m)) for j in range(p)]
            for i in range(n)]


def mat_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def mat_sub(A, B):
    return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def transpose(A):
    return [[A[i][j] for i in range(len(A))] for j in range(len(A[0]))]


def inv2(M):
    """2x2 求逆。"""
    a, b = M[0]
    c, d = M[1]
    det = a * d - b * c
    if abs(det) < 1e-18:
        raise ZeroDivisionError("奇异矩阵")
    return [[d / det, -b / det], [-c / det, a / det]]


# ---------------------------------------------------------------------------
# 卡尔曼滤波
# ---------------------------------------------------------------------------


class KalmanCV:
    """匀速（constant velocity）模型的一维卡尔曼滤波。"""

    def __init__(self, dt, q_accel=0.05, r_meas=0.25, p0=1.0, v0=0.0):
        self.dt = dt
        self.F = [[1.0, dt], [0.0, 1.0]]
        self.H = [[1.0, 0.0]]
        # 过程噪声：由加速度白噪声 q_accel (m/s^2) 驱动的离散 Q
        q = q_accel
        d2, d3, d4 = dt * dt, dt ** 3, dt ** 4
        self.Q = [[q * d4 / 4.0, q * d3 / 2.0],
                  [q * d3 / 2.0, q * d2]]
        self.R = r_meas
        self.x = [[p0], [v0]]
        self.P = [[1.0, 0.0], [0.0, 1.0]]

    # -- 预测 --------------------------------------------------------------
    def predict(self):
        self.x = mat_mul(self.F, self.x)
        FP = mat_mul(self.F, self.P)
        self.P = mat_add(mat_mul(FP, transpose(self.F)), self.Q)
        return self.x

    # -- 更新 --------------------------------------------------------------
    def update(self, z):
        H = self.H
        Ht = transpose(H)                      # 2x1
        PHt = mat_mul(self.P, Ht)              # 2x1
        S = mat_mul(H, PHt)[0][0] + self.R     # 标量
        K = [[PHt[0][0] / S], [PHt[1][0] / S]]  # 2x1
        innov = z - self.x[0][0]
        self.x = [[self.x[0][0] + K[0][0] * innov],
                  [self.x[1][0] + K[1][0] * innov]]
        KH = [[K[0][0] * 1.0, 0.0], [K[1][0] * 1.0, 0.0]]
        I = [[1.0, 0.0], [0.0, 1.0]]
        self.P = mat_mul(mat_sub(I, KH), self.P)
        return self.x

    @property
    def pos(self):
        return self.x[0][0]

    @property
    def vel(self):
        return self.x[1][0]

    def pos_std(self):
        return math.sqrt(max(self.P[0][0], 0.0))


# ---------------------------------------------------------------------------
# 仿真
# ---------------------------------------------------------------------------


def simulate(dt=0.05, duration=10.0, v_true=2.0, p0=0.0,
             meas_sigma=0.5, accel_sigma=0.15, seed=2024,
             q_accel=0.05, r_meas=0.25):
    """真值匀速 + 加速度扰动，测量只含位置噪声。

    返回 dict：ts / p_true / v_true / z / p_hat / v_hat / p_std
    """
    rng = Rng(seed)
    kf = KalmanCV(dt, q_accel=q_accel, r_meas=r_meas, p0=p0, v0=0.0)
    ts, p_true_l, v_true_l, z_l, p_hat, v_hat, p_std = [], [], [], [], [], [], []
    p, v = p0, v_true
    n = int(round(duration / dt))
    for k in range(n):
        t = k * dt
        # 真实运动：轻微加速度扰动
        v += rng.normal(accel_sigma) * dt
        p += v * dt
        z = p + rng.normal(meas_sigma)          # 只能测位置
        kf.predict()
        kf.update(z)
        ts.append(t)
        p_true_l.append(p)
        v_true_l.append(v)
        z_l.append(z)
        p_hat.append(kf.pos)
        v_hat.append(kf.vel)
        p_std.append(kf.pos_std())
    return {"ts": ts, "p_true": p_true_l, "v_true": v_true_l, "z": z_l,
            "p_hat": p_hat, "v_hat": v_hat, "p_std": p_std}


def rmse(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))
