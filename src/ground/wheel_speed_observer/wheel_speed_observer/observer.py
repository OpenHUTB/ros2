"""用轮速传感器 + 龙伯格观测器估计传动轴扭角。

模型（电机转速由编码器测到，当作已知输入 ``u = omega_m``）::

    x1 = 轴扭转角 theta（rad）
    x2 = 车轮转速 omega_w（rad/s）
    u  = 电机转速 omega_m（rad/s）

    x1' = u - x2
    x2' = (k*x1 + c*(u - x2) - Tl) / Jw

因此 ``A = [[0, -1], [k/Jw, -c/Jw]]``、``B = [[1], [c/Jw]]``、
``C = [0, 1]``（只测车轮转速，例如 ABS 齿圈）。

观测器 ``x_hat' = A x_hat + B u + L (y - C x_hat)``，用**对偶**方式
（``A^T``、``C^T`` 上做 Ackermann）设计 ``L``。只用 Python 标准库。
"""

import math
import random

BYTE_TOLERANCE = 1e-9


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b)))
             for j in range(len(b[0]))] for i in range(len(a))]


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def mat_sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def mat_scale(a, k):
    return [[value * k for value in row] for row in a]


def identity(n=2):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def transpose(a):
    return [[a[j][i] for j in range(len(a))] for i in range(len(a[0]))]


def mat_vec(a, v):
    return [sum(a[i][j] * v[j] for j in range(len(v))) for i in range(len(a))]


def inverse2(a):
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    if abs(det) < 1e-15:
        raise ValueError('矩阵奇异，无法求逆')
    return [[a[1][1] / det, -a[0][1] / det], [-a[1][0] / det, a[0][0] / det]]


def eigenvalues2(a):
    trace = a[0][0] + a[1][1]
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    discriminant = trace * trace - 4.0 * det
    if discriminant >= 0.0:
        root = math.sqrt(discriminant)
        return [complex((trace + root) / 2.0), complex((trace - root) / 2.0)]
    root = math.sqrt(-discriminant)
    return [complex(trace / 2.0, root / 2.0), complex(trace / 2.0, -root / 2.0)]


def ackermann_gain(a, b, poles):
    """2 阶系统 Ackermann 极点配置，返回 1x2 增益（``poles`` 为两个期望极点）。"""
    if len(poles) != 2:
        raise ValueError('只支持 2 阶系统')
    ab = mat_mul(a, b)
    ctrb = [[b[0][0], ab[0][0]], [b[1][0], ab[1][0]]]
    det = ctrb[0][0] * ctrb[1][1] - ctrb[0][1] * ctrb[1][0]
    if abs(det) < BYTE_TOLERANCE:
        raise ValueError('系统不可控，无法配置极点')

    a1 = -(poles[0] + poles[1]).real
    a0 = (poles[0] * poles[1]).real
    ident = identity(2)
    phi = mat_add(mat_add(mat_mul(a, a), mat_scale(a, a1)), mat_scale(ident, a0))
    return mat_mul(mat_mul([[0.0, 1.0]], inverse2(ctrb)), phi)


class DrivetrainObserver:
    """电机转速已知、只测车轮转速时的扭角观测器。"""

    def __init__(self, jw=0.6, k=1200.0, c=2.0):
        for name, value in (('Jw', jw), ('k', k)):
            if value <= 0.0:
                raise ValueError(f'{name} 必须为正数')
        if c < 0.0:
            raise ValueError('c 不能为负')

        self.jw = float(jw)
        self.k = float(k)
        self.c = float(c)
        self.a = [[0.0, -1.0], [k / jw, -c / jw]]
        self.b = [[1.0], [c / jw]]
        self.c_matrix = [[0.0, 1.0]]

    # ---------- 能观性 ----------
    def observability_matrix(self):
        ca = mat_mul(self.c_matrix, self.a)
        return [self.c_matrix[0][:], ca[0][:]]

    def observability_determinant(self):
        matrix = self.observability_matrix()
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]

    def is_observable(self, tolerance=1e-9):
        return abs(self.observability_determinant()) > tolerance

    def open_loop_poles(self):
        return eigenvalues2(self.a)

    # ---------- 观测器设计（对偶 Ackermann）----------
    def design(self, wn=None, zeta=0.7):
        """按期望的自然频率/阻尼比设计 ``L``；``wn`` 省略时取开环自然频率的 3 倍。"""
        if wn is None:
            wn = 3.0 * math.sqrt(self.k / self.jw)
        if not 0.0 < zeta < 1.0:
            raise ValueError('zeta 必须在 (0, 1) 内')
        real = -zeta * wn
        imag = wn * math.sqrt(1.0 - zeta * zeta)
        poles = (complex(real, imag), complex(real, -imag))

        a_t = transpose(self.a)
        c_t = transpose(self.c_matrix)
        gain_t = ackermann_gain(a_t, c_t, poles)      # 1x2
        gain = transpose(gain_t)                      # 2x1
        return gain, poles

    def observer_poles(self, gain):
        lc = mat_mul(gain, self.c_matrix)
        return eigenvalues2(mat_sub(self.a, lc))

    # ---------- 仿真 ----------
    def simulate(self, gain, duration=0.5, dt=1e-4, omega_m_func=None,
                 noise_std=0.0, seed=7, initial_error=(0.02, 0.5)):
        """返回 ``[(t, theta_true, theta_hat, error_theta, omega_w)]``。

        ``initial_error`` 是观测器初值与真值的差；``noise_std`` 作用于车轮转速测量。
        """
        rng = random.Random(seed)
        omega_m_func = omega_m_func or (lambda t: 20.0 * math.sin(2.0 * math.pi * t))

        truth = [0.0, 0.0]
        estimate = [truth[0] + initial_error[0], truth[1] + initial_error[1]]
        steps = int(round(duration / dt))
        record = max(1, int(round(0.005 / dt)))
        samples = []

        def derivative_true(state, u):
            theta, omega_w = state
            dtheta = u - omega_w
            domega = (self.k * theta + self.c * (u - omega_w)) / self.jw
            return [dtheta, domega]

        def derivative_estimate(state, u, measured, gain):
            theta, omega_w = state
            dtheta = u - omega_w
            domega = (self.k * theta + self.c * (u - omega_w)) / self.jw
            correction = gain[0][0] * (measured - omega_w)
            return [dtheta + correction,
                    domega + gain[1][0] * (measured - omega_w)]

        for step in range(steps + 1):
            t = step * dt
            u = omega_m_func(t)
            measurement = truth[1] + (rng.gauss(0.0, noise_std) if noise_std else 0.0)

            if step % record == 0:
                samples.append((t, truth[0], estimate[0],
                                estimate[0] - truth[0], truth[1]))

            # 用 RK2（中点法）推进，步长足够小
            k1t = derivative_true(truth, u)
            mid_t = [truth[i] + 0.5 * dt * k1t[i] for i in range(2)]
            k2t = derivative_true(mid_t, u)
            truth = [truth[i] + dt * k2t[i] for i in range(2)]

            k1e = derivative_estimate(estimate, u, measurement, gain)
            mid_e = [estimate[i] + 0.5 * dt * k1e[i] for i in range(2)]
            k2e = derivative_estimate(mid_e, u, measurement, gain)
            estimate = [estimate[i] + dt * k2e[i] for i in range(2)]

        return samples


def error_settle_time(samples, threshold=0.1):
    """扭角估计误差 |e| 最后一次超过 ``threshold * 初始误差`` 的时刻。"""
    initial = abs(samples[0][3])
    if initial == 0.0:
        return 0.0
    band = threshold * initial
    last = None
    for t, _true, _hat, error, _omega in samples:
        if abs(error) > band:
            last = t
    if last is None:
        return 0.0
    if last >= samples[-1][0] - 1e-12:
        return None
    step = samples[1][0] - samples[0][0]
    return last + step
