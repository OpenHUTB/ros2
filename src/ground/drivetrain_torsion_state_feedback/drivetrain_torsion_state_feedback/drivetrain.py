"""双惯量弹性传动的扭振模型与状态反馈极点配置。

模型（只保留扭振模态，忽略整车纵向运动）::

    x1 = 轴扭转角 theta（rad）
    x2 = 相对转速 omega_m - omega_w（rad/s）
    u  = 电机转矩指令（Nm）
    S  = 1/Jm + 1/Jw

    x1' = x2
    x2' = S*(u - k*x1 - c*x2)

于是 ``A = [[0, 1], [-S*k, -S*c]]``、``B = [[0], [S]]``，
``wn = sqrt(S*k)``、``2*zeta*wn = S*c``。用状态反馈 ``u = -K x`` 把闭环极点
放到期望位置，把扭振阻尼比从 ~0.13 提到 0.7 以上。只用 Python 标准库。
"""

import math


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


class TwoInertiaDrivetrain:
    """电机（Jm）经弹簧阻尼轴（k, c）驱动车轮（Jw）的扭振模型。"""

    def __init__(self, jm=0.05, jw=0.6, k=1200.0, c=2.0):
        for name, value in (('Jm', jm), ('Jw', jw), ('k', k)):
            if value <= 0.0:
                raise ValueError(f'{name} 必须为正数')
        if c < 0.0:
            raise ValueError('c 不能为负')

        self.jm = float(jm)
        self.jw = float(jw)
        self.k = float(k)
        self.c = float(c)
        self.s = 1.0 / jm + 1.0 / jw              # S = 1/Jm + 1/Jw
        self.wn_squared = self.s * k
        self.damping_coefficient = self.s * c
        self.a = [[0.0, 1.0], [-self.wn_squared, -self.damping_coefficient]]
        self.b = [[0.0], [self.s]]

    # ---------- 频域特性 ----------
    @property
    def natural_frequency(self):
        return math.sqrt(self.wn_squared)

    @property
    def damping_ratio(self):
        return self.damping_coefficient / (2.0 * self.natural_frequency)

    def poles(self):
        return eigenvalues2(self.a)

    def critical_damping(self):
        """把阻尼比补到 1.0（临界阻尼）所需的轴阻尼系数。"""
        return 2.0 * self.natural_frequency / self.s

    # ---------- 可控性 ----------
    def controllability_matrix(self):
        ab = mat_mul(self.a, self.b)
        return [[self.b[0][0], ab[0][0]], [self.b[1][0], ab[1][0]]]

    def controllability_determinant(self):
        matrix = self.controllability_matrix()
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]

    def is_controllable(self, tolerance=1e-9):
        return abs(self.controllability_determinant()) > tolerance

    # ---------- 极点配置（Ackermann）----------
    @staticmethod
    def desired_poles(wn, zeta):
        if not 0.0 < zeta < 1.0:
            raise ValueError('zeta 必须在 (0, 1) 内，才能得到共轭复极点')
        real = -zeta * wn
        imag = wn * math.sqrt(1.0 - zeta * zeta)
        return complex(real, imag), complex(real, -imag)

    def ackermann_gain(self, poles):
        """按 Ackermann 公式给 2 阶系统算状态反馈增益 ``K``（1x2）。"""
        if len(poles) != 2:
            raise ValueError('只支持 2 阶系统')
        if not self.is_controllable():
            raise ValueError('系统不可控，无法配置极点')

        a1 = -(poles[0] + poles[1]).real          # s^2 + a1*s + a0
        a0 = (poles[0] * poles[1]).real
        ident = identity(2)
        phi = mat_add(mat_add(mat_mul(self.a, self.a), mat_scale(self.a, a1)),
                      mat_scale(ident, a0))
        selector = [[0.0, 1.0]]
        inverse_ctrb = inverse2(self.controllability_matrix())
        return mat_mul(mat_mul(selector, inverse_ctrb), phi)

    def closed_loop(self, gain):
        """返回 ``(A-BK, 极点, 阻尼比, 自然频率)``。"""
        bk = mat_mul(self.b, gain)
        a_cl = mat_sub(self.a, bk)
        poles = eigenvalues2(a_cl)
        magnitude = abs(poles[0])
        zeta = (-poles[0].real / magnitude) if magnitude > 0.0 else 0.0
        return a_cl, poles, zeta, magnitude

    # ---------- 时域仿真 ----------
    def simulate(self, gain=None, duration=0.30, dt=2e-5, initial_twist=0.01):
        """RK4 仿真自由响应，返回 ``[(t, theta, omega_rel, u)]``。

        初值取 ``theta0 = initial_twist``、相对转速 0。``gain`` 为 None 表示开环
        （u = 0，扭振按自身阻尼衰减），否则施加 ``u = -K x``。
        """
        x = [float(initial_twist), 0.0]
        steps = int(round(duration / dt))
        record = max(1, int(round(0.002 / dt)))
        samples = [(0.0, x[0], x[1], 0.0)]

        def derivative(state, u):
            return [state[1],
                    self.s * (u - self.k * state[0] - self.c * state[1])]

        for step in range(1, steps + 1):
            u = 0.0 if gain is None else -(gain[0][0] * x[0] + gain[0][1] * x[1])
            k1 = derivative(x, u)
            x2 = [x[i] + 0.5 * dt * k1[i] for i in range(2)]
            k2 = derivative(x2, u)
            x3 = [x[i] + 0.5 * dt * k2[i] for i in range(2)]
            k3 = derivative(x3, u)
            x4 = [x[i] + dt * k3[i] for i in range(2)]
            k4 = derivative(x4, u)
            x = [x[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
                 for i in range(2)]
            if step % record == 0:
                samples.append((step * dt, x[0], x[1], u))
        return samples
