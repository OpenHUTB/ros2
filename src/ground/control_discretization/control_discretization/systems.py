"""连续二阶系统与四种离散化方法。

连续对象用可控标准型状态空间描述::

    x' = A x + B u,   y = C x
    A = [[0, 1], [-wn^2, -2*zeta*wn]],  B = [[0], [wn^2]],  C = [1, 0]

即 ``G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)``，单位阶跃稳态值为 1。

离散化方法（输入连续 ``(A, B)`` 与采样周期 ``Ts``，输出 ``(F, G)``）：

- ``forward``：前向欧拉，``F = I + A*Ts``，``G = B*Ts``
- ``backward``：后向欧拉，``F = (I - A*Ts)^-1``，``G = (I - A*Ts)^-1 * B*Ts``
- ``tustin``：双线性变换，``F = (I - A*Ts/2)^-1 (I + A*Ts/2)``，``G = (I - A*Ts/2)^-1 * B*Ts``
- ``zoh``：零阶保持的精确离散化，``F = expm(A*Ts)``，``G = A^-1 (F - I) B``
"""

from .matrices import (add, eigenvalues2, eye, inverse2, mul, mul_vector,
                       scale, sub, expm)

METHODS = ('forward', 'backward', 'tustin', 'zoh')

METHOD_LABELS = {
    'forward': '前向欧拉',
    'backward': '后向欧拉',
    'tustin': '双线性(Tustin)',
    'zoh': '零阶保持(ZOH)',
}


class SecondOrderSystem:
    """``G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)``。"""

    def __init__(self, wn=3.0, zeta=0.4):
        if wn <= 0.0:
            raise ValueError('wn 必须为正数')
        if not 0.0 < zeta < 1.0:
            raise ValueError('本模块只讨论欠阻尼 0<zeta<1，以便观察振荡')
        self.wn = float(wn)
        self.zeta = float(zeta)
        self.a = [[0.0, 1.0], [-(wn ** 2), -2.0 * zeta * wn]]
        self.b = [[0.0], [wn ** 2]]
        self.c = [1.0, 0.0]

    def sample_continuous(self, ts, duration, fine_dt=None):
        """在采样时刻 ``t = ts, 2ts, ...`` 上给出连续系统的精确响应。

        用 RK4 在 ``fine_dt``（默认 ``min(1e-4, Ts/50)``）的细网格上积分，
        再落在每个采样时刻上，避免离散化误差与积分误差混在一起。
        """
        fine_dt = fine_dt or min(1e-4, ts / 50.0)
        steps = int(round(duration / ts))
        x = [0.0, 0.0]
        t = 0.0
        samples = []
        for index in range(1, steps + 1):
            target = index * ts
            while t < target - 1e-15:
                h = min(fine_dt, target - t)
                x = self._rk4(x, 1.0, h)
                t += h
            samples.append((target, self.output(x)))
        return samples

    def output(self, x):
        return self.c[0] * x[0] + self.c[1] * x[1]

    def _derivative(self, x, u):
        ax = mul_vector(self.a, x)
        return [ax[0] + self.b[0][0] * u, ax[1] + self.b[1][0] * u]

    def _rk4(self, x, u, dt):
        k1 = self._derivative(x, u)
        x2 = [x[i] + 0.5 * dt * k1[i] for i in range(2)]
        k2 = self._derivative(x2, u)
        x3 = [x[i] + 0.5 * dt * k2[i] for i in range(2)]
        k3 = self._derivative(x3, u)
        x4 = [x[i] + dt * k3[i] for i in range(2)]
        k4 = self._derivative(x4, u)
        return [x[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
                for i in range(2)]


def discretize(system, method, ts):
    """返回 ``(F, G)``；方法名不合法时抛 ``ValueError``。"""
    if ts <= 0.0:
        raise ValueError('Ts 必须为正数')
    if method not in METHODS:
        raise ValueError(f'未知离散化方法 {method}，可选 {METHODS}')

    a, b = system.a, system.b
    identity = eye(2)

    if method == 'forward':
        f = add(identity, scale(a, ts))
        g = scale(b, ts)
        return f, g

    if method == 'backward':
        m = inverse2(sub(identity, scale(a, ts)))
        return m, mul(m, scale(b, ts))

    if method == 'tustin':
        half = scale(a, ts / 2.0)
        m = inverse2(sub(identity, half))
        f = mul(m, add(identity, half))
        # G = (I - A*Ts/2)^-1 * B*Ts
        return f, mul(m, scale(b, ts))

    # zoh: 用增广矩阵 expm 求 G
    f = expm(scale(a, ts))
    # G = A^-1 (F - I) B，A 可逆（行列式 = wn^2 > 0）
    inv_a = inverse2(a)
    g = mul(mul(inv_a, sub(f, identity)), b)
    return f, g


def discrete_poles(system, method, ts):
    f, _ = discretize(system, method, ts)
    return eigenvalues2(f)


def is_stable(system, method, ts):
    return all(abs(pole) < 1.0 - 1e-12 for pole in discrete_poles(system, method, ts))


def simulate_discrete(system, method, ts, duration):
    """在采样点上演离散状态方程，返回 ``[(t, y)]``（区间保持上一拍的值）。"""
    f, g = discretize(system, method, ts)
    steps = int(round(duration / ts))
    x = [0.0, 0.0]
    samples = []
    for step in range(steps):
        u = 1.0
        fx = mul_vector(f, x)
        x = [fx[0] + g[0][0] * u, fx[1] + g[1][0] * u]
        value = system.output(x)
        if value != value or abs(value) > 1e6:      # 发散保护（NaN / 溢出）
            return samples, True
        samples.append(((step + 1) * ts, value))
    return samples, False


def critical_period(system, method, ts_min=1e-3, ts_max=2.0, points=400):
    """扫描采样周期，返回该离散化方法首次失去稳定性的 Ts；始终稳定返回 None。"""
    for index in range(points + 1):
        ts = ts_min + (ts_max - ts_min) * index / points
        if not is_stable(system, method, ts):
            return ts
    return None
