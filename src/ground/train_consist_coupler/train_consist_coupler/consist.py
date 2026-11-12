"""多质点编组模型：车钩弹簧-阻尼-间隙，逐节积分。"""

from .params import G


class Coupler:
    """两个相邻车之间的车钩力。

    约定：返回正值表示"后车把前车往前推"（压钩），负值表示拉钩。
    在间隙内不传力，超过间隙后按弹簧 + 阻尼线性传力。
    """

    def __init__(self, params):
        self.k = params.stiffness
        self.c = params.damping
        self.gap = params.gap

    def force(self, compression, dv):
        if compression > self.gap:
            effective = compression - self.gap
        elif compression < -self.gap:
            effective = compression + self.gap
        else:
            return 0.0
        return self.k * effective + self.c * dv


class Consist:
    """一列车的多质点状态。

    车 1 是头部，车心坐标 x[0] 最靠前；车头前端位置 = x[0] + 车长 / 2。
    """

    def __init__(self, n_cars, car, coupler_params, route):
        self.n = int(n_cars)
        self.car = car
        self.route = route
        self.coupler = Coupler(coupler_params) if self.n > 1 else None
        length = car.length
        # 车心坐标：车头前端正好在 0
        self.x = [-(i + 0.5) * length for i in range(self.n)]
        self.v = [0.0] * self.n
        self.force = [0.0] * self.n      # 实际输出：牵引为正、制动为负

    # ---------- 几何 ----------
    @property
    def head(self):
        return self.x[0] + self.car.length / 2.0

    @property
    def tail(self):
        return self.x[-1] - self.car.length / 2.0

    @property
    def length(self):
        return self.n * self.car.length

    # ---------- 合力 ----------
    @property
    def total_mass(self):
        return self.n * self.car.mass

    def traction_available(self):
        return sum(self.car.traction(v) for v in self.v)

    def brake_available(self, decel):
        return self.n * self.car.mass * decel

    def resistance_total(self):
        return sum(self.car.resistance(v) for v in self.v)

    def grade_force_total(self):
        return sum(self.car.mass * G * self.route.grade(x) for x in self.x)

    def coupler_forces(self):
        """长度 n-1 的列表，第 i 个是车 i 与车 i+1 之间的力。"""
        if self.n == 1:
            return []
        length = self.car.length
        out = []
        for i in range(self.n - 1):
            compression = length - (self.x[i] - self.x[i + 1])
            out.append(self.coupler.force(compression, self.v[i + 1] - self.v[i]))
        return out

    def accelerations(self):
        forces = self.coupler_forces()
        accs = []
        for i in range(self.n):
            net = (self.force[i]
                   - self.car.resistance(self.v[i])
                   - self.car.mass * G * self.route.grade(self.x[i]))
            if i < self.n - 1:
                net += forces[i]        # 后车推前车
            if i > 0:
                net -= forces[i - 1]    # 前车受到的反作用
            accs.append(net / (self.car.gamma * self.car.mass))
        return accs

    def integrate(self, dt):
        accs = self.accelerations()
        for i in range(self.n):
            self.v[i] += accs[i] * dt
            if self.v[i] < 0.0:
                self.v[i] = 0.0
            self.x[i] += self.v[i] * dt
        return accs

    def mean_speed(self):
        return sum(self.v) / self.n


class Actuator:
    """单节车的牵引/制动力一阶滞后。"""

    def __init__(self, params):
        self.tau_traction = params.tau_traction
        self.tau_brake = params.tau_brake
        self.force = 0.0

    def step(self, target, dt):
        tau = self.tau_brake if target < 0.0 else self.tau_traction
        self.force += (target - self.force) * dt / tau
        return self.force
