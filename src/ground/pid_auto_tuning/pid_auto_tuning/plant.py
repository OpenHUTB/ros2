"""差速小车的简化航向被控对象：一阶惯性 + 纯延迟。

只用于 PID 整定实验，不包含轮胎侧偏、道路坡度等真实动力学，
Model:  d(omega)/dt = (gain * u(t - delay) - omega) / tau
        d(theta)/dt = omega
"""


class HeadingPlant:
    def __init__(self, gain=1.0, tau=0.40, delay=0.10, u_max=2.5, dt=0.01):
        self.gain = gain
        self.tau = tau
        self.u_max = u_max
        self.dt = dt
        self.delay_steps = max(1, int(round(delay / dt)))
        self._buffer = [0.0] * self.delay_steps
        self._idx = 0
        self.omega = 0.0
        self.theta = 0.0

    def reset(self):
        self._buffer = [0.0] * self.delay_steps
        self._idx = 0
        self.omega = 0.0
        self.theta = 0.0

    def step(self, u):
        """输入角速度指令 u(rad/s)，返回新的航向角 theta(rad)。"""
        u = max(-self.u_max, min(self.u_max, u))
        delayed = self._buffer[self._idx]
        self._buffer[self._idx] = u
        self._idx = (self._idx + 1) % self.delay_steps
        self.omega += self.dt * (self.gain * delayed - self.omega) / self.tau
        self.theta += self.dt * self.omega
        return self.theta
