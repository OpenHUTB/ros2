"""离线水面世界：恒定流 + 阵风 + 推进器退化 + 传感器噪声。

物理层用完整的船体模型（含附加质量、转弯离心、推进器一阶响应），控制层只能看到
带噪的 GPS / 罗经 / 计程仪读数。两侧严格分离，避免"控制器偷看真值"这种自欺欺人
的仿真。
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from ros2_usv_station_keeping.vehicle import (
    ActuatorState,
    VehicleParams,
    VehicleState,
    integrate,
)


@dataclass(frozen=True)
class EnvironmentConfig:
    """环境参数。"""

    current_x: float = 0.0     # 世界系恒定流速 m/s
    current_y: float = 0.0
    gust_sigma: float = 0.0    # 阵风强度（速度扰动标准差）m/s
    gust_tau: float = 3.5      # 阵风相关时间常数 s
    seed: int = 0

    @property
    def current(self) -> tuple[float, float]:
        return self.current_x, self.current_y

    @property
    def current_speed(self) -> float:
        return math.hypot(self.current_x, self.current_y)


@dataclass(frozen=True)
class NoiseConfig:
    """传感器噪声（1 sigma）。"""

    position: float = 0.02      # m
    heading_deg: float = 0.4    # deg
    velocity: float = 0.01      # m/s


@dataclass
class Observation:
    """控制器能看到的东西。"""

    x: float
    y: float
    psi: float
    u: float
    v: float
    r: float


@dataclass
class SimTrace:
    """一条完整的时间序列，用于出图与指标计算。"""

    time: list[float] = field(default_factory=list)
    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)
    psi: list[float] = field(default_factory=list)
    u: list[float] = field(default_factory=list)
    v: list[float] = field(default_factory=list)
    r: list[float] = field(default_factory=list)
    distance: list[float] = field(default_factory=list)
    crab_deg: list[float] = field(default_factory=list)
    thrust_left: list[float] = field(default_factory=list)
    thrust_right: list[float] = field(default_factory=list)
    true_current: list[tuple[float, float]] = field(default_factory=list)
    est_current: list[tuple[float, float]] = field(default_factory=list)
    efficiency: list[float] = field(default_factory=list)
    health: list[str] = field(default_factory=list)
    deadband: list[bool] = field(default_factory=list)
    target: tuple[float, float] = (0.0, 0.0)

    def append(self, **kwargs) -> None:
        for key, value in kwargs.items():
            getattr(self, key).append(value)


class WaterWorld:
    """推进器效率可以单独设置，用来模拟单侧失效或效率下降。"""

    def __init__(
        self,
        params: VehicleParams | None = None,
        state: VehicleState | None = None,
        env: EnvironmentConfig | None = None,
        efficiency: tuple[float, float] = (1.0, 1.0),
        noise: NoiseConfig | None = None,
    ) -> None:
        self.params = params or VehicleParams()
        self.state = state or VehicleState()
        self.env = env or EnvironmentConfig()
        self.efficiency = efficiency
        self.noise = noise or NoiseConfig()
        self.actuators = ActuatorState()
        self._rng = random.Random(self.env.seed)
        self._gust = [0.0, 0.0]

    # --- 环境 ---------------------------------------------------------
    def _step_gust(self, dt: float) -> None:
        """Ornstein–Uhlenbeck 过程，模拟阵风。"""
        sigma = self.env.gust_sigma
        if sigma <= 0.0:
            self._gust = [0.0, 0.0]
            return
        tau = max(self.env.gust_tau, 1e-3)
        decay = math.exp(-dt / tau)
        noise_scale = sigma * math.sqrt(max(1.0 - decay * decay, 0.0))
        for i in range(2):
            self._gust[i] = self._gust[i] * decay + self._rng.gauss(0.0, noise_scale)

    @property
    def current(self) -> tuple[float, float]:
        """当前时刻的真实流场（恒定流 + 阵风）。"""
        return (
            self.env.current_x + self._gust[0],
            self.env.current_y + self._gust[1],
        )

    # --- 推进 ---------------------------------------------------------
    def step(self, cmd_left: float, cmd_right: float, dt: float) -> VehicleState:
        self._step_gust(dt)
        tau = self.params.thrust_tau
        self.actuators.step(cmd_left, cmd_right, tau, dt)
        actual_left = self.actuators.left * self.efficiency[0]
        actual_right = self.actuators.right * self.efficiency[1]
        self.state = integrate(
            self.state, self.params, actual_left, actual_right, self.current, dt
        )
        return self.state

    def observe(self, quality: float = 1.0) -> Observation:
        """按噪声配置生成一次观测，``quality`` 用于缩放噪声（0 = 理想传感器）。"""
        n = self.noise
        s = self.state
        return Observation(
            x=s.x + self._rng.gauss(0.0, n.position * quality),
            y=s.y + self._rng.gauss(0.0, n.position * quality),
            psi=s.psi + math.radians(self._rng.gauss(0.0, n.heading_deg * quality)),
            u=s.u + self._rng.gauss(0.0, n.velocity * quality),
            v=s.v + self._rng.gauss(0.0, n.velocity * quality),
            r=s.r,
        )

    def true_thrust(self) -> tuple[float, float]:
        """实际作用在船体上的推力（含效率退化）。"""
        return (
            self.actuators.left * self.efficiency[0],
            self.actuators.right * self.efficiency[1],
        )
