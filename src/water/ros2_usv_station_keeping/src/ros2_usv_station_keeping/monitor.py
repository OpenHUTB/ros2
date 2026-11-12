"""状态监视：在线估计流场、判定推进器健康、给出降级策略。

这里解决一个容易被忽略的前提：制导要用到"流速"，但船上没有流速计。可用的只有
GPS 位置、罗经航向和计程仪速度。三者做差就能把流"反解"出来：

    实测世界速度 = (p_k - p_{k-1}) / dt
    船体推进速度 = R(psi) · [u, v]
    流速         ≈ 实测世界速度 - 船体推进速度

差值噪声很大（GPS 抖动会被 1/dt 放大），所以必须低通 + 限幅。这一层做不好，
crab angle 会跟着噪声抖，船在原地画圈。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from .geometry import body_to_world, clamp, world_to_body


class HealthLevel(str, Enum):
    """推进器健康等级。"""

    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class FlowEstimatorConfig:
    """流估计参数。

    关键点：**不能用"位置差分 - 船体速度"来算流**。船在流里被顶住不动时，
    位置差分为 0、船体地速也为 0，残差恒等于 0，估计器会永远认为"没有流"——
    而这恰恰是最需要知道流的时刻。

    正确做法是从动力学反解。纵向与横向各有一个方程：

        m_u·u̇ = X - d_u·(u - c_x)      =>  c_x = u - (X - m_u·u̇)/d_u
        m_v·v̇ = -d_v·(v - c_y) - m_u·u·r  =>  c_y = v + (m_v·v̇ + m_u·u·r)/d_v

    ``u̇``/``v̇`` 由速度差分得到，噪声靠低通压下去。代价是依赖船体参数
    （质量、阻尼）标定准确，这一点在文档的"局限"里写明。
    """

    alpha: float = 0.040         # 纵向低通系数（强平滑，抑制差分噪声）
    alpha_sway: float = 0.010    # 横向低通系数：横向公式的噪声增益是 1.0，必须更慢
    alpha_fast: float = 0.30     # 与估计相差很大时的快速跟随系数
    fast_threshold: float = 0.20 # 触发快速跟随的偏差 m/s
    max_current: float = 1.0     # 流速限幅 m/s
    max_accel: float = 2.0       # 加速度限幅 m/s²，抑制差分野值
    min_thrust_for_update: float = 0.15  # 推力太小（滑行）时不更新纵向估计


class FlowEstimator:
    """基于动力学模型反解流速，输出船体系下的估计值。"""

    def __init__(
        self,
        config: FlowEstimatorConfig | None = None,
        params: "VehicleParams | None" = None,
    ) -> None:
        from .vehicle import VehicleParams

        self.config = config or FlowEstimatorConfig()
        self.params = params or VehicleParams()
        self.current_body: tuple[float, float] = (0.0, 0.0)
        self.valid = False
        self._prev_u: float | None = None
        self._prev_v: float | None = None
        # 推进器一阶响应建模：用"指令"直接反解会把滞后误算成流，静水下凭空多出
        # 0.1 m/s 的假流；先按同样的时间常数把指令过滤成"实际推力"再用。
        self._thrust_est = 0.0

    def reset(self) -> None:
        self.current_body = (0.0, 0.0)
        self.valid = False
        self._prev_u = None
        self._prev_v = None
        self._thrust_est = 0.0

    def update(
        self,
        x: float,
        y: float,
        psi: float,
        u: float,
        v: float,
        r: float,
        thrust_surge: float,
        dt: float,
    ) -> tuple[float, float]:
        cfg = self.config
        if dt <= 0.0:
            raise ValueError("dt 必须为正")

        if self._prev_u is None or self._prev_v is None:
            self._prev_u, self._prev_v = u, v
            return self.current_body

        acc_u = clamp((u - self._prev_u) / dt, -cfg.max_accel, cfg.max_accel)
        acc_v = clamp((v - self._prev_v) / dt, -cfg.max_accel, cfg.max_accel)
        self._prev_u, self._prev_v = u, v

        p = self.params
        tau = max(p.thrust_tau, 1e-3)
        self._thrust_est += min(1.0, dt / tau) * (thrust_surge - self._thrust_est)
        surge_est = u - (self._thrust_est - p.mass_surge * acc_u) / p.damp_surge
        sway_est = v + (p.mass_sway * acc_v + p.mass_surge * u * r) / p.damp_sway

        mag = math.hypot(surge_est, sway_est)
        if mag > cfg.max_current:
            scale = cfg.max_current / mag
            surge_est, sway_est = surge_est * scale, sway_est * scale

        cx, cy = self.current_body
        # 滑行阶段（几乎不出力）纵向公式对推力误差极敏感，干脆不更新纵向分量
        if abs(thrust_surge) < cfg.min_thrust_for_update:
            surge_est = cx

        err = math.hypot(surge_est - cx, sway_est - cy)
        # 快速跟随只在"从零开始捕获"时启用：一旦已有估计，任何大跳变更可能是
        # 推力模型误差或差分噪声，加速跟随只会把假流灌进来
        capturing = math.hypot(cx, cy) < 0.05 and err > cfg.fast_threshold
        a_surge = cfg.alpha_fast if capturing else cfg.alpha
        # 横向单独用更小的系数：m_v/d_v 接近 1，任何加速度噪声都会原样变成假流
        a_sway = cfg.alpha_fast if capturing else cfg.alpha_sway
        self.current_body = (cx + a_surge * (surge_est - cx), cy + a_sway * (sway_est - cy))
        self.valid = True
        return self.current_body

    @property
    def magnitude(self) -> float:
        return math.hypot(*self.current_body)


@dataclass(frozen=True)
class HealthConfig:
    """推进器健康判定参数。"""

    nominal_ratio: float = 0.62   # 效率高于此值视为正常（留出推力模型误差的余量）
    degraded_ratio: float = 0.30  # 低于此值视为严重失效
    min_command: float = 0.80     # 总推力低于该值时不参与判定（信噪比太低）
    alpha: float = 0.05           # 效率估计低通系数：太快会被瞬时工况带偏
    confirm_time: float = 3.0     # 连续满足阈值多久才降级
    recovery_time: float = 3.0    # 恢复也要连续满足这么久，避免抖动


class ThrusterHealth:
    """根据"指令推力 vs 实测加速度反推的推力"估计推进器效率。"""

    def __init__(self, config: HealthConfig | None = None) -> None:
        self.config = config or HealthConfig()
        self.efficiency = 1.0
        self.level = HealthLevel.NOMINAL
        self._pending: HealthLevel | None = None
        self._pending_time = 0.0

    def reset(self) -> None:
        self.efficiency = 1.0
        self.level = HealthLevel.NOMINAL
        self._pending = None
        self._pending_time = 0.0

    def observe(self, command_surge: float, measured_surge: float, dt: float) -> HealthLevel:
        cfg = self.config
        if abs(command_surge) >= cfg.min_command:
            ratio = measured_surge / command_surge
            if ratio < 0.0:
                ratio = 0.0
            elif ratio > 1.6:
                ratio = 1.6
            self.efficiency += cfg.alpha * (ratio - self.efficiency)

        target = self._classify(self.efficiency)
        if target == self.level:
            self._pending = None
            self._pending_time = 0.0
            return self.level

        if self._pending != target:
            self._pending = target
            self._pending_time = 0.0
            return self.level

        self._pending_time += dt
        need = cfg.recovery_time if target == HealthLevel.NOMINAL else cfg.confirm_time
        if self._pending_time >= need:
            self.level = target
            self._pending = None
            self._pending_time = 0.0
        return self.level

    def _classify(self, efficiency: float) -> HealthLevel:
        """带滞回的分类。

        没有滞回就会出现"效率在阈值附近抖动 -> 等级来回切换 -> 降级策略反复生效"
        的连锁反应，实测表现为船原地小幅来回摆、永远不会稳定下来。
        """
        cfg = self.config
        if self.level == HealthLevel.NOMINAL:
            if efficiency < cfg.degraded_ratio:
                return HealthLevel.FAILED
            if efficiency < cfg.nominal_ratio:
                return HealthLevel.DEGRADED
            return HealthLevel.NOMINAL

        if self.level == HealthLevel.DEGRADED:
            if efficiency < cfg.degraded_ratio * 0.80:
                return HealthLevel.FAILED
            if efficiency > cfg.nominal_ratio + 0.07:
                return HealthLevel.NOMINAL
            return HealthLevel.DEGRADED

        # 已判定失效：只有明显恢复才允许降级，否则保持"停机待援"
        if efficiency > cfg.degraded_ratio * 1.25:
            return HealthLevel.DEGRADED
        return HealthLevel.FAILED


@dataclass(frozen=True)
class DegradedPolicy:
    """不同健康等级下的降级策略。"""

    deadband_radius: float
    approach_speed: float
    crab_limit_deg: float
    hold_position: bool = True   # False 表示放弃保位，仅上报漂移（单侧完全失效时）


DEFAULT_POLICY = DegradedPolicy(deadband_radius=0.15, approach_speed=0.35, crab_limit_deg=88.0)
DEGRADED_POLICY = DegradedPolicy(deadband_radius=0.35, approach_speed=0.28, crab_limit_deg=88.0)
# 单侧推进器失效的双推进器船在物理上无法解耦"推力"与"力矩"，继续满推只会原地打转，
# 把电能烧掉却不改变漂移趋势。正确做法是停止挣扎、上报位置，由上层决定是否回收。
FAILED_POLICY = DegradedPolicy(deadband_radius=1.20, approach_speed=0.15, crab_limit_deg=88.0, hold_position=False)


def policy_for(level: HealthLevel) -> DegradedPolicy:
    """按健康等级取策略：越不健康，越接受更大的保持误差，换取可控与省电。"""
    if level == HealthLevel.FAILED:
        return FAILED_POLICY
    if level == HealthLevel.DEGRADED:
        return DEGRADED_POLICY
    return DEFAULT_POLICY


@dataclass
class MonitorSnapshot:
    """一个监视周期的汇总，便于记录与上报。"""

    distance: float = 0.0
    crab_deg: float = 0.0
    current_body: tuple[float, float] = (0.0, 0.0)
    current_magnitude: float = 0.0
    efficiency: float = 1.0
    health: HealthLevel = HealthLevel.NOMINAL
    in_deadband: bool = False
    saturated: bool = False
    extra: dict = field(default_factory=dict)
