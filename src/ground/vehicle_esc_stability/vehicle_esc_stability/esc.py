"""ESC（电子稳定控制）——横摆角速度跟踪 + 侧偏角约束 + 单侧差动制动。

控制思路和量产 ESC 一致，只是把观测换成了真值：

1. 由方向盘转角算期望横摆角速度 r_ref，并用路面附着把 r_ref 限制在
   轮胎能提供的侧向加速度以内（否则期望本身就是不可达的）；
2. 误差 e = r_ref − r 与侧偏角超限一起决定需要的修正横摆力矩强度；
3. 需要正（左转）力矩时给左侧车轮加压、右侧减压，反之亦然。
   对开路面制动时，这套动作自然表现为「释放高附着侧制动」，
   这也正是量产 ESC 在 μ-split 工况下的主要手段。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .dynamics import RoadSpec
from .params import ControlInput, EscConfig, G, VehicleSpec, VehicleState


@dataclass
class EscStatus:
    """一个控制周期的 ESC 内部状态，便于画图与诊断。"""

    reference_yaw_rate: float = 0.0
    yaw_rate_error: float = 0.0
    sideslip: float = 0.0
    intensity: float = 0.0
    active: bool = False
    direction: int = 0
    """+1 施加左转力矩，-1 施加右转力矩，0 不动作。"""

    released: tuple[int, ...] = ()
    added: tuple[int, ...] = ()


class Esc:
    """离散 ESC 控制器，按固定周期调用。"""

    def __init__(self, config: EscConfig | None = None, spec: VehicleSpec | None = None) -> None:
        self.config = config or EscConfig()
        self.spec = spec or VehicleSpec()
        self._prev_error = 0.0
        self._integral = 0.0
        self._intensity = 0.0
        self._active_time = 0.0
        self._peak = 0.0
        self._activations = 0
        self._was_active = False
        self._last = EscStatus()

    # ------------------------------------------------------------ 参考量
    def reference_yaw_rate(self, state: VehicleState, steer: float, road: RoadSpec) -> float:
        """稳态参考横摆角速度，受附着上限约束。"""
        if abs(state.u) < 0.5:
            return 0.0
        wheelbase = self.spec.wheelbase
        k_us = self.config.understeer_gradient
        r_ss = state.u * math.tan(steer) / (wheelbase * (1.0 + k_us * state.u * state.u))

        mu = road.mean
        limit = self.config.reference_margin * mu * G / max(abs(state.u), 3.0)
        return max(-limit, min(limit, r_ss))

    # ------------------------------------------------------------ 主入口
    def step(
        self,
        state: VehicleState,
        control: ControlInput,
        road: RoadSpec,
        dt: float,
    ) -> ControlInput:
        """根据当前状态改写各轮制动指令，返回新的控制量。"""
        cfg = self.config
        driver = min(max(control.driver_brake, 0.0), 1.0)
        base = [driver] * 4

        r_ref = self.reference_yaw_rate(state, control.steer, road)
        error = r_ref - state.r
        sideslip = state.sideslip

        status = EscStatus(
            reference_yaw_rate=r_ref,
            yaw_rate_error=error,
            sideslip=sideslip,
        )

        if not cfg.enabled or abs(state.u) < 3.0:
            self._last = status
            control.wheel_brake = base
            control.esc_active = False
            control.esc_intensity = 0.0
            return control

        # 侧偏角超限时折算成等效横摆误差。β>0 表示速度矢量偏在车头左侧（车头相对
        # 前进方向偏右），要减小 β 得把车头往左带，也就是需要正的横摆力矩；本模型里
        # 正的 demand 恰好压左轮、产生正力矩，所以这里取同号。
        excess = max(0.0, abs(sideslip) - cfg.sideslip_soft)
        beta_term = math.copysign(cfg.sideslip_gain * excess, sideslip) if excess > 0.0 else 0.0

        previous_error = self._prev_error
        error_rate = (error - previous_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        magnitude = abs(error)
        # 前馈：左右附着差越大，制动时被拽偏的力矩越大，先抵消掉大部分。
        # 这一项只看驾驶员制动与路面，不看误差，所以判定「要不要动」时
        # 也得把它算进去，否则对开路面制动开始的几个周期会误判成不用介入。
        split = cfg.split_mu_feedforward * driver * (road.mu_left - road.mu_right)

        idle = (magnitude <= cfg.yaw_rate_deadband and excess <= 0.0
                and abs(self._integral) < 0.02 and abs(split) < 0.02)
        if idle:
            self._integral *= 0.9
            self._intensity *= 0.5
            status.intensity = 0.0
            self._last = status
            control.wheel_brake = base
            control.esc_active = False
            control.esc_intensity = 0.0
            self._update_stats(False, 0.0, dt)
            return control

        # 比例 + 积分 + 微分，积分负责消掉对开路面上残余的稳态横摆偏差
        demand = cfg.yaw_gain * error + cfg.yaw_integral_gain * self._integral
        demand += cfg.yaw_rate_gain * error_rate + beta_term
        demand -= split

        # 一阶低通：误差每 20 ms 更新一次，不带滤波时强度会在 0.02～1.00 之间跳变，
        # 表现为左右轮交替加压、车身一顿一顿。滤波只作用在幅值上，方向仍取 demand 的符号。
        raw = min(1.0, abs(demand))
        if cfg.intensity_tau > 0.0:
            alpha = dt / (cfg.intensity_tau + dt)
            self._intensity += (raw - self._intensity) * alpha
        else:
            self._intensity = raw
        intensity = self._intensity

        # 方向盘反向（期望横摆角速度变号）时清掉积分，避免把上一段的偏差带过来
        if error * previous_error < 0.0 and abs(error) > 0.20:
            self._integral = 0.0

        # 只在没打满时积分，避免饱和后积分继续累积
        if intensity < 1.0:
            self._integral += error * dt
            self._integral = max(-cfg.integral_limit, min(cfg.integral_limit, self._integral))

        if intensity < 0.02:
            status.intensity = 0.0
            self._last = status
            control.wheel_brake = base
            control.esc_active = False
            control.esc_intensity = 0.0
            self._update_stats(False, 0.0, dt)
            return control

        direction = 1 if demand > 0 else -1
        # 左轮下标 0/2，右轮 1/3；需要正力矩时压左轮、松右轮
        add_side = (0, 2) if direction > 0 else (1, 3)
        release_side = (1, 3) if direction > 0 else (0, 2)

        brakes = list(base)
        release = cfg.release_gain * intensity
        for index in release_side:
            brakes[index] = max(0.0, brakes[index] * (1.0 - release))
        # 驾驶员已经在全力制动时只减压不加压：低附着侧再加压只会让它先锁死
        if driver < cfg.brake_add_threshold:
            for index in add_side:
                brakes[index] = min(1.0, brakes[index] + cfg.add_limit * intensity)

        # 车已经明显侧滑时四轮一起补一点制动，把车速降回附着能力以内
        if excess > 0.0:
            extra = cfg.speed_brake_gain * intensity
            brakes = [min(1.0, value + extra) for value in brakes]

        # 前驱车降扭，减轻前轮纵向负荷
        if intensity > cfg.throttle_cut:
            control.throttle = 0.0

        status.intensity = intensity
        status.direction = direction
        status.active = intensity > 0.02
        status.added = tuple(add_side)
        status.released = tuple(release_side)

        control.wheel_brake = [min(max(value, 0.0), 1.0) for value in brakes]
        control.esc_active = status.active
        control.esc_intensity = intensity
        self._last = status
        self._update_stats(status.active, intensity, dt)
        return control

    # ------------------------------------------------------------ 统计
    def _update_stats(self, active: bool, intensity: float, dt: float) -> None:
        if active:
            self._active_time += dt
            self._peak = max(self._peak, intensity)
            if not self._was_active:
                self._activations += 1
        self._was_active = active

    @property
    def status(self) -> EscStatus:
        return self._last

    @property
    def stats(self) -> dict[str, float]:
        return {
            "active_time": self._active_time,
            "peak_intensity": self._peak,
            "activations": float(self._activations),
        }

    def reset(self) -> None:
        self._prev_error = 0.0
        self._integral = 0.0
        self._active_time = 0.0
        self._peak = 0.0
        self._activations = 0
        self._was_active = False
        self._last = EscStatus()
