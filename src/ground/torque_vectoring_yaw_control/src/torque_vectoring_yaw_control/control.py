"""横摆角速度跟踪 + 扭矩矢量分配。

参考值来自线性二自由度模型的稳态解 ``r_ref = u·δ / (L + K·u²)``，再按附着极限
``|r| ≤ μg/u`` 截断；控制器是带前馈的 PI，输出一个横摆力矩需求 ``M_z``，
交给 :func:`~.allocator.allocate` 变成四个轮子的纵向力。

**前馈很关键**：如果只靠误差反馈，轮胎侧偏刚度一变（路面湿滑、载荷转移）就会被
积分项慢慢追，转向响应明显迟钝。前馈直接把稳态需求算出来，PI 只修残差。

关闭扭矩矢量（``enabled=False``）时，分配器只拿总驱动力、力矩需求置零，
等价于"开放式差速器"——这是对照组。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .allocator import AllocResult, allocate, allocate_equal
from .vehicle import VehicleSpec, WheelState

__all__ = ["ControllerConfig", "ControlOutput", "YawController"]


@dataclass
class ControllerConfig:
    kp: float = 5200.0           # 横摆角速度误差 → 力矩，(N·m)/(rad/s)
    ki: float = 9000.0           # 积分增益，(N·m)/(rad)
    i_limit: float = 2200.0      # 积分项上限，N·m
    moment_limit: float = 4200.0 # 力矩指令总限幅，N·m
    speed_kp: float = 900.0      # 纵向速度误差 → 总驱动力，N/(m/s)
    speed_ki: float = 260.0
    force_tau: float = 0.05      # 总驱动力一阶滤波时间常数，s
    moment_tau: float = 0.02     # 力矩指令一阶滤波时间常数，s（执行机构带宽）
    enabled: bool = True         # False = 开放式差速器对照组


@dataclass
class ControlOutput:
    fx: list[float]
    moment_cmd: float
    moment_achieved: float
    force_demand: float
    yaw_rate_ref: float
    saturated: bool
    utilization: float


class YawController:
    def __init__(self, spec: VehicleSpec, cfg: ControllerConfig | None = None):
        self.spec = spec
        self.cfg = cfg or ControllerConfig()
        self._integral = 0.0
        self._speed_integral = 0.0
        self._force_filt = 0.0
        self._moment_filt = 0.0
        self._r_ref_prev: float | None = None

    def reference(self, u: float, steer: float) -> float:
        """参考横摆角速度，已按附着极限截断。"""
        r = self.spec.yaw_rate_reference(u, steer)
        limit = self.spec.yaw_rate_limit(u)
        return max(-limit, min(limit, r))

    def drive_force(self, u: float, u_ref: float, ax_ff: float, dt: float) -> float:
        cfg = self.cfg
        err = u_ref - u
        self._speed_integral += cfg.speed_ki * err * dt
        self._speed_integral = max(-6000.0, min(6000.0, self._speed_integral))
        force = self.spec.mass * ax_ff + cfg.speed_kp * err + self._speed_integral
        return force

    def step(self, *, u: float, r: float, steer: float, u_ref: float,
             ax_ff: float, caps: list[float], dt: float,
             force_demand: float | None = None) -> ControlOutput:
        """单步：算出四轮纵向力。

        ``force_demand`` 给定时直接用它，跳过内置速度环——实车上纵向需求来自
        驱动/制动或 ACC，扭矩矢量只该管横摆力矩，不该在节点里再造一个速度 PI
        去和上层抢权限。留空则退回本模块自带的速度环，便于离线闭环使用。
        """
        cfg = self.cfg
        arms = WheelState.moment_arms(self.spec.track)
        if force_demand is None:
            # 总驱动力做一阶滤波。不滤会出事：纵向力一加上去，附着椭圆里
            # 留给纵向的额度立刻变小，力又被收回来，额度过 20 Hz 地来回跳，
            # 表现为四个轮子的力在高频抖动（电机故障工况下实测出现过极限环）。
            raw = self.drive_force(u, u_ref, ax_ff, dt)
            alpha = dt / (cfg.force_tau + dt) if cfg.force_tau > 0 else 1.0
            self._force_filt += (raw - self._force_filt) * alpha
            force_demand = self._force_filt

        r_ref = self.reference(u, steer)
        integral_before = self._integral
        if cfg.enabled:
            err = r_ref - r
            self._integral += cfg.ki * err * dt
            self._integral = max(-cfg.i_limit, min(cfg.i_limit, self._integral))
            # 速率前馈：想把横摆角速度从 r 变到 r_ref，先得付 Iz·dr_ref/dt 的力矩
            ff = 0.0
            if self._r_ref_prev is not None and dt > 1e-9:
                ff = self.spec.iz * (r_ref - self._r_ref_prev) / dt
            self._r_ref_prev = r_ref
            moment = ff + cfg.kp * err + self._integral
            moment = max(-cfg.moment_limit, min(cfg.moment_limit, moment))
            # 力矩指令也要过一道执行机构带宽：参考值阶跃时前馈项是
            # Iz·dr/dt 的冲激，一步就把指令顶到限幅，四轮力跟着一步跳几千牛。
            # 电机和传动链给不出这种斜率，滤掉 20 ms 更接近实车响应。
            alpha_m = dt / (cfg.moment_tau + dt) if cfg.moment_tau > 0 else 1.0
            self._moment_filt += (moment - self._moment_filt) * alpha_m
            moment = self._moment_filt
        else:
            moment = 0.0

        if cfg.enabled:
            res: AllocResult = allocate(force_demand, moment, caps, arms)
            # 抗积分饱和：分配器顶到附着边界时，力矩根本兑现不了，
            # 此时继续积分只会把积分项喂成一个"常值偏航指令"，松油门后还在拐。
            if res.saturated:
                # 饱和时不仅停止积分，还要让它慢慢泄掉：
                # 否则饱和期间攒下的偏置会在轮胎恢复附着的那一刻突然兑现，
                # 表现为横摆角速度的台阶式跳变。
                self._integral = integral_before * math.exp(-dt / 0.4)
                # 速度环同理：分配不出驱动力的那几帧里积分继续累加，
                # 一旦额度回来就会把积攒的量一次性放出去，形成低频喘振。
                self._speed_integral *= math.exp(-dt / 0.5)
        else:
            # 对照组：等力分配，力矩需求置零，也没有能力去抵消扰动
            res = allocate_equal(force_demand, caps)
        util = 0.0
        for f, c in zip(res.fx, caps):
            if c > 1e-6:
                util = max(util, abs(f) / c)
        return ControlOutput(
            fx=list(res.fx), moment_cmd=moment, moment_achieved=res.achieved_moment,
            force_demand=force_demand, yaw_rate_ref=r_ref,
            saturated=res.saturated, utilization=util,
        )
