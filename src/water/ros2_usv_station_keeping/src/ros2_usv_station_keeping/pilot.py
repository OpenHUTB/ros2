"""把制导 / 控制 / 分配 / 监视串成一个"定点保持驾驶员"。

对外只有一个 ``step()``：给当前观测和时戳，返回左右推力与一份监视快照。这样整套
决策逻辑都能在纯 Python 里离线跑单元测试，ROS 节点只负责收发消息。
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from .allocator import allocate
from .control import ControlConfig, StationKeepingController
from .guidance import GuidanceConfig, StationKeepingGuidance
from .monitor import (
    DEFAULT_POLICY,
    FlowEstimator,
    FlowEstimatorConfig,
    HealthConfig,
    HealthLevel,
    MonitorSnapshot,
    ThrusterHealth,
    policy_for,
)
from .vehicle import VehicleParams, VehicleState


@dataclass(frozen=True)
class PilotConfig:
    """驾驶员总配置。"""

    guidance: GuidanceConfig = GuidanceConfig()
    control: ControlConfig = ControlConfig()
    flow: FlowEstimatorConfig = FlowEstimatorConfig()
    health: HealthConfig = HealthConfig()
    use_crab_compensation: bool = True   # 关闭后用于对照：横流下会持续偏出
    use_degradation: bool = True         # 关闭后失效仍按正常策略工作，用于对照


@dataclass(frozen=True)
class PilotCommand:
    """一个周期的执行指令与监视信息。"""

    left: float
    right: float
    snapshot: MonitorSnapshot
    heading_error: float
    u_cmd: float
    psi_cmd: float
    holding: bool = True


class StationKeepingPilot:
    """欠驱动 USV 的定点保持驾驶员。"""

    def __init__(self, params: VehicleParams | None = None, config: PilotConfig | None = None) -> None:
        self.params = params or VehicleParams()
        self.config = config or PilotConfig()
        self.guidance = StationKeepingGuidance(self.config.guidance)
        self.controller = StationKeepingController(self.config.control)
        self.flow = FlowEstimator(self.config.flow, self.params)
        self.health = ThrusterHealth(self.config.health)
        self._prev_u: float | None = None
        self._prev_thrust = 0.0
        self._runaway_time = 0.0
        self._saturated_time = 0.0

    def reset(self, psi: float = 0.0) -> None:
        self.guidance.reset(psi)
        self.flow.reset()
        self.health.reset()
        self._prev_u = None
        self._prev_thrust = 0.0
        self._runaway_time = 0.0
        self._saturated_time = 0.0

    def step(
        self,
        state: VehicleState,
        target: tuple[float, float],
        dt: float,
        allow_thrust: bool = True,
    ) -> PilotCommand:
        cfg = self.config
        params = self.params

        current_body = self.flow.update(
            state.x, state.y, state.psi, state.u, state.v, state.r, self._prev_thrust, dt
        )
        level = self._check_runaway(state.r, dt)
        policy = policy_for(level) if cfg.use_degradation else DEFAULT_POLICY

        guide_cfg = replace(
            cfg.guidance,
            deadband_radius=policy.deadband_radius,
            approach_speed=policy.approach_speed,
            crab_limit_deg=policy.crab_limit_deg,
        )
        self.guidance.config = guide_cfg

        estimated = current_body if cfg.use_crab_compensation else (0.0, 0.0)
        guide = self.guidance.update(state.x, state.y, state.psi, target, estimated)

        if not policy.hold_position:
            # 已判定为单侧失效：停止挣扎，只上报漂移
            snapshot = MonitorSnapshot(
                distance=guide.distance,
                crab_deg=guide.crab_deg,
                current_body=current_body,
                current_magnitude=(current_body[0] ** 2 + current_body[1] ** 2) ** 0.5,
                efficiency=self.health.efficiency,
                health=level,
                in_deadband=guide.in_deadband,
                saturated=False,
            )
            return PilotCommand(
                left=0.0,
                right=0.0,
                snapshot=snapshot,
                heading_error=0.0,
                u_cmd=0.0,
                psi_cmd=state.psi,
                holding=False,
            )

        # 单侧降效的船每推一次都会带来持续偏转：角速度已经很大时先松推力滑行一段，
        # 否则会陷入"满推 -> 打转 -> 更满推"的循环，把电量烧在转圈上。
        if level == HealthLevel.DEGRADED and abs(state.r) > 0.34:
            allow_thrust = False

        wrench = self.controller.update(
            u_cmd=guide.u_cmd,
            psi_cmd=guide.psi_cmd,
            psi=state.psi,
            u=state.u - current_body[0],   # 控制器追踪的是"相对水"速度，地速会被流拖偏
            r=state.r,
            allow_thrust=allow_thrust and guide.holding,
        )

        # 不用在线估计的效率去放大指令：eff 本身是"指令 vs 实测"算出来的，再拿它去
        # 放大指令会形成正反馈——ratio 越算越小、指令越放越大，几秒内两侧就顶到满推，
        # 且真实故障反而被掩盖。效率只用于诊断与降级决策。
        allocation = allocate(wrench.surge, wrench.yaw, params)
        self._prev_thrust = allocation.left + allocation.right

        # 健康判定必须用"分配后真正下发的推力"，否则限幅/饱和会被误判成推进器失效
        self._update_health(state, current_body, allocation.left + allocation.right, dt)

        # 兜底之二：推力已经打满却还在持续漂移，说明"再怎么推也保不住"。
        # 这种情况继续输出只会烧电，直接退到"停机上报"，把位置留给上层决策。
        if allocation.saturated:
            self._saturated_time += dt
        else:
            self._saturated_time = max(0.0, self._saturated_time - dt)
        if self._saturated_time > 4.0 and self.health.level != HealthLevel.FAILED:
            self.health.level = HealthLevel.FAILED

        snapshot = MonitorSnapshot(
            distance=guide.distance,
            crab_deg=guide.crab_deg,
            current_body=current_body,
            current_magnitude=(current_body[0] ** 2 + current_body[1] ** 2) ** 0.5,
            efficiency=self.health.efficiency,
            health=self.health.level,
            in_deadband=guide.in_deadband,
            saturated=allocation.saturated,
        )
        return PilotCommand(
            left=allocation.left,
            right=allocation.right,
            snapshot=snapshot,
            heading_error=wrench.heading_error,
            u_cmd=guide.u_cmd,
            psi_cmd=guide.psi_cmd,
            holding=guide.holding,
        )

    def _update_health(
        self,
        state: VehicleState,
        current_body: tuple[float, float],
        command_surge: float,
        dt: float,
    ) -> None:
        """用纵向动力学反推实际出力，喂给健康判定。"""
        if self._prev_u is None:
            self._prev_u = state.u
            return
        du = (state.u - self._prev_u) / dt
        u_rel = state.u - current_body[0]
        measured = self.params.mass_surge * du + self.params.damp_surge * u_rel
        self.health.observe(command_surge, measured, dt)
        self._prev_u = state.u

    def _check_runaway(self, yaw_rate: float, dt: float) -> HealthLevel:
        """兜底判据：持续高速打转说明"推了也直不了"（典型是单侧推进器失效）。

        效率估计在强扰动下可能恰好卡在阈值边缘，但"一直在转圈"这个现象本身
        就足以判定无法保位——继续满推只会把电耗在画圈上。
        """
        if abs(yaw_rate) > 0.40:
            self._runaway_time += dt
        else:
            self._runaway_time = max(0.0, self._runaway_time - 0.5 * dt)

        if self._runaway_time > 3.0 and self.health.level != HealthLevel.FAILED:
            self.health.level = HealthLevel.FAILED
        return self.health.level

    @property
    def health_level(self) -> HealthLevel:
        return self.health.level

    @property
    def estimated_current(self) -> tuple[float, float]:
        return self.flow.current_body


def thrust_magnitude(command: PilotCommand) -> float:
    """左右推力的绝对值之和，作为能耗统计的代理量。"""
    return abs(command.left) + abs(command.right)
