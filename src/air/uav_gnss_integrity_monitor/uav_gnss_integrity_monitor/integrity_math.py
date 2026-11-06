"""GNSS 定位完整性判据与失效保护状态机。

本文件只依赖 Python 标准库，不导入 rclpy 或任何 ROS 消息，
因此可以在没有 ROS 2 / 仿真器的机器上直接运行单元测试与离线回放。
ROS 2 节点只负责把这一层的结论转发成话题，判据本身集中在这里。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

__all__ = [
    "State",
    "Fix",
    "Config",
    "Output",
    "Monitor",
    "check_fix",
    "allowed_displacement",
]


class State(str, Enum):
    """对外暴露的四个状态。"""

    INIT = "INIT"            # 还没有任何可信定位
    TRACKING = "TRACKING"    # 定位可信，逐帧更新
    SUSPECT = "SUSPECT"      # 出现坏帧，冻结输出但尚未判定失效
    HOLD = "HOLD"            # 判定失效，持续请求悬停，等待重新捕获


@dataclass(frozen=True)
class Fix:
    """一帧定位结果；位置为本地坐标系下的米，时间为秒。

    fix_type: 0=无定位, 1=2D, 2=3D, 3=RTK 固定解
    num_sats: 参与解算的卫星数
    accuracy: 水平精度估计（米，标准差或厂商给出的精度值）
    """

    t: float
    x: float
    y: float
    z: float
    fix_type: int = 3
    num_sats: int = 12
    accuracy: float = 0.5

    def is_finite(self) -> bool:
        return all(
            math.isfinite(v)
            for v in (
                self.t,
                self.x,
                self.y,
                self.z,
                self.fix_type,
                self.num_sats,
                self.accuracy,
            )
        )


@dataclass(frozen=True)
class Config:
    """判据阈值。默认值对应小型多旋翼在仿真中的低速飞行，可按机型调整。"""

    max_speed: float = 12.0            # 水平最大合理速度 (m/s)
    max_vertical_speed: float = 6.0    # 垂直最大合理速度 (m/s)
    position_margin: float = 1.5       # 允许位移的附加裕度 (m)
    accuracy_limit: float = 3.0        # 水平精度上限 (m)
    min_fix_type: int = 2              # 至少要求 3D 定位
    min_sats: int = 8                  # 最少卫星数
    stale_timeout: float = 1.5         # 两帧之间允许的最大间隔 (s)
    max_extrapolate_dt: float = 0.5    # 位移外推的时间上限，避免长间隔放大跳变阈值
    suspect_limit: int = 2             # 连续坏帧达到该值后进入 HOLD
    recovery_streak: int = 5           # HOLD 后连续可信帧数，满足才恢复
    reacquisition_radius: float = 3.0  # HOLD 期间重新捕获允许离保持点的距离 (m)

    def __post_init__(self) -> None:
        positive = (
            self.max_speed,
            self.max_vertical_speed,
            self.position_margin,
            self.accuracy_limit,
            self.stale_timeout,
            self.max_extrapolate_dt,
            self.reacquisition_radius,
        )
        if any(v <= 0 for v in positive):
            raise ValueError("速度、裕度、超时与门限必须为正数")
        if self.suspect_limit < 1 or self.recovery_streak < 1:
            raise ValueError("suspect_limit 与 recovery_streak 必须 >= 1")
        if self.min_fix_type < 0 or self.min_sats < 0:
            raise ValueError("min_fix_type 与 min_sats 不能为负")


def allowed_displacement(dt: float, cfg: Config) -> Tuple[float, float]:
    """返回 (水平, 垂直) 允许位移，单位米。

    间隔越长本应允许越大位移，但一次长时间失锁后的重新捕获不该被
    「大 dt」合理化，所以外推时间用 max_extrapolate_dt 截断。
    """
    dt_eff = min(max(dt, 0.0), cfg.max_extrapolate_dt)
    return (
        cfg.max_speed * dt_eff + cfg.position_margin,
        cfg.max_vertical_speed * dt_eff + cfg.position_margin,
    )


def check_fix(prev: Optional[Fix], fix: Fix, cfg: Config = Config()) -> Tuple[bool, str]:
    """判断一帧定位是否可信，返回 (是否可信, 原因代码)。

    prev 为上一个可信帧；传 None 时只做质量检查，用于 HOLD 期间的重新捕获。
    """
    if not fix.is_finite():
        return False, "non_finite"
    if prev is not None:
        if fix.t < prev.t:
            return False, "time_reversal"
        dt = fix.t - prev.t
        if dt > cfg.stale_timeout:
            return False, "stale"
    if fix.fix_type < cfg.min_fix_type:
        return False, "fix_type"
    if fix.num_sats < cfg.min_sats:
        return False, "low_sats"
    if fix.accuracy > cfg.accuracy_limit:
        return False, "accuracy"
    if prev is None:
        return True, "quality_ok"

    allowed_h, allowed_v = allowed_displacement(dt, cfg)
    if math.hypot(fix.x - prev.x, fix.y - prev.y) > allowed_h:
        return False, "position_jump"
    if abs(fix.z - prev.z) > allowed_v:
        return False, "altitude_jump"
    return True, "ok"


@dataclass(frozen=True)
class Output:
    """每帧输出。valid=False 时下游应保持悬停并停止使用定位做闭环修正。"""

    state: State
    x: float
    y: float
    z: float
    valid: bool
    action: str            # "track" 或 "hover"
    reason: str
    consecutive_bad: int
    consecutive_good: int


class Monitor:
    """有状态的完整性门控。逐帧调用 update()，输出始终是一个确定的位姿。"""

    def __init__(self, cfg: Config = Config()) -> None:
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.state = State.INIT
        self.last_good: Optional[Fix] = None
        self.last_seen_t: Optional[float] = None
        self._bad = 0
        self._good = 0

    def _emit(self, state: State, pose: Optional[Fix], valid: bool,
              reason: str) -> Output:
        p = pose if pose is not None else Fix(0.0, 0.0, 0.0, 0.0)
        return Output(
            state=state,
            x=p.x,
            y=p.y,
            z=p.z,
            valid=valid,
            action="track" if valid else "hover",
            reason=reason,
            consecutive_bad=self._bad,
            consecutive_good=self._good,
        )

    def note_timeout(self, reason: str = "stream_timeout") -> Optional[Output]:
        """看门狗调用：长时间没有收到新帧时主动进入 HOLD。"""
        if self.last_good is None or self.state == State.HOLD:
            return None
        self.state = State.HOLD
        self._bad = self.cfg.suspect_limit
        self._good = 0
        return self._emit(State.HOLD, self.last_good, False, reason)

    def update(self, fix: Fix) -> Output:
        # 1) 数据流中断：即使本帧本身合法，也要先判定失锁
        gap_reason: Optional[str] = None
        if self.last_seen_t is not None and math.isfinite(fix.t):
            if fix.t - self.last_seen_t > self.cfg.stale_timeout:
                gap_reason = "stream_gap"
        if math.isfinite(fix.t) and (
            self.last_seen_t is None or fix.t > self.last_seen_t
        ):
            self.last_seen_t = fix.t

        # 数据流整体中断时立即失效保护：即使本帧数值合法也不再采信
        if gap_reason and self.last_good is not None and self.state != State.HOLD:
            self.state = State.HOLD
            self._bad = self.cfg.suspect_limit
            self._good = 0
            return self._emit(State.HOLD, self.last_good, False, gap_reason)

        # 2) INIT：只接受质量合格的第一帧
        if self.state == State.INIT:
            ok, reason = check_fix(None, fix, self.cfg)
            if not ok:
                return self._emit(State.INIT, None, False, reason)
            self.last_good = fix
            self.state = State.TRACKING
            self._good = 1
            self._bad = 0
            return self._emit(State.TRACKING, fix, True, reason)

        # 3) HOLD：保持最后可信位姿，只有连续 N 帧在保持点附近才恢复
        if self.state == State.HOLD:
            quality_ok, quality_reason = check_fix(None, fix, self.cfg)
            held = self.last_good
            distance = math.dist(
                (fix.x, fix.y, fix.z), (held.x, held.y, held.z)
            ) if held is not None else math.inf
            near = quality_ok and distance <= self.cfg.reacquisition_radius
            if near:
                self._good += 1
            else:
                self._good = 0
            if self._good >= self.cfg.recovery_streak:
                self.last_good = fix
                self.state = State.TRACKING
                self._bad = 0
                return self._emit(State.TRACKING, fix, True, "reacquired")
            if not quality_ok:
                reason = quality_reason
            elif not near:
                reason = "far_from_hold_pose"
            else:
                reason = "awaiting_reacquisition"
            return self._emit(State.HOLD, held, False, reason)

        # 4) TRACKING / SUSPECT：相对最后可信帧判断运动学合理性
        ok, reason = check_fix(self.last_good, fix, self.cfg)
        if ok:
            self.last_good = fix
            self.state = State.TRACKING
            self._bad = 0
            return self._emit(State.TRACKING, fix, True, reason)

        self._bad += 1
        self._good = 0
        self.state = (
            State.HOLD if self._bad >= self.cfg.suspect_limit else State.SUSPECT
        )
        return self._emit(self.state, self.last_good, False, reason)
