"""脱困策略：分级状态机（倒车 -> 转向 -> 试探），打滑走脉冲模式。

设计原则：

1. **按故障类型换动作**。被顶住/打转要"退出来再换个方向"，轮胎打滑要"停顿-起步"
   （静摩擦大于动摩擦），两者用同一套动作只会白费时间。
2. **分级升级**。同一位置反复触发时不再从第 1 级重来，而是直接加大倒车距离与转向角，
   避免"挣扎-卡住-再挣扎"的死循环。
3. **只做有限次尝试**。连续 3 级都失败就停车上报，交给人工，而不是无休止乱动把车顶到
   更坏的姿态。
4. **数据不可信就停**。里程计超时直接停车，不基于旧位姿继续动作。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .detect import CAUSE_SLIP, Verdict
from .robot import Limits, Twist, clamp_twist, wrap_angle

STATE_CRUISE = "CRUISE"
STATE_BACK_OFF = "BACK_OFF"
STATE_TURN = "TURN"
STATE_PROBE = "PROBE"
STATE_PULSE = "PULSE"
STATE_ABORT = "ABORT"
STATE_STALE = "STALE"


@dataclass(frozen=True)
class RecoveryConfig:
    back_base: float = 0.22            # 第 1 级倒车距离 [m]
    back_step: float = 0.10            # 每升一级追加的倒车距离 [m]
    back_speed: float = 0.14           # 倒车速度 [m/s]
    back_timeout: float = 3.0          # 倒车超时 [s]
    back_min_travel: float = 0.02      # 倒车位移低于此值视为"倒不动" [m]
    stall_time: float = 1.20           # 允许"给了指令还没动"的时间（覆盖反向加速过程）[s]
    turn_rate: float = 0.90            # 原地转向角速度上限 [rad/s]
    turn_p_gain: float = 2.00          # 转向 P 增益
    turn_tolerance: float = 0.07       # 转向到位容差 [rad]
    turn_timeout: float = 3.5          # 转向超时 [s]
    turn_angles: tuple = (0.90, 1.60, 2.60)   # 各级转向角 [rad]
    probe_speed: float = 0.14          # 试探前进速度 [m/s]
    probe_min_travel: float = 0.06     # 试探成功所需位移 [m]
    probe_timeout: float = 0.60        # 试探时长 [s]
    probe_imu_gap: float = 0.20        # 试探验收时允许的角速度偏差 [rad/s]
    pulse_speed: float = 0.26          # 脉冲驱动速度 [m/s]
    pulse_on: float = 0.18             # 脉冲"给油"时长 [s]
    pulse_off: float = 0.28            # 脉冲"停顿"时长 [s]
    pulse_cycles: int = 4              # 脉冲循环次数
    pulse_wiggle: float = 0.16         # 脉冲期间的转向摆动 [rad/s]
    max_level: int = 3                 # 最高尝试等级
    stale_odom: float = 0.50           # 里程计超时 [s]
    memory_radius: float = 2.00        # 记忆半径：同位置再触发 [m]
    memory_time: float = 20.0          # 记忆有效期 [s]
    back_wall_margin: float = 0.30     # 无后向传感器时的倒车距离硬上限 [m]


@dataclass(frozen=True)
class Command:
    """策略输出。``override=True`` 表示这条指令覆盖任务层指令。"""

    v: float = 0.0
    w: float = 0.0
    state: str = STATE_CRUISE
    note: str = ""
    override: bool = False
    level: int = 0

    def as_twist(self) -> Twist:
        return Twist(self.v, self.w)


@dataclass
class _Memory:
    x: float = 0.0
    y: float = 0.0
    t: float = -1e9
    level: int = 0
    valid: bool = False


class RecoverySupervisor:
    """脱困监督器：吃任务层指令 + 检测结论 + 位姿，吐底盘指令。"""

    def __init__(self, config: Optional[RecoveryConfig] = None, limits: Optional[Limits] = None) -> None:
        self.cfg = config or RecoveryConfig()
        self.limits = limits or Limits()
        self.state = STATE_CRUISE
        self.level = 0
        self.cause = "none"
        self.history: list[tuple[float, str]] = []
        self.triggers = 0
        self.escapes = 0
        self._t_state = 0.0
        self._start_pose = (0.0, 0.0, 0.0)
        self._turn_from = 0.0
        self._turn_to = 0.0
        self._turn_index = 0
        self._pulse_phase = 0
        self._pulse_t = 0.0
        self._memory = _Memory()
        self._last_pose = (0.0, 0.0, 0.0)
        self._travel = 0.0
        self._imu_gap = 0.0
        self._family = "back"

    # ------------------------------------------------------------------ 对外
    def clear_abort(self) -> None:
        """人工确认后解除 ABORT。"""
        if self.state == STATE_ABORT:
            self._enter(STATE_CRUISE, 0.0, (self._last_pose[0], self._last_pose[1], self._last_pose[2]), "人工解除")

    def status(self) -> dict:
        return {
            "state": self.state,
            "cause": self.cause,
            "level": self.level,
            "triggers": self.triggers,
            "escapes": self.escapes,
            "history": list(self.history[-8:]),
        }

    def step(
        self,
        t: float,
        task_cmd: Twist,
        verdict: Verdict,
        odom_age: float,
        pose: tuple[float, float, float],
        imu_gap: float = 0.0,
    ) -> Command:
        cfg = self.cfg
        self._last_pose = pose
        self._imu_gap = imu_gap
        if odom_age > cfg.stale_odom:
            self.state = STATE_STALE
            # 启动期尚未收到里程计时 age 为 inf，不能打印成 "infs"
            note = (
                f"里程计 {odom_age:.2f}s 未更新，停车"
                if math.isfinite(odom_age)
                else "尚未收到里程计数据，停车（fail closed）"
            )
            return Command(0.0, 0.0, STATE_STALE, note, True, self.level)

        if self.state == STATE_STALE:
            self.state = STATE_CRUISE

        if self.state == STATE_ABORT:
            return Command(0.0, 0.0, STATE_ABORT, "已放弃：连续 3 级脱困失败，等待人工接管", True, self.level)

        if self.state == STATE_CRUISE:
            if verdict.stuck:
                self._begin(t, verdict, pose)
            else:
                cmd = clamp_twist(task_cmd, self.limits)
                return Command(cmd.v, cmd.w, STATE_CRUISE, "透传任务指令", False, 0)

        return self._advance(t, pose, verdict)

    # ------------------------------------------------------------------ 内部
    def _begin(self, t: float, verdict: Verdict, pose: tuple[float, float, float]) -> None:
        cfg = self.cfg
        self.triggers += 1
        self.cause = verdict.cause
        level = 1
        if self._memory.valid and t - self._memory.t <= cfg.memory_time:
            near = math.hypot(pose[0] - self._memory.x, pose[1] - self._memory.y) <= cfg.memory_radius
            if near:
                level = min(self._memory.level + 1, cfg.max_level)
        self.level = level
        self._turn_index = 0
        self._family = "pulse" if verdict.cause == CAUSE_SLIP else "back"
        start = STATE_PULSE if self._family == "pulse" else STATE_BACK_OFF
        self._enter(start, t, pose, f"检测到{verdict.cause_zh}（置信度 {verdict.confidence:.2f}），从第 {level} 级开始脱困")

    def _enter(self, state: str, t: float, pose: tuple[float, float, float], note: str) -> None:
        self.state = state
        self._t_state = t
        self._start_pose = pose
        self._travel = 0.0
        self._pulse_phase = 0
        self._pulse_t = t
        self.history.append((t, f"{state}:{note}"))

    def _advance(self, t: float, pose: tuple[float, float, float], verdict: Verdict) -> Command:
        cfg = self.cfg
        elapsed = t - self._t_state
        self._travel = max(self._travel, math.hypot(pose[0] - self._start_pose[0], pose[1] - self._start_pose[1]))

        if self.state == STATE_BACK_OFF:
            target = min(cfg.back_base + cfg.back_step * (self.level - 1), cfg.back_wall_margin + cfg.back_base)
            if self._travel >= target:
                return self._to_turn(t, pose)
            # 注意：从正速度反转到倒车要 ~0.4 s 的加速过程，判定窗口必须留够，
            # 否则会在车还没动起来时就误判为"倒不动"并空转升级。
            # 超时上限随目标距离放宽，否则调大倒车距离会先被超时卡住。
            t_limit = cfg.back_timeout + target / max(cfg.back_speed, 1e-6)
            stalled = elapsed > t_limit or (
                elapsed > cfg.stall_time and self._travel < cfg.back_min_travel
            )
            if stalled:
                return self._escalate(t, pose, "倒车没有让出空间")
            return self._out(-cfg.back_speed, 0.0, "倒车让位")

        if self.state == STATE_TURN:
            err = wrap_angle(self._turn_to - pose[2])
            if abs(err) <= cfg.turn_tolerance or elapsed > cfg.turn_timeout:
                return self._to_probe(t, pose)
            w = max(-cfg.turn_rate, min(cfg.turn_rate, cfg.turn_p_gain * err))
            # 纯原地转向在墙角常常转不动，给一点倒车分量卸载
            return self._out(-0.02, w, "原地转向到空旷方向")

        if self.state == STATE_PROBE:
            # 验收"真的动起来了"：里程计有位移，且陀螺仪确认车身没在乱转
            if self._travel >= cfg.probe_min_travel and self._imu_gap <= cfg.probe_imu_gap:
                return self._succeed(t, pose)
            if elapsed > cfg.probe_timeout:
                return self._escalate(t, pose, "试探前进仍无位移")
            return self._out(cfg.probe_speed, 0.0, "试探前进")

        if self.state == STATE_PULSE:
            return self._pulse(t, pose, elapsed)

        return self._out(0.0, 0.0, "未知状态，停车")

    def _pulse(self, t: float, pose: tuple[float, float, float], elapsed: float) -> Command:
        cfg = self.cfg
        cycle = cfg.pulse_on + cfg.pulse_off
        total = cycle * cfg.pulse_cycles
        if elapsed >= total or self._travel >= 0.30:
            return self._to_probe(t, pose)
        phase = elapsed % cycle
        sign = 1.0 if (int(elapsed // cycle) % 2 == 0) else -1.0
        if phase < cfg.pulse_on:
            return self._out(cfg.pulse_speed, cfg.pulse_wiggle * sign, "脉冲给油（利用静摩擦）")
        return self._out(0.0, 0.0, "脉冲停顿")

    def _to_turn(self, t: float, pose: tuple[float, float, float]) -> Command:
        cfg = self.cfg
        angle = cfg.turn_angles[min(self.level - 1, len(cfg.turn_angles) - 1)]
        sign = 1.0 if self._turn_index % 2 == 0 else -1.0
        self._turn_index += 1
        self._turn_from = pose[2]
        self._turn_to = wrap_angle(pose[2] + sign * angle)
        self._enter(STATE_TURN, t, pose, f"转向 {sign * angle:+.2f} rad")
        return self._advance(t, pose, Verdict())

    def _to_probe(self, t: float, pose: tuple[float, float, float]) -> Command:
        self._enter(STATE_PROBE, t, pose, "试探前进")
        return self._advance(t, pose, Verdict())

    def _succeed(self, t: float, pose: tuple[float, float, float]) -> Command:
        self.escapes += 1
        self._memory = _Memory(x=pose[0], y=pose[1], t=t, level=self.level, valid=True)
        self._enter(STATE_CRUISE, t, pose, f"脱困成功（第 {self.level} 级）")
        self.level = 0
        return self._out(0.0, 0.0, "脱困成功，交回任务层")

    def _escalate(self, t: float, pose: tuple[float, float, float], why: str) -> Command:
        cfg = self.cfg
        self.level += 1
        if self.level > cfg.max_level:
            self._enter(STATE_ABORT, t, pose, why)
            self.history.append((t, "ABORT:连续失败，停车上报"))
            self.level = cfg.max_level
            return Command(0.0, 0.0, STATE_ABORT, f"连续 {cfg.max_level} 级失败（{why}），停车等待人工", True, self.level)
        start = STATE_PULSE if self.cause == CAUSE_SLIP else STATE_BACK_OFF
        if self.cause == CAUSE_SLIP:
            # 打滑时两类动作可以互替：脉冲（保持位置）与倒车（换地面）。
            # 一类动作失败就换另一类，而不是在同一类上加码。
            self._family = "back" if self._family == "pulse" else "pulse"
            start = STATE_PULSE if self._family == "pulse" else STATE_BACK_OFF
        else:
            self._family = "back"
            start = STATE_BACK_OFF
        self._enter(start, t, pose, f"升级到第 {self.level} 级（换动作族）：{why}")
        return self._advance(t, pose, Verdict())

    def _out(self, v: float, w: float, note: str) -> Command:
        cmd = clamp_twist(Twist(v, w), self.limits)
        return Command(cmd.v, cmd.w, self.state, note, True, self.level)
