"""落脚点、支撑区间与 ZMP 参考轨迹。

步态时序（以 12 步为例）：
- 第 i 只脚在 t_i = i * step_time 落地，支撑 stance_duration = 0.6 s；
- 相邻脚落地间隔 step_time = 0.5 s，因此双支撑时长 0.1 s；
- 单脚支撑时 ZMP 参考在脚掌中心；双脚支撑时沿前进方向从后脚中心
  线性过渡到前脚中心，保证参考轨迹连续。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Footstep:
    """单只脚的落地信息。"""

    index: int
    side: str
    x: float
    land_time: float
    lift_time: float


@dataclass(frozen=True)
class SupportState:
    """某一时刻的支撑多边形和 ZMP 参考。"""

    time: float
    left: float
    right: float
    zmp_ref: float
    phase: str
    stance_indices: tuple[int, ...]
    center: float

    @property
    def width(self) -> float:
        return self.right - self.left


class FootstepPlan:
    """离线双足落脚计划。

    half_length 按题设为足底几何半长 0.09 m。为了在仅靠 ZMP 的模型里
    给踝关节保留 0.03 m 的边界调节余量，ZMP 可行域半宽取 0.12 m。
    两个量分开保存，便于在 README 中说明这一复现假设。
    """

    def __init__(
        self,
        n_steps: int = 12,
        step_length: float = 0.35,
        step_time: float = 0.5,
        stance_ratio: float = 0.6,
        half_length: float = 0.09,
        zmp_edge_margin: float = 0.03,
        swing_height: float = 0.06,
    ) -> None:
        if n_steps < 2:
            raise ValueError("n_steps 至少为 2")
        if step_length <= 0 or step_time <= 0:
            raise ValueError("步长和步时必须为正")
        if not 0.5 < stance_ratio < 1.0:
            raise ValueError("stance_ratio 必须位于 (0.5, 1) 才能形成双支撑")

        self.n_steps = int(n_steps)
        self.step_length = float(step_length)
        self.step_time = float(step_time)
        self.stance_ratio = float(stance_ratio)
        self.stance_duration = 2.0 * self.step_time * self.stance_ratio
        self.half_length = float(half_length)
        self.zmp_edge_margin = float(zmp_edge_margin)
        self.support_half_width = self.half_length + self.zmp_edge_margin
        self.swing_height = float(swing_height)
        self.cycle_time = 2.0 * self.step_time

        self.steps: list[Footstep] = []
        for i in range(self.n_steps):
            land = i * self.step_time
            self.steps.append(
                Footstep(
                    index=i,
                    side="L" if i % 2 == 0 else "R",
                    x=i * self.step_length,
                    land_time=land,
                    lift_time=land + self.stance_duration,
                )
            )

        self.duration = (self.n_steps - 1) * self.step_time + self.stance_duration

    # ------------------------------------------------------------------
    # 基本查询
    # ------------------------------------------------------------------
    def foot_center(self, index: int) -> float:
        return self.steps[index].x

    def stance_interval(self, index: int) -> tuple[float, float]:
        step = self.steps[index]
        return step.land_time, step.lift_time

    def active_steps(self, t: float, eps: float = 1e-9) -> list[Footstep]:
        """返回时刻 t 处于支撑相的脚。"""
        active = [
            s
            for s in self.steps
            if s.land_time - eps <= t <= s.lift_time + eps
        ]
        if active:
            return active
        if t < 0.0:
            return [self.steps[0]]
        # 数值上正好越界时沿用最近一次支撑，避免空区间。
        if t >= self.steps[-1].land_time:
            return [self.steps[-1]]
        for s in reversed(self.steps):
            if s.land_time <= t:
                return [s]
        return [self.steps[0]]

    def support_state(self, t: float) -> SupportState:
        """计算 (lo, hi, p_ref) 以及支撑相类型。"""
        if not math.isfinite(t):
            raise ValueError("t 必须为有限值")
        # 仿真终点恰好是最后一只脚离地时刻，钳位可避免浮点边界问题。
        t_clamped = min(max(float(t), 0.0), self.duration)
        active = self.active_steps(t_clamped)
        if len(active) == 1:
            step = active[0]
            return SupportState(
                time=t_clamped,
                left=step.x - self.support_half_width,
                right=step.x + self.support_half_width,
                zmp_ref=step.x,
                phase="single",
                stance_indices=(step.index,),
                center=step.x,
            )

        old, new = active[0], active[1]
        denom = old.lift_time - new.land_time
        alpha = 1.0 if denom <= 0 else (t_clamped - new.land_time) / denom
        alpha = min(max(alpha, 0.0), 1.0)
        # 沿前进方向以后脚 -> 前脚线性转移，保证 ZMP 连续且始终位于并集内。
        p_ref = (1.0 - alpha) * old.x + alpha * new.x
        return SupportState(
            time=t_clamped,
            left=old.x - self.support_half_width,
            right=new.x + self.support_half_width,
            zmp_ref=p_ref,
            phase="double",
            stance_indices=(old.index, new.index),
            center=0.5 * (old.x + new.x),
        )

    # ------------------------------------------------------------------
    # 可视化辅助
    # ------------------------------------------------------------------
    def foot_pose(self, index: int, t: float) -> tuple[float, float, bool]:
        """返回第 index 只脚在 t 时刻的 (x, z, 是否支撑)。

        摆动相用平滑三次插值和正弦抬脚曲线。
        """
        if index < 0 or index >= self.n_steps:
            return float(index * self.step_length), 0.0, False
        step = self.steps[index]
        if step.land_time <= t <= step.lift_time:
            return step.x, 0.0, True

        if t < step.land_time:
            if index >= 2:
                prev = self.steps[index - 2]
                start = prev.lift_time
                x0, x1 = prev.x, step.x
            else:
                start = step.land_time - self.cycle_time + 0.1 * self.step_time
                x0, x1 = step.x - self.step_length, step.x
            if t <= start:
                return x0, 0.0, False
            u = min(max((t - start) / max(step.land_time - start, 1e-9), 0.0), 1.0)
            return self._swing_curve(x0, x1, u)

        # 支撑结束后的摆动：去往同一只脚的下一个落点。
        if index + 2 < self.n_steps:
            nxt = self.steps[index + 2]
            start = step.lift_time
            u = min(max((t - start) / max(nxt.land_time - start, 1e-9), 0.0), 1.0)
            return self._swing_curve(step.x, nxt.x, u)
        return step.x, 0.0, False

    def foot_pose_by_side(self, side: str, t: float) -> tuple[float, float, bool]:
        """按左右脚查询当前姿态。

        若该侧脚正处于摆动相（离地后飞向下一落点），返回摆动姿态；
        否则返回该侧最近一只已落地脚的姿态。供动画使用。
        """
        if side not in ("L", "R"):
            raise ValueError("side 必须是 'L' 或 'R'")
        indices = [i for i, s in enumerate(self.steps) if s.side == side]
        if not indices:
            return 0.0, 0.0, False

        # 正在摆动：land_time 已过、lift_time 未到，且 t 落在上一落点之后的摆动态。
        for i in indices:
            step = self.steps[i]
            if step.land_time - 1e-9 <= t < step.lift_time:
                return self.foot_pose(i, t)
            if i + 2 < self.n_steps:
                nxt = self.steps[i + 2]
                if step.lift_time - 1e-9 <= t < nxt.land_time:
                    return self.foot_pose(i, t)

        landed = [i for i in indices if self.steps[i].land_time <= t + 1e-9]
        idx = landed[-1] if landed else indices[0]
        return self.foot_pose(idx, t)

    def _swing_curve(self, x0: float, x1: float, u: float) -> tuple[float, float, bool]:
        # smootherstep 使起落点速度为 0，画面更自然。
        s = u * u * u * (u * (6.0 * u - 15.0) + 10.0)
        x = (1.0 - s) * x0 + s * x1
        z = self.swing_height * math.sin(math.pi * u)
        return x, z, False

    def current_legs(self, t: float) -> tuple[tuple[float, float, bool], tuple[float, float, bool]]:
        """返回左右腿当前视觉姿态，供火柴人动画使用。"""
        state = self.support_state(t)
        stance = set(state.stance_indices)

        def pick(side: str) -> tuple[float, float, bool]:
            if side == "L":
                indices = [i for i, s in enumerate(self.steps) if s.side == "L"]
            else:
                indices = [i for i, s in enumerate(self.steps) if s.side == "R"]
            # 已经落地的最近一只脚；若仍在摆动，则使用其摆动姿态。
            landed = [i for i in indices if self.steps[i].land_time <= t + 1e-9]
            if landed:
                idx = landed[-1]
                x, z, is_stance = self.foot_pose(idx, t)
                return x, z, idx in stance or is_stance
            idx = indices[0]
            return self.foot_pose(idx, t)

        return pick("L"), pick("R")

    # ------------------------------------------------------------------
    # 批量采样
    # ------------------------------------------------------------------
    def reference_samples(self, duration: float, dt: float = 1e-3) -> dict[str, np.ndarray]:
        n = int(round(duration / dt))
        times = np.arange(n + 1, dtype=float) * dt
        lo = np.empty(n + 1)
        hi = np.empty(n + 1)
        ref = np.empty(n + 1)
        phase = np.empty(n + 1, dtype=object)
        for k, t in enumerate(times):
            s = self.support_state(t)
            lo[k], hi[k], ref[k], phase[k] = s.left, s.right, s.zmp_ref, s.phase
        return {"time": times, "left": lo, "right": hi, "zmp_ref": ref, "phase": phase}
