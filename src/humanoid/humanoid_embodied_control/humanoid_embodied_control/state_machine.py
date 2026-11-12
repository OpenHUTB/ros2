# -*- coding: utf-8 -*-
"""任务状态机：把"感知 + 规划 + 步态"串成一条可解释的行为序列。

::

    IDLE ──► WALK ──► AVOID ──► WALK ──► ... ──► REACHED
              │          │
              └──────────┴──► BLOCKED（前方被堵死 / 摔倒）

为什么用显式状态机而不是一个 if-else 堆
----------------------------------------
三个状态各自有独立的进入/退出条件与迟滞（hysteresis）：没有迟滞时，
"前方距离"在阈值附近抖动会让 AVOID 与 WALK 每帧互切，步态相位被反复重置，
机器人表现为原地抽搐。把迟滞值和切换条件写在状态机里，是唯一能单测的地方。
"""

from __future__ import annotations

from enum import Enum


class State(str, Enum):
    """任务状态。"""

    IDLE = "IDLE"
    WALK = "WALK"
    AVOID = "AVOID"
    REACHED = "REACHED"
    BLOCKED = "BLOCKED"
    FALLEN = "FALLEN"


class TaskStateMachine:
    """跟踪"走向目标"任务的状态迁移。

    参数
    ----
    goal_tolerance : float
        进入 REACHED 的到达半径（米）。
    obstacle_slow : float
        前方障碍距离低于它时降速（不切状态）。
    obstacle_stop : float
        前方障碍距离低于它时进入 AVOID。
    avoid_clear : float
        从 AVOID 退出需要的前方距离。**必须严格大于** ``obstacle_stop``，
        否则会在阈值附近反复横跳。
    max_blocked_time : float
        在 AVOID 内持续超过该时长且没有进展 → 判 BLOCKED（真的过不去）。
    """

    def __init__(self, goal_tolerance=0.35, obstacle_slow=1.2, obstacle_stop=0.65,
                 avoid_clear=1.0, max_blocked_time=12.0):
        if not (avoid_clear > obstacle_stop):
            raise ValueError("avoid_clear 必须大于 obstacle_stop（迟滞要求）")
        self.goal_tolerance = float(goal_tolerance)
        self.obstacle_slow = float(obstacle_slow)
        self.obstacle_stop = float(obstacle_stop)
        self.avoid_clear = float(avoid_clear)
        self.max_blocked_time = float(max_blocked_time)

        self.state = State.IDLE
        self.avoid_time = 0.0
        self.transitions = []

    # ------------------------------------------------------------------
    def reset(self):
        self.state = State.IDLE
        self.avoid_time = 0.0
        self.transitions = []

    def _to(self, new_state, reason, t):
        if new_state is not self.state:
            self.transitions.append({"t": round(float(t), 3),
                                     "from": self.state.value,
                                     "to": new_state.value,
                                     "reason": reason})
            self.state = new_state
            if new_state is State.AVOID:
                self.avoid_time = 0.0

    # ------------------------------------------------------------------
    def update(self, t, goal_distance, front_clearance, fallen=False):
        """推进一次状态机，返回新的 :class:`State`。

        goal_distance   : 到目标的直线距离（米）
        front_clearance : 正前方可通行距离（米）
        fallen          : 是否已摔倒（摔倒优先级最高，直接终止任务）
        """
        if fallen:
            self._to(State.FALLEN, "tilt_or_height", t)
            return self.state

        if self.state in (State.REACHED, State.FALLEN):
            return self.state

        if goal_distance <= self.goal_tolerance:
            self._to(State.REACHED, "goal_within_tolerance", t)
            return self.state

        if self.state is State.IDLE:
            self._to(State.WALK, "start", t)
            return self.state

        if self.state is State.WALK:
            if front_clearance < self.obstacle_stop:
                self._to(State.AVOID, "front_obstacle", t)
            return self.state

        if self.state is State.AVOID:
            self.avoid_time += 0.0  # 由 tick_avoid 累积
            if front_clearance >= self.avoid_clear:
                self._to(State.WALK, "path_clear", t)
            elif self.avoid_time > self.max_blocked_time:
                self._to(State.BLOCKED, "no_progress", t)
            return self.state

        return self.state

    def tick_avoid(self, dt: float):
        """在 AVOID 状态下累积停滞时间（每个控制周期调用一次）。"""
        if self.state is State.AVOID:
            self.avoid_time += float(dt)

    # ------------------------------------------------------------------
    def speed_scale(self, front_clearance) -> float:
        """由前方距离给出速度缩放 [0,1]。

        线性降速而不是"开关"：速度跳变会让步态相位突然加速，机器人会前倾。
        从 ``obstacle_slow`` 到 ``obstacle_stop`` 之间线性从 1 降到 0.25，
        再近就交给状态机处理。
        """
        if front_clearance >= self.obstacle_slow:
            return 1.0
        if front_clearance <= self.obstacle_stop:
            return 0.25
        span = self.obstacle_slow - self.obstacle_stop
        return 0.25 + 0.75 * (front_clearance - self.obstacle_stop) / span
