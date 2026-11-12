"""过渡剖面管理：悬停 → 加速过渡 → 巡航 → 减速过渡 → 悬停，含掉高保护。

这里只负责"该飞多快、该飞多高"，不碰倾转角——倾转角由分配律决定。
这样"走廊"才是**约束**而不是轨迹，掉出走廊时才真的会掉高度。
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["PilotConfig", "TransitionPilot"]


@dataclass
class PilotConfig:
    hover_alt: float = 30.0
    cruise_speed: float = 20.0
    accel: float = 0.6            # 空速指令的斜坡加速度，m/s²
    hover_hold: float = 6.0       # 起步悬停，s
    cruise_hold: float = 25.0     # 巡航保持，s
    sink_abort: float = 2.0       # 触发保护的下沉率，m/s
    sink_abort_time: float = 0.8  # 持续多久才触发，s
    alt_drop_abort: float = 8.0   # 相对目标高度允许的下沉，m


class TransitionPilot:
    """状态机：``hover → up → cruise → down → done``，异常进入 ``abort``。"""

    def __init__(self, cfg: PilotConfig | None = None):
        self.cfg = cfg or PilotConfig()
        self.reset()

    def reset(self) -> None:
        self.mode = "hover"
        self.u_ref = 0.0
        self.sink_timer = 0.0
        self.abort_reason: str | None = None
        self._t_cruise = 0.0

    def update(self, t: float, h: float, u: float, w: float, dt: float) -> tuple[float, float]:
        """返回 ``(高度指令, 空速指令)``。"""
        c = self.cfg
        self._check_abort(t, h, w, dt)

        if self.mode == "abort":
            self.u_ref = max(0.0, self.u_ref - 3.0 * dt)
            return c.hover_alt, self.u_ref

        if self.mode == "hover":
            self.u_ref = 0.0
            if t >= c.hover_hold:
                self.mode = "up"
        elif self.mode == "up":
            self.u_ref = min(c.cruise_speed, self.u_ref + c.accel * dt)
            if self.u_ref >= c.cruise_speed - 1e-9:
                self.mode = "cruise"
                self._t_cruise = t
        elif self.mode == "cruise":
            self.u_ref = c.cruise_speed
            if t - self._t_cruise >= c.cruise_hold:
                self.mode = "down"
        elif self.mode == "down":
            self.u_ref = max(0.0, self.u_ref - c.accel * dt)
            if self.u_ref <= 1e-9:
                self.mode = "done"
        elif self.mode == "done":
            self.u_ref = 0.0

        return c.hover_alt, self.u_ref

    def _check_abort(self, t: float, h: float, w: float, dt: float) -> None:
        if self.mode == "abort":
            return
        c = self.cfg
        if w < -c.sink_abort:
            self.sink_timer += dt
        else:
            self.sink_timer = max(0.0, self.sink_timer - 0.5 * dt)
        if self.sink_timer >= c.sink_abort_time:
            self.mode = "abort"
            self.abort_reason = f"下沉率 {w:.2f} m/s 持续 {self.sink_timer:.1f} s"
        elif h < c.hover_alt - c.alt_drop_abort:
            self.mode = "abort"
            self.abort_reason = f"高度损失 {c.hover_alt - h:.1f} m"

    @property
    def finished(self) -> bool:
        return self.mode in ("done", "abort") and abs(self.u_ref) < 0.2
