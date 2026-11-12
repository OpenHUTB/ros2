"""驾驶员模型：纯跟踪（pure pursuit）。

工况如果只给开环方向盘角度，车辆一旦偏离就再也回不来，
指标里全是「漂了多远」。用参考路径 + 纯跟踪把横向位置闭到环里，
才能看出 ESC 真正的作用：车被拽偏时能不能守住路径。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class ReferencePath:
    """由一串采样点描述的参考路径（世界坐标）。"""

    points: list[tuple[float, float]] = field(default_factory=list)
    lane_width: float = 3.5

    @classmethod
    def double_lane_change(cls, offset: float = 3.5, x_start: float = 15.0,
                           length: float = 45.0, step: float = 0.5) -> "ReferencePath":
        """双移线：在 length 长度内横向平移 offset，之后保持平移后的车道。"""

        pts: list[tuple[float, float]] = []
        x_end = x_start + length
        x = 0.0
        while x <= x_end + 160.0:
            if x <= x_start:
                y = 0.0
            elif x >= x_end:
                y = offset
            else:
                ratio = (x - x_start) / length
                y = offset * 0.5 * (1.0 - math.cos(math.pi * ratio))
            pts.append((x, y))
            x += step
        return cls(points=pts, lane_width=abs(offset))

    def sample(self, s: float) -> tuple[float, float]:
        """按弧长取点，超出范围时取端点。"""
        if not self.points:
            return 0.0, 0.0
        if s <= 0.0:
            return self.points[0]
        travelled = 0.0
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if travelled + seg >= s:
                ratio = (s - travelled) / seg
                return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio
            travelled += seg
        # 超出末端时沿最后一段方向外推，避免目标点跑到车后面
        xl, yl = self.points[-1]
        xp, yp = self.points[-2] if len(self.points) > 1 else self.points[-1]
        dx, dy = xl - xp, yl - yp
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return self.points[-1]
        extra = s - travelled
        return xl + dx / norm * extra, yl + dy / norm * extra

    def project(self, x: float, y: float) -> tuple[float, float, float]:
        """把世界坐标投影到路径上，返回 (弧长, 横向误差, 切向角)。"""
        best = None
        travelled = 0.0
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            dx, dy = x1 - x0, y1 - y0
            seg_sq = dx * dx + dy * dy
            seg = math.sqrt(seg_sq)
            if seg < 1e-9:
                continue
            t = ((x - x0) * dx + (y - y0) * dy) / seg_sq
            t = max(0.0, min(1.0, t))
            px, py = x0 + dx * t, y0 + dy * t
            dist = math.hypot(x - px, y - py)
            if best is None or dist < best[0]:
                # 用叉积判断偏在路径哪一侧，左正右负
                cross = dx * (y - py) - dy * (x - px)
                sign = 1.0 if cross >= 0.0 else -1.0
                best = (dist, sign * dist, travelled + seg * t, math.atan2(dy, dx))
            travelled += seg
        if best is None:
            return 0.0, 0.0, 0.0
        return best[2], best[1], best[3]


@dataclass
class PurePursuitDriver:
    """纯跟踪驾驶员。"""

    path: ReferencePath
    wheelbase: float = 2.5
    base_lookahead: float = 4.0
    speed_lookahead: float = 0.15
    max_steer: float = math.radians(6.0)
    steer_rate: float = math.radians(240.0)
    """方向盘最大转速，rad/s，用来让动作像人为操作而不是瞬时阶跃。"""

    _last_steer: float = 0.0

    def lookahead(self, speed: float) -> float:
        return self.base_lookahead + self.speed_lookahead * abs(speed)

    def steer(self, x: float, y: float, yaw: float, speed: float, dt: float) -> float:
        s, _, _ = self.path.project(x, y)
        target = self.path.sample(s + self.lookahead(speed))
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return 0.0
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        delta = steer - self._last_steer
        limit = self.steer_rate * dt
        delta = max(-limit, min(limit, delta))
        self._last_steer += delta
        return self._last_steer

    def reset(self) -> None:
        self._last_steer = 0.0
