"""Bug2 反应式避障状态机。

状态：
    GOAL_SEEKING  直线朝目标走；前方被障碍挡住时记录 hit_point，切到 WALL_FOLLOWING
    WALL_FOLLOWING 沿障碍边界绕圈；当轨迹再次穿越 起点-目标 连线(Lm)，且交点比 hit_point
                   更靠近目标时，切回 GOAL_SEEKING
    REACHED       到达目标
"""

import math
from enum import Enum

from . import config
from .geometry import circle_obstacle_nearest, dist, point_segment_dist


class State(Enum):
    GOAL_SEEKING = "goal_seeking"
    WALL_FOLLOWING = "wall_following"
    REACHED = "reached"


class Bug2:
    def __init__(self, start=config.START, goal=config.GOAL, obstacle=config.OBSTACLE):
        self.pos = list(start)
        self.goal = goal
        self.obstacle = obstacle
        self.state = State.GOAL_SEEKING
        self.hit_point = None
        self.trail = [tuple(start)]

    # ---------- 内部 ----------
    def _toward_goal(self):
        d = dist(self.pos, self.goal)
        if d < 1e-6:
            return 0.0, 0.0
        return (self.goal[0] - self.pos[0]) / d, (self.goal[1] - self.pos[1]) / d

    def _blocks_goal(self):
        """朝目标方向上，障碍是否进入机器人前方安全距离。"""
        _, gap = circle_obstacle_nearest(self.pos, self.obstacle)
        # 朝目标投影方向上的障碍间隙
        ux, uy = self._toward_goal()
        cx, cy, _ = self.obstacle
        to_obs = (cx - self.pos[0], cy - self.pos[1])
        proj = to_obs[0] * ux + to_obs[1] * uy
        return 0 < proj < 120 and gap < config.WALL_MARGIN

    def _on_lm(self, tol=config.HIT_TOL):
        """当前位置是否在 起点-目标 连线附近。"""
        d, _ = point_segment_dist(self.pos, config.START, self.goal)
        return d < tol

    def _closer_to_goal_than_hit(self):
        if self.hit_point is None:
            return False
        return dist(self.pos, self.goal) < dist(self.hit_point, self.goal)

    # ---------- 主步进 ----------
    def step(self):
        """推进一步，返回新状态。"""
        if self.state == State.REACHED:
            return self.state

        if dist(self.pos, self.goal) < config.GOAL_TOL:
            self.state = State.REACHED
            return self.state

        if self.state == State.GOAL_SEEKING:
            if self._blocks_goal():
                self.hit_point = tuple(self.pos)
                self.state = State.WALL_FOLLOWING
            else:
                ux, uy = self._toward_goal()
                self.pos[0] += ux * config.SPEED
                self.pos[1] += uy * config.SPEED

        elif self.state == State.WALL_FOLLOWING:
            surface, gap = circle_obstacle_nearest(self.pos, self.obstacle)
            # 沿障碍切线方向走，同时略微向内修正保持 WALL_MARGIN
            cx, cy, _ = self.obstacle
            rx, ry = self.pos[0] - cx, self.pos[1] - cy
            rlen = math.hypot(rx, ry) or 1e-6
            # 切线（逆时针绕障碍）
            tx, ty = -ry / rlen, rx / rlen
            # 法向（指向障碍中心）
            nx, ny = -rx / rlen, -ry / rlen
            # 间距修正：太靠近就往外，太远就往内
            correction = (config.WALL_MARGIN - gap) * 0.05
            self.pos[0] += (tx + nx * correction) * config.SPEED
            self.pos[1] += (ty + ny * correction) * config.SPEED

            # 绕回 Lm 且更靠近目标 -> 切回直线
            if self._on_lm() and self._closer_to_goal_than_hit():
                self.state = State.GOAL_SEEKING

        self.trail.append(tuple(self.pos))
        return self.state
