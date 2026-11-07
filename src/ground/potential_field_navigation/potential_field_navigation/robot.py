"""小车运动学与势场导航状态机。"""

import math

from . import config, field


class Robot:
    def __init__(self, start=None, goal=None, obstacles=None):
        self.start = config.START if start is None else start
        self.goal = config.GOAL if goal is None else goal
        self.obstacles = list(config.OBSTACLES if obstacles is None else obstacles)
        self.p = self.start
        self.heading = math.atan2(self.goal[1] - self.p[1],
                                  self.goal[0] - self.p[0])
        self.trail = [self.p]
        self.steps = 0
        self.done = False
        self.status = "moving"
        self.history = [self.distance()]
        self.escape_left = 0
        self.min_clearance = self.clearance()

    def distance(self):
        return math.hypot(self.goal[0] - self.p[0], self.goal[1] - self.p[1])

    def clearance(self):
        """到最近障碍表面的距离，负数表示已侵入。"""
        best = float("inf")
        for cx, cy, r in self.obstacles:
            best = min(best, math.hypot(self.p[0] - cx, self.p[1] - cy) - r)
        return best

    def stalled(self):
        """最近若干帧离目标几乎没变近，判定为局部极小点。"""
        if len(self.history) <= config.STALL_WINDOW:
            return False
        return self.history[-1] >= self.history[-config.STALL_WINDOW] - config.STALL_PROGRESS

    def step(self):
        if self.done:
            return self.status

        d = self.distance()
        if d <= config.GOAL_TOL:
            self.done = True
            self.status = "arrived"
            return self.status

        if self.escape_left == 0 and self.stalled():
            self.escape_left = config.ESCAPE_FRAMES
            self.status = "escaping"

        bias = config.ESCAPE_BIAS if self.escape_left > 0 else None
        if self.escape_left > 0:
            self.escape_left -= 1

        ux, uy = field.desired_heading(self.p, self.goal, self.obstacles, bias=bias)
        want = math.atan2(uy, ux)
        diff = want - self.heading
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        diff = max(-config.MAX_TURN, min(config.MAX_TURN, diff))
        self.heading += diff

        self.p = (self.p[0] + config.SPEED * math.cos(self.heading),
                  self.p[1] + config.SPEED * math.sin(self.heading))
        self.trail.append(self.p)
        self.steps += 1
        self.history.append(self.distance())
        self.min_clearance = min(self.min_clearance, self.clearance())
        if self.escape_left == 0 and self.status == "escaping":
            self.status = "moving"
        return self.status
