"""自行车运动学小车：按前轮转角推进，沿赛道循线。"""

import math

from . import config
from .track import cross_track_error, nearest_point
from .pid import PID


class Car:
    def __init__(self, track):
        self.track = track
        self.x = config.TRACK_X0 + 10.0
        self.y = config.TRACK_BASE + config.TRACK_AMP * math.sin(config.TRACK_K * self.x)
        self.heading = math.atan2(
            config.TRACK_AMP * config.TRACK_K * math.cos(config.TRACK_K * self.x), 1.0)
        self.pid = PID(config.KP, config.KI, config.KD, config.MAX_TOTAL)
        self.trail = [(self.x, self.y)]
        self.err = 0.0

    def step(self):
        idx = nearest_point(self.track, (self.x, self.y))
        self.err = cross_track_error(self.track, (self.x, self.y), self.heading, idx)
        # 误差为正表示偏右，需要左打舵（负转角）
        steer = -self.pid.step(self.err)
        steer = max(-config.MAX_STEER, min(config.MAX_STEER, steer))
        self.heading += steer * config.SPEED / config.WHEELBASE
        self.x += config.SPEED * math.cos(self.heading)
        self.y += config.SPEED * math.sin(self.heading)
        self.trail.append((self.x, self.y))
        return self.err

    def finished(self):
        return self.x > config.TRACK_X1
