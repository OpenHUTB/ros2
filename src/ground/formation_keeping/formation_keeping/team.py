"""编队状态机：领航者沿圆周运动，跟随者追期望队形位置。"""


import math
from collections import deque


from . import config, formation


# 轨迹与误差记录的长度上限，避免长时间运行时内存与绘制开销无限增长。
# 60 FPS 下 3000 点约等于 50 秒窗口，足够观察收敛过程与稳态保持。
TRAIL_MAXLEN = 3000
ERROR_WINDOW = 3000


class Team:
    def __init__(self):
        self.step_count = 0
        self.leader_pos, self.leader_heading = formation.leader_state(0)
        self.followers = list(config.INIT_POSITIONS)
        self.trails = [deque([p], maxlen=TRAIL_MAXLEN) for p in self.followers]
        self.leader_trail = deque([self.leader_pos], maxlen=TRAIL_MAXLEN)
        self.errors = []


    def positions(self):
        return [self.leader_pos] + list(self.followers)


    def desired(self):
        return formation.desired_positions(self.leader_pos, self.leader_heading)


    def error(self):
        return formation.formation_error(self.positions(), self.desired())


    def settled(self):
        return (self.step_count >= config.SETTLE_STEPS
                and self.error() <= config.TOL)


    def step(self):
        self.step_count += 1
        self.leader_pos, self.leader_heading = formation.leader_state(self.step_count)
        self.leader_trail.append(self.leader_pos)


        desired = self.desired()[1:]
        new_followers = []
        for p, d in zip(self.followers, desired):
            dx = d[0] - p[0]
            dy = d[1] - p[1]
            ex = config.GAIN * dx
            ey = config.GAIN * dy
            n = math.hypot(ex, ey)
            if n > config.MAX_STEP:
                ex = ex * config.MAX_STEP / n
                ey = ey * config.MAX_STEP / n
            new_followers.append((p[0] + ex, p[1] + ey))
        self.followers = new_followers
        for i, p in enumerate(self.followers):
            self.trails[i].append(p)


        err = self.error()
        self.errors.append(err)
        if len(self.errors) > ERROR_WINDOW:
            del self.errors[0]
        return err
