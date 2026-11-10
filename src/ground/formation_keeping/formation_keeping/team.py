"""编队状态机：领航者沿圆周运动，跟随者追期望队形位置。"""


import math


from . import config, formation




class Team:
    def __init__(self):
        self.step_count = 0
        self.leader_pos, self.leader_heading = formation.leader_state(0)
        self.followers = list(config.INIT_POSITIONS)
        self.trails = [[p] for p in self.followers]
        self.leader_trail = [self.leader_pos]
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
        return err
