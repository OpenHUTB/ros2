"""机械臂状态机：逐帧插值到逆解，记录末端轨迹，按序切换目标。"""

import math

from . import config, kinematics


class Arm:
    def __init__(self, targets=None):
        self.q1 = 0.35
        self.q2 = 0.60
        self.targets = list(config.TARGETS if targets is None else targets)
        self.index = 0
        self.trail = []
        self.reached = []
        self.last_status = "moving"
        self.elbow, self.tip = kinematics.forward_kinematics(self.q1, self.q2)

    @property
    def target(self):
        if self.index < len(self.targets):
            return self.targets[self.index]
        return None

    def step(self):
        """推进一帧，返回本帧的状态字符串。"""
        target = self.target
        if target is None:
            self.last_status = "done"
            return self.last_status

        elbow, tip = kinematics.forward_kinematics(self.q1, self.q2)
        dist = math.hypot(tip[0] - target[0], tip[1] - target[1])

        if dist <= config.REACH_TOL:
            self.reached.append(target)
            self.index += 1
            self.last_status = "reached"
            return self.last_status

        sol = kinematics.inverse_kinematics(target)
        if sol is None:
            # 目标不在工作空间，跳过它
            self.index += 1
            self.last_status = "unreachable"
            return self.last_status

        q1_des, q2_des = sol
        q1_des, q2_des, clipped = kinematics.clamp_joints(q1_des, q2_des)
        d1 = config.STEP_RATIO
        self.q1 = kinematics.wrap(self.q1 + (q1_des - self.q1) * d1)
        self.q2 = kinematics.wrap(self.q2 + (q2_des - self.q2) * d1)
        self.elbow, self.tip = kinematics.forward_kinematics(self.q1, self.q2)
        self.trail.append(self.tip)
        self.last_status = "clipped" if clipped else "moving"
        return self.last_status

    def error(self):
        """末端到当前目标的距离；没有目标时返回 0。"""
        target = self.target
        if target is None:
            return 0.0
        _, tip = kinematics.forward_kinematics(self.q1, self.q2)
        return math.hypot(tip[0] - target[0], tip[1] - target[1])
