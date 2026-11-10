# -*- coding: utf-8 -*-
"""protection.py —— 跌倒判据与三种响应策略

判据不是自己拍的：倾角 30°、角速度 2.0 rad/s 这组部署阈值来自
OpenHorizon Labs 的 Asimov 1 safe-fall 策略配置（Apache-2.0），
见 https://github.com/openhorizon-labs/asimov-v1-policies 。
他们的训练范围是倾角 20°~45°、角速度 2~4 rad/s，本模块的阈值扫描
（main.py --sweep）覆盖的正是这个区间，用来说明「阈值不是越小越好」。

三种策略共用同一个检测器，唯一区别是触发之后做什么：
    rigid   —— 冻结触发瞬间的关节角，僵硬地倒下（没有任何保护的基线）
    limp    —— 力矩全部置零，像断电一样瘫软
    protect —— 切换到保护姿态：前伸撑地、屈肘缓冲、抬头、屈膝

这样对照才干净：三组在触发前的控制完全一样，差异只来自触发后的响应。
"""

import numpy as np

from . import config


class FallDetector:
    """倾角 / 角速度双判据。

    mode="or"  任一超限即触发（Asimov 的部署默认行为，响应最快）
    mode="and" 两者同时超限才触发（更保守，小扰动下的误触发更少）

    触发是一次性的：一旦置位就保持，不会因为后面姿态回摆而取消，
    这与真实跌倒保护的状态机一致（保护动作中途取消反而更危险）。
    """

    def __init__(self, tilt_threshold_deg=30.0, tilt_rate_threshold=2.0, mode="or"):
        self.tilt_threshold = float(np.radians(tilt_threshold_deg))
        self.rate_threshold = float(tilt_rate_threshold)
        self.mode = mode
        self.triggered = False
        self.trigger_time = None
        self.trigger_tilt = None
        self.trigger_rate = None

    def reset(self):
        self.triggered = False
        self.trigger_time = None
        self.trigger_tilt = None
        self.trigger_rate = None

    def update(self, tilt, tilt_rate, t=None):
        """喂当前倾角与角速度，返回是否「刚刚」触发。"""
        if self.triggered:
            return False
        over_tilt = abs(tilt) >= self.tilt_threshold
        over_rate = abs(tilt_rate) >= self.rate_threshold
        fired = (over_tilt or over_rate) if self.mode == "or" else (over_tilt and over_rate)
        if fired:
            self.triggered = True
            self.trigger_time = t
            self.trigger_tilt = float(tilt)
            self.trigger_rate = float(tilt_rate)
        return fired

    def direction(self):
        """触发方向：'forward'（前扑）或 'backward'（后仰）。"""
        if self.trigger_tilt is None:
            return "forward"
        return "forward" if self.trigger_tilt >= 0 else "backward"


class FallController:
    """站立 PD + 跌倒响应状态机。

    站立阶段：关节 PD 维持 config.STAND_POSE，踝关节额外叠加
    「倾角 + 倾角速率 + 质心水平速度」三项反馈。没有这三项，纯 PD
    在 1.8 s 内就会自己前倾倒地（docs 里有这组对照数据）。

    触发之后：三项踝反馈立即关闭 —— 已经决定要倒了，再让踝关节按倾角
    输出 300 N·m 的力矩只会把腿甩起来，反而加大头部冲击。
    """

    def __init__(self, params=None):
        p = dict(config.DEFAULTS)
        if params:
            p.update(params)
        self.p = p
        self.kp = float(p["kp"])
        self.kv = float(p["kv"])
        self.kp_protect = float(p["protect_kp"])
        self.kv_protect = float(p["protect_kv"])
        self.limit = float(p["torque_limit"])
        self.ankle_lean = float(p["ankle_lean_gain"])
        self.ankle_rate = float(p["ankle_lean_rate_gain"])
        self.ankle_com_vel = float(p["ankle_com_vel_gain"])

        self.detector = FallDetector(
            tilt_threshold_deg=p["tilt_threshold_deg"],
            tilt_rate_threshold=p["tilt_rate_threshold"],
            mode=p["detector_mode"],
        )
        self.strategy = "rigid"
        self.protect_target = None
        self.frozen_target = None

    def reset(self, strategy="rigid"):
        self.strategy = strategy
        self.detector.reset()
        self.protect_target = None
        self.frozen_target = None

    # ------------------------------------------------------------------
    def _choose_protect_pose(self, direction):
        if self.p.get("protect_direction") == "forward":
            return dict(config.PROTECT_FORWARD)
        if self.p.get("protect_direction") == "backward":
            return dict(config.PROTECT_BACKWARD)
        return dict(config.PROTECT_FORWARD if direction == "forward" else config.PROTECT_BACKWARD)

    def step(self, sim, t):
        """计算并下发一帧力矩，返回本帧的状态字典。"""
        tilt = sim.tilt()
        rate = sim.tilt_rate()
        just_triggered = self.detector.update(tilt, rate, t)

        if just_triggered:
            if self.strategy == "rigid":
                # 冻结触发瞬间的姿态，之后僵硬地倒下去
                self.frozen_target = {n: sim.joint_pos(n) for n in config.JOINT_NAMES}
            elif self.strategy == "protect":
                self.protect_target = self._choose_protect_pose(self.detector.direction())

        triggered = self.detector.triggered
        pose = None
        kp, kv = self.kp, self.kv
        ankle_feedback = True

        if not triggered:
            pose = config.STAND_POSE
        elif self.strategy == "rigid":
            pose = self.frozen_target or config.STAND_POSE
            ankle_feedback = False
        elif self.strategy == "protect":
            pose = self.protect_target or config.PROTECT_FORWARD
            kp, kv = self.kp_protect, self.kv_protect
            ankle_feedback = False
        # limp：走下面的 zero_torque 分支

        if self.strategy == "limp" and triggered:
            sim.zero_torque()
        else:
            ff = 0.0
            if ankle_feedback:
                com_vel = sim.com_vel()
                ff = (self.ankle_lean * tilt
                      + self.ankle_rate * rate
                      + self.ankle_com_vel * float(com_vel[0]))
            for name in config.JOINT_NAMES:
                torque = kp * (pose.get(name, 0.0) - sim.joint_pos(name)) - kv * sim.joint_vel(name)
                if ankle_feedback and name in ("ankle_pitch_l", "ankle_pitch_r"):
                    torque += ff
                sim.set_torque(name, torque, self.limit)

        return {
            "t": t,
            "tilt": tilt,
            "tilt_rate": rate,
            "triggered": triggered,
            "just_triggered": just_triggered,
            "mode": self._mode_name(triggered),
        }

    def _mode_name(self, triggered):
        if not triggered:
            return "stand"
        if self.strategy == "limp":
            return "limp"
        if self.strategy == "rigid":
            return "rigid"
        return "protect"
