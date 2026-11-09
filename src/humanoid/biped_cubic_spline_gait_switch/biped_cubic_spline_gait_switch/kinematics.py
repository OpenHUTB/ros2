#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平面双足机器人运动学：腿部 IK/FK、连杆位置与质心。

约定（矢状面 x-z）：
- x 向前，z 向上；
- 髋角 a1 从竖直向下方向量起，向前为正；
- 膝角 a2 为屈膝角，0 为伸直，向后弯曲为正；
- 踝角 a3 使脚掌保持水平，a3 = -(a1 - a2)。
"""

import math
from typing import Dict, List, Sequence

import numpy as np


class BipedModel:
    def __init__(self, thigh: float, shank: float, foot_length: float,
                 ankle_height: float, torso_length: float,
                 masses: Dict[str, float]):
        self.thigh = float(thigh)
        self.shank = float(shank)
        self.foot_length = float(foot_length)
        self.ankle_height = float(ankle_height)
        self.torso_length = float(torso_length)
        self.m_torso = float(masses["torso"])
        self.m_thigh = float(masses["thigh"])
        self.m_shank = float(masses["shank"])
        self.m_foot = float(masses["foot"])

    def leg_ik(self, hip: Sequence[float], ankle: Sequence[float]) -> np.ndarray:
        """由髋/踝位置求 (髋角, 膝角, 踝角)。"""
        dx = float(ankle[0]) - float(hip[0])
        dz = float(ankle[1]) - float(hip[1])
        d = math.hypot(dx, dz)
        d = max(abs(self.thigh - self.shank) + 1e-4,
                min(self.thigh + self.shank - 1e-4, d))
        phi = math.atan2(dx, -dz)

        cos_gamma = (self.thigh ** 2 + self.shank ** 2 - d ** 2) / (
            2.0 * self.thigh * self.shank)
        gamma = math.acos(max(-1.0, min(1.0, cos_gamma)))
        knee = math.pi - gamma

        cos_beta = (self.thigh ** 2 + d ** 2 - self.shank ** 2) / (
            2.0 * self.thigh * d)
        beta = math.acos(max(-1.0, min(1.0, cos_beta)))
        hip_angle = phi + beta
        shank_angle = hip_angle - knee
        ankle_angle = -shank_angle
        return np.array([hip_angle, knee, ankle_angle])

    def leg_fk(self, hip: Sequence[float], q: Sequence[float]) -> Dict[str, np.ndarray]:
        """由髋位置与三关节角求膝、踝、脚掌端点。"""
        a1, a2, a3 = float(q[0]), float(q[1]), float(q[2])
        hip = np.array([float(hip[0]), float(hip[1])])
        knee_dir = np.array([math.sin(a1), -math.cos(a1)])
        knee = hip + self.thigh * knee_dir
        shank_angle = a1 - a2
        shank_dir = np.array([math.sin(shank_angle), -math.cos(shank_angle)])
        ankle = knee + self.shank * shank_dir
        foot_angle = shank_angle + a3
        # 脚掌方向（水平时 foot_angle = 0）
        foot_dir = np.array([math.cos(foot_angle), math.sin(foot_angle)])
        heel = ankle - 0.5 * self.foot_length * foot_dir
        toe = ankle + 0.5 * self.foot_length * foot_dir
        return {
            "hip": hip, "knee": knee, "ankle": ankle,
            "heel": heel, "toe": toe,
            "shank_angle": np.array([shank_angle]),
        }

    def pose(self, base_x: float, hip_z: float, lean: float,
             q: Sequence[float]) -> Dict[str, object]:
        """给定基座与 6 个关节角，返回左右腿连杆与躯干。"""
        hip = np.array([float(base_x), float(hip_z)])
        left = self.leg_fk(hip, q[0:3])
        right = self.leg_fk(hip, q[3:6])
        torso_dir = np.array([math.sin(lean), math.cos(lean)])
        torso_top = hip + self.torso_length * torso_dir
        return {
            "hip": hip, "left": left, "right": right,
            "torso_top": torso_top, "torso_dir": torso_dir,
        }

    def center_of_mass(self, base_x: float, hip_z: float, lean: float,
                       q: Sequence[float]) -> np.ndarray:
        links = self.pose(base_x, hip_z, lean, q)
        hip = links["hip"]
        torso_c = hip + 0.5 * self.torso_length * links["torso_dir"]
        total_m = self.m_torso
        com = self.m_torso * torso_c
        for side in ("left", "right"):
            leg = links[side]
            thigh_c = 0.5 * (leg["hip"] + leg["knee"])
            shank_c = 0.5 * (leg["knee"] + leg["ankle"])
            foot_c = leg["ankle"]
            total_m += self.m_thigh + self.m_shank + self.m_foot
            com += self.m_thigh * thigh_c + self.m_shank * shank_c + self.m_foot * foot_c
        return com / total_m

    def support_polygon(self, base_x: float, hip_z: float, lean: float,
                        q: Sequence[float], contacts: Sequence[bool]) -> tuple:
        """返回支撑脚的 [min_x, max_x]，无支撑返回 (None, None)。"""
        links = self.pose(base_x, hip_z, lean, q)
        xs: List[float] = []
        for side, contact in zip(("left", "right"), contacts):
            if not contact:
                continue
            leg = links[side]
            heel_x = float(leg["heel"][0])
            toe_x = float(leg["toe"][0])
            xs.extend([heel_x, toe_x])
        if not xs:
            return None, None
        return min(xs), max(xs)
