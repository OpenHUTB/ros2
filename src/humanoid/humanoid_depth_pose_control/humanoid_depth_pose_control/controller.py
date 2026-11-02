"""controller.py —— 基于预测姿态的比例平衡控制"""
from __future__ import annotations
import numpy as np
from .sim import DEFAULT_STAND_TARGET


class ProportionalController:
    """根据预测的躯干pitch/roll/yaw角，输出关节修正目标。"""
    def __init__(self, kp_ankle=2.0, kp_hip=1.5, kp_neck=0.8):
        self.kp_ankle = kp_ankle
        self.kp_hip = kp_hip
        self.kp_neck = kp_neck

    def target_for(self, pitch, roll, yaw=0.0):
        """pitch>0=前倾, roll>0=右倾, yaw>0=左转。返回15维关节目标角。"""
        target = DEFAULT_STAND_TARGET.copy()
        # pitch 修正：前倾→踝背屈+髋后倾
        target[3] += self.kp_ankle * pitch   # ankle_pitch_l
        target[8] += self.kp_ankle * pitch   # ankle_pitch_r
        target[1] -= self.kp_hip * pitch     # hip_pitch_l
        target[6] -= self.kp_hip * pitch     # hip_pitch_r
        # roll 修正：右倾→踝外翻+髋侧倾
        target[4] += self.kp_ankle * roll    # ankle_roll_l
        target[9] -= self.kp_ankle * roll    # ankle_roll_r
        target[0] -= self.kp_hip * roll      # hip_roll_l
        target[5] += self.kp_hip * roll     # hip_roll_r
        # yaw 修正：躯干偏转→颈部反向跟随，保持头部朝向正前方
        # 注：yaw不直接影响重心投影，故不参与平衡修正，仅通过颈部跟随
        target[14] -= self.kp_neck * yaw    # neck
        return target
