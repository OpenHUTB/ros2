"""controller.py —— 基于预测姿态的比例平衡控制"""
from __future__ import annotations
import numpy as np
from .sim import DEFAULT_STAND_TARGET


class ProportionalController:
    """根据预测的躯干pitch/roll/yaw角，输出关节修正目标。

    增益按实测标定：站姿平衡（sim.step 的 ankle strategy）已经在做踝关节修正，
    控制器再叠一层同向踝修正相当于把增益加大。原实现用 kp_ankle=2.0 /
    kp_hip=1.5，实测把无扰动站姿从 1.7° 拖到 40°、比不加控制还差 20 倍；
    现在的 0.2 / 0.1 在 2.4 s 无扰动下倾角 6.5°，40 N 推力存活 950 步
    （1.90 s），比纯平衡层的 913 步略好。
    """
    def __init__(self, kp_ankle=0.2, kp_hip=0.1, kp_neck=0.3):
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
