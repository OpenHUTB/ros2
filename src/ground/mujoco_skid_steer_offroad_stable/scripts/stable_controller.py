import numpy as np

class AttitudeStableController:
    def __init__(self):
        # 姿态稳定PID参数（俯仰、侧倾）
        self.kp_pitch = 22.0
        self.kd_pitch = 3.2
        self.kp_roll = 20.0
        self.kd_roll = 3.0
        self.last_pitch = 0.0
        self.last_roll = 0.0

    def compute_compensate(self, pitch, roll, pitch_dot, roll_dot):
        # 地形扰动补偿力矩
        comp_pitch = self.kp_pitch * pitch + self.kd_pitch * pitch_dot
        comp_roll = self.kp_roll * roll + self.kd_roll * roll_dot
        return comp_pitch, comp_roll

