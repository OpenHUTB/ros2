# -*- coding: utf-8 -*-
"""quadrotor.py —— 四旋翼简化动力学模型。

模型定位：够真实到能复现"跟踪需要持续修正"的闭环特性，又不至于引入
外部仿真器(Unreal/AirSim)依赖，保证任何一台装了 numpy 的机器都能跑。

状态量：位置 p(m)、速度 v(m/s)、加速度 a(m/s^2,一阶滞后内环)、偏航角 yaw(rad)。
控制量：机体前向速度 vx、右向速度 vy、世界系垂直速度 vz、期望偏航角 yaw_des。

动力学：
    a_cmd = clip(Kp * (v_des - v) + a_ff, -A_MAX, A_MAX)
    da/dt = (a_cmd - a) / TAU          # 推力/姿态响应滞后
    dv/dt = a - DRAG * v + wind(t)     # 阻尼 + 阵风扰动
    dp/dt = v
"""
import numpy as np

from . import params as P


class Quadrotor:
    """四旋翼质点+一阶姿态滞后的简化模型。"""

    def __init__(self, p0=(0.0, 0.0, P.H_DES), yaw0=0.0, dt=P.DT):
        self.p = np.array(p0, dtype=float)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.yaw = float(yaw0)
        self.dt = dt
        self.v_des = np.zeros(3)      # 世界系期望速度
        self.yaw_des = float(yaw0)
        self.kp_v = 3.2

    # ---------- 接口 ----------
    def set_cmd(self, v_body, vz, yaw_des=None):
        """设置速度指令。v_body=(前向,右向)，vz 为世界系垂直速度。"""
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        vx_b, vy_b = float(v_body[0]), float(v_body[1])
        v_world = np.array([
            cy * vx_b - sy * vy_b,
            sy * vx_b + cy * vy_b,
            float(vz),
        ])
        self.v_des = np.array([
            np.clip(v_world[0], -P.V_MAX, P.V_MAX),
            np.clip(v_world[1], -P.V_MAX, P.V_MAX),
            np.clip(v_world[2], -P.VZ_MAX, P.VZ_MAX),
        ])
        if yaw_des is not None:
            self.yaw_des = float(yaw_des)

    def hover(self):
        self.v_des = np.zeros(3)

    def camera_pose(self):
        """相机光心的世界坐标(机体坐标偏移随偏航旋转)。"""
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        ox, oy, oz = P.CAM_OFFSET
        return np.array([
            self.p[0] + cy * ox - sy * oy,
            self.p[1] + sy * ox + cy * oy,
            self.p[2] + oz,
        ])

    # ---------- 积分 ----------
    def step(self, wind=None):
        wind = np.zeros(3) if wind is None else np.asarray(wind, dtype=float)
        # 内环：速度误差 -> 加速度，限幅
        a_cmd = self.kp_v * (self.v_des - self.v)
        a_cmd[0:2] = np.clip(a_cmd[0:2], -P.A_MAX, P.A_MAX)
        a_cmd[2] = np.clip(a_cmd[2], -P.A_MAX, P.A_MAX)
        # 一阶滞后
        self.a = self.a + (a_cmd - self.a) * (self.dt / P.TAU)
        # 阻尼 + 阵风
        a_total = self.a - P.DRAG * self.v + wind
        self.v = self.v + a_total * self.dt
        self.p = self.p + self.v * self.dt
        # 高度/速度硬限幅，避免数值发散
        if self.p[2] < 0.4:
            self.p[2] = 0.4
            self.v[2] = max(self.v[2], 0.0)
        self.v[0:2] = np.clip(self.v[0:2], -P.V_MAX, P.V_MAX)
        # 偏航
        dyaw = np.clip(self.yaw_des - self.yaw, -P.YAW_RATE_MAX * self.dt,
                       P.YAW_RATE_MAX * self.dt)
        self.yaw += dyaw
        return self.p.copy()
