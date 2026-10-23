# -*- coding: utf-8 -*-
"""
reacher_env.py
Reacher(2关节平面机械臂) MuJoCo 环境封装。

- 感知输入(状态): [sin(q1), cos(q1), sin(q2), cos(q2), dx, dy]
    dx,dy = 目标点 - 末端点 在全局坐标系的差(来自关节编码器 + 目标检测)
- 控制输出: 肩/肘两个关节的扭矩 ctrl[2]
- 提供解析逆运动学 + PD 控制器, 用于生成监督学习演示数据
"""
import math

import numpy as np
import mujoco

L1 = 0.15   # 上臂长度
L2 = 0.12   # 前臂长度


def forward_kinematics(q1, q2):
    """正运动学: 由关节角求末端全局坐标。"""
    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y


def inverse_kinematics(tx, ty):
    """逆运动学: 2关节平面臂解析解, 返回 (q1, q2) (肘在下方正解)。"""
    d = math.hypot(tx, ty)
    d = min(max(d, 1e-6), L1 + L2 - 1e-6)
    c2 = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    q2 = math.acos(c2)
    q1 = math.atan2(ty, tx) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


class ReacherEnv:
    def __init__(self, xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        # 索引
        self.q1_idx = self.model.joint("shoulder").qposadr[0]
        self.q2_idx = self.model.joint("elbow").qposadr[0]
        self.v1_idx = self.model.joint("shoulder").dofadr[0]
        self.v2_idx = self.model.joint("elbow").dofadr[0]
        self.fingertip_id = self.model.site("fingertip").id
        self.target_id = self.model.site("target").id
        self.target_mocap_id = self.model.body("targetmocap").mocapid[0]
        # 动作上下限
        self.ctrl_range = self.model.actuator_ctrlrange.copy()
        self.dt = self.model.opt.timestep

    # ---------- 目标与重置 ----------
    def set_target(self, xy):
        """用 mocap 放置目标点。"""
        x, y = xy
        self.data.mocap_pos[self.target_mocap_id] = [x, y, 0.0]
        self.data.mocap_quat[self.target_mocap_id] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)

    def random_target(self, joint_limit=1.8):
        """在可达环内随机目标点(保证逆解在关节限位内, 演示可达)。"""
        while True:
            r = np.random.uniform(0.05, L1 + L2 - 0.05)
            a = np.random.uniform(0, 2 * math.pi)
            tx, ty = r * math.cos(a), r * math.sin(a)
            q1, q2 = inverse_kinematics(tx, ty)
            if abs(q1) <= joint_limit and abs(q2) <= joint_limit:
                return (tx, ty)

    def reset(self, target_xy=None, init_q=(0.0, 0.0)):
        mujoco.mj_resetData(self.model, self.data)
        if target_xy is None:
            target_xy = self.random_target()
        self.set_target(target_xy)
        self.data.qpos[self.q1_idx] = init_q[0]
        self.data.qpos[self.q2_idx] = init_q[1]
        self.data.qvel[self.v1_idx] = 0.0
        self.data.qvel[self.v2_idx] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return target_xy

    # ---------- 感知与几何 ----------
    def q(self):
        return (self.data.qpos[self.q1_idx], self.data.qpos[self.q2_idx])

    def qvel(self):
        return (self.data.qvel[self.v1_idx], self.data.qvel[self.v2_idx])

    def fingertip_xy(self):
        return self.data.site_xpos[self.fingertip_id][:2].copy()

    def target_xy(self):
        return self.data.site_xpos[self.target_id][:2].copy()

    def distance(self):
        """末端到目标的欧氏距离。"""
        d = self.fingertip_xy() - self.target_xy()
        return math.hypot(d[0], d[1])

    def state(self):
        """感知状态: [sin q1, cos q1, sin q2, cos q2, dx, dy]。"""
        q1, q2 = self.q()
        ft = self.fingertip_xy()
        tg = self.target_xy()
        dx = tg[0] - ft[0]
        dy = tg[1] - ft[1]
        return np.array([math.sin(q1), math.cos(q1),
                         math.sin(q2), math.cos(q2),
                         dx, dy], dtype=np.float64)

    # ---------- 控制 ----------
    def step(self, ctrl):
        """施加扭矩并推进一步。"""
        ctrl = np.clip(np.asarray(ctrl, dtype=np.float64).ravel(),
                       self.ctrl_range[:, 0], self.ctrl_range[:, 1])
        self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)

    def pd_control(self, q_des, kp=15.0, kd=4.0):
        """PD 位置控制: 由目标关节角求扭矩。"""
        q1, q2 = self.q()
        v1, v2 = self.qvel()
        u1 = kp * (q_des[0] - q1) - kd * v1
        u2 = kp * (q_des[1] - q2) - kd * v2
        return np.array([u1, u2])

    def ik_pd_ctrl(self, target_xy, kp=15.0, kd=4.0):
        """逆运动学 + PD: 由目标点直接求扭矩(演示控制器)。"""
        q_des = inverse_kinematics(target_xy[0], target_xy[1])
        return self.pd_control(q_des, kp, kd)
