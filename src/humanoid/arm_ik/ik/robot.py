"""
ik.robot — 7 自由度人形机械臂 DH 模型与正运动学
=================================================

关节布局（从基座到末端）：
    肩(3自由度) → 肘(1自由度) → 腕(3自由度)
    共 7 自由度，冗余度 = 7 - 6 = 1

DH 参数约定（Craig 标准）：
    a_i    : 连杆长度
    alpha_i: 连杆扭转角
    d_i    : 连杆偏置
    theta_i: 关节角（变量）
"""
import numpy as np
from dataclasses import dataclass


def dh_transform(theta, d, a, alpha):
    """单步 DH 齐次变换矩阵 (4x4)。"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,      -st,     0,      a    ],
        [st*ca,   ct*ca,  -sa,   -sa*d ],
        [st*sa,   ct*sa,   ca,    ca*d ],
        [0,       0,       0,      1    ],
    ])


@dataclass
class Arm7DOF:
    """7 自由度人形手臂（肩3 + 肘1 + 腕3）。

    尺寸单位：米
    """
    L_shoulder: float = 0.20   # 肩到肘距离
    L_elbow:    float = 0.20   # 肘到腕距离
    L_wrist:    float = 0.10   # 腕到末端工具距离

    # 关节限位 (rad)
    q_min: np.ndarray = None
    q_max: np.ndarray = None

    def __post_init__(self):
        if self.q_min is None:
            self.q_min = np.array([-2.6, -1.8, -2.8, -1.6, -2.8, -1.6, -2.8])
        if self.q_max is None:
            self.q_max = np.array([ 2.6,  1.8,  2.8,  1.6,  2.8,  1.6,  2.8])

    def dh_params(self, q):
        """返回 7 个关节的 DH 参数表 (7,4): [theta, d, a, alpha]。"""
        Ls, Le, Lw = self.L_shoulder, self.L_elbow, self.L_wrist
        # 标准 anthropomorphic 手臂 DH 参数
        rows = [
            # theta      d       a       alpha
            [q[0],       0.0,    0.0,    np.pi/2],   # 肩 1
            [q[1],       0.0,    0.0,   -np.pi/2],   # 肩 2
            [q[2],       Ls,     0.0,    np.pi/2],   # 肩 3
            [q[3],       0.0,    Le,    -np.pi/2],   # 肘
            [q[4],       0.0,    0.0,    np.pi/2],   # 腕 1
            [q[5],       Lw,     0.0,   -np.pi/2],   # 腕 2
            [q[6],       0.0,    0.0,    0.0      ],   # 腕 3
        ]
        return np.array(rows)

    def fk(self, q):
        """正运动学：返回末端 4x4 齐次变换矩阵。"""
        rows = self.dh_params(q)
        T = np.eye(4)
        for row in rows:
            T = T @ dh_transform(row[0], row[1], row[2], row[3])
        return T

    def fk_all_frames(self, q):
        """返回 8 个关节的 4x4 位姿（含基座）。"""
        rows = self.dh_params(q)
        Ts = [np.eye(4)]
        T = np.eye(4)
        for row in rows:
            T = T @ dh_transform(row[0], row[1], row[2], row[3])
            Ts.append(T)
        return Ts

    def ee_position(self, q):
        """末端位置 (3,)。"""
        return self.fk(q)[:3, 3]

    def ee_pose(self, q):
        """末端位姿 (位置 3 + 旋转矩阵 3x3)。"""
        T = self.fk(q)
        return T[:3, 3], T[:3, :3]
