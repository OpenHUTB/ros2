# -*- coding: utf-8 -*-
"""串联机械臂的正运动学与几何雅可比。

关节约定
--------
每个关节用一个「平移 + 绕轴转动」描述：

    T_i = Translate(origin_i) · Rot(axis_i, q_i)

整条链：

    T_ee = T_0 · T_1 · … · T_{n−1} · Translate(tool)

``origin_i`` 是在**前一个关节坐标系**下、从上一个关节指向本关节的位移，
``tool`` 是最后一个关节到末端执行器的固定偏移。这种写法比 DH 参数直观，
也好手工验算（2R 平面臂的解析解可以逐项对照）。

雅可比
------
用几何法而不是解析求导，因为末端位置对关节角的偏导在数值上容易出问题。

对转动关节 i：

    z_i = R_{0..i−1} · axis_i          （关节轴在世界系下的方向）
    p_i = 位置（世界系）
    Jv_i = z_i × (p_ee − p_i)
    Jw_i = z_i

这样拼出 6×n 的雅可比，前 3 行是线速度，后 3 行是角速度。
"""

from __future__ import annotations

import math

from linalg import (
    compose,
    cross,
    identity,
    mat3_mul,
    mat3_vec,
    norm,
    normalize,
    rodrigues,
    zeros,
)

__all__ = [
    "Joint",
    "Chain",
    "planar_2r_fk",
    "planar_2r_ik",
    "make_planar_2r",
    "make_humanoid_arm",
]


class Joint(object):
    """一个转动关节。

    参数
    ----
    name    关节名
    axis    转动轴，在本关节坐标系下（会被归一化）
    origin  相对上一个关节的位移（平移），单位米
    limits  (下限, 上限)，弧度；None 表示不限位
    """

    def __init__(self, name, axis=(0.0, 0.0, 1.0), origin=(0.0, 0.0, 0.0),
                 limits=None):
        if norm(axis) < 1e-12:
            raise ValueError("关节 %s 的转轴不能是零向量" % name)
        self.name = str(name)
        self.axis = normalize(axis)
        self.origin = [float(x) for x in origin]
        if limits is not None:
            lo, hi = limits
            if lo > hi:
                raise ValueError("关节 %s 的限位下限大于上限" % name)
            self.limits = (float(lo), float(hi))
        else:
            self.limits = None

    def clamp(self, q):
        """把关节角夹到限位内。"""
        if self.limits is None:
            return q
        lo, hi = self.limits
        return lo if q < lo else (hi if q > hi else q)

    def within_limits(self, q, tol=1e-9):
        if self.limits is None:
            return True
        lo, hi = self.limits
        return (lo - tol) <= q <= (hi + tol)

    def __repr__(self):
        return "Joint(%s, axis=%s)" % (self.name, self.axis)


class Chain(object):
    """串联关节链。"""

    def __init__(self, joints, tool=(0.0, 0.0, 0.0), name="arm"):
        if not joints:
            raise ValueError("关节链不能为空")
        self.joints = list(joints)
        self.tool = [float(x) for x in tool]
        self.name = name

    # ------------------------------------------------------------------
    @property
    def dof(self):
        return len(self.joints)

    def reach(self):
        """完全伸直时末端到基座的最大距离（工作空间半径的上界）。"""
        total = norm(self.tool)
        for j in self.joints:
            total += norm(j.origin)
        return total

    # ------------------------------------------------------------------
    def frames(self, q):
        """返回每个关节的 (z_i, p_i) 以及末端位置。

        z_i 是关节轴在世界系下的方向，p_i 是关节位置（世界系）。
        实现上先平移再记录、最后转动，保证 z_i / p_i 对应"第 i 个关节的轴"。
        """
        if len(q) != self.dof:
            raise ValueError("关节角数量 %d 与自由度 %d 不符"
                             % (len(q), self.dof))

        r = identity(3)
        p = [0.0, 0.0, 0.0]
        zs = []
        ps = []

        for joint, qi in zip(self.joints, q):
            # 先平移到关节位置
            off = mat3_vec(r, joint.origin)
            p = [p[0] + off[0], p[1] + off[1], p[2] + off[2]]
            # 记录该关节的轴与位置
            zs.append(mat3_vec(r, joint.axis))
            ps.append(list(p))
            # 再绕轴转动
            r = mat3_mul(r, rodrigues(joint.axis, qi))

        tool = mat3_vec(r, self.tool)
        p_ee = [p[0] + tool[0], p[1] + tool[1], p[2] + tool[2]]
        return zs, ps, p_ee, r

    def fk(self, q):
        """正运动学，返回 (位置, 3x3 旋转矩阵)。"""
        _zs, _ps, p_ee, r = self.frames(q)
        return p_ee, r

    def fk_matrix(self, q):
        """正运动学，返回 4x4 齐次变换。"""
        p, r = self.fk(q)
        return compose(r, p)

    # ------------------------------------------------------------------
    def jacobian(self, q):
        """6×n 几何雅可比。前 3 行线速度，后 3 行角速度。"""
        zs, ps, p_ee, _r = self.frames(q)
        n = self.dof
        j = zeros(6, n)
        for i in range(n):
            z = zs[i]
            d = [p_ee[0] - ps[i][0],
                 p_ee[1] - ps[i][1],
                 p_ee[2] - ps[i][2]]
            v = cross(z, d)
            j[0][i] = v[0]
            j[1][i] = v[1]
            j[2][i] = v[2]
            j[3][i] = z[0]
            j[4][i] = z[1]
            j[5][i] = z[2]
        return j

    def position_jacobian(self, q):
        """只取线速度部分（3×n）。位置逆解用这个就够了。"""
        return self.jacobian(q)[:3]

    # ------------------------------------------------------------------
    def clamp_all(self, q):
        return [j.clamp(x) for j, x in zip(self.joints, q)]

    def all_within_limits(self, q, tol=1e-9):
        return all(j.within_limits(x, tol) for j, x in zip(self.joints, q))

    def __repr__(self):
        return "Chain(%s, dof=%d, reach=%.3f m)" % (
            self.name, self.dof, self.reach())


# ---------------------------------------------------------------------------
# 2R 平面臂：有解析解，用来验证数值逆解
# ---------------------------------------------------------------------------
def planar_2r_fk(q, l1, l2):
    """2R 平面臂的解析正解。"""
    q0, q1 = q
    x = l1 * math.cos(q0) + l2 * math.cos(q0 + q1)
    y = l1 * math.sin(q0) + l2 * math.sin(q0 + q1)
    return [x, y, 0.0]


def planar_2r_ik(x, y, l1, l2, elbow_up=False):
    """2R 平面臂的解析逆解。

    返回关节角列表；目标超出工作空间时抛 ValueError。

        cos q1 = (x² + y² − l1² − l2²) / (2 l1 l2)
    """
    r2 = x * x + y * y
    c1 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if c1 < -1.0 - 1e-9 or c1 > 1.0 + 1e-9:
        raise ValueError("目标 (%.4f, %.4f) 超出 2R 臂工作空间" % (x, y))
    c1 = max(-1.0, min(1.0, c1))
    q1 = math.acos(c1)
    if elbow_up:
        q1 = -q1
    q0 = math.atan2(y, x) - math.atan2(l2 * math.sin(q1),
                                      l1 + l2 * math.cos(q1))
    return [q0, q1]


def make_planar_2r(l1=1.0, l2=1.0, name="planar_2r"):
    """构造一个 2R 平面臂。

    关节 0 在原点、关节 1 在 (l1, 0, 0)，末端再延伸 l2。
    """
    if l1 <= 0 or l2 <= 0:
        raise ValueError("连杆长度必须为正")
    return Chain(
        [Joint("q0", axis=(0, 0, 1), origin=(0, 0, 0)),
         Joint("q1", axis=(0, 0, 1), origin=(l1, 0, 0))],
        tool=(l2, 0, 0),
        name=name,
    )


def make_humanoid_arm(name="humanoid_arm"):
    """一个 7 自由度的拟人手臂（肩 3 + 肘 1 + 腕 3）。

    尺寸按成年人上肢近似：上臂 0.28 m、前臂 0.25 m、手 0.08 m，
    完全伸直时够到约 0.61 m——比 2R 臂更接近真实人形机器人，而且
    **7 > 6 是冗余的**，可以研究零空间自运动。
    """
    return Chain(
        [
            Joint("shoulder_yaw", axis=(0, 0, 1), origin=(0, 0, 0),
                  limits=(-math.pi, math.pi)),
            Joint("shoulder_pitch", axis=(0, 1, 0), origin=(0, 0, 0),
                  limits=(-math.pi / 2, math.pi / 2)),
            Joint("shoulder_roll", axis=(1, 0, 0), origin=(0, 0, 0),
                  limits=(-math.pi / 2, math.pi / 2)),
            Joint("elbow", axis=(0, 1, 0), origin=(0, 0, -0.28),
                  limits=(0.0, 2.6)),
            Joint("wrist_yaw", axis=(0, 0, 1), origin=(0, 0, -0.25),
                  limits=(-math.pi / 2, math.pi / 2)),
            Joint("wrist_pitch", axis=(0, 1, 0), origin=(0, 0, -0.06),
                  limits=(-math.pi / 2, math.pi / 2)),
            Joint("wrist_roll", axis=(0, 0, 1), origin=(0, 0, -0.06),
                  limits=(-math.pi, math.pi)),
        ],
        tool=(0, 0, -0.08),
        name=name,
    )
