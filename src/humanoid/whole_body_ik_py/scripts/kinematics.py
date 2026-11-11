"""关节链、正运动学与几何雅可比。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .linalg import rotation_matrix


@dataclass
class Joint:
    """单个转动关节。

    关节变换为 T_i = Translate(origin_i) * Rot(axis_i, q_i)。
    origin_i 是相对上一个关节坐标系的平移，axis_i 是同一坐标系中的转轴。
    """

    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = -np.pi
    upper: float = np.pi

    def __post_init__(self) -> None:
        self.origin = np.asarray(self.origin, dtype=float).reshape(3)
        self.axis = np.asarray(self.axis, dtype=float).reshape(3)
        norm = float(np.linalg.norm(self.axis))
        if norm < 1e-12:
            raise ValueError(f"关节 {self.name} 的转轴不能为零向量")
        self.axis = self.axis / norm
        if not np.isfinite(self.lower) or not np.isfinite(self.upper):
            raise ValueError(f"关节 {self.name} 的限位必须是有限值")
        if self.lower >= self.upper:
            raise ValueError(f"关节 {self.name} 的下限必须小于上限")


@dataclass
class FKResult:
    """一次正运动学计算的全部中间结果。"""

    position: np.ndarray
    rotation: np.ndarray
    joint_positions: np.ndarray
    joint_axes: np.ndarray
    frames: tuple[np.ndarray, ...]


class SerialChain:
    """由转动关节和末端工具平移构成的串联链。"""

    def __init__(self, name: str, joints: Sequence[Joint], tool: np.ndarray):
        self.name = name
        self.joints = list(joints)
        self.tool = np.asarray(tool, dtype=float).reshape(3)
        if len(self.joints) == 0:
            raise ValueError("关节链至少需要一个关节")

    @property
    def dof(self) -> int:
        return len(self.joints)

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.lower for j in self.joints], dtype=float)

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.upper for j in self.joints], dtype=float)

    @property
    def q_mid(self) -> np.ndarray:
        """关节限位中点。7 自由度链的肘关节中点非零，可避开全零奇异。"""
        return 0.5 * (self.lower + self.upper)

    def _check_q(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.dof:
            raise ValueError(f"期望 {self.dof} 个关节角，实际得到 {q.size} 个")
        if not np.all(np.isfinite(q)):
            raise ValueError("关节角包含 NaN 或 inf")
        return q

    def forward(self, q: np.ndarray) -> FKResult:
        """正运动学，返回末端位姿及每个关节的轴/位置。"""
        q = self._check_q(q)
        T = np.eye(4, dtype=float)
        joint_positions: list[np.ndarray] = []
        joint_axes: list[np.ndarray] = []
        frames: list[np.ndarray] = []

        for qi, joint in zip(q, self.joints):
            T_origin = np.eye(4, dtype=float)
            T_origin[:3, 3] = joint.origin
            T = T @ T_origin
            joint_positions.append(T[:3, 3].copy())

            Rj = rotation_matrix(joint.axis, float(qi))
            T_rot = np.eye(4, dtype=float)
            T_rot[:3, :3] = Rj
            T = T @ T_rot
            joint_axes.append(T[:3, :3] @ joint.axis)
            frames.append(T.copy())

        T_tool = np.eye(4, dtype=float)
        T_tool[:3, 3] = self.tool
        T_ee = T @ T_tool
        frames.append(T_ee.copy())
        return FKResult(
            position=T_ee[:3, 3].copy(),
            rotation=T_ee[:3, :3].copy(),
            joint_positions=np.asarray(joint_positions, dtype=float),
            joint_axes=np.asarray(joint_axes, dtype=float),
            frames=tuple(frames),
        )

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """几何雅可比，返回 6 x n：前三行为线速度，后三行为角速度。"""
        fk = self.forward(q)
        J = np.zeros((6, self.dof), dtype=float)
        for i in range(self.dof):
            z_i = fk.joint_axes[i]
            p_i = fk.joint_positions[i]
            J[:3, i] = np.cross(z_i, fk.position - p_i)
            J[3:, i] = z_i
        return J

    def link_points(self, q: np.ndarray) -> np.ndarray:
        """用于绘图：从基座到末端工具点的折线点列。"""
        fk = self.forward(q)
        pts = [np.zeros(3, dtype=float)]
        pts.extend(fk.joint_positions.tolist())
        pts.append(fk.position.copy())
        return np.asarray(pts, dtype=float)


def make_2r_arm(l1: float = 1.0, l2: float = 1.0) -> SerialChain:
    """x-y 平面内的 2R 平面臂。

    q0 为肩关节角，q1 为肘关节角；末端位置满足
    x = l1 cos(q0) + l2 cos(q0 + q1)，y = l1 sin(q0) + l2 sin(q0 + q1)。
    """
    joints = [
        Joint("shoulder", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), -2.8, 2.8),
        Joint("elbow", np.array([l1, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), -2.8, 2.8),
    ]
    return SerialChain("2R planar arm", joints, np.array([l2, 0.0, 0.0]))


def make_7dof_arm(
    upper_arm: float = 0.30,
    forearm: float = 0.26,
    tool_length: float = 0.08,
) -> SerialChain:
    """7 自由度人形手臂：肩 3 + 肘 1 + 腕 3。

    零位时手臂沿 +x 伸展，肘关节限位中点为 1.3 rad，故默认初值不是奇异位形。
    末端工具沿前臂方向再伸出 tool_length。
    """
    shoulder = np.array([0.0, 0.0, 0.30])
    wrist_origin = np.array([forearm, 0.0, 0.0])
    joints = [
        Joint("shoulder_yaw", shoulder.copy(), np.array([0.0, 0.0, 1.0]), -2.8, 2.8),
        Joint("shoulder_pitch", np.zeros(3), np.array([0.0, 1.0, 0.0]), -1.5, 1.5),
        Joint("shoulder_roll", np.zeros(3), np.array([1.0, 0.0, 0.0]), -2.0, 2.0),
        Joint("elbow", np.array([upper_arm, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), 0.0, 2.6),
        Joint("wrist_yaw", wrist_origin.copy(), np.array([0.0, 0.0, 1.0]), -2.0, 2.0),
        Joint("wrist_pitch", np.zeros(3), np.array([0.0, 1.0, 0.0]), -1.5, 1.5),
        Joint("wrist_roll", np.zeros(3), np.array([1.0, 0.0, 0.0]), -2.5, 2.5),
    ]
    return SerialChain("7-DOF humanoid arm", joints, np.array([tool_length, 0.0, 0.0]))


def _planar_ik(
    x: float,
    z: float,
    l1: float,
    l2: float,
) -> tuple[float, float]:
    """平面 2R 反解（q 绕 +y 转，+q 使 x 轴向 -z 转）。"""
    d = float(np.hypot(x, z))
    d = min(max(d, abs(l1 - l2) + 1e-12), l1 + l2 - 1e-12)
    cos_phi = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    phi = float(np.arccos(np.clip(cos_phi, -1.0, 1.0)))
    theta = float(np.arctan2(-z, x))
    alpha = theta - float(np.arctan2(l2 * np.sin(phi), l1 + l2 * np.cos(phi)))
    return alpha, phi


def reference_circle_pose(
    chain: SerialChain,
    center: Sequence[float],
    radius: float,
    angle: float,
    plane: str = "xz",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """为 7 自由度手臂生成圆形全位姿参考点。

    位置严格落在半径 radius 的圆上；姿态由一个可行的平面 2R 解析解给出，
    因此该全位姿目标一定在机构可达范围内。返回值是 (位置, 旋转, 参考关节角)。
    """
    if chain.dof != 7:
        raise ValueError("参考圆轨迹只针对 7 自由度手臂定义")
    if plane not in ("xz", "xy"):
        raise ValueError("plane 必须是 'xz' 或 'xy'")
    center = np.asarray(center, dtype=float).reshape(3)
    radius = float(radius)
    if radius <= 0.0:
        raise ValueError("半径必须为正")

    base = chain.joints[0].origin
    l1 = float(np.linalg.norm(chain.joints[3].origin))
    l2 = float(np.linalg.norm(chain.joints[4].origin) + np.linalg.norm(chain.tool))
    if l1 <= 0.0 or l2 <= 0.0:
        raise ValueError("手臂连杆长度必须为正")

    if plane == "xz":
        target = center + np.array([radius * np.cos(angle), 0.0, radius * np.sin(angle)])
        v = target - base
        alpha, phi = _planar_ik(float(v[0]), float(v[2]), l1, l2)
        q = np.zeros(7, dtype=float)
        q[1] = alpha
        q[3] = phi
        R_target = rotation_matrix(np.array([0.0, 1.0, 0.0]), alpha + phi)
        return target, R_target, q

    target = center + np.array(
        [radius * np.cos(angle), radius * np.sin(angle), 0.0]
    )
    v = target - base
    psi = float(np.arctan2(v[1], v[0]))
    rho = float(np.hypot(v[0], v[1]))
    alpha, phi = _planar_ik(rho, float(v[2]), l1, l2)
    q = np.zeros(7, dtype=float)
    q[0] = psi
    q[1] = alpha
    q[3] = phi
    R_target = rotation_matrix(np.array([0.0, 0.0, 1.0]), psi) @ rotation_matrix(
        np.array([0.0, 1.0, 0.0]), alpha + phi
    )
    return target, R_target, q
