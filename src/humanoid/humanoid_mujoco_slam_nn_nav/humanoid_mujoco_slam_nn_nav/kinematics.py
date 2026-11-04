"""双足逆运动学（解析解）与正向运动学（用于自检）。

关节链（每侧）::

    pelvis ── hip_roll(x) ── hip_pitch(y) ── knee_pitch(y) ── ankle_pitch(y) ── ankle_roll(x) ── foot

约定：
* 世界系 x 前、y 左、z 上；yaw 绕 z 轴，逆时针为正；
* ``hip_pitch`` / ``knee_pitch`` / ``ankle_pitch`` 绕 +y 轴，正角度使肢体向后摆；
* ``hip_roll`` / ``ankle_roll`` 绕 +x 轴，正角度使肢体向 +y（左）倾斜。

解析解的依据：髋 roll 只把"腿平面"绕 x 轴旋转，旋转之后就是一个平面二连杆问题。
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

from .model import (ANKLE_TO_SOLE, HIP_OFFSET_Y, HIP_OFFSET_Z, SHIN_LEN, THIGH_LEN)

LEGS = ("l", "r")
SOLE_HALF_H = 0.030


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# ---------------------------------------------------------------------------
# 正 / 逆运动学
# ---------------------------------------------------------------------------
def hip_position(side: str, pelvis_pos, yaw: float) -> np.ndarray:
    """髋关节在世界系中的位置。"""
    off = np.array([0.0, HIP_OFFSET_Y if side == "l" else -HIP_OFFSET_Y, HIP_OFFSET_Z])
    return np.asarray(pelvis_pos, dtype=float) + rot_z(yaw) @ off


def leg_fk(side: str, q: Dict[str, float], pelvis_pos, yaw: float) -> Tuple[np.ndarray, np.ndarray]:
    """正向运动学：6 个关节角 -> (踝关节世界坐标, 脚掌姿态矩阵)。"""
    roll, pitch, knee = q["hip_roll"], q["hip_pitch"], q["knee_pitch"]
    r_base = rot_z(yaw + q.get("hip_yaw", 0.0)) @ rot_x(roll)
    d1 = r_base @ rot_y(pitch) @ np.array([0.0, 0.0, -1.0])
    d2 = r_base @ rot_y(pitch + knee) @ np.array([0.0, 0.0, -1.0])
    ankle = hip_position(side, pelvis_pos, yaw) + THIGH_LEN * d1 + SHIN_LEN * d2
    r_foot = (r_base @ rot_y(pitch + knee)
              @ rot_y(q["ankle_pitch"]) @ rot_x(q["ankle_roll"]))
    return ankle, r_foot


def leg_ik(side: str, pelvis_pos, yaw: float, ankle_target, foot_yaw: float | None = None,
           knee_hint: float = 0.35) -> Dict[str, float]:
    """解析逆运动学。

    * ``ankle_target``：踝关节世界坐标（脚底 = 踝 - (0,0,ANKLE_TO_SOLE+SOLE_HALF_H)）；
    * ``foot_yaw``：期望的脚掌朝向（默认与骨盆同向），由髋 yaw 关节承担；
    * 超出腿长时按比例裁剪，保证返回角度始终可用。
    """
    target = np.asarray(ankle_target, dtype=float)
    leg_yaw = 0.0 if foot_yaw is None else float(np.clip(foot_yaw - yaw, -0.75, 0.75))
    d = rot_z(-(yaw + leg_yaw)) @ (target - hip_position(side, pelvis_pos, yaw))
    dx, dy, dz = float(d[0]), float(d[1]), float(d[2])

    lateral = math.hypot(dy, dz)
    dist = math.hypot(dx, lateral)
    d_max = (THIGH_LEN + SHIN_LEN) * 0.995
    d_min = abs(THIGH_LEN - SHIN_LEN) + 1e-3
    dist_c = min(max(dist, d_min), d_max)
    if dist > 1e-9:
        dx *= dist_c / dist
        lateral *= dist_c / dist

    cos_k = (dist_c ** 2 - THIGH_LEN ** 2 - SHIN_LEN ** 2) / (2 * THIGH_LEN * SHIN_LEN)
    knee = math.acos(max(-1.0, min(1.0, cos_k)))
    cos_b = (THIGH_LEN ** 2 + dist_c ** 2 - SHIN_LEN ** 2) / (2 * THIGH_LEN * dist_c)
    beta = math.acos(max(-1.0, min(1.0, cos_b)))
    phi = math.atan2(dx, lateral)

    pitch = -(phi + beta)
    roll = math.atan2(dy, -dz) if lateral > 1e-9 else 0.0
    roll = max(-0.85, min(0.85, roll))
    knee = max(knee, 0.02)

    # 脚掌水平：hip_yaw + roll 补偿之后，pitch + knee + ankle_pitch = 0
    ankle_pitch = -(pitch + knee)
    del knee_hint
    return {
        "hip_yaw": leg_yaw,
        "hip_roll": roll,
        "hip_pitch": pitch,
        "knee_pitch": knee,
        "ankle_pitch": ankle_pitch,
        "ankle_roll": -roll,
    }


# ---------------------------------------------------------------------------
# 站立姿态
# ---------------------------------------------------------------------------
def stand_ankle_z() -> float:
    """脚底刚好贴地时的踝关节高度。"""
    return ANKLE_TO_SOLE + SOLE_HALF_H


def pelvis_height_for_crouch(knee: float) -> float:
    """对称微蹲（hip=-knee/2）时骨盆中心的高度。"""
    a1 = knee / 2.0
    return stand_ankle_z() + (THIGH_LEN + SHIN_LEN) * math.cos(a1) - HIP_OFFSET_Z


def default_stand(pelvis_xy=(0.0, 0.0), yaw: float = 0.0, knee: float = 0.30):
    """返回 (骨盆位置, 关节角字典)：双脚平行、与髋同宽、脚底贴地。"""
    pelvis = np.array([pelvis_xy[0], pelvis_xy[1], pelvis_height_for_crouch(knee)])
    q: Dict[str, float] = {
        "waist_yaw": 0.0,
        "shoulder_pitch_l": 0.12, "shoulder_pitch_r": -0.12,
        "elbow_pitch_l": -0.25, "elbow_pitch_r": -0.25,
    }
    for s in LEGS:
        off = HIP_OFFSET_Y if s == "l" else -HIP_OFFSET_Y
        ankle = pelvis + rot_z(yaw) @ np.array([0.0, off, 0.0])
        ankle[2] = stand_ankle_z()
        q.update({f"{k}_{s}": v for k, v in leg_ik(s, pelvis, yaw, ankle).items()})
    return pelvis, q
