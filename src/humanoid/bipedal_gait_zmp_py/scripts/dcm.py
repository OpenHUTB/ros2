"""发散运动分量（DCM / capture point）反馈控制。"""

from __future__ import annotations

from .lipm import OMEGA


def dcm(x: float, v: float, omega: float = OMEGA) -> float:
    """xi = x + x_dot / omega。"""
    return x + v / omega


def dcm_reference(p_ref: float, v_avg: float, omega: float = OMEGA) -> float:
    """xi_ref = p_ref + v_avg / omega。"""
    return p_ref + v_avg / omega


def zmp_command(
    xi: float,
    xi_ref: float,
    p_ref: float,
    gain: float,
    omega: float = OMEGA,
) -> float:
    """p_cmd = p_ref + K (xi - xi_ref)。"""
    del omega  # 保留参数以明确该控制律与 LIPM 频率的关系
    return p_ref + gain * (xi - xi_ref)


def clip_zmp(p_cmd: float, left: float, right: float) -> float:
    """把 ZMP 限幅到支撑区间。"""
    if left > right:
        raise ValueError("支撑区间左端不能大于右端")
    return min(max(p_cmd, left), right)


def stability_margin(p: float, left: float, right: float) -> float:
    """ZMP 到支撑区间边界的最小距离；区间内为正，边界上为 0。"""
    return min(p - left, right - p)
