"""双推进器推力分配，含饱和处理与单侧降效补偿。

推进器只能沿船头方向出力，无法产生侧向力——所以"分配"这件事在这里的含义是：
把期望的总推力 ``X`` 和艏摇力矩 ``N`` 拆成左右两个标量推力，并尽量在某个推进器
触及上限时保住总推力，而不是让整条船一起掉速。
"""
from __future__ import annotations

from dataclasses import dataclass

from .geometry import clamp
from .vehicle import VehicleParams


@dataclass(frozen=True)
class AllocationResult:
    """一次分配的结果。"""

    left: float
    right: float
    saturated: bool
    shortfall: float   # 因限幅而损失的总推力（N），正常时为 0


def allocate(
    surge: float,
    yaw: float,
    params: VehicleParams,
    efficiency: tuple[float, float] = (1.0, 1.0),
) -> AllocationResult:
    """把（纵向力, 力矩）拆成左右推力，并考虑推进器效率退化。

    ``efficiency`` 为（左, 右）两侧的实际出力比例，1.0 为健康。退化时按比例放大
    指令，使实际出力尽量接近期望——但放大后仍受物理限幅约束。
    """
    spacing = params.thruster_spacing
    if spacing <= 0.0:
        raise ValueError("推进器间距必须为正")

    left_cmd = surge * 0.5 - yaw / spacing
    right_cmd = surge * 0.5 + yaw / spacing

    eff_l = max(efficiency[0], 0.05)
    eff_r = max(efficiency[1], 0.05)
    left_cmd /= eff_l
    right_cmd /= eff_r

    lo, hi = params.thrust_min, params.thrust_max
    left = clamp(left_cmd, lo, hi)
    right = clamp(right_cmd, lo, hi)

    lost_left = left_cmd - left
    lost_right = right_cmd - right

    # 一侧被削时，把差额尽量补给另一侧，优先保住总推力
    if abs(lost_left) > 1e-9 and abs(lost_right) < 1e-9:
        right = clamp(right + lost_left, lo, hi)
    elif abs(lost_right) > 1e-9 and abs(lost_left) < 1e-9:
        left = clamp(left + lost_right, lo, hi)

    shortfall = (surge - (left * eff_l + right * eff_r))
    saturated = abs(lost_left) > 1e-9 or abs(lost_right) > 1e-9
    return AllocationResult(
        left=left,
        right=right,
        saturated=saturated,
        shortfall=shortfall,
    )


def yaw_authority(surge: float, params: VehicleParams) -> float:
    """在当前总推力下还能产生多大的力矩（用于判断是否"转不动"）。"""
    spacing = params.thruster_spacing
    lo, hi = params.thrust_min, params.thrust_max
    half = clamp(surge * 0.5, lo, hi)
    return min(hi - half, half - lo) * spacing
