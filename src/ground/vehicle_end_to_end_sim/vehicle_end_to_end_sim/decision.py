"""决策逻辑 —— 三级状态机：安全区 / 预警区 / 刹车区（AEB）。

依据《基于端到端感知与物理动力学的车载仿真系统》的端到端设计，把"感知到的
前方最近障碍物距离"映射为期望速度变化：

| 距离区间                        | 状态   | 速度行为                     |
|---------------------------------|--------|------------------------------|
| dist >= WARN_DIST_PIXEL         | SAFE   | 正常加速至目标车速           |
| SAFE_BRAKE <= dist < WARN_DIST  | WARN   | 速度保持不变（仅橙色预警）   |
| dist < SAFE_BRAKE_DIST_PIXEL    | BRAKE  | AEB 恒减速（每帧 -0.03）平滑刹停 |

设计要点：预警区与刹车区严格分离，实现"人机共驾，机器不抢方向盘"——只有进入
安全红线才触发物理制动。
"""

import math
from enum import Enum

from . import config


class DriveState(Enum):
    """行车状态，驱动 UI 提示。"""
    SAFE = 'safe'    # 安全区：正常巡航
    WARN = 'warn'    # 预警区：橙色警告，请转弯
    BRAKE = 'brake'  # 刹车区：红色警告，AEB 减速


def decide(car, target_speed_pixel, min_dist):
    """依据前方最近距离决策，就地更新 car.v_pixel。

    参数
    ----
    car : Car
        车辆对象，读 / 写 v_pixel。
    target_speed_pixel : float
        目标车速（像素/帧）。
    min_dist : float
        前方最近障碍物距离（像素），由 perception 计算。

    返回
    ----
    (DriveState, bool) : (行车状态, 是否触发紧急制动)。
    """
    if min_dist < config.SAFE_BRAKE_DIST_PIXEL:
        # 进入安全红线：AEB 恒减速度刹车，滑行一段真实物理距离后缓缓停住
        car.v_pixel -= config.BRAKE_DECEL
        if car.v_pixel < 0.0:
            car.v_pixel = 0.0
        return DriveState.BRAKE, True

    if min_dist < config.WARN_DIST_PIXEL:
        # 预警区：只提示，不改变物理速度，等待驾驶员操作
        return DriveState.WARN, False

    # 安全区：正常加速至目标车速
    if car.v_pixel < target_speed_pixel:
        car.v_pixel += config.ACCEL_STEP
    else:
        car.v_pixel = target_speed_pixel
    return DriveState.SAFE, False
