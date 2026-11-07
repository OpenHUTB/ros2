"""线性倒立摆模型（LIPM）的数学工具。

LIPM 假设质心高度恒定，ZMP 与质心的关系为：

    p = c - (z_c / g) * a

其中 p 为 ZMP 水平坐标，c 为质心水平坐标，a 为质心水平加速度，
z_c 为质心高度，g 为重力加速度。
"""

import math

GRAVITY = 9.81


def omega(com_height):
    """返回倒立摆的自然角频率 omega = sqrt(g / z_c)。"""
    if com_height <= 0.0:
        raise ValueError('com_height 必须为正')
    return math.sqrt(GRAVITY / com_height)


def zmp_from_com(com_pos, com_acc, com_height):
    """由质心位置与加速度计算 ZMP，参数与返回值均为 (x, y)。"""
    factor = com_height / GRAVITY
    return (com_pos[0] - factor * com_acc[0],
            com_pos[1] - factor * com_acc[1])


def com_acceleration(com_pos, zmp_pos, com_height):
    """由 ZMP 与质心位置计算质心加速度（LIPM 正向动力学）。

    与 zmp_from_com 互为逆运算：a = (c - p) / (z_c / g)。
    """
    factor = com_height / GRAVITY
    return ((com_pos[0] - zmp_pos[0]) / factor,
            (com_pos[1] - zmp_pos[1]) / factor)
