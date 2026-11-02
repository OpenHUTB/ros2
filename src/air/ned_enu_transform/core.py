#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ned_enu_transform/core.py - NED <-> ENU 坐标转换核心算法（纯 Python，无 ROS 依赖）

背景：OpenHUTB/AirSim 模拟器使用 NED（North-East-Down）机载坐标系，
     ROS 2 (REP-103) 使用 ENU（East-North-Up）坐标系。二者差异：

     位置/速度:  x_NED = x_ENU,  y_NED = -y_ENU,  z_NED = -z_ENU
     偏航角  :  yaw_ENU = -yaw_NED （ENU 逆时针为正, NED 顺时针为正）
     四元数  :  q_ENU = (w, x, -y, -z) （绕 z 轴翻转）

将核心算法与 ROS 解耦，便于在无 ROS 环境下单元测试。
"""

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """将角度归一化到 (-pi, pi]。"""
    angle = math.fmod(angle, TWO_PI)
    if angle <= -math.pi:
        angle += TWO_PI
    elif angle > math.pi:
        angle -= TWO_PI
    return angle


def ned_to_enu_position(x: float, y: float, z: float):
    """位置/速度: NED -> ENU（y、z 取反）。"""
    return (x, -y, -z)


def enu_to_ned_position(x: float, y: float, z: float):
    """位置/速度: ENU -> NED（转换与逆变换相同，是自反变换）。"""
    return (x, -y, -z)


def ned_to_enu_yaw(yaw_ned: float) -> float:
    """偏航角: NED(顺时针为正, rad) -> ENU(逆时针为正, rad)。"""
    return wrap_angle(-yaw_ned)


def enu_to_ned_yaw(yaw_enu: float) -> float:
    """偏航角: ENU -> NED。"""
    return wrap_angle(-yaw_enu)


def ned_to_enu_quaternion(w: float, x: float, y: float, z: float):
    """姿态四元数 (w,x,y,z): NED -> ENU（y、z 分量取反）。"""
    return (w, x, -y, -z)


def enu_to_ned_quaternion(w: float, x: float, y: float, z: float):
    """姿态四元数 (w,x,y,z): ENU -> NED。"""
    return (w, x, -y, -z)


def yaw_to_quaternion_enu(yaw_enu: float):
    """ENU 偏航角 -> ENU 四元数 (w, x, y, z)，绕 z 轴旋转。"""
    return (math.cos(yaw_enu / 2.0), 0.0, 0.0, math.sin(yaw_enu / 2.0))


def quaternion_to_yaw_enu(w: float, x: float, y: float, z: float) -> float:
    """ENU 四元数 -> ENU 偏航角 (rad)。"""
    return wrap_angle(2.0 * math.atan2(z, w))


def ned_pose_to_enu_pose(position_ned, yaw_ned: float):
    """完整位姿转换: (NED 位置, NED 偏航) -> (ENU 位置, ENU 偏航)。"""
    pos_enu = ned_to_enu_position(*position_ned)
    return pos_enu, ned_to_enu_yaw(yaw_ned)
