# -*- coding: utf-8 -*-
"""出生与落稳的公共实现。

评审指出 check_hill.py 修好了出生位姿，但 make_gif.py 没跟着改，
导致动图还是"旧场景"渲染的（穿透 −26.9 mm、起点初速 −0.332 m/s）。
这里把两处逻辑抽成一个函数，两边都只能走这一条路。

三个必须在同一处保证的细节：
  1. 车身按坡度俯仰 −α（否则三点/四点支撑对不上，落稳就翻）
  2. 沿斜面法向抬高 **轮半径 + 轮毂下移量**（漏了会陷地）
  3. 落稳阶段**带刹**（否则测量窗口一开始就带着初速）
"""

import math

import mujoco

from . import config


def place_on_slope(model, data, slope_deg=None):
    """把车按坡度摆正并放到接触面上，返回车身 origin 到接触面的距离。"""
    slope = math.radians(config.SLOPE_DEG if slope_deg is None else slope_deg)
    body_h = config.WHEEL_RADIUS + config.WHEEL_DROP

    mujoco.mj_resetData(model, data)
    # 斜面法向 = (sin(−α), 0, cos(−α))；x 分量符号写反会让车陷进地面
    nx, nz = math.sin(-slope), math.cos(-slope)
    data.qpos[0] = body_h * nx
    data.qpos[2] = body_h * nz
    data.qpos[3] = math.cos(slope / 2)      # 绕 y 轴俯仰 −α 的四元数
    data.qpos[5] = -math.sin(slope / 2)
    mujoco.mj_forward(model, data)
    return body_h


def settle_with_brake(model, data, controller, qadr, vadr, seconds=None):
    """带刹落稳，返回落稳后的轮角基准（后续误差都相对它计算）。"""
    seconds = config.SETTLE_S if seconds is None else seconds
    dt = float(model.opt.timestep)
    # 先把当前轮角当作基准，误差从 0 开始（不能用绝对角，否则会把
    # 落稳过程中滚过的角度当成偏差，控制器反而把车往前推）
    phi_ref = [float(data.qpos[a]) for a in qadr]
    for _ in range(int(seconds / dt)):
        for i, (qa, va) in enumerate(zip(qadr, vadr)):
            tau = controller.torque(float(data.qpos[qa]) - phi_ref[i],
                                    float(data.qvel[va]))
            data.ctrl[i] = tau / config.ACTUATOR_GEAR
        mujoco.mj_step(model, data)
    # 再补一段纯 PD，把残余速度压到接近 0
    for _ in range(int(0.2 / dt)):
        for i, (qa, _va) in enumerate(zip(qadr, vadr)):
            tau = controller.torque(float(data.qpos[qa]) - phi_ref[i], 0.0)
            data.ctrl[i] = tau / config.ACTUATOR_GEAR
        mujoco.mj_step(model, data)
    phi_ref = [float(data.qpos[a]) for a in qadr]
    return phi_ref
