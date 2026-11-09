#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定量制动测试（无需 ROS 与图形界面，可独立运行）

测试内容：
  1) 不同巡航驱动力下的紧急制动：记录触发车速、停止结果、车头到悬崖边缘的余量
  2) 无制动对照：验证越过边缘后车辆真实坠落（z 方向自由度存在）

用法：python3 brake_test.py
注意：模型与控制参数须与 cliff_brake_sim.py 保持一致（改动请同步）。
"""
import mujoco
import numpy as np

# ---------------- 参数（与 cliff_brake_sim.py 保持一致） ----------------
CLIFF_X = 2.5
CTRL_DT = 0.01
KV_BRAKE = 150.0
FMAX_BRAKE = 100.0
BRAKE_DIST = 0.3
RELEASE_DIST = 0.5

XML_MODEL = """
<mujoco>
    <option gravity="0 0 -9.81" timestep="0.002"/>
    <worldbody>
        <light pos="0 0 3"/>
        <geom name="floor" type="box" size="3.75 2.0 0.1" pos="-1.25 0 -0.1"
              rgba="0.8 0.8 0.8 1" friction="0.05 0.005 0.0001"/>
        <geom type="box" size="5 2 0.1" pos="7.5 0 -2.0"
              rgba="0.2 0.2 0.2 1" friction="0.05 0.005 0.0001"/>
        <body name="chassis" pos="0 0 0.05">
            <geom type="box" size="0.2 0.15 0.05" rgba="0.2 0.6 0.8 1"
                  friction="0.05 0.005 0.0001"/>
            <joint name="chassis_x" type="slide" axis="1 0 0" damping="0.5"/>
            <joint name="chassis_z" type="slide" axis="0 0 1" damping="0.5"/>
            <site name="sensor_site" pos="0.2 0 0" size="0.02" rgba="1 0 0 1"/>
        </body>
    </worldbody>
    <actuator>
        <motor joint="chassis_x" ctrlrange="-150 150"/>
    </actuator>
</mujoco>
"""


def fresh():
    model = mujoco.MjModel.from_xml_string(XML_MODEL)
    data = mujoco.MjData(model)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sensor_site")
    xd = model.joint("chassis_x").dofadr[0]
    zadr = model.joint("chassis_z").qposadr[0]
    return model, data, site, xd, zadr


def dist_of(data, site):
    return CLIFF_X - data.site_xpos[site][0]


def run_brake(drive_force):
    """真实工况：以 drive_force 巡航 → 距离触发阈值 → 制动。"""
    model, data, site, xd, zadr = fresh()
    v_trigger = None
    braking = False
    next_ctrl = 0.0
    while data.time < 30.0:
        if data.time >= next_ctrl:               # 与 sim 节点相同的 100Hz 控制节拍
            next_ctrl += CTRL_DT
            d = dist_of(data, site)
            if not braking and d < BRAKE_DIST:
                braking = True
                v_trigger = data.qvel[xd]
            elif braking and d > RELEASE_DIST:
                braking = False
            vx = data.qvel[xd]
            data.ctrl[0] = (float(np.clip(-KV_BRAKE * vx, -FMAX_BRAKE, 0.0))
                            if braking else drive_force)
        mujoco.mj_step(model, data)
        if data.qpos[zadr] < -0.2:               # 重心跌落台面 → 坠落
            return "FELL", v_trigger, None
        if braking and abs(data.qvel[xd]) < 1e-3:
            for _ in range(500):                 # 停稳后再观察 1s（仿真时间）
                mujoco.mj_step(model, data)
            if data.qpos[zadr] < -0.2:
                return "FELL", v_trigger, None
            d_final = dist_of(data, site)
            # 余量 >0 安全停住；<0 表示车头越界但重心仍在台上（悬空搁住）
            return ("STOPPED" if d_final > 0 else "OVERHANG"), v_trigger, d_final
    return "TIMEOUT", v_trigger, None


def run_fall(drive_force):
    """无制动对照：恒定驱动力直冲悬崖，验证跌落自由度真实存在。"""
    model, data, site, xd, zadr = fresh()
    data.ctrl[0] = drive_force
    while data.time < 30.0:
        mujoco.mj_step(model, data)
        if data.qpos[zadr] < -1.0:               # 已坠落到下层平台附近
            return data.time, data.qpos[zadr]
    return None, None


if __name__ == '__main__':
    print("不同巡航驱动力下的紧急制动测试（触发 %.2fm，滞回解除 %.2fm）" % (BRAKE_DIST, RELEASE_DIST))
    print("%12s %12s %10s %14s" % ("驱动力(N)", "触发车速(m/s)", "结果", "车头余量(m)"))
    for f in (15.0, 20.0, 30.0, 50.0, 100.0):
        status, v_t, margin = run_brake(f)
        v_str = "%.2f" % v_t if v_t is not None else "-"
        m_str = "%.3f" % margin if margin is not None else "-"
        print("%12.0f %12s %10s %14s" % (f, v_str, status, m_str))

    print()
    print("无制动对照（恒驱动力 15N 直冲悬崖）：")
    t_fall, z_fall = run_fall(15.0)
    if t_fall is not None:
        print("  车辆于 t=%.2fs 坠落至 z=%.2fm（跌落自由度验证通过）" % (t_fall, z_fall))
    else:
        print("  未检测到坠落——请检查 z 自由度是否被正确添加！")
