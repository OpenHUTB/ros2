# -*- coding: utf-8 -*-
# MuJoCo 小车自动打靶：车固定，炮塔自动识别立着的靶子并转动瞄准，依次击倒
import math
import time
import numpy as np
import mujoco
import mujoco.viewer

XML = """
<mujoco>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" iterations="60"/>
  <default>
    <geom friction="1.0 0.005 0.0001" solref="0.02 1" solimp="0.9 0.95 0.001"/>
    <joint damping="0.05" armature="0.02"/>
  </default>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <light pos="-3 3 4" dir="1 -1 -1" diffuse="0.3 0.3 0.35"/>
    <geom name="floor" type="plane" size="8 8 0.05" rgba="0.75 0.75 0.75 1"/>
    <geom name="zone" type="box" pos="2.0 0 0.001" size="2.0 2.9 0.001" rgba="0.50 0.60 0.50 1"/>
    <geom name="line" type="box" pos="0.4 0 0.002" size="0.02 2.9 0.002" rgba="0.90 0.90 0.90 1"/>
    <body name="car" pos="0 0 0.05">
      <geom name="chassis" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.20 0.45 0.85 1" mass="4.0"/>
      <body name="turret" pos="0.28 0 0.12">
        <joint name="turret_j" type="hinge" axis="0 0 1"/>
        <geom name="barrel" type="box" pos="0.15 0 0" size="0.15 0.03 0.03" rgba="0.25 0.25 0.25 1"/>
        <geom name="beam" type="cylinder" pos="2.0 0 0" size="0.012 1.7" euler="0 1.5707963 0" rgba="0.10 1.00 0.30 0.30" contype="0" conaffinity="0"/>
      </body>
      <body name="c_fl" pos="0.16 0.125 0">
        <joint name="c_fl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_fr" pos="0.16 -0.125 0">
        <joint name="c_fr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_rl" pos="-0.16 0.125 0">
        <joint name="c_rl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_rr" pos="-0.16 -0.125 0">
        <joint name="c_rr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
    </body>
    <body name="t0" pos="1.55 -1.25 0">
      <joint name="tj0" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.85 0.25 0.25 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx0" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy0" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t1" pos="2.00 0.00 0">
      <joint name="tj1" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.95 0.60 0.15 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx1" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy1" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t2" pos="1.55 1.25 0">
      <joint name="tj2" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.90 0.85 0.20 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx2" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy2" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t3" pos="3.00 -1.35 0">
      <joint name="tj3" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.30 0.70 0.35 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx3" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy3" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t4" pos="3.40 0.00 0">
      <joint name="tj4" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.25 0.55 0.85 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx4" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy4" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t5" pos="3.00 1.35 0">
      <joint name="tj5" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.60 0.35 0.80 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx5" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy5" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="ball0" pos="10 10 0.05">
      <freejoint name="bj0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball1" pos="10 10.2 0.05">
      <freejoint name="bj1"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball2" pos="10 10.4 0.05">
      <freejoint name="bj2"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball3" pos="10 10.6 0.05">
      <freejoint name="bj3"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball4" pos="10 10.8 0.05">
      <freejoint name="bj4"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball5" pos="10 11.0 0.05">
      <freejoint name="bj5"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball6" pos="10 11.2 0.05">
      <freejoint name="bj6"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball7" pos="10 11.4 0.05">
      <freejoint name="bj7"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball8" pos="10 11.6 0.05">
      <freejoint name="bj8"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball9" pos="10 11.8 0.05">
      <freejoint name="bj9"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball10" pos="10 12.0 0.05">
      <freejoint name="bj10"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball11" pos="10 12.2 0.05">
      <freejoint name="bj11"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="turret" joint="turret_j" kv="20" ctrlrange="-7 7" forcerange="-30 30"/>
  </actuator>
</mujoco>
"""

m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)

# 锁定准星的十字 geom（平时透明，锁定目标时亮红）
cx_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"cx{i}") for i in range(6)]
cy_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"cy{i}") for i in range(6)]

# 六个靶子的位置：前三近、后三远（车在原点）
TARGETS = [
    (1.55, -1.25), (2.00, 0.00), (1.55, 1.25),
    (3.00, -1.35), (3.40, 0.00), (3.00, 1.35),
]
# 球的 qpos/qvel 起始：炮塔0+轮1-4+靶5-10
BALL_Q = 11
BALL_V = 11


def clamp(v, lo=-15.0, hi=15.0):
    return max(lo, min(hi, v))


n = 0
si = 0
cool = 0
hit = [0] * 6
last_tid = -1
scan = 1.0
scan_t = 0
tid = -1
with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 9.0
    viewer.cam.elevation = -40
    viewer.cam.azimuth = 90
    while viewer.is_running():
        if n >= 9000:
            d.ctrl[:] = 0
            print("时间到")
            break
        # 全部击倒则结束
        if all(abs(d.qpos[5 + i]) > 0.25 for i in range(6)):
            d.ctrl[:] = 0
            print("全部击倒")
            break
        if si >= 12:
            d.ctrl[:] = 0
            print("球用尽")
            break
        if n < 400:
            # 开场雷达扫描：激光左右扫场找目标
            d.ctrl[0] = 5.0 * scan
            scan_t += 1
            if scan_t > 130:
                scan = -scan
                scan_t = 0
        elif cool > 0:
            cool -= 1
            d.ctrl[0] = 0
        else:
            # 智能识别：雷达式选炮塔当前朝向最近的、还立着的靶
            tid = -1
            best = 999.0
            for i in range(6):
                if abs(d.qpos[5 + i]) < 0.25:
                    tx, ty = TARGETS[i]
                    a = math.atan2(ty, tx)
                    err = (a - d.qpos[0] + math.pi) % (2 * math.pi) - math.pi
                    if abs(err) < best:
                        best = abs(err)
                        tid = i
            if tid == -1:
                d.ctrl[0] = 0
            else:
                tx, ty = TARGETS[tid]
                want = math.atan2(ty, tx)
                err_t = (want - d.qpos[0] + math.pi) % (2 * math.pi) - math.pi
                d.ctrl[0] = clamp(err_t * 6.0 - d.qvel[0] * 2.0)
                if abs(err_t) < 0.05 and abs(d.qvel[0]) < 0.05:
                    # 炮塔对准了，发射
                    bq = BALL_Q + si * 7
                    bv = BALL_V + si * 6
                    yaw_t = d.qpos[0]
                    mx = 0.28 + 0.35 * math.cos(yaw_t)
                    my = 0.35 * math.sin(yaw_t)
                    dx = tx - mx
                    dy = ty - my
                    dist = math.hypot(dx, dy)
                    # 自动控力度：固定飞行时间，距离越远球速越大
                    t = 0.45
                    speed = dist / t
                    if hit[tid] > 0:
                        speed *= 1.25  # 没打中的靶补枪自动加力
                    vx = speed * dx / dist
                    vy = speed * dy / dist
                    vz = (0.25 - 0.17 + 0.5 * 9.81 * t * t) / t
                    d.qpos[bq:bq + 3] = [mx, my, 0.17]
                    d.qpos[bq + 3:bq + 7] = [0, 0, 0, 1]
                    d.qvel[bv:bv + 3] = [vx, vy, vz]
                    d.qvel[bv + 3:bv + 6] = [0, 0, 0]
                    mujoco.mj_forward(m, d)
                    if tid != last_tid:
                        print(f"锁定靶{tid} 距离{dist:.2f}m")
                        last_tid = tid
                    print(f"第{si + 1}发 发射 力度{speed:.1f}")
                    hit[tid] += 1
                    cool = 300
                    si += 1
        mujoco.mj_step(m, d)
        # 锁定准星：被锁定的靶子上方亮起红叉
        for i in range(6):
            on = 1.0 if i == tid else 0.0
            m.geom_rgba[cx_ids[i]] = [1.0, 1.0, 1.0, on]
            m.geom_rgba[cy_ids[i]] = [1.0, 1.0, 1.0, on]
        viewer.cam.lookat[0] = 0.6 * math.cos(d.qpos[0])
        viewer.cam.lookat[1] = 0.6 * math.sin(d.qpos[0])
        viewer.sync()
        time.sleep(0.002)
        if n % 500 == 0:
            ang = [f"{d.qpos[5 + i]:.1f}" for i in range(6)]
            print(f"step={n} 炮塔={d.qpos[0]:.2f} 靶角度={ang}")
        n += 1
