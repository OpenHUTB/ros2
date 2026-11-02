# -*- coding: utf-8 -*-
# MuJoCo 双车自动跟车（前车变速直行，后车自动调速保持车距）
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
    <geom name="floor" type="plane" size="12 12 0.05" rgba="0.75 0.75 0.75 1"/>
    <geom name="m0" type="box" pos="0 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m2" type="box" pos="2 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m4" type="box" pos="4 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m6" type="box" pos="6 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m8" type="box" pos="8 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m10" type="box" pos="10 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m12" type="box" pos="12 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <geom name="m14" type="box" pos="14 0 0.003" size="0.01 1.6 0.002" rgba="0.35 0.35 0.35 0.5"/>
    <body name="leader" pos="2.5 0 0.048">
      <freejoint name="leader_root"/>
      <geom name="chassis_l" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.20 0.45 0.85 1" mass="4.0"/>
      <body name="w_fl" pos="0.16 0.125 0">
        <joint name="w_fl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="w_fr" pos="0.16 -0.125 0">
        <joint name="w_fr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="w_rl" pos="-0.16 0.125 0">
        <joint name="w_rl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="w_rr" pos="-0.16 -0.125 0">
        <joint name="w_rr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
    </body>
    <body name="follower" pos="0 0 0.048">
      <freejoint name="follower_root"/>
      <geom name="chassis_f" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.85 0.55 0.15 1" mass="4.0"/>
      <body name="w2_fl" pos="0.16 0.125 0">
        <joint name="w2_fl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="w2_fr" pos="0.16 -0.125 0">
        <joint name="w2_fr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="w2_rl" pos="-0.16 0.125 0">
        <joint name="w2_rl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="w2_rr" pos="-0.16 -0.125 0">
        <joint name="w2_rr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="fl" joint="w_fl" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="fr" joint="w_fr" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="rl" joint="w_rl" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="rr" joint="w_rr" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="f2fl" joint="w2_fl" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="f2fr" joint="w2_fr" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="f2rl" joint="w2_rl" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="f2rr" joint="w2_rr" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
  </actuator>
</mujoco>
"""

m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)

TARGET = 1.2  # 期望车距（米）
ALPHA = 0.003  # 后车反应滤波系数：越小变速延迟越明显（约 0.67s）
filt = 10.0


with mujoco.viewer.launch_passive(m, d) as viewer:
    # 相机锁定后车（橙色）俯视：每帧手动对准
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 5.0
    viewer.cam.elevation = -50
    viewer.cam.azimuth = 90
    n = 0
    while viewer.is_running():
        if n >= 60000:
            d.ctrl[:] = 0
            print("时间到")
            break
        # 视点对准后车
        viewer.cam.lookat[0] = d.qpos[11]
        viewer.cam.lookat[1] = d.qpos[12]
        viewer.cam.lookat[2] = 0.05
        # 前车（蓝色）大幅变速直行：正弦 2.1s 一周期，0.05~1.15 m/s
        lead_ctrl = 12.0 + 11.0 * math.sin(n * 0.006)
        d.ctrl[0] = lead_ctrl
        d.ctrl[1] = lead_ctrl
        d.ctrl[2] = lead_ctrl
        d.ctrl[3] = lead_ctrl
        # 后车（橙色）：以前车速度为前馈 + 距离误差修正，经过滤延迟响应，保持 1.2m
        gap = d.qpos[0] - d.qpos[11]
        target = lead_ctrl + 8.0 * (gap - TARGET)
        filt += ALPHA * (target - filt)
        speed = max(2.0, min(26.0, filt))
        d.ctrl[4] = speed
        d.ctrl[5] = speed
        d.ctrl[6] = speed
        d.ctrl[7] = speed
        mujoco.mj_step(m, d)
        viewer.sync()
        time.sleep(0.002)
        if n % 40 == 0:
            print(f"step={n} lead_x={d.qpos[0]:.2f} follow_x={d.qpos[11]:.2f} gap={gap:.2f} lead_ctrl={lead_ctrl:.1f} f_ctrl={speed:.1f}")
        n += 1
