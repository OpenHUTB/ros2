#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import math
import time

XML = """
<mujoco>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" iterations="60"/>
  <default>
    <geom friction="1.0 0.005 0.0001" solref="0.02 1" solimp="0.9 0.95 0.001"/>
    <joint damping="0.05" armature="0.005"/>
  </default>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="12 12 0.05" rgba="0.75 0.75 0.75 1"/>
    <geom name="g_back" type="box" pos="0 1.5 0.2" size="0.9 0.06 0.2" rgba="0.7 0.25 0.25 1"/>
    <geom name="g_left" type="box" pos="0.9 0.75 0.2" size="0.06 0.75 0.2" rgba="0.7 0.25 0.25 1"/>
    <geom name="g_right" type="box" pos="-0.9 0.75 0.2" size="0.06 0.75 0.2" rgba="0.7 0.25 0.25 1"/>
    <body name="car" pos="1.5 -1.0 0.048">
      <freejoint name="car_root"/>
      <geom name="chassis" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.20 0.45 0.85 1" mass="4.0"/>
      <body name="wheel_fl" pos="0.16 0.125 0">
        <joint name="wheel_fl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="1.2 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="wheel_fr" pos="0.16 -0.125 0">
        <joint name="wheel_fr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="1.2 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="wheel_rl" pos="-0.16 0.125 0">
        <joint name="wheel_rl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="1.2 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="wheel_rr" pos="-0.16 -0.125 0">
        <joint name="wheel_rr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="1.2 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="fl" joint="wheel_fl" kv="20" ctrlrange="-60 60" forcerange="-15 15"/>
    <velocity name="fr" joint="wheel_fr" kv="20" ctrlrange="-60 60" forcerange="-15 15"/>
    <velocity name="rl" joint="wheel_rl" kv="20" ctrlrange="-60 60" forcerange="-15 15"/>
    <velocity name="rr" joint="wheel_rr" kv="20" ctrlrange="-60 60" forcerange="-15 15"/>
  </actuator>
</mujoco>
"""

m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)


def pose():
    x, y = d.qpos[0], d.qpos[1]
    w, qx, qy, qz = d.qpos[3], d.qpos[4], d.qpos[5], d.qpos[6]
    yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return x, y, yaw


phase = 0
step = 0
with mujoco.viewer.launch_passive(m, d) as viewer:
    with viewer.lock():
        viewer.cam.lookat[:] = [0, 0.3, 0]
        viewer.cam.distance = 9.0
        viewer.cam.elevation = -55
    while viewer.is_running():
        x, y, yaw = pose()
        if phase == 0:
            l = r = -3.0
            if x <= 0.1:
                phase = 1
        elif phase == 1:
            l, r = 4.0, -4.0
            if yaw <= -1.55:
                phase = 2
        elif phase == 2:
            l = r = -7.0
            if y >= 0.9:
                phase = 3
        else:
            l = r = 0.0

        d.ctrl[0] = d.ctrl[2] = l
        d.ctrl[1] = d.ctrl[3] = r
        mujoco.mj_step(m, d)
        viewer.sync()
        if step % 200 == 0:
            names = ["直行", "掉头", "直退", "停车"]
            print("%s  x=%.2f y=%.2f yaw=%.2f" % (names[phase], x, y, yaw))
        step += 1
        time.sleep(0.002)
