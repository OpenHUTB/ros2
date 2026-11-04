# -*- coding: utf-8 -*-
# MuJoCo 侧方停车仿真：小车自动倒车入位
import math, time, numpy as np, mujoco, mujoco.viewer

XML = """
<mujoco>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <default>
    <geom friction="1.0 0.005 0.0001"/>
    <joint damping="0.5" armature="0.05"/>
  </default>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <light pos="-3 3 4" dir="1 -1 -1" diffuse="0.3 0.3 0.35"/>
    <geom name="road" type="plane" size="8 4 0.05" rgba="0.4 0.4 0.4 1"/>
    <geom name="curb" type="box" pos="2.5 1.2 0.05" size="2.0 0.1 0.1" rgba="0.7 0.7 0.7 1"/>
    <geom name="line1" type="box" pos="1.8 0.5 0.005" size="0.02 0.8 0.005" rgba="1 1 1 0.5" contype="0" conaffinity="0"/>
    <geom name="line2" type="box" pos="3.2 0.5 0.005" size="0.02 0.8 0.005" rgba="1 1 1 0.5" contype="0" conaffinity="0"/>
    <geom name="line3" type="box" pos="2.5 0.1 0.005" size="1.4 0.02 0.005" rgba="1 1 1 0.5" contype="0" conaffinity="0"/>
    <body name="car_front" pos="1.5 0.7 0.1"><geom type="box" size="0.3 0.15 0.1" rgba="0.8 0.2 0.2 1" mass="5"/></body>
    <body name="car_back" pos="3.5 0.7 0.1"><geom type="box" size="0.3 0.15 0.1" rgba="0.2 0.2 0.8 1" mass="5"/></body>
    <body name="car" pos="0 0 0.1">
      <joint name="car_x" type="slide" axis="1 0 0"/>
      <joint name="car_y" type="slide" axis="0 1 0"/>
      <joint name="car_yaw" type="hinge" axis="0 0 1"/>
      <geom name="body" type="box" size="0.25 0.13 0.08" rgba="0.9 0.6 0.1 1" mass="1.5"/>
      <geom name="fw" type="box" pos="0.18 0.13 0" size="0.04 0.02 0.04" rgba="0.1 0.1 0.1 1"/>
      <geom name="fl" type="box" pos="0.18 -0.13 0" size="0.04 0.02 0.04" rgba="0.1 0.1 0.1 1"/>
      <geom name="rw" type="box" pos="-0.18 0.13 0" size="0.04 0.02 0.04" rgba="0.1 0.1 0.1 1"/>
      <geom name="rl" type="box" pos="-0.18 -0.13 0" size="0.04 0.02 0.04" rgba="0.1 0.1 0.1 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="mx" joint="car_x" gear="5"/>
    <motor name="my" joint="car_y" gear="5"/>
    <motor name="myaw" joint="car_yaw" gear="5"/>
  </actuator>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)

car_x_q = m.joint("car_x").qposadr[0]
car_y_q = m.joint("car_y").qposadr[0]
car_yaw_q = m.joint("car_yaw").qposadr[0]

# 关键帧：(时间, x, y, yaw)
keyframes = [
    (0.0,  0.0,  0.0,  0.0),   # 起点
    (3.0,  3.8,  0.0,  0.0),   # 开过车位，停住
    (5.0,  3.6,  0.1, -0.4),   # 倒车打方向
    (7.0,  3.0,  0.35,-0.6),   # 斜着倒进去
    (9.0,  2.5,  0.5,  0.0),   # 停好，车身平行
    (12.0, 2.5,  0.5,  0.0),   # 停住不动
]

def get_state(t):
    for i in range(len(keyframes)-1):
        t0, x0, y0, yaw0 = keyframes[i]
        t1, x1, y1, yaw1 = keyframes[i+1]
        if t0 <= t <= t1:
            s = (t - t0) / (t1 - t0)
            s = s * s * (3 - 2*s)
            return x0 + (x1-x0)*s, y0 + (y1-y0)*s, yaw0 + (yaw1-yaw0)*s
    return keyframes[-1][1], keyframes[-1][2], keyframes[-1][3]

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 7.0; viewer.cam.elevation = -50; viewer.cam.azimuth = 90
    t0 = time.time()
    while viewer.is_running():
        t = time.time() - t0
        tx, ty, tyaw = get_state(t)
        cx = d.qpos[car_x_q]; cy = d.qpos[car_y_q]; cyaw = d.qpos[car_yaw_q]
        err_x = tx - cx; err_y = ty - cy
        err_yaw = (tyaw - cyaw + math.pi) % (2*math.pi) - math.pi
        d.ctrl[0] = err_x * 5.0
        d.ctrl[1] = err_y * 5.0
        d.ctrl[2] = err_yaw * 8.0
        mujoco.mj_step(m, d)
        viewer.cam.lookat[0] = 2.0; viewer.cam.lookat[1] = 0.3
        viewer.sync()
