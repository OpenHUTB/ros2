# -*- coding: utf-8 -*-
# MuJoCo 侧方停车仿真：小车自动倒车入位
import time, mujoco, mujoco.viewer

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
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)

x_qp = m.joint("car_x").qposadr[0]
y_qp = m.joint("car_y").qposadr[0]
yaw_qp = m.joint("car_yaw").qposadr[0]
x_qv = m.joint("car_x").dofadr[0]
y_qv = m.joint("car_y").dofadr[0]
yaw_qv = m.joint("car_yaw").dofadr[0]

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

dt = m.opt.timestep
T_END = keyframes[-1][0]
with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 7.0; viewer.cam.elevation = -50; viewer.cam.azimuth = 90
    while viewer.is_running():
        step_start = time.time()
        t = min(d.time, T_END)
        tx, ty, tyaw = get_state(t)
        px, py, pyaw = get_state(max(0.0, t - dt))

        d.qpos[x_qp] = tx
        d.qpos[y_qp] = ty
        d.qpos[yaw_qp] = tyaw
        d.qvel[x_qv] = (tx - px) / dt
        d.qvel[y_qv] = (ty - py) / dt
        d.qvel[yaw_qv] = (tyaw - pyaw) / dt

        mujoco.mj_step(m, d)
        viewer.cam.lookat[0] = 2.0; viewer.cam.lookat[1] = 0.3
        viewer.sync()

        # 按仿真步长对齐墙钟，避免全速灌完 12 秒轨迹
        slack = dt - (time.time() - step_start)
        if slack > 0:
            time.sleep(slack)
