# -*- coding: utf-8 -*-
# MuJoCo 动态障碍局部避障：自车沿道路开到终点，绕开横穿的移动障碍
import time, math, mujoco, mujoco.viewer

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
    <geom name="road" type="plane" size="8 3 0.05" rgba="0.4 0.4 0.4 1"/>
    <geom name="start" type="box" pos="-5 0 0.01" size="0.3 0.6 0.01" rgba="0.2 0.6 0.2 0.5" contype="0" conaffinity="0"/>
    <geom name="goal" type="box" pos="5 0 0.01" size="0.3 0.6 0.01" rgba="0.8 0.2 0.2 0.5" contype="0" conaffinity="0"/>
    <body name="car" pos="0 0 0.1">
      <joint name="car_x" type="slide" axis="1 0 0"/>
      <joint name="car_y" type="slide" axis="0 1 0"/>
      <joint name="car_yaw" type="hinge" axis="0 0 1"/>
      <geom name="body" type="box" size="0.25 0.13 0.08" rgba="0.9 0.6 0.1 1" mass="1.5"/>
    </body>
    <body name="obs1" pos="-2 0 0.1">
      <joint name="obs1_y" type="slide" axis="0 1 0"/>
      <geom type="box" size="0.22 0.22 0.1" rgba="0.2 0.2 0.8 1" mass="2"/>
    </body>
    <body name="obs2" pos="0 0 0.1">
      <joint name="obs2_y" type="slide" axis="0 1 0"/>
      <geom type="box" size="0.22 0.22 0.1" rgba="0.2 0.2 0.8 1" mass="2"/>
    </body>
    <body name="obs3" pos="2 0 0.1">
      <joint name="obs3_y" type="slide" axis="0 1 0"/>
      <geom type="box" size="0.22 0.22 0.1" rgba="0.2 0.2 0.8 1" mass="2"/>
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

obs_y_qp = [m.joint(f"obs{i}_y").qposadr[0] for i in (1, 2, 3)]
obs_y_qv = [m.joint(f"obs{i}_y").dofadr[0] for i in (1, 2, 3)]

d.qpos[x_qp] = -5.0
d.qpos[y_qp] = 0.0
d.qpos[yaw_qp] = 0.0

GOAL = (5.0, 0.0)
V_MAX = 0.85
R_REP = 1.3
K_REP = 1.4
K_TAN = 0.3
K_MID = 0.5
K_YAW = 3.0
STOP = 0.15
# 障碍: 固定 x, 横向振幅, 角频率, 初相
obstacles = [
    (-2.0, 0.5, 0.6, 0.0),
    ( 0.0, 0.5, 0.5, 2.1),
    ( 2.0, 0.5, 0.7, 4.0),
]

dt = m.opt.timestep
T_END = 30.0
min_clear = float("inf")
reached = False

with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 9.0; viewer.cam.elevation = -50; viewer.cam.azimuth = 90
    while viewer.is_running() and d.time < T_END:
        step_start = time.time()
        t = min(d.time, T_END)

        # 障碍横向往复
        for i, (ox, amp, w, ph) in enumerate(obstacles):
            oy = amp * math.sin(w * t + ph)
            d.qpos[obs_y_qp[i]] = oy
            d.qvel[obs_y_qv[i]] = amp * w * math.cos(w * t + ph)

        x = d.qpos[x_qp]; y = d.qpos[y_qp]; yaw = d.qpos[yaw_qp]
        gx, gy = GOAL
        dx, dy = gx - x, gy - y
        dist_goal = math.hypot(dx, dy)

        # 目标吸引，取单位方向
        fx = dx / dist_goal if dist_goal > 1e-3 else 0.0
        fy = dy / dist_goal if dist_goal > 1e-3 else 0.0
        fy += -K_MID * y

        for i, (ox, amp, w, ph) in enumerate(obstacles):
            oy = amp * math.sin(w * t + ph)
            rx, ry = x - ox, y - oy
            dist = math.hypot(rx, ry)
            if dist < min_clear:
                min_clear = dist
            if dist < R_REP and dist > 1e-3:
                f = K_REP * (1.0 / dist - 1.0 / R_REP) / dist
                fx += f * rx / dist
                fy += f * ry / dist
                # 奇偶障碍交替从不同侧绕
                s = 1 if i % 2 == 0 else -1
                ft = K_TAN / dist
                tx, ty = -ry / dist, rx / dist
                fx += s * ft * tx
                fy += s * ft * ty

        yaw_cmd = math.atan2(fy, fx)
        err = (yaw_cmd - yaw + math.pi) % (2 * math.pi) - math.pi
        yaw += K_YAW * err * dt

        v = V_MAX * min(1.0, dist_goal / 1.0)
        if dist_goal < STOP:
            v = 0.0
            reached = True
            break

        vx = v * math.cos(yaw)
        vy = v * math.sin(yaw)
        d.qpos[x_qp] = x + vx * dt
        d.qpos[y_qp] = y + vy * dt
        d.qpos[yaw_qp] = yaw
        d.qvel[x_qv] = vx
        d.qvel[y_qv] = vy
        d.qvel[yaw_qv] = K_YAW * err

        mujoco.mj_step(m, d)
        viewer.cam.lookat[0] = 0.0; viewer.cam.lookat[1] = 0.0
        viewer.sync()

        slack = dt - (time.time() - step_start)
        if slack > 0:
            time.sleep(slack)

elapsed = d.time
final_dist = math.hypot(GOAL[0] - d.qpos[x_qp], GOAL[1] - d.qpos[y_qp])
if reached:
    print(f"已到达终点，用时 {elapsed:.1f} s，最小车障间距 {min_clear:.2f} m")
else:
    print(f"未到达目标：距终点 {final_dist:.2f} m，用时 {elapsed:.1f} s，最小车障间距 {min_clear:.2f} m")
