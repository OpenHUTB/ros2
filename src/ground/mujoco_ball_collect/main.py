# -*- coding: utf-8 -*-
# MuJoCo 搜集小球仿真：差速小车在平面上自动收集散布的小球
# 运行: python3 main.py  (Ubuntu 20.04 / mujoco 2.3.7)
import math
import time
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
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.5 0.6 0.85" rgb2="0.85 0.9 1.0" width="512" height="512"/>
    <texture name="ground" type="2d" builtin="checker" rgb1="0.88 0.88 0.88" rgb2="0.93 0.93 0.93" width="512" height="512"/>
    <material name="gnd" texture="ground" texrepeat="5 5"/>
    <material name="ball_red" rgba="0.95 0.25 0.25 1" emission="0.0" specular="0.4" shininess="0.6"/>
    <material name="ball_blue" rgba="0.25 0.4 0.95 1" emission="0.0" specular="0.4" shininess="0.6"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="8 8 0.02" material="gnd"/>
    <geom name="line_n" type="box" pos="0 2.4 0.005" size="3 0.02 0.005" rgba="1 1 1 0.35"/>
    <geom name="line_s" type="box" pos="0 -2.4 0.005" size="3 0.02 0.005" rgba="1 1 1 0.35"/>
    <geom name="line_e" type="box" pos="2.4 0 0.005" size="0.02 3 0.005" rgba="1 1 1 0.35"/>
    <geom name="line_w" type="box" pos="-2.4 0 0.005" size="0.02 3 0.005" rgba="1 1 1 0.35"/>
    <geom name="home_cross1" type="box" pos="0 0 0.01" size="0.02 0.25 0.005" rgba="0.1 0.55 0.35 0.5"/>
    <geom name="home_cross2" type="box" pos="0 0 0.01" size="0.25 0.02 0.005" rgba="0.1 0.55 0.35 0.5"/>

    <!-- 小车：底盘 + 收集筐 + 四轮 -->
    <body name="collector" pos="0 0 0.048">
      <freejoint name="collector_root"/>
      <geom name="chassis" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.20 0.45 0.85 1" mass="4.0"/>
      <body name="basket" pos="0 0 0.14">
        <geom name="b_floor" type="box" size="0.20 0.13 0.01" mass="0.1" rgba="0.1 0.55 0.35 1"/>
        <geom name="b_w1" type="box" size="0.20 0.005 0.07" pos="0 -0.13 0.04" mass="0.05" rgba="0.1 0.55 0.35 1"/>
        <geom name="b_w2" type="box" size="0.20 0.005 0.07" pos="0 0.13 0.04" mass="0.05" rgba="0.1 0.55 0.35 1"/>
        <geom name="b_w3" type="box" size="0.005 0.13 0.07" pos="-0.20 0 0.04" mass="0.05" rgba="0.1 0.55 0.35 1"/>
        <geom name="b_w4" type="box" size="0.005 0.13 0.07" pos="0.20 0 0.04" mass="0.05" rgba="0.1 0.55 0.35 1"/>
      </body>
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

    <!-- 6 个待收集小球 -->
    <body name="ball0" pos="1.2 1.2 0.12">
      <freejoint name="b0_j"/>
      <geom name="ball0_geom" type="sphere" size="0.07" contype="0" conaffinity="0" material="ball_red"/>
    </body>
    <body name="ball1" pos="1.8 -1.4 0.12">
      <freejoint name="b1_j"/>
      <geom name="ball1_geom" type="sphere" size="0.07" contype="0" conaffinity="0" material="ball_red"/>
    </body>
    <body name="ball2" pos="-1.6 1.0 0.12">
      <freejoint name="b2_j"/>
      <geom name="ball2_geom" type="sphere" size="0.07" contype="0" conaffinity="0" material="ball_red"/>
    </body>
    <body name="ball3" pos="-1.3 -1.6 0.12">
      <freejoint name="b3_j"/>
      <geom name="ball3_geom" type="sphere" size="0.07" contype="0" conaffinity="0" material="ball_red"/>
    </body>
    <body name="ball4" pos="0.3 1.8 0.12">
      <freejoint name="b4_j"/>
      <geom name="ball4_geom" type="sphere" size="0.07" contype="0" conaffinity="0" material="ball_blue"/>
    </body>
    <body name="ball5" pos="-0.5 -1.9 0.12">
      <freejoint name="b5_j"/>
      <geom name="ball5_geom" type="sphere" size="0.07" contype="0" conaffinity="0" material="ball_blue"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="fl" joint="w_fl" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="fr" joint="w_fr" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="rl" joint="w_rl" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
    <velocity name="rr" joint="w_rr" kv="20" ctrlrange="-60 60" forcerange="-100 100"/>
  </actuator>
</mujoco>
"""

N_BALL = 6
BALL_POS_SLICE = 7          # freejoint qpos: 3 位置 + 4 四元数
COLLECT_DIST = 0.35         # 车与球距离小于该值视为收集
V_MAX = 26.0
V_MIN = 3.0
TURN_K = 4.0
ANIM_STEPS = 26

m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
mujoco.mj_resetData(m, d)

# qpos: collector 0-6、轮 7-10、球从 11 起每 7 个；qvel: 球从 10 起每 6 个
BALL_START = 11
BALL_VEL_START = 10
BALL_VEL_SLICE = 6
collected = [False] * N_BALL
collected_n = 0
anim_t = [-1.0] * N_BALL        # 吸入动画进度，-1 表示动画结束
anim_sx = [0.0] * N_BALL        # 收集瞬间球的 x（动画起点）
anim_sy = [0.0] * N_BALL        # 收集瞬间球的 y（动画起点）


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 6.5
    viewer.cam.elevation = -50
    viewer.cam.azimuth = 90
    viewer.cam.lookat[0] = 0.0
    viewer.cam.lookat[1] = 0.0
    viewer.cam.lookat[2] = 0.05
    n = 0
    while viewer.is_running():
        cx = d.qpos[0]
        cy = d.qpos[1]

        # 找最近的未收集球
        target = None
        best = 1e9
        for i in range(N_BALL):
            if collected[i]:
                continue
            bx = d.qpos[BALL_START + i * BALL_POS_SLICE]
            by = d.qpos[BALL_START + i * BALL_POS_SLICE + 1]
            dist = math.hypot(bx - cx, by - cy)
            if dist < best:
                best = dist
                target = i

        if target is None:
            d.ctrl[:] = 0
            # 收完后球待在筐里
            for i in range(N_BALL):
                idx = BALL_START + i * BALL_POS_SLICE
                d.qpos[idx] = d.qpos[0]
                d.qpos[idx + 1] = d.qpos[1]
                d.qpos[idx + 2] = 0.25
                d.qpos[idx + 3] = 1.0
                d.qpos[idx + 4] = 0.0
                d.qpos[idx + 5] = 0.0
                d.qpos[idx + 6] = 0.0
                d.qvel[BALL_VEL_START + i * BALL_VEL_SLICE] = 0.0
                d.qvel[BALL_VEL_START + i * BALL_VEL_SLICE + 1] = 0.0
                d.qvel[BALL_VEL_START + i * BALL_VEL_SLICE + 2] = 0.0
            if n % 200 == 0:
                print("step=%d 全部小球已收集完毕！" % n)
            mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(0.002)
            n += 1
            if n > 62000:
                break
            continue
        bx = d.qpos[BALL_START + target * BALL_POS_SLICE]
        by = d.qpos[BALL_START + target * BALL_POS_SLICE + 1]

        # 差速转向 + 按距离调速
        yaw = yaw_from_quat(d.qpos[3:7])
        err = math.atan2(by - cy, bx - cx) - yaw
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        speed = V_MIN + (V_MAX - V_MIN) * min(1.0, best / 3.0)
        l = speed - TURN_K * err
        r = speed + TURN_K * err
        l = max(V_MIN, min(V_MAX, l))
        r = max(V_MIN, min(V_MAX, r))
        d.ctrl[0] = l
        d.ctrl[1] = r
        d.ctrl[2] = l
        d.ctrl[3] = r

        # 接近即收集，球开始弹入筐
        if best < COLLECT_DIST:
            collected[target] = True
            collected_n += 1
            anim_sx[target] = bx
            anim_sy[target] = by
            anim_t[target] = 0.0
            print("step=%d 收集第%d个球(idx=%d) 剩余=%d" % (n, collected_n, target, N_BALL - collected_n))

        # 未收集球锁悬浮高度防下落
        for i in range(N_BALL):
            if not collected[i]:
                d.qpos[BALL_START + i * BALL_POS_SLICE + 2] = 0.12
                d.qvel[BALL_VEL_START + i * BALL_VEL_SLICE + 2] = 0.0

        # 已收集球先弹入动画，再进筐跟随
        for i in range(N_BALL):
            if not collected[i]:
                continue
            idx = BALL_START + i * BALL_POS_SLICE
            if anim_t[i] >= 0.0:
                anim_t[i] += 1.0 / ANIM_STEPS
                t = min(1.0, anim_t[i])
                e = t * t * (3.0 - 2.0 * t)
                d.qpos[idx] = anim_sx[i] + (d.qpos[0] - anim_sx[i]) * e
                d.qpos[idx + 1] = anim_sy[i] + (d.qpos[1] - anim_sy[i]) * e
                d.qpos[idx + 2] = 0.12 + (0.25 - 0.12) * e
                if t >= 1.0:
                    anim_t[i] = -1.0
            else:
                d.qpos[idx] = d.qpos[0]
                d.qpos[idx + 1] = d.qpos[1]
                d.qpos[idx + 2] = 0.25
            d.qpos[idx + 3] = 1.0
            d.qpos[idx + 4] = 0.0
            d.qpos[idx + 5] = 0.0
            d.qpos[idx + 6] = 0.0

        mujoco.mj_step(m, d)

        viewer.sync()
        time.sleep(0.002)
        if n % 2000 == 0:
            print("step=%d 已收集=%d/%d" % (n, collected_n, N_BALL))
        n += 1
        if n > 62000:
            print("超时，已收集=%d/%d" % (collected_n, N_BALL))
            break

print("完成：收集 %d/%d 个球" % (collected_n, N_BALL))
