# -*- coding: utf-8 -*-
"""MuJoCo 双机械臂协同搬运（Dual-Arm Cooperative Transport）——单文件版

两台二连杆机械臂（带垂直升降关节与夹爪）对称布置，
协同完成一次「抓取-抬升-平移-放下」：把一块长木板从 A 区搬到 B 区。
两臂同步动作，末端各画一条黄色轨迹线。
模型 XML 已内嵌在本文件，无需额外文件。

运行（虚拟机 / 无显卡环境）：
    pip3 install --user mujoco pillow numpy
    sudo apt install -y libosmesa6-dev
    MUJOCO_GL=osmesa python3 mujoco_arm_dual_single.py

输出：同目录 demo_dual.gif
"""
import os, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

OUT_GIF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_dual.gif")

# ================= 内嵌模型 =================
ARM_XML = """
<mujoco model="arm_dual">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="640" offheight="480"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256" rgb1="0.52 0.52 0.54" rgb2="0.42 0.42 0.44"/>
    <material name="floor_mat" texture="grid" texrepeat="4 4" reflectance="0.2"/>
    <material name="tip_mat" rgba="1 0 0 1"/>
  </asset>
  <worldbody>
    <light name="sun" pos="0.3 0.3 1.5" dir="0 0 -1" diffuse="0.55 0.55 0.55" ambient="0.3 0.3 0.3" castshadow="false"/>
    <light name="fill" pos="-0.3 0.3 1.0" dir="0.5 0 -1" diffuse="0.3 0.3 0.35" ambient="0.25 0.25 0.28" castshadow="false"/>
    <light name="side" pos="0.1 -0.5 0.7" dir="0.6 0.3 -0.75" diffuse="0.35 0.35 0.4" ambient="0.12 0.12 0.12" castshadow="false"/>
    <camera name="front" pos="0.3 0 1.5" zaxis="0 0 -1" fovy="38"/>
    <geom name="floor" type="plane" size="2 2 0.1" pos="0.35 0 -0.02" material="floor_mat"/>

    <!-- B 点（黄色地面标记，木板目标区中心） -->
    <geom name="mark_b" type="sphere" size="0.018" pos="0.35 -0.15 0.012" rgba="0.95 0.75 0.1 1"/>

    <!-- 长木板（free joint，初始在 A 区地面，中心 z=0.045；位置由代码控制） -->
    <body name="board" pos="0.35 0.30 0.045">
      <freejoint name="board_free"/>
      <geom name="board_g" type="box" size="0.20 0.04 0.025" density="200" rgba="0.30 0.60 0.35 1" condim="4" friction="1 0.005 0.001"/>
    </body>

    <!-- 机械臂1（左，底座 (0.10, 0)，蓝橙） -->
    <body name="arm1_base" pos="0.10 0 0.15">
      <geom name="arm1_base_g" type="cylinder" size="0.06 0.05" pos="0 0 0" rgba="0.35 0.35 0.4 1"/>
      <joint name="shoulder1" type="hinge" axis="0 0 1" pos="0 0 0"/>
      <body name="arm1_link1" pos="0 0 0">
        <geom name="arm1_link1_g" type="capsule" fromto="0 0 0 0.25 0 0" size="0.03" rgba="0.15 0.45 0.85 1"/>
        <joint name="elbow1" type="hinge" axis="0 0 1" pos="0.25 0 0"/>
        <body name="arm1_link2" pos="0.25 0 0">
          <geom name="arm1_link2_g" type="capsule" fromto="0 0 0 0.25 0 0" size="0.026" rgba="0.85 0.55 0.2 1"/>
          <body name="arm1_lift" pos="0.25 0 0">
            <joint name="lift1" type="slide" axis="0 0 1" range="-0.20 0.15"/>
            <geom name="arm1_lift_conn" type="cylinder" size="0.006 0.06" pos="0 0 0.28" rgba="0.45 0.45 0.5 1"/>
            <body name="arm1_gripper" pos="0 0 0.02">
              <joint name="grip1" type="hinge" axis="0 0 1" range="-0.55 0"/>
              <geom name="arm1_grip_base" type="box" size="0.012 0.018 0.012" pos="0.03 0 0.08" rgba="0.5 0.5 0.55 1"/>
              <geom name="arm1_finger1" type="box" size="0.02 0.005 0.02" pos="0.03 0.025 0.08" rgba="0.35 0.35 0.4 1"/>
              <geom name="arm1_finger2" type="box" size="0.02 0.005 0.02" pos="0.03 -0.025 0.08" rgba="0.35 0.35 0.4 1"/>
              <geom name="arm1_tip" type="sphere" size="0.018" pos="0.0 0 0.16" material="tip_mat"/>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- 机械臂2（右，底座 (0.60, 0)，红橙） -->
    <body name="arm2_base" pos="0.60 0 0.15">
      <geom name="arm2_base_g" type="cylinder" size="0.06 0.05" pos="0 0 0" rgba="0.35 0.35 0.4 1"/>
      <joint name="shoulder2" type="hinge" axis="0 0 1" pos="0 0 0"/>
      <body name="arm2_link1" pos="0 0 0">
        <geom name="arm2_link1_g" type="capsule" fromto="0 0 0 0.25 0 0" size="0.03" rgba="0.50 0.25 0.60 1"/>
        <joint name="elbow2" type="hinge" axis="0 0 1" pos="0.25 0 0"/>
        <body name="arm2_link2" pos="0.25 0 0">
          <geom name="arm2_link2_g" type="capsule" fromto="0 0 0 0.25 0 0" size="0.026" rgba="0.85 0.55 0.2 1"/>
          <body name="arm2_lift" pos="0.25 0 0">
            <joint name="lift2" type="slide" axis="0 0 1" range="-0.20 0.15"/>
            <geom name="arm2_lift_conn" type="cylinder" size="0.006 0.06" pos="0 0 0.28" rgba="0.45 0.45 0.5 1"/>
            <body name="arm2_gripper" pos="0 0 0.02">
              <joint name="grip2" type="hinge" axis="0 0 1" range="-0.55 0"/>
              <geom name="arm2_grip_base" type="box" size="0.012 0.018 0.012" pos="0.03 0 0.08" rgba="0.5 0.5 0.55 1"/>
              <geom name="arm2_finger1" type="box" size="0.02 0.005 0.02" pos="0.03 0.025 0.08" rgba="0.35 0.35 0.4 1"/>
              <geom name="arm2_finger2" type="box" size="0.02 0.005 0.02" pos="0.03 -0.025 0.08" rgba="0.35 0.35 0.4 1"/>
              <geom name="arm2_tip" type="sphere" size="0.018" pos="0.0 0 0.16" rgba="0.3 0.55 0.95 1"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="shoulder1_m" joint="shoulder1" ctrlrange="-30 30" gear="1"/>
    <motor name="elbow1_m" joint="elbow1" ctrlrange="-30 30" gear="1"/>
    <motor name="lift1_m" joint="lift1" ctrlrange="-50 50" gear="1"/>
    <motor name="shoulder2_m" joint="shoulder2" ctrlrange="-30 30" gear="1"/>
    <motor name="elbow2_m" joint="elbow2" ctrlrange="-30 30" gear="1"/>
    <motor name="lift2_m" joint="lift2" ctrlrange="-50 50" gear="1"/>
  </actuator>
</mujoco>
"""

L1 = 0.25
L2 = 0.25
ARM1_XY = (0.10, 0.0)    # 臂1 底座
ARM2_XY = (0.60, 0.0)    # 臂2 底座
PLANE_Z = 0.15           # 机械臂水平运动平面
GRIP_Z = 0.02            # 夹爪中心相对升降平面的高度
TIP_Z = 0.16             # 末端红球相对夹爪中心的高度（校准/轨迹用）

# 任务点（木板两端夹持点，世界坐标）
A_L = (0.15, 0.30)       # A 区木板左端
A_R = (0.55, 0.30)       # A 区木板右端
B_L = (0.15, -0.15)      # B 区木板左端
B_R = (0.55, -0.15)      # B 区木板右端
BOARD_Z_A = 0.045        # A 区木板中心高度（放地面）
BOARD_Z_B = 0.045        # B 区木板中心高度（放地面）
BOARD_C = (0.35, 0.30)   # A 区木板中心
BOARD_CB = (0.35, -0.15) # B 区木板中心

# 控制参数
FPS = 24
KP_H = 100.0
KD_H = 25.0


def ik2(x, y):
    d = math.hypot(x, y)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2),
                                       L1 + L2 * math.cos(q2))
    return q1, q2


def tip_pos(q1, q2, lift, base_x, base_y):
    """某臂末端（末端红球）世界坐标。"""
    ex = base_x + L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    ey = base_y + L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    ez = PLANE_Z + lift + GRIP_Z + TIP_Z
    return ex, ey, ez


def calibrate(model, data, renderer, cam):
    """DLT 相机标定：渲染多组姿态，用臂1 红色末端球的像素位置反解 3x4 投影矩阵。"""
    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid]

    a1, a2, al = jq("shoulder1"), jq("elbow1"), jq("lift1")
    pts = []
    targets = [(0.25, 0.25), (0.35, 0.20), (0.45, 0.25), (0.28, 0.05),
               (0.40, 0.05), (0.50, -0.05), (0.25, -0.10), (0.40, -0.12),
               (0.48, 0.15), (0.30, 0.15)]
    for i, (tx, ty) in enumerate(targets):
        lift = 0.08 if i % 2 == 0 else -0.02
        q = ik2(tx - ARM1_XY[0], ty - ARM1_XY[1])
        if q is None:
            continue
        data.qpos[a1] = q[0]
        data.qpos[a2] = q[1]
        data.qpos[al] = lift
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()
        r = rgb[:, :, 0].astype(int)
        g = rgb[:, :, 1].astype(int)
        b = rgb[:, :, 2].astype(int)
        mask = (r > 120) & (g < 60) & (b < 60)  # 红 tip（区分橙色连杆暗部）
        if mask.sum() < 5:
            continue
        ys, xs = np.nonzero(mask)
        px, py = float(xs.mean()), float(ys.mean())
        ex, ey, ez = tip_pos(q[0], q[1], lift, ARM1_XY[0], ARM1_XY[1])
        pts.append((ex, ey, ez, px, py))
        if len(pts) >= 10:
            break
    if len(pts) < 6:
        raise RuntimeError("标定点不足（%d 个），请检查末端球可见性" % len(pts))
    A, Bv = [], []
    for (x, y, z, px, py) in pts:
        A.append([x, y, z, 1, 0, 0, 0, 0, -px * x, -px * y, -px * z])
        Bv.append(px)
        A.append([0, 0, 0, 0, x, y, z, 1, -py * x, -py * y, -py * z])
        Bv.append(py)
    sol, _, _, _ = np.linalg.lstsq(np.array(A), np.array(Bv), rcond=None)
    P = np.zeros(12)
    P[:11] = sol
    P[11] = 1.0
    err = 0.0
    for (x, y, z, px, py) in pts:
        v = P.reshape(3, 4) @ np.array([x, y, z, 1.0])
        err += math.hypot(v[0] / v[2] - px, v[1] / v[2] - py)
    print("相机标定：%d 个点，平均重投影误差 %.2f px" % (len(pts), err / len(pts)))
    return P.reshape(3, 4)


def proj(P, x, y, z):
    v = P @ np.array([x, y, z, 1.0])
    return v[0] / v[2], v[1] / v[2]


def main():
    model = mujoco.MjModel.from_xml_string(ARM_XML)
    data = mujoco.MjData(model)

    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    s1_a, s1_v = jq("shoulder1")
    e1_a, e1_v = jq("elbow1")
    l1_a, l1_v = jq("lift1")
    g1_a, g1_v = jq("grip1")
    s2_a, s2_v = jq("shoulder2")
    e2_a, e2_v = jq("elbow2")
    l2_a, l2_v = jq("lift2")
    g2_a, g2_v = jq("grip2")
    board = jq("board_free")
    g1_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm1_gripper")
    g2_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm2_gripper")

    q0 = ik2(A_L[0] - ARM1_XY[0], A_L[1] - ARM1_XY[1])
    data.qpos[s1_a] = q0[0]
    data.qpos[e1_a] = q0[1]
    data.qpos[s2_a] = q0[0]
    data.qpos[e2_a] = q0[1]
    data.qpos[l1_a] = 0.08
    data.qpos[l2_a] = 0.08
    data.qpos[g1_a] = -0.4
    data.qpos[g2_a] = -0.4
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, 480, 640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array([0.35, 0.05, 0.08])
    cam.distance = 1.05
    cam.azimuth = 30
    cam.elevation = 25

    P = calibrate(model, data, renderer, cam)

    # ---- 任务调度（时间轴）----
    # 每段: (结束时间, 臂1x, 臂1y, 臂1lift, 臂1grip, 臂2x, 臂2y, 臂2lift, 臂2grip, 是否吸附)
    # 夹持点即木板两端（世界坐标）。下降时夹爪中心 z = 0.15 + lift + 0.02 = 0.045
    #   → lift = -0.125
    lift_up = 0.08
    lift_down = -0.125
    grip_open, grip_close = -0.4, 0.0
    ph = [
        (0.5, A_L[0], A_L[1], lift_down, grip_open,
              A_R[0], A_R[1], lift_down, grip_open, False),
        (1.0, A_L[0], A_L[1], lift_down, grip_close,
              A_R[0], A_R[1], lift_down, grip_close, True),
        (1.6, A_L[0], A_L[1], lift_up, grip_close,
              A_R[0], A_R[1], lift_up, grip_close, True),
        (3.0, B_L[0], B_L[1], lift_up, grip_close,
              B_R[0], B_R[1], lift_up, grip_close, True),
        (3.5, B_L[0], B_L[1], lift_down, grip_close,
              B_R[0], B_R[1], lift_down, grip_close, True),
        (3.9, B_L[0], B_L[1], lift_down, grip_open,
              B_R[0], B_R[1], lift_down, grip_open, False),
        (4.6, B_L[0], B_L[1], lift_up, grip_open,
              B_R[0], B_R[1], lift_up, grip_open, False),
    ]
    total = 4.6
    total_frames = int(total * FPS)

    print("Dual Arm：A 区木板 → B 区协同搬运，共 %d 帧（%.1f 秒）" %
          (total_frames, total))
    frames = []
    traj1, traj2 = [], []
    attached = False
    placed = False
    phase_i = 0

    def set_board(cx, cy, z, yaw):
        qadr, dofadr = board
        data.qpos[qadr:qadr + 3] = [cx, cy, z]
        data.qpos[qadr + 3:qadr + 7] = [math.cos(yaw / 2.0), 0.0, 0.0,
                                        math.sin(yaw / 2.0)]
        data.qvel[dofadr:dofadr + 6] = 0.0

    set_board(BOARD_C[0], BOARD_C[1], BOARD_Z_A, 0.0)

    for i in range(total_frames):
        t = i / FPS
        while phase_i < len(ph) and t >= ph[phase_i][0]:
            phase_i += 1
        seg = ph[min(phase_i, len(ph) - 1)]
        (x1, y1, l1, gr1, x2, y2, l2, gr2, ga) = seg[1:10]

        q1 = ik2(x1 - ARM1_XY[0], y1 - ARM1_XY[1])
        q2 = ik2(x2 - ARM2_XY[0], y2 - ARM2_XY[1])
        if q1 is None or q2 is None:
            print("目标不可达:", (x1, y1), (x2, y2))
            return
        data.ctrl[0] = KP_H * (q1[0] - data.qpos[s1_a]) - KD_H * data.qvel[s1_v]
        data.ctrl[1] = KP_H * (q1[1] - data.qpos[e1_a]) - KD_H * data.qvel[e1_v]
        data.ctrl[2] = 0.0
        data.ctrl[3] = KP_H * (q2[0] - data.qpos[s2_a]) - KD_H * data.qvel[s2_v]
        data.ctrl[4] = KP_H * (q2[1] - data.qpos[e2_a]) - KD_H * data.qvel[e2_v]
        data.ctrl[5] = 0.0

        # 吸附：两臂都降到木板中心高度（误差<6mm）且都闭合后才吸附
        if ga:
            z1 = data.xpos[g1_body][2]
            z2 = data.xpos[g2_body][2]
            if (not attached and abs(z1 - BOARD_Z_A) < 0.006
                    and abs(z2 - BOARD_Z_A) < 0.006
                    and abs(data.qpos[g1_a]) < 0.05
                    and abs(data.qpos[g2_a]) < 0.05):
                attached = True
        else:
            if attached:
                attached = False
                placed = True

        # 更新木板位姿
        if attached:
            p1 = data.xpos[g1_body]
            p2 = data.xpos[g2_body]
            mx, my = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
            mz = (p1[2] + p2[2]) / 2.0
            myaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            set_board(mx, my, mz, myaw)
        elif placed:
            set_board(BOARD_CB[0], BOARD_CB[1], BOARD_Z_B, 0.0)
        else:
            set_board(BOARD_C[0], BOARD_C[1], BOARD_Z_A, 0.0)

        for _ in range(3):
            mujoco.mj_step(model, data)
        # 升降/夹爪直接位置过渡
        data.qvel[l1_v] = 0.0
        data.qpos[l1_a] += (l1 - data.qpos[l1_a]) * 0.35
        data.qvel[g1_v] = 0.0
        data.qpos[g1_a] += (gr1 - data.qpos[g1_a]) * 0.25
        data.qvel[l2_v] = 0.0
        data.qpos[l2_a] += (l2 - data.qpos[l2_a]) * 0.35
        data.qvel[g2_v] = 0.0
        data.qpos[g2_a] += (gr2 - data.qpos[g2_a]) * 0.25
        mujoco.mj_forward(model, data)

        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        e1x, e1y, e1z = tip_pos(data.qpos[s1_a], data.qpos[e1_a],
                                data.qpos[l1_a], ARM1_XY[0], ARM1_XY[1])
        e2x, e2y, e2z = tip_pos(data.qpos[s2_a], data.qpos[e2_a],
                                data.qpos[l2_a], ARM2_XY[0], ARM2_XY[1])
        traj1.append(proj(P, e1x, e1y, e1z))
        traj2.append(proj(P, e2x, e2y, e2z))

        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)
        if len(traj1) >= 2:
            draw.line(traj1, fill=(255, 200, 0), width=4)
        if len(traj2) >= 2:
            draw.line(traj2, fill=(255, 200, 0), width=4)
        frames.append(im)

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    print("渲染完成，合成 GIF...")
    frames[0].save(OUT_GIF, save_all=True, append_images=frames[1:],
                   duration=int(1000 / FPS), loop=0)
    print("已生成效果图：%s（%d 帧，%.1f 秒）" % (OUT_GIF, len(frames),
                                            len(frames) / FPS))
    qadr, _ = board
    print("  木板最终位置: (%.3f, %.3f, %.3f)" %
          (data.qpos[qadr], data.qpos[qadr + 1], data.qpos[qadr + 2]))


if __name__ == "__main__":
    main()
