# -*- coding: utf-8 -*-
"""MuJoCo 二连杆机械臂抓取搬运（Pick & Place）

功能：机械臂末端（带垂直升降关节与夹爪）自动完成：
移动到球上方 → 下降 → 夹取（逻辑吸附）→ 抬升 → 移动到目标点 B → 下降 → 释放。

相机为斜视角 3D，用 DLT 校准把末端 3D 位置投影到像素，绘制黄色搬运轨迹。

运行（虚拟机 / 无显卡环境）：
    pip3 install --user mujoco pillow numpy
    sudo apt install -y libosmesa6-dev
    MUJOCO_GL=osmesa python3 mujoco_arm_pick.py

输出：同目录 demo_pick.gif
"""
import os, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_gripper.xml")
OUT_GIF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_pick.gif")

L1 = 0.30
L2 = 0.30
PLANE_Z = 0.15           # 机械臂水平运动平面（抬高，避开球与托盘）
GRIP_Z = 0.02            # 夹爪中心相对升降平面的高度
TIP_Z = 0.11             # 末端红球相对夹爪中心的高度（校准/轨迹用）

# 任务点
PICK = (0.18, 0.20)      # 取球点 A
PLACE = (0.45, -0.15)    # 放球点 B
BALL_Z = 0.08            # 球心高度（A/B 托盘柱顶）

# 控制参数
FPS = 24
KP_H = 100.0             # 水平关节 PD（快速到达目标）
KD_H = 25.0
KP_L = 200.0             # 升降关节 PD
KD_L = 20.0


def ik2(x, y):
    d = math.hypot(x, y)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2),
                                       L1 + L2 * math.cos(q2))
    return q1, q2


def tip_pos(q1, q2, lift):
    """末端（末端红球）世界坐标。"""
    ex = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    ey = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    ez = PLANE_Z + lift + GRIP_Z + TIP_Z
    return ex, ey, ez


def calibrate(model, data, renderer, cam):
    """DLT 相机标定：渲染多组姿态，用红色末端球的像素位置反解 3x4 投影矩阵。"""
    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid]

    a1, a2, al = jq("shoulder"), jq("elbow"), jq("lift_j")
    pts = []
    # 标定姿态：末端在工作区（画面内）的 10 组，含不同升降高度
    targets = [(0.22, 0.10), (0.30, 0.12), (0.38, 0.08), (0.26, 0.02),
               (0.34, 0.02), (0.42, -0.02), (0.24, -0.06), (0.32, -0.06),
               (0.40, -0.10), (0.30, -0.04)]
    for i, (tx, ty) in enumerate(targets):
        lift = 0.08 if i % 2 == 0 else -0.02
        q = ik2(tx, ty)
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
        mask = (r > 200) & (g < 90) & (b < 90)  # 纯红 tip（link2 已改橙黄排除）
        if mask.sum() < 5:
            continue
        ys, xs = np.nonzero(mask)
        px, py = float(xs.mean()), float(ys.mean())
        ex, ey, ez = tip_pos(q[0], q[1], lift)
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
    # 重投影误差
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
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    # 关节索引：qpos 与 qvel 布局不同（free joint 四元数），分别取偏移
    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    q1_a, q1_v = jq("shoulder")
    q2_a, q2_v = jq("elbow")
    lift_a, lift_v = jq("lift_j")
    grip_a, _ = jq("grip_j")
    ball_qadr = jq("ball_free")[0]
    ball_dofadr = jq("ball_free")[1]
    gripper_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")

    # 初始姿态：末端已在 A 点上方（ik 姿态），夹爪张开、高位
    q0 = ik2(*PICK)
    data.qpos[q1_a] = q0[0]
    data.qpos[q2_a] = q0[1]
    data.qpos[lift_a] = 0.08
    data.qpos[grip_a] = -0.4            # 夹爪张开
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, 480, 640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array([0.3, 0.05, 0.08])
    cam.distance = 0.8
    cam.azimuth = 30
    cam.elevation = -55

    # ---- 相机标定 ----
    P = calibrate(model, data, renderer, cam)

    # ---- 任务调度（时间轴）----
    # 每段: (结束时间, 目标x, 目标y, 目标lift, 目标grip, 是否吸附球)
    # 球心在托盘柱顶 z=0.08；夹爪中心 z = 0.15 + lift + 0.02，
    # 抓取/放下时 lift=-0.09（夹爪中心 z=0.08），高位 lift=0.08（z=0.25）
    lift_low, lift_high = -0.09, 0.08
    grip_open, grip_close = -0.4, 0.0
    phases = [
        (0.5, PICK[0], PICK[1], lift_high, grip_open, False),   # 保持就位
        (1.0, PICK[0], PICK[1], lift_low, grip_open, False),    # 下降到球心高度
        (1.5, PICK[0], PICK[1], lift_low, grip_close, True),    # 闭合夹爪并吸附（接近球时生效）
        (2.2, PICK[0], PICK[1], lift_high, grip_close, True),   # 抬升
        (5.4, PLACE[0], PLACE[1], lift_high, grip_close, True), # 移到 B 上方
        (7.0, PLACE[0], PLACE[1], lift_low, grip_close, True),  # 下降到 B（留足时间）
        (7.2, PLACE[0], PLACE[1], lift_low, grip_open, False),  # 张开夹爪释放
        (7.8, PLACE[0], PLACE[1], lift_low, grip_open, False),  # 保持低位让球落稳
        (8.4, PLACE[0], PLACE[1], lift_high, grip_open, False), # 收回
    ]
    total = 8.4
    total_frames = int(total * FPS)

    print("Pick & Place：A(%s) → B(%s)，共 %d 帧（%.1f 秒）" %
          (PICK, PLACE, total_frames, total))
    frames = []
    traj = []
    attached = False
    released = False
    phase_i = 0

    for i in range(total_frames):
        t = i / FPS
        while phase_i < len(phases) and t >= phases[phase_i][0]:
            phase_i += 1
        # 当前目标（取最近一个已开始的阶段）
        gx, gy, gl, gg, ga = phases[min(phase_i, len(phases) - 1)][1:6]

        q = ik2(gx, gy)
        if q is None:
            print("目标不可达:", gx, gy)
            return
        data.ctrl[0] = KP_H * (q[0] - data.qpos[q1_a]) - KD_H * data.qvel[q1_v]
        data.ctrl[1] = KP_H * (q[1] - data.qpos[q2_a]) - KD_H * data.qvel[q2_v]
        data.ctrl[2] = KP_L * (gl - data.qpos[lift_a]) - KD_L * data.qvel[lift_v]

        # 吸附球：夹爪降到球心高度附近时吸附（几何与球无接触，无反弹）
        if ga:
            gripper_z = data.xpos[gripper_body][2]
            if not attached and abs(gripper_z - (BALL_Z + 0.02)) < 0.02:
                attached = True
            if attached:
                ball_pos = data.xpos[gripper_body].copy()
            else:
                ball_pos = np.array([PICK[0], PICK[1], BALL_Z])
        else:
            if attached:
                attached = False
                released = True
            ball_pos = (np.array([PLACE[0], PLACE[1], BALL_Z])
                        if released else np.array([PICK[0], PICK[1], BALL_Z]))
        data.qpos[ball_qadr:ball_qadr + 3] = ball_pos
        data.qpos[ball_qadr + 3:ball_qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[ball_dofadr:ball_dofadr + 6] = 0.0

        for _ in range(3):
            mujoco.mj_step(model, data)
        # 夹爪角度（视觉直接设定，缓慢过渡）
        data.qpos[grip_a] += (gg - data.qpos[grip_a]) * 0.25
        mujoco.mj_forward(model, data)

        # 渲染
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        # 末端轨迹（夹爪中心 3D → 像素）
        ex, ey, ez = tip_pos(data.qpos[q1_a], data.qpos[q2_a], data.qpos[lift_a])
        traj.append(proj(P, ex, ey, ez))

        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)
        if len(traj) >= 2:
            draw.line(traj, fill=(255, 200, 0), width=4)
        frames.append(im)

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    print("渲染完成，合成 GIF...")
    frames[0].save(OUT_GIF, save_all=True, append_images=frames[1:],
                   duration=int(1000 / FPS), loop=0)
    print("已生成效果图：%s（%d 帧，%.1f 秒）" % (OUT_GIF, len(frames),
                                            len(frames) / FPS))


if __name__ == "__main__":
    main()
