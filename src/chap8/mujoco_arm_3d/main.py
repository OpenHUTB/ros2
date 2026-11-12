#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 三自由度机械臂空间螺旋线仿真（单文件，XML 内嵌）

功能：三自由度机械臂（肩方位 yaw + 肩俯仰 shoulder + 肘俯仰 elbow）的末端
在 3D 空间中沿圆柱螺旋线运动（水平画圆 + 垂直匀速上升），
用解析空间 IK 实时解算三个关节角，逐帧渲染并合成动态效果图。

运行（有桌面 / 有 GL 的机器）：
    pip install mujoco pillow numpy
    python3 main.py

运行（无显示器的虚拟机，推荐 egl；osmesa 在部分 VMware 上会崩，见 README）：
    MUJOCO_GL=egl python3 main.py

输出：默认写到仓库内 docs/img/mujoco_arm_3d_demo.gif（文档站配图，只此一份，不另存 demo_arm3d.gif）；
      不在仓库结构里（例如被 catkin 安装到 lib/ 之下）自动退到当前工作目录，
      也可以显式指定：python3 main.py --out ./mujoco_arm_3d_demo.gif
"""
import os, math, sys

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

# ============ 模型（XML 内嵌） ============
MODEL_XML = r"""
<mujoco model="arm3d">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.6 0.6 0.6" diffuse="0.4 0.4 0.4"/>
  </visual>
  <worldbody>
    <light name="l1" diffuse="0.75 0.75 0.75" pos="0.6 0.6 1.4" dir="-0.4 -0.4 -1" castshadow="false"/>
    <light name="l2" diffuse="0.35 0.35 0.4" pos="-0.8 -0.6 1.2" dir="0.6 0.4 -1" castshadow="false"/>
    <geom name="floor" type="plane" size="1.2 1.2 0.02" rgba="0.88 0.88 0.9 1"/>
    <!-- 参考圆：螺旋线在地面的水平投影，半径与 RADIUS=0.200 一致（16 个绿点，每 22.5°）。
         这些 pos 是按 RADIUS 手算写死的常量：改动 RADIUS（本文件顶部）或轨迹半径时，必须同步改下面 16 行，
         否则地面绿点会和实际轨迹悄悄对不上，图上看不出来但对比就会露馅。 -->
    <body name="ref" pos="0 0 0.001">
      <geom name="rc0"  type="sphere" size="0.012" pos="0.200 0.000 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc1"  type="sphere" size="0.012" pos="0.185 0.077 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc2"  type="sphere" size="0.012" pos="0.141 0.141 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc3"  type="sphere" size="0.012" pos="0.077 0.185 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc4"  type="sphere" size="0.012" pos="0.000 0.200 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc5"  type="sphere" size="0.012" pos="-0.077 0.185 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc6"  type="sphere" size="0.012" pos="-0.141 0.141 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc7"  type="sphere" size="0.012" pos="-0.185 0.077 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc8"  type="sphere" size="0.012" pos="-0.200 0.000 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc9"  type="sphere" size="0.012" pos="-0.185 -0.077 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc10" type="sphere" size="0.012" pos="-0.141 -0.141 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc11" type="sphere" size="0.012" pos="-0.077 -0.185 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc12" type="sphere" size="0.012" pos="0.000 -0.200 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc13" type="sphere" size="0.012" pos="0.077 -0.185 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc14" type="sphere" size="0.012" pos="0.141 -0.141 0" rgba="0.1 0.8 0.3 1"/>
      <geom name="rc15" type="sphere" size="0.012" pos="0.185 -0.077 0" rgba="0.1 0.8 0.3 1"/>
    </body>
    <!-- 底座 + 三自由度机械臂 -->
    <body name="base" pos="0 0 0">
      <geom name="basegeom" type="cylinder" size="0.07 0.10" pos="0 0 0.10" rgba="0.32 0.32 0.38 1"/>
      <geom name="pillar"  type="cylinder" size="0.035 0.035" pos="0 0 0.205" rgba="0.45 0.45 0.5 1"/>
      <joint name="yaw" type="hinge" axis="0 0 1" pos="0 0 0.24" range="-12.566371 12.566371"/>
      <body name="l1" pos="0 0 0.24">
        <geom name="link1" type="capsule" fromto="0 0 0 0.30 0 0" size="0.035" rgba="0.2 0.42 0.92 1"/>
        <joint name="shoulder" type="hinge" axis="0 -1 0" range="-3.141593 3.141593"/>
        <body name="l2" pos="0.30 0 0">
          <geom name="link2" type="capsule" fromto="0 0 0 0.30 0 0" size="0.028" rgba="0.95 0.55 0.12 1"/>
          <joint name="elbow" type="hinge" axis="0 -1 0" range="-3.141593 3.141593"/>
          <body name="tip" pos="0.30 0 0">
            <geom name="tipball" type="sphere" size="0.042" rgba="0.85 0.05 0.85 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m_yaw" joint="yaw" ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_shoulder" joint="shoulder" ctrlrange="-30 30" ctrllimited="true"/>
    <motor name="m_elbow" joint="elbow" ctrlrange="-30 30" ctrllimited="true"/>
  </actuator>
</mujoco>
"""

# ============ 参数 ============
L1 = 0.30                # 第一段杆长（m）
L2 = 0.30                # 第二段杆长（m）
BASE_Z = 0.24            # 肩俯仰铰点高度（m，与 XML 一致）

# 螺旋线轨迹（世界坐标）
RADIUS = 0.20            # 螺旋半径（m）
Z_LO = 0.32              # 螺旋起点高度（m）
Z_HI = 0.56              # 螺旋终点高度（m）
TURNS = 1.5              # 圈数
PERIOD = 6.0             # 每圈秒数
DURATION = 9.0           # 总时长（s）

# 渲染与控制
FPS = 24
PHYS_DT = 0.002          # 与 XML <option timestep> 一致
# PD 对照路径的物理子步数：1/FPS = 41.7 ms 需 ≈ 21 个 2 ms 子步，物理时间才能跟轨迹时间对齐，
# 否则机械臂实际速度只有轨迹速度的约 14%，末端会全程"追不上"螺旋线。
# 注意：默认的运动学驱动（下面 USE_PD=False 分支）不调用 mj_step，这个数不参与默认路径。
SUBSTEPS = int(round((1.0 / FPS) / PHYS_DT))
# 默认运动学驱动（末端严格贴合轨迹）；设 ARM3D_PD=1 可切回 PD 位置控制做对照。
# 注意：PD 路径是**反面示例**——2 ms 步长下带宽不足，五组增益（KP/KD=40/12 … 400/80）
#     末端误差仍有 100~540 mm、肘关节来回震荡，只用来演示「为什么不推荐这条路」。
#     不要照抄这里的 KP/KD 去接别的模型。
USE_PD = os.environ.get("ARM3D_PD", "") == "1"
KP = float(os.environ.get("ARM3D_KP", 40.0))
KD = float(os.environ.get("ARM3D_KD", 12.0))
WIDTH, HEIGHT = 640, 480


def ik3(x, y, z):
    """三自由度空间逆运动学（yaw + shoulder + elbow），返回 (q0, q1, q2) 或 None。

    几何：先由水平投影求方位角 q0；再在臂平面（r-z）内用 2 连杆解析解求 q1、q2。
    模型约定：shoulder/elbow 绕 -Y 轴，正角度使连杆向 +Z 方向（向上）弯折。
    """
    r = math.hypot(x, y)
    dz = z - BASE_Z
    d = math.hypot(r, dz)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))          # 肘上姿势
    q1 = math.atan2(dz, r) - math.atan2(L2 * math.sin(q2),
                                        L1 + L2 * math.cos(q2))
    q0 = math.atan2(y, x)
    return q0, q1, q2


def fk3(q0, q1, q2):
    """三自由度正运动学：关节角 -> 末端世界坐标（与 ik3 互查用）。"""
    x_arm = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    z_arm = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    x = x_arm * math.cos(q0)
    y = x_arm * math.sin(q0)
    z = BASE_Z + z_arm
    return x, y, z


def calibrate(model, data, renderer, cam, qidx):
    """DLT 相机标定：渲染多组姿态，用红色末端球的像素位置反解 3x4 投影矩阵。

    手算 lookAt 投影与 MuJoCo 实际相机（azimuth/elevation 约定、fovy 换算）容易有偏差，
    标定后黄线才能和真实末端像素严格重合。
    """
    qa_y, qa_s, qa_e = qidx
    # 标定点在三维空间里分散分布（不同半径 + 不同高度 + 不同方位），
    # 只沿螺旋取点会近似共面，DLT 外推会失效。
    # 高度覆盖必须盖住整条轨迹（螺旋 z = 0.32→0.56 m），否则上段属于外推区，
    # 残差会明显变大（y 方向尤甚）。
    cands = []
    layers = [(0.14, 0.10), (0.24, 0.22), (0.34, 0.26), (0.44, 0.20), (0.54, 0.16)]
    for li, (z, r) in enumerate(layers):
        for k in range(3):
            ang = 2.0 * math.pi * (k / 3.0) + li * 0.7
            cands.append((r * math.cos(ang), r * math.sin(ang), z))
    pts = []
    for (x, y, z) in cands:
        q = ik3(x, y, z)
        if q is None:
            continue
        data.qpos[qa_y] = q[0]
        data.qpos[qa_s] = q[1]
        data.qpos[qa_e] = q[2]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()
        r = rgb[:, :, 0].astype(int)
        gg = rgb[:, :, 1].astype(int)
        bb = rgb[:, :, 2].astype(int)
        # 品红是画面里唯一「红、蓝都明显高于绿」的颜色，用相对判据避开光照冲淡
        mask = (r > 110) & (r - gg > 40) & (bb - gg > 40)
        if mask.sum() < 40:
            continue
        ys, xs = np.nonzero(mask)
        pts.append((x, y, z, float(xs.mean()), float(ys.mean())))

    # 候选点比需要的多时，沿序列均匀抽样，保证 z 方向铺满整条轨迹，
    # 不能只取前 10 个（那样 z 只覆盖到 0.34，螺旋上段全是外推）。
    if len(pts) > 10:
        idx = sorted({int(round(k * (len(pts) - 1) / 9.0)) for k in range(10)})
        pts = [pts[k] for k in idx]
    if len(pts) < 6:
        raise RuntimeError("标定点不足（%d 个），请检查末端球是否可见" % len(pts))

    def dlt(ps):
        A, Bv = [], []
        for (x, y, z, px, py) in ps:
            A.append([x, y, z, 1, 0, 0, 0, 0, -px * x, -px * y, -px * z])
            Bv.append(px)
            A.append([0, 0, 0, 0, x, y, z, 1, -py * x, -py * y, -py * z])
            Bv.append(py)
        sol, _, _, _ = np.linalg.lstsq(np.array(A), np.array(Bv), rcond=None)
        P = np.zeros(12)
        P[:11] = sol
        P[11] = 1.0
        return P.reshape(3, 4)

    Pm = dlt(pts)
    # 剔除被遮挡/误判的离群点后重解一次
    keep = []
    for (x, y, z, px, py) in pts:
        v = Pm @ np.array([x, y, z, 1.0])
        if math.hypot(v[0] / v[2] - px, v[1] / v[2] - py) < 8.0:
            keep.append((x, y, z, px, py))
    if len(keep) >= 6 and len(keep) < len(pts):
        Pm = dlt(keep)
    err = 0.0
    for (x, y, z, px, py) in keep:
        v = Pm @ np.array([x, y, z, 1.0])
        err += math.hypot(v[0] / v[2] - px, v[1] / v[2] - py)
    print("相机标定：%d/%d 个点有效，平均重投影误差 %.2f px"
          % (len(keep), len(pts), err / max(1, len(keep))))
    # 标定完把姿态归零，避免影响后续控制起点
    data.qpos[qa_y] = data.qpos[qa_s] = data.qpos[qa_e] = 0.0
    mujoco.mj_forward(model, data)
    return Pm


def proj(P, x, y, z):
    v = P @ np.array([x, y, z, 1.0])
    return v[0] / v[2], v[1] / v[2]


def resolve_out(argv):
    """决定 GIF 输出路径：--out > 仓库内 docs/img > 当前工作目录。

    在仓库里跑时直接更新文档站配图（只此一份）；被 catkin / 安装到别处时，
    `../../../docs/img` 可能落到不可写目录（如 /home/docs），所以先判断是否真在仓库结构里，
    不满足就退到当前目录，避免硬编码路径导致 PermissionError。
    """
    for i, a in enumerate(argv):
        if a == "--out" and i + 1 < len(argv):
            return os.path.abspath(argv[i + 1]), "--out 指定"

    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))
    img = os.path.join(repo, "docs", "img")
    if os.path.isfile(os.path.join(repo, "mkdocs.yml")) and os.path.isdir(img) and os.access(img, os.W_OK):
        return os.path.join(img, "mujoco_arm_3d_demo.gif"), "仓库内 docs/img（文档站配图）"

    return os.path.join(os.getcwd(), "mujoco_arm_3d_demo.gif"), "当前工作目录（不在仓库结构里，可用 --out 指定）"


def main():
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)

    # 关节索引（按名字查）
    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name.encode())
        if jid < 0:
            raise RuntimeError('joint %s not found' % name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    qa_y, va_y = jq('yaw')
    qa_s, va_s = jq('shoulder')
    qa_e, va_e = jq('elbow')
    tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b'tip')

    # 渲染器 + 斜视角自由相机
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array([0.0, 0.0, 0.46])
    cam.distance = 1.70
    cam.azimuth = 50.0
    cam.elevation = 22.0

    # DLT 相机标定：黄线像素与真实末端严格对齐
    P = calibrate(model, data, renderer, cam, (qa_y, qa_s, qa_e))

    frames = []
    traj = []                      # 末端轨迹点（像素）
    errs = []                      # 末端与目标的逐帧跟随误差（m）
    al = []                        # 投影点与画面球心的逐帧像素残差 (dx, dy)
    q_prev = None                  # 上一帧目标关节角（用于 yaw 解缠）
    total_frames = int(DURATION * FPS)

    if USE_PD:
        print("三自由度机械臂空间螺旋线：半径 %.2f m，高度 %.2f→%.2f m，%g 圈，共 %d 帧，"
              "PD 对照路径，每帧 %d 个 %g ms 物理子步" %
              (RADIUS, Z_LO, Z_HI, TURNS, total_frames, SUBSTEPS, PHYS_DT * 1000))
    else:
        print("三自由度机械臂空间螺旋线：半径 %.2f m，高度 %.2f→%.2f m，%g 圈，共 %d 帧，"
              "运动学驱动（IK 解直写 qpos + mj_forward，不做物理积分）" %
              (RADIUS, Z_LO, Z_HI, TURNS, total_frames))
    import time
    t0 = time.time()

    for i in range(total_frames):
        t = i / FPS
        ang = 2.0 * math.pi * (t % PERIOD) / PERIOD
        # 螺旋：水平圆 + 垂直匀速上升（并伴随半径轻微变化，轨迹更立体）
        frac = t / DURATION
        tx = RADIUS * math.cos(ang)
        ty = RADIUS * math.sin(ang)
        tz = Z_LO + (Z_HI - Z_LO) * frac

        q = ik3(tx, ty, tz)
        if q is None:
            print("  [warn] 目标点在工作空间外：x=%.2f y=%.2f z=%.2f" % (tx, ty, tz))
            continue
        # yaw 解缠：螺旋 1.5 圈时 atan2 会在 ±pi 处跳变，需保持关节角连续
        q0 = q[0]
        if q_prev is not None:
            while q0 - q_prev[0] > math.pi:
                q0 -= 2.0 * math.pi
            while q0 - q_prev[0] < -math.pi:
                q0 += 2.0 * math.pi
        q = (q0, q[1], q[2])
        q_prev = q

        if USE_PD:
            # PD 位置控制（三关节）：保留的对照路径，默认不启用
            data.ctrl[0] = KP * (q[0] - data.qpos[qa_y]) - KD * data.qvel[va_y]
            data.ctrl[1] = KP * (q[1] - data.qpos[qa_s]) - KD * data.qvel[va_s]
            data.ctrl[2] = KP * (q[2] - data.qpos[qa_e]) - KD * data.qvel[va_e]
            for _ in range(SUBSTEPS):
                mujoco.mj_step(model, data)
        else:
            # 运动学驱动：把 IK 解直接写进 qpos 再 mj_forward，末端严格落在螺旋线上。
            # PD 在 2 ms 步长下带宽不足（实测各增益下末端误差仍有 100~540 mm，肘关节来回震荡），
            # 所以演示默认用运动学驱动，保证「末端沿螺旋线运动」与画面一致。
            data.qpos[qa_y] = q[0]
            data.qpos[qa_s] = q[1]
            data.qpos[qa_e] = q[2]
            data.qvel[va_y] = data.qvel[va_s] = data.qvel[va_e] = 0.0
            mujoco.mj_forward(model, data)

        # 末端跟随误差：实际末端 tip 与当前目标螺旋点的距离，逐帧记录
        p_tip = data.xpos[tip_id]
        errs.append(float(np.linalg.norm(np.array(
            [p_tip[0] - tx, p_tip[1] - ty, p_tip[2] - tz]))))

        # 渲染当前帧
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        # 实际末端 3D -> 像素（DLT 标定的投影矩阵，与渲染相机一致）
        px, py = proj(P, p_tip[0], p_tip[1], p_tip[2])
        traj.append((int(px), int(py)))

        # 投影对齐自检：逐帧把 DLT 投影点与画面里末端球的像素质心对比。
        # 画面里只有这一个品红球，球心即像素质心；球被遮住或改颜色会检测失败（届时本项跳过）。
        rr = rgb[:, :, 0].astype(int)
        g2 = rgb[:, :, 1].astype(int)
        b2 = rgb[:, :, 2].astype(int)
        m2 = (rr > 110) & (rr - g2 > 40) & (b2 - g2 > 40)
        if m2.sum() > 10:
            ys2, xs2 = np.nonzero(m2)
            al.append((px - float(xs2.mean()), py - float(ys2.mean())))

        # 叠加黄色轨迹线
        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)
        if len(traj) >= 2:
            for k in range(1, len(traj)):
                draw.line([traj[k - 1], traj[k]], fill=(255, 200, 0), width=4)
        frames.append(im)

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    print("渲染完成，耗时 %.1f 秒，合成 GIF..." % (time.time() - t0))
    if errs:
        print("末端跟随误差：首帧 %.1f mm，末帧 %.1f mm，最大 %.1f mm"
              "（构造性自检：qpos 直接取 IK 解、mj_forward 只做正解，与 IK 输入同源，非独立验证）"
              % (errs[0] * 1000.0, errs[-1] * 1000.0, max(errs) * 1000.0))
    if al:
        ax = np.array([a[0] for a in al])
        ay = np.array([a[1] for a in al])
        print("投影对齐残差（%d 帧全帧）：x 平均 %.2f / 最大 %.2f px，y 平均 %.2f / 最大 %.2f px"
              % (len(al), abs(ax).mean(), abs(ax).max(), abs(ay).mean(), abs(ay).max()))
        print("末帧对齐：dx %.2f px，dy %.2f px，偏差 %.2f px"
              % (al[-1][0], al[-1][1], math.hypot(al[-1][0], al[-1][1])))
    # 输出路径：--out 显式指定 > 仓库内 docs/img > 当前工作目录（catkin / 安装环境都能跑）
    out, why = resolve_out(sys.argv[1:])
    print("输出位置：%s（%s）" % (out, why))
    try:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    except OSError as e:
        print("无法创建输出目录：%s" % e)
        print("请用 --out 指定一个可写路径，例如：python3 main.py --out ./mujoco_arm_3d_demo.gif")
        sys.exit(1)
    if frames:
        # GIF 时间轴精度只有 10 ms：24 fps 的 41.7 ms 会被写成 40 ms（4 centiseconds），
        # 所以实际播放时长 = 帧数 × 40 ms = 8.64 s，比按帧率算的名义 9.0 s 短。
        delay_ms = max(10, int((1000.0 / FPS) / 10) * 10)
        try:
            frames[0].save(out, save_all=True, append_images=frames[1:],
                           duration=delay_ms, loop=0)
        except OSError as e:
            print("写入失败：%s" % e)
            print("请用 --out 指定一个可写路径，例如：python3 main.py --out ./mujoco_arm_3d_demo.gif")
            sys.exit(1)
        size_mb = os.path.getsize(out) / 1024.0 / 1024.0
        print("已生成效果图：%s（%d 帧，每帧 %d ms，实际 %.2f s，名义 %.1f s，%.2f MB）"
              % (out, len(frames), delay_ms, len(frames) * delay_ms / 1000.0,
                 len(frames) / FPS, size_mb))
    else:
        print("未生成任何帧！请检查模型与渲染后端。")
        sys.exit(1)


if __name__ == "__main__":
    main()
