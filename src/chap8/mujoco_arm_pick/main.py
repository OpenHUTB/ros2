#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 二连杆机械臂抓取搬运（Pick & Place）

功能：机械臂末端（带垂直升降关节与夹爪）自动完成：
移动到球上方 → 下降 → 夹取（逻辑吸附）→ 抬升 → 移动到目标点 B → 下降 → 释放。

相机为斜视角 3D，用 DLT 校准把末端 3D 位置投影到像素，绘制黄色搬运轨迹。

运行（虚拟机 / 无显卡环境）：
    pip3 install --user mujoco pillow numpy
    sudo apt install -y libosmesa6-dev
    MUJOCO_GL=osmesa python3 main.py
    python3 main.py --out ./demo_pick.gif     # 指定输出路径

输出：--out 指定 > 仓库 docs/img > 系统临时目录（不往源码树里写）
"""
import math, os, sys, tempfile

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "arm_gripper.xml")


def resolve_out(argv):
    """输出路径：--out > 仓库 docs/img > 系统临时目录（不往源码树里写）。"""
    for i, a in enumerate(argv):
        if a == "--out" and i + 1 < len(argv):
            p = os.path.abspath(argv[i + 1])
            d = os.path.dirname(p) or "."
            if not os.path.isdir(d) or not os.access(d, os.W_OK):
                print("无法写入输出目录：%s" % d)
                print("请用 --out 指定一个可写路径，例如：python3 main.py --out ./demo_pick.gif")
                sys.exit(1)
            return p, "--out 指定"
    root = os.path.dirname(os.path.dirname(HERE))
    img = os.path.join(root, "docs", "img")
    if (os.path.isfile(os.path.join(root, "mkdocs.yml"))
            and os.path.isdir(img) and os.access(img, os.W_OK)):
        return os.path.join(img, "mujoco_arm_pick_demo.gif"), "仓库 docs/img"
    return os.path.join(tempfile.gettempdir(), "mujoco_arm_pick_demo.gif"), "系统临时目录"

L1 = 0.30
L2 = 0.30
PLANE_Z = 0.15           # 机械臂水平运动平面（抬高，避开球与托盘）
GRIP_Z = 0.02            # 夹爪中心相对升降平面的高度
TIP_Z = 0.06             # 末端红球相对夹爪中心的高度（校准/轨迹用）

# 任务点
PICK = (0.18, 0.20)      # 取球点 A
PLACE = (0.45, -0.15)    # 放球点 B
BALL_Z = 0.08            # 球心高度（A/B 托盘柱顶）

# 控制参数
FPS = 24
GIF_MS = 40              # GIF 每帧时长（ms）；GIF 时间精度只有 10 ms，24 fps 的 41 ms 会被截成 40
# 关节用「每帧按固定比例逼近目标」的一阶插值驱动（运动学），不做物理积分。
# 原因：24 fps 的采样周期是 41.7 ms，而本模型的杆件惯量很小，纯 PD 在这个采样率下
# 要么欠采样震荡（实测肘关节来回甩到 -1.5 rad），要么为了稳定把增益压到 0.5 s 才到位；
# 本项目要演示的是 IK 解算 + 九段时间轴调度 + 抓取搬运流程，不是力矩控制，
# 用确定性插值反而让「什么时候到球心、什么时候到 B 点」可复现、可核对。
ALPHA_H = 0.10           # 水平关节每帧逼近比例（76 帧后到 99.9%）
ALPHA_L = 0.25           # 升降关节每帧逼近比例（12 帧后到 96.8%）
ALPHA_G = 0.25           # 夹爪开合每帧逼近比例


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
    """DLT 相机标定：渲染多组姿态，用红色末端球的像素位置反解 3x4 投影矩阵。

    标定会临时改写 qpos/qvel，返回前还原并 mj_forward —— 否则正式动画的第一帧
    会从最后一个标定姿态起步（黄色轨迹起点多出一段绕行）。
    """
    saved_qpos = data.qpos.copy()
    saved_qvel = data.qvel.copy()

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
    data.qpos[:] = saved_qpos
    data.qvel[:] = saved_qvel
    mujoco.mj_forward(model, data)
    return P.reshape(3, 4)


def proj(P, x, y, z):
    v = P @ np.array([x, y, z, 1.0])
    return v[0] / v[2], v[1] / v[2]


def main():
    out_gif, out_why = resolve_out(sys.argv[1:])
    print("输出位置：%s（%s）" % (out_gif, out_why))
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    print("驱动方式：关节空间一阶插值（每帧 alpha_h=%.2f / alpha_l=%.2f，不做物理积分）"
          % (ALPHA_H, ALPHA_L))

    # 关节索引：qpos 与 qvel 布局不同（free joint 四元数），分别取偏移
    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    q1_a, q1_v = jq("shoulder")
    q2_a, q2_v = jq("elbow")
    lift_a, lift_v = jq("lift_j")
    grip_a, grip_v = jq("grip_j")
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
    cam.lookat = np.array([0.28, 0.02, 0.10])
    cam.distance = 1.70
    cam.azimuth = 30
    cam.elevation = -58

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
    attach_count = 0
    attach_frame = release_frame = -1
    attach_grip = release_grip = None
    attach_ball = last_attach_ball = release_ball = None

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
        # 关节逼近目标（一阶插值，确定性）
        data.qpos[q1_a] += (q[0] - data.qpos[q1_a]) * ALPHA_H
        data.qpos[q2_a] += (q[1] - data.qpos[q2_a]) * ALPHA_H
        data.qpos[lift_a] += (gl - data.qpos[lift_a]) * ALPHA_L

        # 吸附球：用「升降到位 + 夹爪闭合」两个确定性条件判断
        # （原来的 abs(gripper_z - (BALL_Z+0.02)) < 0.02 在稳态下差值恰好 0.02，
        #   只能靠浮点误差才成立，换 MuJoCo 版本或调 PD 参数就会永久不吸附）
        if ga:
            lifted_low = data.qpos[lift_a] <= lift_low + 0.02
            grip_closed_now = data.qpos[grip_a] >= grip_close - 0.10
            if not attached and lifted_low and grip_closed_now:
                attached = True
                attach_count += 1
                attach_frame = i
                attach_grip = data.xpos[gripper_body].copy()
                attach_ball = np.array([PICK[0], PICK[1], BALL_Z])
            if attached:
                ball_pos = data.xpos[gripper_body].copy()
                last_attach_ball = ball_pos
            else:
                ball_pos = np.array([PICK[0], PICK[1], BALL_Z])
        else:
            if attached:
                attached = False
                released = True
                release_frame = i
                release_grip = data.xpos[gripper_body].copy()
                release_ball = np.array([PLACE[0], PLACE[1], BALL_Z])
            ball_pos = (np.array([PLACE[0], PLACE[1], BALL_Z])
                        if released else np.array([PICK[0], PICK[1], BALL_Z]))
        data.qpos[ball_qadr:ball_qadr + 3] = ball_pos
        data.qpos[ball_qadr + 3:ball_qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[ball_dofadr:ball_dofadr + 6] = 0.0

        # 夹爪开合（同样用插值）
        data.qpos[grip_a] += (gg - data.qpos[grip_a]) * ALPHA_G
        data.qvel[:] = 0.0
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
        frames.append(im.quantize(colors=128))

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    # ---- 搬运结果核对 ----
    # 注意：不能拿「代码刚赋进去的球位置」跟「同一个目标点」比 —— 那个偏差恒为 0，
    # 无论搬运成不成都不可能失败。这里换成三项互相独立的检查。
    print("\n搬运结果核对：")

    # 1) 吸附是否真的触发过（控制事件计数，不是赋值）
    ok1 = (attach_count == 1)
    print("  1) 吸附触发：%d 次（要求 1 次）%s"
          % (attach_count, "PASS" if ok1 else "FAIL（夹爪没降到球心高度，或没闭合）"))
    if attach_frame >= 0:
        print("     发生在第 %d 帧（t=%.2f s），夹爪中心 z=%.4f m"
              % (attach_frame, attach_frame / FPS, attach_grip[2]))

    # 2) 释放瞬间夹爪中心到 B 点的距离（PD 没收敛就会失败）
    if release_grip is None:
        dist_b = float("nan")
        ok2 = False
    else:
        dist_b = float(np.linalg.norm(release_grip - np.array([PLACE[0], PLACE[1], BALL_Z])))
        ok2 = dist_b < 0.05
    print("  2) 释放瞬间夹爪中心到 B 点：%.4f m（要求 < 0.0500 m）%s"
          % (dist_b, "PASS" if ok2 else "FAIL（没走到 B 就松手了）"))

    # 3) 吸附期间球是否真跟着夹爪走完 A→B（吸附失效就会失败）
    if attach_ball is None or last_attach_ball is None:
        d3 = 0.0
        ok3 = False
    else:
        d3 = float(np.linalg.norm(last_attach_ball - attach_ball))
        ok3 = d3 > 0.30
    print("  3) 吸附期间球从 A 到 B 的位移：%.4f m（要求 > 0.3000 m）%s"
          % (d3, "PASS" if ok3 else "FAIL（球没跟着夹爪走）"))

    assert ok1 and ok2 and ok3, ("搬运失败：吸附=%s / 到位=%s / 搬动=%s"
                                 % (ok1, ok2, ok3))
    print("搬运判定：PASS（球被夹起、送到 B 点、并在 B 点释放）")

    print("渲染完成，合成 GIF...")
    # 不加 optimize=True：Pillow 会把连续相同帧合并，文件里的帧数会少于内存帧数；
    # 因此下面的帧数与时长都从写好的 GIF 读回实际值，不用内存里的 len(frames)。
    frames[0].save(out_gif, save_all=True, append_images=frames[1:],
                   duration=GIF_MS, loop=0)
    size_mb = os.path.getsize(out_gif) / 1024.0 / 1024.0
    try:
        with Image.open(out_gif) as gif:
            n_real = gif.n_frames
            total_ms = 0
            for k in range(n_real):
                gif.seek(k)
                total_ms += gif.info.get("duration", GIF_MS)
    except Exception:
        n_real, total_ms = len(frames), len(frames) * GIF_MS
    print("已生成效果图：%s（内存 %d 帧 / 文件内实际 %d 帧，每帧约 %d ms，合计 %.2f 秒，%.2f MB）"
          % (out_gif, len(frames), n_real, GIF_MS, total_ms / 1000.0, size_mb))
    if size_mb > 5.0:
        print("提示：该 GIF 超过仓库动图 5 MB 的限制，本地产物仅供本地查看；"
              "要更新仓库配图请先降低 FPS/分辨率或二次压缩。")


if __name__ == "__main__":
    main()
