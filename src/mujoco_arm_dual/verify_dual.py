# -*- coding: utf-8 -*-
"""双机械臂协同搬运 · 验证脚本（IK 数值精度 + 实时渲染）

用法（虚拟机 / 本机，放在 mujoco_arm_dual_single.py 同目录）：
    1) IK 数值精度核对（无窗口，osmesa 即可）：
           MUJOCO_GL=osmesa python3 verify_dual.py ik
    2) 实时渲染验证（有窗口，用 glfw + 桌面，软件 GL 即可）：
           MUJOCO_GL=glfw python3 verify_dual.py gui

输出：
    ik 模式：每个夹持点逆解→正运动学互查误差（毫米），全部 < 0.01mm 即通过
    gui 模式：弹出 MuJoCo 查看器，完整播放「下降→夹取→抬升→平移→放下」4.6 秒
"""
import os, math, sys

import numpy as np
import mujoco
from mujoco import viewer

import mujoco_arm_dual_single as m  # 复用单文件里的模型与控制参数

L1, L2 = m.L1, m.L2
ARM1_XY, ARM2_XY = m.ARM1_XY, m.ARM2_XY


# ========== 1. IK 数值精度核对 ==========
def verify_ik():
    # 夹持点 + 标定覆盖点，两个底座都核对
    points = [m.A_L, m.A_R, m.B_L, m.B_R,
              (0.30, 0.15), (0.48, 0.15), (0.28, 0.05), (0.50, -0.05)]
    print("IK 数值精度核对（逆解 → 正运动学互查）")
    print("%-14s %-14s %-14s %s" % ("底座", "目标点", "正解位置", "误差(mm)"))
    max_err = 0.0
    for base, tag in [(ARM1_XY, "arm1(0.10,0)"), (ARM2_XY, "arm2(0.60,0)")]:
        for (tx, ty) in points:
            q = m.ik2(tx - base[0], ty - base[1])
            if q is None:
                print("%-14s %-14s 不可达" % (tag, str((tx, ty))))
                continue
            x = base[0] + L1 * math.cos(q[0]) + L2 * math.cos(q[0] + q[1])
            y = base[1] + L1 * math.sin(q[0]) + L2 * math.sin(q[0] + q[1])
            err = math.hypot(x - tx, y - ty) * 1000.0  # m -> mm
            max_err = max(max_err, err)
            print("%-14s %-14s (%.4f, %.4f)  %8.4f" %
                  (tag, str((tx, ty)), x, y, err))
    print("\n最大误差: %.4f mm（< 0.01mm 即通过，浮点精度内）" % max_err)
    ok = max_err < 0.01
    print("IK 精度验证: %s" % ("通过" if ok else "未通过"))
    return ok


# ========== 2. 实时渲染验证（有窗口） ==========
def run_gui():
    model = mujoco.MjModel.from_xml_string(m.ARM_XML)
    data = mujoco.MjData(model)

    def jq(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid], model.jnt_dofadr[jid]

    s1_a, s1_v = jq("shoulder1"); e1_a, e1_v = jq("elbow1")
    l1_a, l1_v = jq("lift1");     g1_a, g1_v = jq("grip1")
    s2_a, s2_v = jq("shoulder2"); e2_a, e2_v = jq("elbow2")
    l2_a, l2_v = jq("lift2");     g2_a, g2_v = jq("grip2")
    board = jq("board_free")
    g1_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm1_gripper")
    g2_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm2_gripper")

    q0 = m.ik2(m.A_L[0] - ARM1_XY[0], m.A_L[1] - ARM1_XY[1])
    data.qpos[s1_a] = q0[0]; data.qpos[e1_a] = q0[1]
    data.qpos[s2_a] = q0[0]; data.qpos[e2_a] = q0[1]
    data.qpos[l1_a] = 0.08;  data.qpos[l2_a] = 0.08
    data.qpos[g1_a] = -0.4;  data.qpos[g2_a] = -0.4
    mujoco.mj_forward(model, data)

    lift_up, lift_down = 0.08, -0.125
    grip_open, grip_close = -0.4, 0.0
    ph = [
        (0.5,  m.A_L[0], m.A_L[1], lift_down, grip_open,
               m.A_R[0], m.A_R[1], lift_down, grip_open, False),
        (1.0,  m.A_L[0], m.A_L[1], lift_down, grip_close,
               m.A_R[0], m.A_R[1], lift_down, grip_close, True),
        (1.6,  m.A_L[0], m.A_L[1], lift_up, grip_close,
               m.A_R[0], m.A_R[1], lift_up, grip_close, True),
        (3.0,  m.B_L[0], m.B_L[1], lift_up, grip_close,
               m.B_R[0], m.B_R[1], lift_up, grip_close, True),
        (3.5,  m.B_L[0], m.B_L[1], lift_down, grip_close,
               m.B_R[0], m.B_R[1], lift_down, grip_close, True),
        (3.9,  m.B_L[0], m.B_L[1], lift_down, grip_open,
               m.B_R[0], m.B_R[1], lift_down, grip_open, False),
        (4.6,  m.B_L[0], m.B_L[1], lift_up, grip_open,
               m.B_R[0], m.B_R[1], lift_up, grip_open, False),
    ]
    total = 4.6
    total_frames = int(total * m.FPS)
    attached = False
    placed = False
    phase_i = 0

    def set_board(cx, cy, z, yaw):
        qadr, dofadr = board
        data.qpos[qadr:qadr + 3] = [cx, cy, z]
        data.qpos[qadr + 3:qadr + 7] = [math.cos(yaw / 2.0), 0.0, 0.0,
                                        math.sin(yaw / 2.0)]
        data.qvel[dofadr:dofadr + 6] = 0.0

    set_board(m.BOARD_C[0], m.BOARD_C[1], m.BOARD_Z_A, 0.0)

    print("实时渲染验证：打开 MuJoCo 查看器，播放完整搬运流程（%.1f 秒）" % total)
    with viewer.launch_passive(model, data) as v:
        for i in range(total_frames):
            t = i / m.FPS
            while phase_i < len(ph) and t >= ph[phase_i][0]:
                phase_i += 1
            seg = ph[min(phase_i, len(ph) - 1)]
            (x1, y1, l1, gr1, x2, y2, l2, gr2, ga) = seg[1:10]

            q1 = m.ik2(x1 - ARM1_XY[0], y1 - ARM1_XY[1])
            q2 = m.ik2(x2 - ARM2_XY[0], y2 - ARM2_XY[1])
            data.ctrl[0] = m.KP_H * (q1[0] - data.qpos[s1_a]) - m.KD_H * data.qvel[s1_v]
            data.ctrl[1] = m.KP_H * (q1[1] - data.qpos[e1_a]) - m.KD_H * data.qvel[e1_v]
            data.ctrl[3] = m.KP_H * (q2[0] - data.qpos[s2_a]) - m.KD_H * data.qvel[s2_v]
            data.ctrl[4] = m.KP_H * (q2[1] - data.qpos[e2_a]) - m.KD_H * data.qvel[e2_v]

            if ga:
                z1 = data.xpos[g1_body][2]; z2 = data.xpos[g2_body][2]
                if (not attached and abs(z1 - m.BOARD_Z_A) < 0.006
                        and abs(z2 - m.BOARD_Z_A) < 0.006
                        and abs(data.qpos[g1_a]) < 0.05
                        and abs(data.qpos[g2_a]) < 0.05):
                    attached = True
            else:
                if attached:
                    attached = False
                    placed = True

            if attached:
                p1 = data.xpos[g1_body]; p2 = data.xpos[g2_body]
                set_board((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0,
                          (p1[2] + p2[2]) / 2.0,
                          math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
            elif placed:
                set_board(m.BOARD_CB[0], m.BOARD_CB[1], m.BOARD_Z_B, 0.0)
            else:
                set_board(m.BOARD_C[0], m.BOARD_C[1], m.BOARD_Z_A, 0.0)

            for _ in range(3):
                mujoco.mj_step(model, data)
            data.qvel[l1_v] = 0.0; data.qpos[l1_a] += (l1 - data.qpos[l1_a]) * 0.35
            data.qvel[g1_v] = 0.0; data.qpos[g1_a] += (gr1 - data.qpos[g1_a]) * 0.25
            data.qvel[l2_v] = 0.0; data.qpos[l2_a] += (l2 - data.qpos[l2_a]) * 0.35
            data.qvel[g2_v] = 0.0; data.qpos[g2_a] += (gr2 - data.qpos[g2_a]) * 0.25
            mujoco.mj_forward(model, data)
            v.sync()
            if i % 20 == 0:
                print("  实时帧 %d/%d (t=%.1fs)" % (i + 1, total_frames, t))
    qadr, _ = board
    print("实时渲染验证: 完成，木板最终位置 (%.3f, %.3f, %.3f)" %
          (data.qpos[qadr], data.qpos[qadr + 1], data.qpos[qadr + 2]))


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "ik"
    if mode == "ik":
        verify_ik()
    elif mode == "gui":
        run_gui()
    else:
        print("用法: python3 verify_dual.py [ik|gui]")
