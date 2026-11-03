# -*- coding: utf-8 -*-
"""MuJoCo 二连杆机械臂画圆 —— 验证脚本

用法：
    MUJOCO_GL=osmesa python3 verify_ik.py ik    # IK 数值精度：逆解→正运动学互查
    MUJOCO_GL=glfw   python3 verify_ik.py gui   # 实时渲染：glfw 查看器播放完整 8s 画圆

对应评论中的两项未验证项：
  1) 有窗口环境实时渲染（glfw 查看器，非 osmesa 离屏）；
  2) IK 解算数值精度的单独核对（不依赖渲染与控制闭环）。
"""
import os, math, sys, time

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from mujoco import viewer

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_2link.xml")

# 与 mujoco_arm_ik.py 一致的参数
L1 = 0.30
L2 = 0.30
CX, CY = 0.30, 0.0
RADIUS = 0.15
PERIOD = 6.0
DURATION = 8.0
FPS = 24
KP = 30.0
KD = 8.0


def ik2(x, y):
    """二连杆平面机械臂解析逆运动学（与主程序一致）。"""
    d = math.hypot(x, y)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2),
                                       L1 + L2 * math.cos(q2))
    return q1, q2


def fk2(q1, q2):
    """二连杆正运动学：关节角 → 末端位置，用于与逆解互查。"""
    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y


def check_ik():
    """IK 数值精度：圆上 8 个等分点，逆解 → 正运动学回代，统计最大误差。"""
    n = 8
    max_err = 0.0
    print("IK 数值精度检查：圆上 %d 个等分点（圆心 (%.2f, %.2f)，半径 %.2f m）"
          % (n, CX, CY, RADIUS))
    for k in range(n):
        ang = 2.0 * math.pi * k / n
        tx = CX + RADIUS * math.cos(ang)
        ty = CY + RADIUS * math.sin(ang)
        q = ik2(tx, ty)
        if q is None:
            print("  点 %d (%.3f, %.3f) 超出工作空间！" % (k, tx, ty))
            return 1
        fx, fy = fk2(q[0], q[1])
        err_mm = math.hypot(fx - tx, fy - ty) * 1000.0
        max_err = max(max_err, err_mm)
        print("  点%2d 目标(%.4f, %.4f) -> q(%.4f, %.4f) -> 末端(%.4f, %.4f)  误差 %.4f mm"
              % (k, tx, ty, q[0], q[1], fx, fy, err_mm))
    print("结论：IK 最大误差 = %.4f mm（%s）" %
          (max_err, "通过" if max_err < 0.01 else "未通过"))
    return 0 if max_err < 0.01 else 1


def run_gui():
    """实时渲染：glfw 查看器（有窗口）播放完整 8 秒画圆流程。"""
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    total_frames = int(DURATION * FPS)
    print("实时渲染：glfw 查看器播放完整画圆（%.1f 秒 / %d 帧），关闭窗口或超时后退出..."
          % (DURATION, total_frames))
    try:
        with viewer.launch_passive(model, data) as v:
            for i in range(total_frames):
                t = i / FPS
                ang = 2.0 * math.pi * (t % PERIOD) / PERIOD
                tx = CX + RADIUS * math.cos(ang)
                ty = CY + RADIUS * math.sin(ang)
                q = ik2(tx, ty)
                if q is None:
                    continue
                data.ctrl[0] = KP * (q[0] - data.qpos[0]) - KD * data.qvel[0]
                data.ctrl[1] = KP * (q[1] - data.qpos[1]) - KD * data.qvel[1]
                for _ in range(3):
                    mujoco.mj_step(model, data)
                v.sync()
                time.sleep(max(0.0, 1.0 / 60.0 - 0.005))
            print("实时渲染完成：完整播放 %d 帧 / %.1f 秒" %
                  (total_frames, DURATION))
    except Exception as e:
        print("实时渲染失败：%s" % e)
        return 1
    return 0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "ik"
    if mode == "ik":
        sys.exit(check_ik())
    elif mode == "gui":
        sys.exit(run_gui())
    else:
        print("用法：python3 verify_ik.py {ik|gui}")
        sys.exit(1)
