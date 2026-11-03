# -*- coding: utf-8 -*-
"""MuJoCo 二连杆机械臂逆运动学（IK）画圆轨迹

功能：机械臂末端沿圆形轨迹运动，用解析 2 连杆 IK 实时解算关节角，
PD 位置控制驱动关节，渲染每帧动画并合成 demo.gif 效果图。

运行（虚拟机 / 无显卡环境）：
    pip install mujoco pillow
    MUJOCO_GL=osmesa python3 mujoco_arm_ik.py

输出：同目录 demo.gif
"""
import os, math

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_2link.xml")
OUT_GIF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo.gif")

# 连杆长度（与模型一致）
L1 = 0.30
L2 = 0.30

# 圆形轨迹参数
CX, CY = 0.30, 0.0      # 圆心（世界坐标，x 朝前、y 朝左）
RADIUS = 0.15           # 半径
PERIOD = 6.0            # 走完一圈的秒数
DURATION = 8.0          # 总时长（秒）

# 渲染与控制参数
FPS = 24
KP = 30.0               # 关节 PD 比例增益
KD = 8.0                # 关节 PD 微分增益


def ik2(x, y):
    """二连杆平面机械臂解析逆运动学，返回 (q1, q2)；超出工作空间返回 None。"""
    d = math.hypot(x, y)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    cos_q2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))   # 肘下姿势（q2 > 0）
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2),
                                       L1 + L2 * math.cos(q2))
    return q1, q2


def project(wx, wy, wz=0.06):
    """把末端点（世界坐标）投影到渲染图像像素坐标。

    相机为正上方俯视（elevation=-90、distance=0.62、fovy=45）：
    相机位于 (0.3, 0, 0.67)，圆轨迹在俯视投影下就是正圆。
    """
    zc = 0.67 - wz
    f = 579.4                       # 240 / tan(22.5°)
    px = 320 + (wx - 0.3) / zc * f
    py = 240 - wy / zc * f
    return int(px), int(py)


def main():
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    # 渲染器：MUJOCO_GL=osmesa 为无显卡软渲染（虚拟机可用）
    renderer = mujoco.Renderer(model, 480, 640)
    # 自由相机：正上方俯视，圆形轨迹投影为正圆
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = np.array([0.3, 0.0, 0.05])
    cam.distance = 0.62
    cam.azimuth = 0
    cam.elevation = -90

    frames = []
    traj = []            # 末端轨迹点（像素），只保留最近一圈
    max_traj = int(PERIOD * FPS)
    total_frames = int(DURATION * FPS)

    print("开始 IK 轨迹跟踪：圆心 (%.2f, %.2f) 半径 %.2f，共 %d 帧" %
          (CX, CY, RADIUS, total_frames))
    t0 = __import__("time").time()

    for i in range(total_frames):
        t = i / FPS
        ang = 2.0 * math.pi * (t % PERIOD) / PERIOD
        tx = CX + RADIUS * math.cos(ang)
        ty = CY + RADIUS * math.sin(ang)

        q = ik2(tx, ty)
        if q is None:
            print("  [warn] 目标点在机械臂工作空间外：x=%.2f y=%.2f" % (tx, ty))
            continue

        # PD 位置控制（关节力矩 = kp*(目标角-当前角) - kd*角速度）
        data.ctrl[0] = KP * (q[0] - data.qpos[0]) - KD * data.qvel[0]
        data.ctrl[1] = KP * (q[1] - data.qpos[1]) - KD * data.qvel[1]
        for _ in range(3):                      # 细分步进，控制更平滑
            mujoco.mj_step(model, data)

        # 渲染当前画面帧
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render().copy()

        # 末端目标点直接投影到像素（正俯视透视，圆轨迹投影为正圆）
        traj.append(project(tx, ty))
        if len(traj) > max_traj:
            traj = traj[-max_traj:]

        # 在帧上叠加末端已走过的轨迹（黄色粗线）
        im = Image.fromarray(rgb)
        draw = ImageDraw.Draw(im)
        if len(traj) >= 2:
            for k in range(1, len(traj)):
                draw.line([traj[k - 1], traj[k]], fill=(255, 200, 0), width=4)
        frames.append(im)

        if (i + 1) % (total_frames // 4) == 0:
            print("  已渲染 %d/%d 帧（%.0f%%）" % (i + 1, total_frames,
                                                  (i + 1) * 100.0 / total_frames))

    print("渲染完成，耗时 %.1f 秒，合成 GIF..." % (__import__("time").time() - t0))
    if frames:
        frames[0].save(OUT_GIF, save_all=True, append_images=frames[1:],
                       duration=int(1000 / FPS), loop=0)
        print("已生成效果图：%s（%d 帧，%.1f 秒）" % (OUT_GIF, len(frames),
                                                  len(frames) / FPS))
    else:
        print("未生成任何帧！请检查模型文件与渲染后端。")
        import sys
        sys.exit(1)


if __name__ == "__main__":
    main()
