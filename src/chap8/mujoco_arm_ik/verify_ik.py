# -*- coding: utf-8 -*-
"""MuJoCo 二连杆机械臂画圆 —— 验证脚本

用法：
    python3 verify_ik.py ik     # IK 解算精度：逆解 -> **引擎前向运动学**回代（独立读数）
    python3 verify_ik.py gui    # 实时渲染：在 glfw 查看器里播放并叠加轨迹

两种口径的区别（不能互相替代）：
  * 本脚本 ik 模式测的是「IK 解算精度」——只验证解析逆解算得对不对，
    用 MuJoCo 自己的前向运动学（mj_kinematics + site 读数）回代，不涉及控制与物理；
  * main.py 打印的是「仿真侧跟踪误差」——末端实际位置 vs 目标点，含 PD 控制与步进。

渲染后端：未显式设置 MUJOCO_GL 时按平台自动选择（Windows/有 DISPLAY -> glfw，否则 osmesa）。
"""
import math
import os
import sys
import time

if "MUJOCO_GL" not in os.environ:
    if sys.platform.startswith("win") or os.environ.get("DISPLAY"):
        os.environ["MUJOCO_GL"] = "glfw"
    else:
        os.environ["MUJOCO_GL"] = "osmesa"

import numpy as np
import mujoco

MODEL = os.environ.get("MUJOCO_ARM_IK_MODEL") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "arm_2link.xml")

# 与 main.py 一致的参数
L1 = 0.30
L2 = 0.30
CX, CY = 0.30, 0.0
RADIUS = 0.15
PERIOD = 6.0
DURATION = 8.0
FPS = 24
KP = 600.0
KD = 15.0
KP2 = 1400.0
KD2 = 18.0
PHYS_DT = 0.002
SUBSTEPS = int(round((1.0 / FPS) / PHYS_DT))


# 逆解直接从主程序导入，避免两处实现各自演化后验证不再覆盖 main.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from main import ik2  # noqa: E402


def engine_fk(model, data, tip, q1, q2):
    """用 MuJoCo 引擎的前向运动学读末端位置（不依赖手写正解公式）。"""
    data.qpos[0] = q1
    data.qpos[1] = q2
    mujoco.mj_kinematics(model, data)
    return float(data.site_xpos[tip][0]), float(data.site_xpos[tip][1])


def check_ik():
    """IK 解算精度：圆上 8 个等分点，解析逆解 -> 引擎前向运动学回代。"""
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip_site")
    if tip < 0:
        print("[error] 模型里找不到 site 'tip_site'")
        return 1

    n = 8
    max_err = 0.0
    print("IK 解算精度（逆解 -> 引擎前向运动学回代，不含控制与物理）")
    print("圆上 %d 个等分点，圆心 (%.2f, %.2f)，半径 %.2f m" % (n, CX, CY, RADIUS))
    for k in range(n):
        ang = 2.0 * math.pi * k / n
        tx = CX + RADIUS * math.cos(ang)
        ty = CY + RADIUS * math.sin(ang)
        q = ik2(tx, ty)
        if q is None:
            print("  点 %d (%.3f, %.3f) 超出工作空间！" % (k, tx, ty))
            return 1
        fx, fy = engine_fk(model, data, tip, q[0], q[1])
        err_mm = math.hypot(fx - tx, fy - ty) * 1000.0
        max_err = max(max_err, err_mm)
        print("  点%2d 目标(%.4f, %.4f) -> q(%.4f, %.4f) -> 引擎末端(%.4f, %.4f)  误差 %.4f mm"
              % (k, tx, ty, q[0], q[1], fx, fy, err_mm))
    print("结论：IK 回代最大误差 = %.4f mm（%s）" %
          (max_err, "通过" if max_err < 0.01 else "未通过"))
    print("注：这只是解算精度。末端实际跟踪精度由 main.py 运行时打印，含 PD 控制与物理步进。")
    return 0 if max_err < 0.01 else 1


def run_gui():
    """实时渲染：glfw 查看器播放完整画圆，并用 user_scn 叠加黄色轨迹点。"""
    try:
        import mujoco.viewer  # 显式导入子模块：`import mujoco` 不会自动加载 viewer
    except Exception as e:
        print("无法加载 mujoco.viewer：%s" % e)
        print("该模式需要图形窗口，请在桌面环境或有 DISPLAY 的机器上运行。")
        return 1

    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip_site")
    total_frames = int(DURATION * FPS)
    print("实时渲染：glfw 查看器播放完整画圆（%.1f 秒 / %d 帧），关闭窗口或跑完即退出..."
          % (DURATION, total_frames))
    traj = []
    try:
        with mujoco.viewer.launch_passive(model, data) as v:
            for i in range(total_frames):
                ang = 2.0 * math.pi * ((i / FPS) % PERIOD) / PERIOD
                q = ik2(CX + RADIUS * math.cos(ang), CY + RADIUS * math.sin(ang))
                if q is None:
                    continue
                for _ in range(SUBSTEPS):
                    data.ctrl[0] = KP * (q[0] - data.qpos[0]) - KD * data.qvel[0]
                    data.ctrl[1] = KP2 * (q[1] - data.qpos[1]) - KD2 * data.qvel[1]
                    mujoco.mj_step(model, data)
                traj.append((float(data.site_xpos[tip][0]),
                             float(data.site_xpos[tip][1]),
                             float(data.site_xpos[tip][2])))
                if len(traj) > int(PERIOD * FPS):
                    traj = traj[-int(PERIOD * FPS):]

                # 用 user_scn 把轨迹画成黄色小球（查看器里能看到，不依赖外部叠加）
                v.user_scn.ngeom = 0
                for j in range(0, len(traj), 4):
                    p = traj[j]
                    g = v.user_scn.geoms[v.user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.008, 0.0, 0.0]),
                        np.array(p),
                        np.eye(3).flatten(),
                        np.array([1.0, 1.0, 0.0, 1.0]))
                    v.user_scn.ngeom += 1
                v.sync()
                time.sleep(1.0 / FPS)
            print("实时渲染完成：完整播放 %d 帧 / %.1f 秒" % (total_frames, DURATION))
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
