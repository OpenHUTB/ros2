#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import os
import time

# 优先从 ROS 包 share 目录查找 scene.xml；直接 python3 main.py 运行时回退到脚本同目录
try:
    from ament_index_python.packages import get_package_share_directory
    xml_path = os.path.join(get_package_share_directory("mujoco_car_follow"), "scene.xml")
except Exception:
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

TARGET = 0.5       # 纵向目标车心距（m）
KP_X, KD_X = 1.2, 0.6   # 纵向 PD 增益
KP_Y, KD_Y = 1.5, 0.6   # 横向 PD 增益
FMAX = 1.0         # 输出力限幅
DEADBAND = 0.02    # 小死区，抑制微抖


def pd(err, vel, kp, kd, fmax=FMAX):
    """PD 比例控制 + 速度反馈，带饱和限幅。"""
    if abs(err) < DEADBAND and abs(vel) < 0.02:
        return 0.0
    u = kp * err - kd * vel
    return max(-fmax, min(fmax, u))


t = 0.0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t += model.opt.timestep

        # 引导车：x 方向匀速前进，y 方向做正弦摆动走出 S 形
        data.ctrl[0] = 0.2
        data.ctrl[1] = 0.3 * np.cos(t * 2)

        lead_x, lead_y = data.qpos[0], data.qpos[1]
        follow_x, follow_y = data.qpos[2], data.qpos[3]
        vfx, vfy = data.qvel[2], data.qvel[3]

        dx = lead_x - follow_x      # 纵向车心距
        dy = lead_y - follow_y      # 横向偏差

        # PD 控制：误差大加速靠近，误差小自动减速，速度反馈抑制超调与抖振
        data.ctrl[2] = pd(dx - TARGET, vfx, KP_X, KD_X)
        data.ctrl[3] = pd(dy, vfy, KP_Y, KD_Y)

        print(f"车心距 dx={dx:.3f} dy={dy:.3f} | 引导({lead_x:.3f},{lead_y:.3f}) 跟随({follow_x:.3f},{follow_y:.3f})")

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
