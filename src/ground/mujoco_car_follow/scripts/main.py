#!/usr/bin/env python3
"""mujoco_car_follow：引导车在 ±3 m 内往返走 S 形轨迹，
跟随车用「前馈 + PD」控制保持 0.5 m 的真实车心距。
"""
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

TARGET = 0.5              # 纵向目标车心距（m），按两车车身中心的世界坐标计算
LEAD_X_FORCE = 0.2        # 引导车纵向推力（N），稳态速度 = 0.2/0.3 = 0.667 m/s
LEAD_Y_AMP = 0.3          # 引导车横向正弦振幅（m）
LEAD_Y_FREQ = 2.0         # 引导车横向正弦角频率（rad/s）

KP_X, KD_X = 25.0, 2.0    # 纵向 PD 增益
KP_Y, KD_Y = 15.0, 1.5    # 横向 PD 增益
FMAX = 1.0                # 单个电机输出限幅（N）
X_LIMIT = 3.0             # 引导车往返边界（m）
FF_RATE = 0.02            # 前馈力每步最大变化量（N），折返时平滑过渡
LOG_INTERVAL = 0.5        # 日志打印间隔（s），避免每秒 100 行刷屏


def resolve_scene_path():
    """优先从 ROS 包 share 目录查找 scene.xml，直接运行脚本时回退到脚本目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("mujoco_car_follow"), "scene.xml")
        if os.path.exists(path):
            return path
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (os.path.join(here, "scene.xml"),
                      os.path.join(here, os.pardir, "scene.xml")):
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
    return os.path.join(here, os.pardir, "scene.xml")


def joint_index(model, name):
    """返回 (qpos 地址, dof 地址)。"""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return model.jnt_qposadr[jid], model.jnt_dofadr[jid]


def clamp(value, limit=FMAX):
    return max(-limit, min(limit, value))


def main():
    model = mujoco.MjModel.from_xml_path(resolve_scene_path())
    data = mujoco.MjData(model)

    # 关节地址
    q_lead_x, v_lead_x = joint_index(model, "lead_x")
    q_lead_y, v_lead_y = joint_index(model, "lead_y")
    q_follow_x, v_follow_x = joint_index(model, "follow_x")
    q_follow_y, v_follow_y = joint_index(model, "follow_y")

    # body 的初始偏移：slide 关节 qpos 是相对 body 初始位置的位移，
    # 因此世界坐标 = body.pos + qpos（这是之前距离算少的根因）
    lead_x0 = model.body("lead_car").pos[0]
    lead_y0 = model.body("lead_car").pos[1]
    follow_x0 = model.body("follow_car").pos[0]
    follow_y0 = model.body("follow_car").pos[1]

    t = 0.0
    next_log = 0.0
    direction = 1.0                 # 引导车 x 方向：+1 前进，-1 折返
    feedforward = 0.0               # 平滑后的纵向前馈力，折返时不突变
    dx_samples, dy_samples = [], []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            t += model.opt.timestep

            # ---------- 引导车：±3 m 内往返 + 横向正弦，走 S 形 ----------
            if data.qpos[q_lead_x] > X_LIMIT:
                direction = -1.0
            elif data.qpos[q_lead_x] < -X_LIMIT:
                direction = 1.0
            data.ctrl[0] = LEAD_X_FORCE * direction
            data.ctrl[1] = LEAD_Y_AMP * np.cos(LEAD_Y_FREQ * t)

            # ---------- 世界坐标（含 body 初始偏移） ----------
            lead_x = lead_x0 + data.qpos[q_lead_x]
            lead_y = lead_y0 + data.qpos[q_lead_y]
            follow_x = follow_x0 + data.qpos[q_follow_x]
            follow_y = follow_y0 + data.qpos[q_follow_y]
            vfx = data.qvel[v_follow_x]
            vfy = data.qvel[v_follow_y]

            dx = lead_x - follow_x      # 真实纵向车心距
            dy = lead_y - follow_y      # 真实横向偏差

            # ---------- 跟随车：前馈抵消阻尼负载 + PD 消除稳态误差 ----------
            # 前馈取引导车当前推力并做变化率限幅，折返瞬间不会引起冲击
            feedforward += max(-FF_RATE, min(FF_RATE, data.ctrl[0] - feedforward))
            data.ctrl[2] = clamp(feedforward + KP_X * (dx - TARGET) - KD_X * vfx)
            data.ctrl[3] = clamp(data.ctrl[1] + KP_Y * dy - KD_Y * vfy)

            if t >= next_log:
                next_log += LOG_INTERVAL
                dx_samples.append(dx)
                dy_samples.append(dy)
                print(f"[t={t:6.2f}s] 真实车心距={dx:.3f} m  "
                      f"横向残差={dy:+.3f} m | "
                      f"引导({lead_x:+.3f},{lead_y:+.3f}) "
                      f"跟随({follow_x:+.3f},{follow_y:+.3f})", flush=True)

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)

    if dx_samples:
        dx_arr = np.array(dx_samples)
        dy_arr = np.array(dy_samples)
        print("\n===== 统计（去掉前 3 s 过渡段） =====")
        print(f"纵向车心距：均值 {dx_arr[6:].mean():.3f} m，"
              f"最大偏差 {np.abs(dx_arr[6:] - TARGET).max():.3f} m")
        print(f"横向残差：均值 {dy_arr[6:].mean():+.3f} m，"
              f"最大偏差 {np.abs(dy_arr[6:]).max():.3f} m")


if __name__ == "__main__":
    main()
