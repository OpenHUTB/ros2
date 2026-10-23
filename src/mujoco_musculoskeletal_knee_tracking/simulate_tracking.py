#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线轨迹跟踪仿真 (不依赖 ROS2, 用于生成运行效果 GIF 与评价图)

用法:
    py -3.14 simulate_tracking.py

输出:
    tracking.gif           运行效果动图 (<10 MB)
    tracking_eval.png      目标 vs 实际 + 肌肉激活曲线
    tracking_metrics.txt   RMSE / 超调量 / 稳定时间
"""
import math
import os

import numpy as np

import mujoco
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "mujoco_musculoskeletal_knee_tracking",
                     "model", "muscle_knee.xml")

# 参数
DT = 0.005
DURATION = 10.0
TRAJ_BIAS_DEG = 30.0
TRAJ_AMP_DEG = 40.0
TRAJ_FREQ_HZ = 0.4
KP = 30.0
KD = 3.0
MAX_TORQUE = 25.0          # 肌肉等效最大力矩(N·m), 用于激活归一化
LIMIT_DEG = 90.0
FRAME_EVERY = 8          # 每 8 步(0.04s)渲染一帧 GIF

def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))

def reference_deg(t):
    return TRAJ_BIAS_DEG + TRAJ_AMP_DEG * 0.5 * (1 - math.cos(2 * math.pi * TRAJ_FREQ_HZ * t))


def main():
    with open(MODEL, "r", encoding="utf-8") as f:
        xml = f.read()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    i_flex = model.actuator("flexor").id
    i_ext = model.actuator("extensor").id

    n = int(DURATION / DT)
    times = np.linspace(0, DURATION, n)
    ref = np.array([reference_deg(t) for t in times])
    ang = np.zeros(n)
    act_f = np.zeros(n)
    act_e = np.zeros(n)
    torq = np.zeros(n)

    renderer = mujoco.Renderer(model, 360, 480)  # (height, width) 降分辨率省体积
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat = [0.0, 0.0, -0.35]
    cam.distance = 1.3
    cam.azimuth = 90.0
    cam.elevation = -10.0
    frames = []

    for i in range(n):
        t = times[i]
        q = data.qpos[0]
        qd = data.qvel[0]
        err = math.radians(ref[i]) - q
        torque = KP * err - KD * qd

        if torque >= 0:
            flex, ext = clamp(torque / MAX_TORQUE), 0.0
        else:
            flex, ext = 0.0, clamp(-torque / MAX_TORQUE)

        if not (math.isfinite(flex) and math.isfinite(ext)):
            flex = ext = 0.0
        if math.degrees(q) >= LIMIT_DEG and torque > 0:
            flex = 0.0

        data.ctrl[i_flex] = flex
        data.ctrl[i_ext] = ext
        mujoco.mj_step(model, data)

        ang[i] = math.degrees(q)
        act_f[i], act_e[i] = flex, ext
        torq[i] = data.qfrc_actuator[0]

        if i % FRAME_EVERY == 0:
            renderer.update_scene(data, camera=cam)
            img = renderer.render()
            frames.append(Image.fromarray(img))

    # ---- 指标 ----
    rmse = float(np.sqrt(np.mean((ang - ref) ** 2)))
    overshoot = float(np.max(np.maximum(ang - ref, 0)))  # 简化正超调
    settle = float(np.argmax(np.abs(ang - ref) < 3.0)) * DT if np.any(np.abs(ang - ref) < 3.0) else DURATION
    metrics = (
        f"RMSE     : {rmse:6.2f} deg\n"
        f"正超调   : {overshoot:6.2f} deg\n"
        f"收敛时间 : {settle:6.2f} s (误差<3deg)\n"
        f"仿真时长 : {DURATION} s  步长: {DT} s\n"
        f"轨迹     : 正弦 bias={TRAJ_BIAS_DEG} amp={TRAJ_AMP_DEG} f={TRAJ_FREQ_HZ}Hz\n"
        f"控制器   : PD kp={KP} kd={KD}  max_torque={MAX_TORQUE}\n"
    )
    with open(os.path.join(HERE, "tracking_metrics.txt"), "w", encoding="utf-8") as f:
        f.write(metrics)
    print(metrics)

    # ---- 评价图 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    axes[0].plot(times, ref, "--", label="reference", color="tab:green")
    axes[0].plot(times, ang, "-", label="actual", color="tab:blue")
    axes[0].set_ylabel("Knee angle [deg]")
    axes[0].legend(); axes[0].grid(True)
    axes[0].set_title("Sine trajectory tracking (PD + antagonistic muscles)")
    axes[1].plot(times, act_f, label="flexor activation", color="tab:red")
    axes[1].plot(times, act_e, label="extensor activation", color="tab:blue")
    axes[1].set_ylabel("Muscle activation [0,1]"); axes[1].set_xlabel("Time [s]")
    axes[1].legend(); axes[1].grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "tracking_eval.png"), dpi=110)
    print("已输出 tracking_eval.png")

    # ---- GIF ----
    gif = os.path.join(HERE, "tracking.gif")
    frames[0].save(gif, save_all=True, append_images=frames[1:], duration=int(FRAME_EVERY * DT * 1000), loop=0)
    size_mb = os.path.getsize(gif) / 1e6
    print(f"已输出 tracking.gif ({size_mb:.2f} MB, {len(frames)} 帧)")
    assert size_mb < 10.0, "GIF 超过 10MB, 需降低帧率/分辨率"

    print("离线仿真完成 ✓")


if __name__ == "__main__":
    main()
