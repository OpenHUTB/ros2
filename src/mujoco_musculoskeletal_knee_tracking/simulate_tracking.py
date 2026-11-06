#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线轨迹跟踪仿真（不依赖 ROS 2，用于生成运行效果 GIF 与评价图）。

用法:
    python3 simulate_tracking.py                # 渲染 GIF（需要图形环境）
    python3 simulate_tracking.py --no-render    # 无图形环境：只算指标与曲线图
    python3 simulate_tracking.py --duration 10 --out-dir out

输出:
    tracking.gif            运行效果动图（< 5 MB，仅渲染时生成）
    tracking_eval.png       目标 vs 实际 + 肌肉激活曲线
    tracking_metrics.txt    RMSE / 超调量 / 稳定时间
"""
import argparse
import math
import os

import numpy as np

import mujoco

from mujoco_musculoskeletal_knee_tracking.controller import pd_to_activation

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "mujoco_musculoskeletal_knee_tracking",
                     "model", "muscle_knee.xml")

DT = 0.005
MAX_TORQUE = 25.0
LIMIT_DEG = 90.0
FRAME_EVERY = 8


def reference_deg(t, bias, amp, freq):
    return bias + amp * 0.5 * (1.0 - math.cos(2.0 * math.pi * freq * t))


def parse_args():
    ap = argparse.ArgumentParser(description="膝关节轨迹跟踪离线仿真")
    ap.add_argument("--no-render", action="store_true",
                    help="跳过 MuJoCo 渲染（无图形/远程环境使用）")
    ap.add_argument("--duration", type=float, default=10.0, help="仿真时长 [s]")
    ap.add_argument("--bias", type=float, default=30.0, help="轨迹偏置 [deg]")
    ap.add_argument("--amp", type=float, default=40.0, help="轨迹幅值 [deg]")
    ap.add_argument("--freq", type=float, default=0.4, help="正弦频率 [Hz]")
    ap.add_argument("--kp", type=float, default=30.0)
    ap.add_argument("--kd", type=float, default=3.0)
    ap.add_argument("--out-dir", default=HERE)
    return ap.parse_args()


def make_renderer(model, height=320, width=400):
    """尝试创建渲染器；失败时返回 None（供无图形环境降级）。"""
    try:
        return mujoco.Renderer(model, height, width)
    except Exception as exc:                     # mujoco.FatalError 等
        print(f"[WARN] 无法创建 MuJoCo 渲染器：{exc}")
        print("[WARN] 已跳过 GIF 渲染，指标与曲线图不受影响。"
              "如需动图请在带图形界面的环境运行，或设置 MUJOCO_GL=egl。")
        return None


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    with open(MODEL, "r", encoding="utf-8") as f:
        xml = f.read()
    model = mujoco.MjModel.from_xml_string(xml)
    # 让参数里的步长真正生效，保证时间轴与轨迹基准一致
    model.opt.timestep = DT
    data = mujoco.MjData(model)

    i_flex = model.actuator("flexor").id
    i_ext = model.actuator("extensor").id

    n = int(args.duration / DT)
    times = np.linspace(0, args.duration, n)
    ref = np.array([reference_deg(t, args.bias, args.amp, args.freq) for t in times])
    ang = np.zeros(n)
    act_f = np.zeros(n)
    act_e = np.zeros(n)

    renderer = None
    if not args.no_render:
        renderer = make_renderer(model)
    frames = []
    cam = None
    if renderer is not None:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat = [0.0, 0.0, -0.35]
        cam.distance = 1.3
        cam.azimuth = 90.0
        cam.elevation = -10.0

    for i in range(n):
        q = data.qpos[0]
        qd = data.qvel[0]
        err = math.radians(ref[i]) - q
        flex, ext = pd_to_activation(
            err, qd, args.kp, args.kd, MAX_TORQUE,
            limit_reached=(math.degrees(q) >= LIMIT_DEG),
        )
        data.ctrl[i_flex] = flex
        data.ctrl[i_ext] = ext
        mujoco.mj_step(model, data)

        ang[i] = math.degrees(q)
        act_f[i], act_e[i] = flex, ext

        if renderer is not None and i % FRAME_EVERY == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    # ---- 指标 ----
    rmse = float(np.sqrt(np.mean((ang - ref) ** 2)))
    overshoot = float(np.max(np.maximum(ang - ref, 0.0)))
    settled = np.abs(ang - ref) < 3.0
    settle = float(np.argmax(settled)) * DT if settled.any() else args.duration
    metrics = (
        f"RMSE     : {rmse:6.2f} deg\n"
        f"正超调   : {overshoot:6.2f} deg\n"
        f"收敛时间 : {settle:6.2f} s (误差<3deg)\n"
        f"仿真时长 : {args.duration} s  步长: {DT} s\n"
        f"轨迹     : 正弦 bias={args.bias} amp={args.amp} f={args.freq}Hz\n"
        f"控制器   : PD kp={args.kp} kd={args.kd}  max_torque={MAX_TORQUE}\n"
        f"渲染     : {'关闭 (--no-render)' if args.no_render else '开启'}\n"
    )
    with open(os.path.join(args.out_dir, "tracking_metrics.txt"), "w", encoding="utf-8") as f:
        f.write(metrics)
    print(metrics)

    # ---- 评价图 ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 1, figsize=(10, 7))
        axes[0].plot(times, ref, "--", label="reference", color="tab:green")
        axes[0].plot(times, ang, "-", label="actual", color="tab:blue")
        axes[0].set_ylabel("Knee angle [deg]")
        axes[0].legend()
        axes[0].grid(True)
        axes[0].set_title("Sine trajectory tracking (PD + antagonistic muscles)")
        axes[1].plot(times, act_f, label="flexor activation", color="tab:red")
        axes[1].plot(times, act_e, label="extensor activation", color="tab:blue")
        axes[1].set_ylabel("Muscle activation [0,1]")
        axes[1].set_xlabel("Time [s]")
        axes[1].legend()
        axes[1].grid(True)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out_dir, "tracking_eval.png"), dpi=110)
        print("已输出 tracking_eval.png")
    except ImportError:
        print("[WARN] 未安装 matplotlib，跳过 tracking_eval.png")

    # ---- GIF ----
    if renderer is not None and frames:
        from PIL import Image
        gif = os.path.join(args.out_dir, "tracking.gif")
        frames[0].save(gif, save_all=True, append_images=frames[1:],
                       duration=int(FRAME_EVERY * DT * 1000), loop=0,
                       optimize=False)
        size_mb = os.path.getsize(gif) / 1e6
        print(f"已输出 tracking.gif ({size_mb:.2f} MB, {len(frames)} 帧)")
        if size_mb >= 5.0:
            print("[WARN] GIF 超过仓库要求的 5 MB，请降低分辨率或增大 FRAME_EVERY")
    else:
        print("未生成 tracking.gif（渲染已关闭或不可用）")

    print("离线仿真完成")


if __name__ == "__main__":
    main()
