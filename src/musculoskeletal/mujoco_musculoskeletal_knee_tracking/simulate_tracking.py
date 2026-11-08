#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线轨迹跟踪仿真（不依赖 ROS 2，用于生成运行效果 GIF 与评价图）。

用法:
    python3 simulate_tracking.py                # 渲染 GIF（需要图形环境）
    python3 simulate_tracking.py --no-render    # 无图形环境：只算指标与曲线图
    python3 simulate_tracking.py --duration 10 --out-dir results

输出（默认写到 results/）:
    tracking.gif            运行效果动图（< 5 MB，仅渲染时生成）
    tracking_eval.png       正弦跟踪：目标 vs 实际 + 肌肉激活
    tracking_step.png       阶跃响应：超调与调节时间
    tracking_metrics.txt    RMS / 最大误差 / 超调 / 调节时间 / 稳态误差

指标口径说明:
    正弦轨迹下不存在“超调”“调节时间”这类指标（参考一直在动），
    因此正弦段只报告 RMS 与最大绝对误差；超调/调节时间/稳态误差
    由单独的阶跃实验给出。
"""
import argparse
import math
import os

import numpy as np

import mujoco
from PIL import Image

from mujoco_musculoskeletal_knee_tracking import config_loader
from mujoco_musculoskeletal_knee_tracking.controller import pd_to_activation

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "mujoco_musculoskeletal_knee_tracking",
                     "model", "muscle_knee.xml")
FRAME_EVERY = 8
DEFAULT_SIM_SECONDS = 10.0     # 离线仿真的正弦段时长，与节点 sim.duration 无关


def reference_deg(t, bias, amp, freq):
    return bias + amp * 0.5 * (1.0 - math.cos(2.0 * math.pi * freq * t))


def parse_args():
    ap = argparse.ArgumentParser(description="膝关节轨迹跟踪离线仿真")
    ap.add_argument("--no-render", action="store_true",
                    help="跳过 MuJoCo 渲染（无图形/远程环境使用）")
    ap.add_argument("--duration", type=float, default=DEFAULT_SIM_SECONDS,
                    help="正弦段仿真时长 [s]，默认 10 s（离线仿真专用，"
                         "与节点内置轨迹的 sim.duration 无关）")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results"))
    return ap.parse_args()


def make_model(dt):
    with open(MODEL, "r", encoding="utf-8") as f:
        xml = f.read()
    model = mujoco.MjModel.from_xml_string(xml)
    # 让 params.yaml 的 sim.dt 真正决定物理步长，保证时间轴与轨迹一致
    model.opt.timestep = dt
    return model


def make_renderer(model, height=320, width=400):
    """尝试创建渲染器；失败时返回 None（供无图形环境降级）。"""
    try:
        return mujoco.Renderer(model, height, width)
    except Exception as exc:                     # mujoco.FatalError 等
        print(f"[WARN] 无法创建 MuJoCo 渲染器：{exc}")
        print("[WARN] 已跳过 GIF 渲染，指标与曲线图不受影响。"
              "如需动图请在带图形界面的环境运行，或设置 MUJOCO_GL=egl。")
        return None


def simulate(target_fn, duration, cfg, renderer=None):
    """跑一段仿真。target_fn(t) 返回目标角度（deg）。"""
    dt = cfg["sim.dt"]
    model = make_model(dt)
    data = mujoco.MjData(model)
    i_flex = model.actuator("flexor").id
    i_ext = model.actuator("extensor").id

    n = int(round(duration / dt)) + 1
    times = np.arange(n) * dt                 # 采样间隔严格等于 dt
    ref = np.array([target_fn(t) for t in times])
    # 初值对齐到轨迹起点，避免把启动瞬态算进跟踪误差
    data.qpos[0] = math.radians(ref[0])
    mujoco.mj_forward(model, data)
    ang = np.zeros(n)
    act_f = np.zeros(n)
    act_e = np.zeros(n)

    cam = None
    frames = []
    if renderer is not None:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat = [0.0, 0.0, -0.35]
        cam.distance = 1.3
        cam.azimuth = 90.0
        cam.elevation = -10.0

    lim_hi = math.radians(cfg["ctrl.limit_deg"])
    lim_lo = math.radians(cfg["ctrl.limit_low_deg"])

    for i in range(n):
        q = data.qpos[0]
        qd = data.qvel[0]
        err = math.radians(ref[i]) - q
        flex, ext = pd_to_activation(
            err, qd, cfg["ctrl.kp"], cfg["ctrl.kd"], cfg["ctrl.max_torque"],
            limit_reached=(q >= lim_hi),
            limit_low_reached=(q <= lim_lo),
        )
        data.ctrl[i_flex] = flex
        data.ctrl[i_ext] = ext
        mujoco.mj_step(model, data)

        ang[i] = math.degrees(data.qpos[0])
        act_f[i], act_e[i] = flex, ext

        if renderer is not None and i % FRAME_EVERY == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(Image.fromarray(renderer.render().copy()))

    return {"t": times, "ref": ref, "ang": ang,
            "act_f": act_f, "act_e": act_e, "frames": frames}


def metrics_sine(res):
    err = res["ang"] - res["ref"]
    return {
        "rmse": float(np.sqrt(np.mean(err ** 2))),
        "max_err": float(np.max(np.abs(err))),
    }


def metrics_step(res, t_step, target):
    """阶跃响应：超调、调节时间(±2% 带宽)、稳态误差(下垂)。"""
    t, ang = res["t"], res["ang"]
    after = t >= t_step
    y = ang[after]
    tt = t[after]
    start = res["ref"][0]
    span = target - start
    os_pct = float(max(0.0, (y.max() - target) / span * 100.0)) if span else 0.0

    band = 0.02 * abs(span)
    inside = np.abs(y - target) <= band
    settle = None
    for k in range(len(inside) - 1, -1, -1):
        if not inside[k]:
            settle = float(tt[k + 1] - t_step) if k + 1 < len(tt) else None
            break
    else:
        settle = 0.0

    # 补充：进入 ±5° 的时间（用来说明"能跟上大趋势，但停不到目标"）
    loose = np.abs(y - target) <= 5.0
    t_5 = float(tt[np.argmax(loose)] - t_step) if loose.any() else None

    tail = y[-max(1, len(y) // 10):]
    return {
        "overshoot_pct": os_pct,
        "settle_s": settle,
        "settle_5deg_s": t_5,
        "steady_err_deg": float(target - tail.mean()),   # 正=低于目标(下垂)
    }


def write_metrics(path, cfg, sine, step):
    sym = '-' if step["steady_err_deg"] >= 0 else '+'
    if step["settle_s"] is None:
        settle_txt = ("未进入 ±2% 带"
                      f"（稳态下垂 {abs(step['steady_err_deg']):.2f} deg 超出带宽")
        if step["settle_5deg_s"] is not None:
            settle_txt += f"；进入 ±5° 需 {step['settle_5deg_s']:.2f} s"
        settle_txt += "）"
    else:
        settle_txt = f"{step['settle_s']:6.2f} s (±2% 带宽)"
    text = (
        "== 正弦跟踪（参考持续变化，不适用超调/调节时间）==\n"
        f"RMSE        : {sine['rmse']:6.2f} deg\n"
        f"最大绝对误差 : {sine['max_err']:6.2f} deg\n"
        "\n== 阶跃响应（20deg -> 50deg）==\n"
        f"超调        : {step['overshoot_pct']:6.2f} %\n"
        f"调节时间    : {settle_txt}\n"
        f"稳态误差    : {sym}{abs(step['steady_err_deg']):5.2f} deg"
        f"（{'低于' if sym == '-' else '高于'}目标，"
        "主因是重力矩未做前馈补偿：PD 在稳态误差为 0 时输出为 0）\n"
        "\n== 参数 ==\n"
        f"步长 sim.dt : {cfg['sim.dt']} s\n"
        f"PD          : kp={cfg['ctrl.kp']} kd={cfg['ctrl.kd']}\n"
        f"max_torque  : {cfg['ctrl.max_torque']} N·m"
        "（= 单肌 500 N × wrap 半径 0.035 m）\n"
        f"限位        : {cfg['ctrl.limit_low_deg']} ~ {cfg['ctrl.limit_deg']} deg\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return text


def plot_sine(res, out, cfg):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    axes[0].plot(res["t"], res["ref"], "--", label="reference", color="tab:green")
    axes[0].plot(res["t"], res["ang"], "-", label="actual", color="tab:blue")
    axes[0].set_ylabel("Knee angle [deg]")
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_title(f"Sine tracking (PD kp={cfg['ctrl.kp']} kd={cfg['ctrl.kd']}, "
                      f"max_torque={cfg['ctrl.max_torque']} N.m)")
    axes[1].plot(res["t"], res["act_f"], label="flexor activation", color="tab:red")
    axes[1].plot(res["t"], res["act_e"], label="extensor activation", color="tab:blue")
    axes[1].set_ylabel("Muscle activation [0,1]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_step(res, t_step, target, out, m):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(res["t"], res["ref"], "--", label="reference", color="tab:green")
    ax.plot(res["t"], res["ang"], "-", label="actual", color="tab:blue")
    ax.axvline(t_step, color="gray", lw=0.8, ls=":")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Knee angle [deg]")
    settle_txt = (f"settle {m['settle_s']:.2f}s" if m["settle_s"] is not None
                  else "settle n/a (2% band)")
    ax.set_title(f"Step 20->50 deg | overshoot {m['overshoot_pct']:.1f}% | "
                 f"{settle_txt} | droop {m['steady_err_deg']:.2f} deg",
                 fontsize=10)
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    args = parse_args()
    cfg = config_loader.load(HERE)
    dt = cfg["sim.dt"]
    duration = args.duration
    os.makedirs(args.out_dir, exist_ok=True)

    # ---- 正弦段（含 GIF 渲染）----
    renderer = None if args.no_render else make_renderer(
        make_model(dt))
    sine = simulate(
        lambda t: reference_deg(t, cfg["trajectory.bias_deg"],
                               cfg["trajectory.amplitude_deg"],
                               cfg["trajectory.frequency_hz"]),
        duration, cfg, renderer=renderer)
    if renderer is not None:
        renderer.close()
    sine_m = metrics_sine(sine)

    # ---- 阶跃段（不带渲染）----
    t_step, start_deg = 0.5, 20.0
    target_deg = 50.0
    step = simulate(
        lambda t: start_deg if t < t_step else target_deg,
        t_step + 6.0, cfg)
    step_m = metrics_step(step, t_step, target_deg)

    text = write_metrics(os.path.join(args.out_dir, "tracking_metrics.txt"),
                         cfg, sine_m, step_m)
    print(text)

    try:
        plot_sine(sine, os.path.join(args.out_dir, "tracking_eval.png"), cfg)
        plot_step(step, t_step, target_deg,
                  os.path.join(args.out_dir, "tracking_step.png"), step_m)
        print(f"已输出 {args.out_dir}/tracking_eval.png 与 tracking_step.png")
    except ImportError:
        print("[WARN] 未安装 matplotlib，跳过曲线图")

    # ---- GIF ----
    frames = sine["frames"]
    if frames:
        gif = os.path.join(args.out_dir, "tracking.gif")
        frames[0].save(gif, save_all=True, append_images=frames[1:],
                       duration=int(FRAME_EVERY * dt * 1000), loop=0,
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
