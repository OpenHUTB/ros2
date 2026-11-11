#!/usr/bin/env python3
"""无头全流程测试：二连杆阻抗控制推墙，生成三联曲线图与指标文件。

用法（在包目录或 devel/lib/<pkg> 下执行）：
    python3 scripts/headless_test.py                 # 10 s，产物写仓库 docs/img/chap8/
    python3 scripts/headless_test.py --out /tmp/imp   # 指定输出目录
    python3 scripts/headless_test.py --seconds 22 --steady-end 10   # 长跑，但仍按 [5,10] s 统计

装到 catkin 的 devel/install 布局后源码树不可写，产物自动退到 ~/.cache/<pkg>/output；
模型路径由 output_paths.resolve_model_path() 解析（源码树 / devel / install 三种布局都覆盖）。

统计口径：接触力等"稳态"指标取 t ∈ [steady_start, steady_end] 窗口的均值，默认 [5.0, 10.0] s
（0～0.4 s 是接触建立、之后有接触模型蠕变，把前段或后段算进均值会偏高/偏低，
评审、节点曲线图与文档必须用同一个窗口才能对上）。
"""
import argparse
import json
import os
import sys

import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from impedance_controller import ImpedanceController
from output_paths import resolve_out_dir, resolve_model_path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
STEADY_START = 5.0   # 稳态统计窗口起点（秒）
STEADY_END = 10.0    # 稳态统计窗口终点（秒）= 默认仿真时长
KP = [80.0, 50.0]
BD = [8.0, 5.0]
X_DES = np.array([0.62, 0.0])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="二连杆阻抗控制无头测试")
    ap.add_argument("--out", default=None, help="输出目录（默认仓库 docs/img/chap8，或 ~/.cache/<pkg>/output）")
    ap.add_argument("--model", default=None, help="模型 XML 路径（默认自动按源码树/devel/install 查找）")
    ap.add_argument("--seconds", type=float, default=STEADY_END, help="仿真时长（秒）")
    ap.add_argument("--steady-start", type=float, default=STEADY_START, help="稳态统计窗口起点（秒）")
    ap.add_argument("--steady-end", type=float, default=None,
                    help="稳态统计窗口终点（秒），默认与仿真时长相同；长跑时固定成 10 与节点口径一致")
    return ap.parse_args(argv)


def run_sim(model_path, seconds, steady_start, steady_end):
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)

    ctrl = ImpedanceController(Kp=KP, Bd=BD)
    x_des = X_DES  # 目标在墙内，持续推墙

    t_list, x_list, fx_list, q1_list, q2_list, ncon_list = [], [], [], [], [], []
    steps = int(round(seconds / m.opt.timestep))
    every = max(1, int(round(0.02 / m.opt.timestep)))  # 20 ms 采样一次

    for i in range(steps):
        q = d.qpos[:2].copy()
        qdot = d.qvel[:2].copy()
        tau, F, x = ctrl.compute_torque(q, qdot, x_des)
        d.ctrl[:] = np.clip(tau, -5, 5)
        mujoco.mj_step(m, d)

        if i % every == 0:
            fx = 0.0
            for ci in range(d.ncon):
                cf = np.zeros(6)
                mujoco.mj_contactForce(m, d, ci, cf)
                fx += cf[0]
            t_list.append(d.time)
            x_list.append(x.copy())
            fx_list.append(fx)
            q1_list.append(q[0])
            q2_list.append(q[1])
            ncon_list.append(d.ncon)

    return {
        "m": m,
        "d": d,
        "t": np.array(t_list),
        "x": np.array(x_list),
        "fx": np.array(fx_list),
        "q1": np.array(q1_list),
        "q2": np.array(q2_list),
        "ncon": np.array(ncon_list),
        "steps": steps,
        "every": every,
        "steady_start": steady_start,
        "steady_end": steady_end,
    }


def summarize(res):
    t_arr = res["t"]
    x_arr = res["x"]
    fx_arr = res["fx"]
    # 稳态指标：只用 [steady_start, steady_end] 窗口（默认 [5,10] s）
    mask = (t_arr >= res["steady_start"]) & (t_arr <= res["steady_end"])
    steady_fx = float(np.mean(fx_arr[mask]))
    steady_x = float(np.mean(x_arr[mask, 0]))
    steady_y = float(np.mean(x_arr[mask, 1]))
    steady_q1 = float(np.mean(res["q1"][mask]))
    steady_q2 = float(np.mean(res["q2"][mask]))
    peak_fx = float(np.max(fx_arr))
    first_contact = float(t_arr[np.argmax(fx_arr > 0.0)]) if np.any(fx_arr > 0.0) else float("nan")
    contact_frames = int(np.sum(fx_arr > 0.0))
    return {
        "mask": mask,
        "steady_fx": steady_fx,
        "steady_x": steady_x,
        "steady_y": steady_y,
        "steady_q1": steady_q1,
        "steady_q2": steady_q2,
        "peak_fx": peak_fx,
        "first_contact": first_contact,
        "contact_frames": contact_frames,
    }


def plot(res, s, out_dir, steady_start):
    t_arr = res["t"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(t_arr, res["fx"], "r-", lw=1.5)
    axes[0].set_ylabel("Contact Force X (N)")
    axes[0].set_title("Impedance Control on Wall — Steady Force = %.1f N (t in [%.1f, %.1f] s)"
                      % (s["steady_fx"], res["steady_start"], res["steady_end"]))
    axes[0].grid(alpha=0.3)
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].axvline(steady_start, color="gray", ls=":", lw=0.8)
    axes[0].axvline(res["steady_end"], color="gray", ls=":", lw=0.8)

    axes[1].plot(t_arr, res["x"][:, 0], "b-", label="tip x")
    axes[1].plot(t_arr, res["x"][:, 1], "g-", label="tip y")
    axes[1].axhline(0.58, color="gray", ls="--", label="wall surface")
    axes[1].set_ylabel("Tip Position (m)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    axes[2].plot(t_arr, res["q1"], label="q1")
    axes[2].plot(t_arr, res["q2"], label="q2")
    axes[2].set_ylabel("Joint Angle (rad)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.join(out_dir, "impedance_curves.png")
    fig.savefig(png, dpi=120)
    plt.close(fig)
    return png


def main(argv=None):
    args = parse_args(argv)
    steady_end = args.steady_end if args.steady_end is not None else args.seconds
    OUT = resolve_out_dir(ROOT, args.out)
    model_path = resolve_model_path(SCRIPT_DIR, args.model)

    res = run_sim(model_path, args.seconds, args.steady_start, steady_end)
    s = summarize(res)
    m, d = res["m"], res["d"]

    print("=" * 56)
    print("二连杆阻抗控制仿真结果")
    print("=" * 56)
    print("模型          : %s" % model_path)
    print("仿真时长      : %.1f s（%d 步 @ %.1f ms）" % (args.seconds, res["steps"], m.opt.timestep * 1000))
    print("稳态窗口      : [%.1f, %.1f] s，%d 个采样点"
          % (args.steady_start, steady_end, int(s["mask"].sum())))
    print("初始 tip      : (%.4f, %.4f) m" % (res["x"][0, 0], res["x"][0, 1]))
    print("稳态 tip      : (%.4f, %.4f) m" % (s["steady_x"], s["steady_y"]))
    print("稳态接触力    : %.2f N（峰值 %.2f N，首次接触 %.2f s）"
          % (s["steady_fx"], s["peak_fx"], s["first_contact"]))
    print("接触点数      : 末步 ncon=%d，窗口内最大 ncon=%d，有接触的采样帧 %d/%d"
          % (d.ncon, int(np.max(res["ncon"])), s["contact_frames"], len(res["t"])))
    print("稳态关节角    : q1=%.4f, q2=%.4f rad（末步 q1=%.4f, q2=%.4f）"
          % (s["steady_q1"], s["steady_q2"], d.qpos[0], d.qpos[1]))

    png = plot(res, s, OUT, args.steady_start)
    print("\n曲线图: %s" % png)

    metrics = {
        "steady_window_s": [args.steady_start, steady_end],
        "steady_window_samples": int(s["mask"].sum()),
        "steady_contact_force_N": round(s["steady_fx"], 2),
        "peak_contact_force_N": round(s["peak_fx"], 2),
        "first_contact_s": round(s["first_contact"], 3),
        "steady_tip_x_m": round(s["steady_x"], 4),
        "steady_tip_y_m": round(s["steady_y"], 4),
        "initial_tip": [round(float(res["x"][0, 0]), 4), round(float(res["x"][0, 1]), 4)],
        "steady_q_rad": [round(s["steady_q1"], 4), round(s["steady_q2"], 4)],
        "final_qpos": [round(float(d.qpos[0]), 4), round(float(d.qpos[1]), 4)],
        "contact_points_final": int(d.ncon),
        "contact_points_max": int(np.max(res["ncon"])),
        "contact_frames": s["contact_frames"],
        "sample_period_s": round(float(m.opt.timestep) * res["every"], 4),
        "controller": {"Kp": KP, "Bd": BD, "x_des": X_DES.tolist(), "torque_clip": 5.0},
        "env": {
            "mujoco": mujoco.__version__,
            "numpy": np.__version__,
            "python": sys.version.split()[0],
        },
    }
    json_path = os.path.join(OUT, "metrics.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print("指标  : %s（运行时产物，按仓库规范不入库）" % json_path)


if __name__ == "__main__":
    main()
