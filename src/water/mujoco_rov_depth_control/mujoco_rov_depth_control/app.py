#!/usr/bin/env python3
"""mujoco_rov_depth_control — 水下 ROV 定深控制实验（MuJoCo 物理引擎）。

为什么用 MuJoCo
---------------
OpenHUTB 自带 MuJoCo（`mujoco/` 目录里有 mujoco.dll 与官方模型），它的水下机器人/潜水器
方向就是基于 MuJoCo 插件做的。HoloOcean 那套需要另外下载数 GB 的世界包和独立渲染器，
在没有对应资源时跑不起来；而 MuJoCo 的刚体+推力+阻尼模型足够把「定深控制」这条
控制算法链路完整跑通，且完全可复现。

被控对象（简化 ROV 模型）
------------------------
- 近中性浮力刚体（浮力与重力抵消），只受推进器推力 + 水体阻尼；
- 阻尼包含线性项与二次项，贴近真实水下运动特性；
- 垂向一个推进器做深度控制，水平一个推进器做匀速巡航；
- 控制器：深度 PD + 积分（抗稳态偏差），推力限幅 ±max_thrust。

实验内容
--------
按 `mission.depth_setpoints` 依次定深（−5 m → −2 m → −0.5 m），每个目标保持若干秒，
记录深度、推力、误差，最后输出 `depth_log.csv` 与 `result.png`（深度跟踪曲线 + 推力曲线）。

运行
----
    python main.py

作者：陈曦（湖南工商大学 人工智能与低空经济）
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
DEFAULT_PARAMS = os.path.join(HERE, "config", "params.yaml")


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="MuJoCo 水下 ROV 定深控制实验")
    p.add_argument("--save-dir", default=None)
    p.add_argument("--params", default=DEFAULT_PARAMS)
    return p


def rov_xml(cfg):
    m = cfg["model"]
    sx, sy, sz = [float(v) for v in m["size"]]
    mass = float(m["mass"])
    return """
<mujoco model="rov">
  <option timestep="{dt}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <body name="rov" pos="0 0 -0.2">
      <freejoint/>
      <geom name="hull" type="box" size="{sx} {sy} {sz}" density="{density}" rgba="0.2 0.6 0.9 1"/>
      <geom name="fin" type="box" size="{sx} 0.02 0.10" pos="0 {sy} 0" density="500" rgba="0.9 0.5 0.1 1"/>
    </body>
  </worldbody>
</mujoco>
""".format(dt=float(cfg["control"]["dt"]), sx=sx / 2, sy=sy / 2, sz=sz / 2,
           density=mass / (sx * sy * sz))


def buoyancy_force(mass):
    """近中性浮力：只留 2% 的正浮力，让 ROV 有轻微上浮趋势（更接近真实配平偏差）。"""
    return mass * 9.81 * 1.02


def run(cfg):
    import mujoco

    out_cfg = cfg["output"]
    save_dir = os.path.join(HERE, out_cfg["save_dir"])
    os.makedirs(save_dir, exist_ok=True)

    model = mujoco.MjModel.from_xml_string(rov_xml(cfg))
    data = mujoco.MjData(model)
    mcfg = cfg["model"]
    ccfg = cfg["control"]
    mission = cfg["mission"]

    dt = float(ccfg["dt"])
    control_every = int(ccfg["control_every"])
    kp, kd, ki = float(ccfg["kp"]), float(ccfg["kd"]), float(ccfg["ki"])
    max_thrust = float(mcfg["max_thrust"])
    fwd = float(mcfg["forward_thrust"])
    drag_l = float(mcfg["drag_linear"])
    drag_q = float(mcfg["drag_quadratic"])
    buoy = buoyancy_force(float(mcfg["mass"]))

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    total_steps = int(len(mission["depth_setpoints"]) * float(mission["hold_time"]) / dt)

    print("[rov] 目标深度序列: %s，每个保持 %.1f s，仿真步长 %.3f s"
          % (mission["depth_setpoints"], float(mission["hold_time"]), dt))

    rows = []
    thrust = 0.0
    setpoint = None
    # MuJoCo 会自己施加重力，所以外力里只需要补「浮力」，不能再减一次重量
    m_actual = float(model.body_mass[body_id])
    buoy = m_actual * 9.81 * 1.02
    max_vz = float(ccfg.get("max_vertical_speed", 0.8))
    kv = float(ccfg.get("velocity_gain", 4.0))
    print("[rov] 实际机体质量 %.2f kg，净浮力 %.1f N" % (m_actual, buoy - m_actual * 9.81))

    for step in range(total_steps):
        t = step * dt
        seg = min(int(t / float(mission["hold_time"])), len(mission["depth_setpoints"]) - 1)
        new_setpoint = float(mission["depth_setpoints"][seg])
        if setpoint is None or new_setpoint != setpoint:
            setpoint = new_setpoint
            print("[rov] t=%.1fs 切换到目标深度 %.2f m" % (t, setpoint))

        if step % control_every == 0:
            z = float(data.qpos[2])
            vz = float(data.qvel[2])
            err = setpoint - z                     # 目标 - 当前（负=还要下潜）
            # 串级控制：位置误差 -> 期望垂速 -> 期望加速度 -> 推力
            # 串级控制：位置误差 -> 期望垂速 -> 期望加速度 -> 推力
            v_des = max(-max_vz, min(max_vz, float(ccfg.get("position_gain", 0.8)) * err))
            a_des = kv * (v_des - vz)
            thrust = m_actual * a_des - (buoy - m_actual * 9.81)
            thrust = max(-max_thrust, min(max_thrust, thrust))
            rows.append({"t": round(t, 3), "setpoint": setpoint, "depth": round(z, 4),
                         "error": round(err, 4), "thrust": round(thrust, 3),
                         "vz": round(vz, 4), "segment": seg})

        # 外力：垂向推力 + 净浮力 + 水体阻尼（MuJoCo 本身不含流体阻力，这里显式建模）
        v = data.qvel[0:3].copy()
        drag = -drag_l * v - drag_q * np.abs(v) * v
        data.xfrc_applied[body_id, 0:3] = [fwd + drag[0], drag[1],
                                           thrust + buoy + drag[2]]
        mujoco.mj_step(model, data)

    # ---- 统计：按「连续相同的目标深度」分组，取每组最后 settle_time 秒的稳态误差 ----
    settle = float(mission["settle_time"])
    groups = []
    for r in rows:
        if not groups or groups[-1]["setpoint"] != r["setpoint"]:
            groups.append({"setpoint": r["setpoint"], "rows": []})
        groups[-1]["rows"].append(r)
    seg_stats = []
    for gi, g in enumerate(groups):
        tail = [r for r in g["rows"] if r["t"] >= g["rows"][-1]["t"] - settle]
        errs = [abs(r["error"]) for r in tail]
        seg_stats.append({"segment": gi, "setpoint": g["setpoint"],
                          "settle_error": float(np.mean(errs)) if errs else float("nan"),
                          "max_error": float(np.max(errs)) if errs else float("nan")})
        print("[rov] 定深 %d：目标 %.2f m，稳态平均误差 %.3f m，最大 %.3f m"
              % (gi, g["setpoint"], seg_stats[-1]["settle_error"], seg_stats[-1]["max_error"]))

    with open(os.path.join(save_dir, "depth_log.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # ---- 画图 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in rows]
    depth = [r["depth"] for r in rows]
    sp = [r["setpoint"] for r in rows]
    thrusts = [r["thrust"] for r in rows]

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t, depth, color="#1f77b4", lw=2, label="ROV depth")
    axes[0].plot(t, sp, "--", color="#d62728", lw=1.5, label="setpoint")
    axes[0].set_ylabel("depth (m)")
    axes[0].grid(alpha=.3)
    axes[0].legend(loc="lower right")
    axes[0].set_title("ROV depth keeping (cascade: position -> velocity -> thrust), max settle error %.3f m"
                      % max(s["settle_error"] for s in seg_stats))
    axes[1].plot(t, thrusts, color="#2ca02c", lw=1.7)
    axes[1].set_ylabel("vertical thrust (N)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(save_dir, "result.png"), dpi=110)
    plt.close(fig)

    with open(os.path.join(save_dir, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write("水下 ROV 定深控制实验摘要\n")
        fh.write("物理引擎: MuJoCo %s\n" % mujoco.__version__)
        fh.write("ROV 质量 %.1f kg，最大垂向推力 %.0f N\n" % (float(mcfg["mass"]), max_thrust))
        fh.write("控制律: 串级控制 u = m*(kv*(v_des - vz)) - 净浮力，v_des = clip(kp_pos*e, ±%.1f m/s)\n"
                 % float(ccfg.get("max_vertical_speed", 1.0)))
        fh.write("参数: kp_pos=%.2f 1/s, kv=%.1f 1/s, 推力限幅 ±%.0f N\n"
                 % (float(ccfg.get("position_gain", 0.8)), kv, max_thrust))
        for s in seg_stats:
            fh.write("目标 %.2f m -> 稳态平均误差 %.3f m，最大误差 %.3f m\n"
                     % (s["setpoint"], s["settle_error"], s["max_error"]))
        fh.write("采样点: %d（控制 50 Hz）\n" % len(rows))

    worst = max(s["settle_error"] for s in seg_stats)
    ok = worst <= 0.20
    print("[rov] 最差稳态误差 %.3f m" % worst)
    print("[rov] 结论：%s" % ("PASS（三段定深均收敛）" if ok else "FAIL（定深误差偏大）"))
    return 0 if ok else 1


def main():
    args = build_argparser().parse_args()
    cfg = load_params(args.params)
    if args.save_dir:
        cfg["output"]["save_dir"] = args.save_dir
    try:
        return run(cfg)
    except KeyboardInterrupt:
        print("\n[rov] 用户中断")
        return 130


if __name__ == "__main__":
    sys.exit(main())
