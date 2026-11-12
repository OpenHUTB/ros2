#!/usr/bin/env python3
"""mujoco_rov_depth_control — 水下 ROV 定深控制实验（MuJoCo 物理引擎）。

被控对象
--------
0.4 x 0.3 x 0.3 m 的 ROV 刚体（机体质量由 `model.hull_mass` 给出），尾部一块尾鳍
（尺寸/密度在 `model.fin` 里显式配置），整体接近中性浮力（净浮力 = 重量的 2%）。
水体作用显式建模为线性阻尼 + 二次阻尼：MuJoCo 本身不含浮力与流体阻力，
必须在 `xfrc_applied` 里补上；而重力由引擎自行施加，因此补浮力时不能再减一次重量。

控制器
------
串级结构（不是单环 PD）：
    位置误差 e -> 期望垂速 v_des = clip(kp_pos * e, ±v_max)
               -> 期望加速度 a_des = kv * (v_des - v_z)
               -> 推力 u = m * a_des - 净浮力，限幅 ±max_thrust
物理步长 dt 与控制分频 control_every 决定控制频率（默认 0.002 s × 25 = **20 Hz**）。

实验内容
--------
按 `mission.segments` 依次定深（-5 m -> -2 m -> -0.5 m），每段的保持时长在配置里显式给出，
记录深度/推力/误差，输出 `depth_log.csv`、`result.png` 与 `summary.md`。

运行
----
    python main.py
"""
import argparse
import csv
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # module root
PKG_NAME = "mujoco_rov_depth_control"


def _resolve_params():
    """源码树里用 <module>/config/params.yaml；colcon 安装后用 ament share 目录里的副本。"""
    local = os.path.join(HERE, "config", "params.yaml")
    if os.path.isfile(local):
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(PKG_NAME), "config", "params.yaml")
    except Exception:
        return local


def _output_root():
    """源码树里写 <module>/docs；安装后写当前工作目录下的 docs。"""
    return HERE if os.path.isdir(os.path.join(HERE, "config")) else os.getcwd()


DEFAULT_PARAMS = _resolve_params()
GRAVITY = 9.81


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def build_argparser():
    p = argparse.ArgumentParser(description="MuJoCo 水下 ROV 定深控制实验")
    p.add_argument("--save-dir", default=None, help="证据输出目录（默认 <模块>/docs）")
    p.add_argument("--params", default=DEFAULT_PARAMS, help="参数文件")
    return p


def rov_xml(cfg):
    """按配置生成 ROV 的 MJCF 模型（只有 box 基本几何体，无外部 mesh）。"""
    m = cfg["model"]
    sx, sy, sz = [float(v) for v in m["size"]]
    hull_mass = float(m["hull_mass"])
    fin = m["fin"]
    fx, fy, fz = [float(v) for v in fin["size"]]
    hull_density = hull_mass / (sx * sy * sz)
    return """
<mujoco model="rov">
  <option timestep="{dt}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <body name="rov" pos="0 0 -0.2">
      <freejoint/>
      <geom name="hull" type="box" size="{hx} {hy} {hz}" density="{hull_density}" rgba="0.2 0.6 0.9 1"/>
      <geom name="fin" type="box" size="{fx} {fy} {fz}" pos="0 {fy_pos} 0" density="{fin_density}" rgba="0.9 0.5 0.1 1"/>
    </body>
  </worldbody>
</mujoco>
""".format(dt=float(cfg["control"]["dt"]),
           hx=sx / 2, hy=sy / 2, hz=sz / 2, hull_density=hull_density,
           fx=fx / 2, fy=fy / 2, fz=fz / 2, fy_pos=sy / 2,
           fin_density=float(fin["density"]))


def cascade_thrust(error, vz, cfg, mass, net_buoyancy):
    """串级控制律的纯函数形式（便于单元测试）。

    error: 目标深度 - 当前深度（负值表示还需要下潜）
    vz:    当前垂向速度（m/s，向下为负）
    返回限幅后的垂向推力（N）。
    """
    c = cfg["control"]
    v_max = float(c["max_vertical_speed"])
    v_des = max(-v_max, min(v_max, float(c["position_gain"]) * error))
    a_des = float(c["velocity_gain"]) * (v_des - vz)
    u = mass * a_des - net_buoyancy
    limit = float(cfg["model"]["max_thrust"])
    return max(-limit, min(limit, u))


def schedule(cfg):
    """把 mission.segments 展开成 [(t_start, t_end, depth), ...]。"""
    segments = cfg["mission"]["segments"]
    out = []
    t = 0.0
    for depth, hold in segments:
        out.append((t, t + float(hold), float(depth)))
        t += float(hold)
    return out


def run(cfg, save=True):
    import mujoco

    out_cfg = cfg["output"]
    save_dir = os.path.join(_output_root(), out_cfg["save_dir"])
    if save:
        os.makedirs(save_dir, exist_ok=True)

    model = mujoco.MjModel.from_xml_string(rov_xml(cfg))
    data = mujoco.MjData(model)
    mcfg = cfg["model"]
    ccfg = cfg["control"]
    mission = cfg["mission"]

    dt = float(ccfg["dt"])
    control_every = int(ccfg["control_every"])
    control_hz = 1.0 / (dt * control_every)
    max_thrust = float(mcfg["max_thrust"])
    fwd = float(mcfg["forward_thrust"])
    drag_l = float(mcfg["drag_linear"])
    drag_q = float(mcfg["drag_quadratic"])

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    mass = float(model.body_mass[body_id])
    # buoyancy: 引擎会自己施加 -m*g，所以要补的是「向上浮力 B」；
    # net_buoyancy = B - m*g 是配平残差，控制器用它做前馈。
    buoyancy = mass * GRAVITY * float(mcfg["buoyancy_ratio"])
    net_buoyancy = buoyancy - mass * GRAVITY

    segs = schedule(cfg)
    total_time = segs[-1][1]
    total_steps = int(round(total_time / dt))

    print("[rov] 任务: %s（每段 (深度, 保持秒数)）" % (mission["segments"],))
    print("[rov] 机体质量 %.2f kg（hull %.2f + fin %.2f），净浮力 %.2f N，控制频率 %.0f Hz"
          % (mass, float(mcfg["hull_mass"]), mass - float(mcfg["hull_mass"]),
             net_buoyancy, control_hz))

    rows = []
    thrust = 0.0
    seg_index = -1

    for step in range(total_steps):
        t = step * dt
        # 当前处于第几段
        while seg_index + 1 < len(segs) and t >= segs[seg_index + 1][0]:
            seg_index += 1
            print("[rov] t=%.1fs 切换到目标深度 %.2f m（保持 %.1f s）"
                  % (t, segs[seg_index][2], segs[seg_index][1] - segs[seg_index][0]))
        setpoint = segs[seg_index][2]

        if step % control_every == 0:
            z = float(data.qpos[2])
            vz = float(data.qvel[2])
            err = setpoint - z                     # 目标 - 当前（负值 = 还要下潜）
            thrust = cascade_thrust(err, vz, cfg, mass, net_buoyancy)
            rows.append({"t": round(t, 3), "segment": seg_index, "setpoint": setpoint,
                         "depth": round(z, 4), "error": round(err, 4),
                         "vz": round(vz, 4), "thrust": round(thrust, 3)})

        # 外力：垂向推力 + 净浮力 + 水体阻尼（MuJoCo 不含浮力/阻力，这里显式补）
        v = data.qvel[0:3].copy()
        drag = -drag_l * v - drag_q * np.abs(v) * v
        data.xfrc_applied[body_id, 0:3] = [fwd + drag[0], drag[1], thrust + buoyancy + drag[2]]
        mujoco.mj_step(model, data)

    # ---- 统计：每段最后 settle_time 秒的稳态误差 ----
    settle = float(mission["settle_time"])
    seg_stats = []
    for si in range(len(segs)):
        seg_rows = [r for r in rows if r["segment"] == si]
        if not seg_rows:
            continue
        tail = [r for r in seg_rows if r["t"] >= seg_rows[-1]["t"] - settle]
        errs = [abs(r["error"]) for r in tail]
        seg_stats.append({"segment": si, "setpoint": seg_rows[0]["setpoint"],
                          "settle_error": float(np.mean(errs)),
                          "max_error": float(np.max(errs))})
        print("[rov] 定深 %d：目标 %.2f m，稳态平均误差 %.3f m，最大 %.3f m"
              % (si, seg_stats[-1]["setpoint"], seg_stats[-1]["settle_error"],
                 seg_stats[-1]["max_error"]))

    worst = max(s["settle_error"] for s in seg_stats)
    threshold = float(mission["pass_threshold"])
    ok = worst <= threshold

    if save:
        write_evidence(cfg, mujoco, save_dir, rows, seg_stats, mass, net_buoyancy,
                       control_hz, worst, threshold, save)
        print("[rov] 证据已写入 %s" % save_dir)

    print("[rov] 最差稳态误差 %.3f m（阈值 %.3f m）" % (worst, threshold))
    print("[rov] 结论：%s" % ("PASS（三段定深均收敛）" if ok else "FAIL（定深误差超过阈值）"))
    return 0 if ok else 1


def write_evidence(cfg, mujoco, save_dir, rows, seg_stats, mass, net_buoyancy,
                   control_hz, worst, threshold, save):
    import csv

    with open(os.path.join(save_dir, "depth_log.csv"), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t"] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t, [r["depth"] for r in rows], color="#1f77b4", lw=2, label="ROV depth")
    axes[0].plot(t, [r["setpoint"] for r in rows], "--", color="#d62728", lw=1.5, label="setpoint")
    axes[0].set_ylabel("depth (m)")
    axes[0].grid(alpha=.3)
    axes[0].legend(loc="lower right")
    axes[0].set_title("ROV depth keeping (cascade: position -> velocity -> thrust), "
                      "worst settle error %.3f m" % worst)
    axes[1].plot(t, [r["thrust"] for r in rows], color="#2ca02c", lw=1.7)
    axes[1].set_ylabel("vertical thrust (N)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(save_dir, "result.png"), dpi=110)
    plt.close(fig)

    with open(os.path.join(save_dir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("水下 ROV 定深控制实验摘要\n")
        fh.write("物理引擎: MuJoCo %s\n" % mujoco.__version__)
        fh.write("任务: %s（每段 = (目标深度 m, 保持 s)）\n" % (cfg["mission"]["segments"],))
        fh.write("机体质量: %.2f kg（hull %.2f kg + fin %.2f kg），净浮力: %.2f N\n"
                 % (mass, float(cfg["model"]["hull_mass"]),
                    mass - float(cfg["model"]["hull_mass"]), net_buoyancy))
        fh.write("控制律: 串级 u = m*(kv*(v_des - vz)) - 净浮力, v_des = clip(kp_pos*e, ±%.1f m/s)\n"
                 % float(cfg["control"]["max_vertical_speed"]))
        fh.write("增益: kp_pos=%.2f 1/s, kv=%.1f 1/s；推力限幅 ±%.0f N\n"
                 % (float(cfg["control"]["position_gain"]),
                    float(cfg["control"]["velocity_gain"]),
                    float(cfg["model"]["max_thrust"])))
        fh.write("控制频率: %.0f Hz（dt=%.3f s, control_every=%d），采样点 %d\n"
                 % (control_hz, float(cfg["control"]["dt"]),
                    int(cfg["control"]["control_every"]), len(rows)))
        for s in seg_stats:
            fh.write("目标 %.2f m -> 稳态平均误差 %.3f m，最大误差 %.3f m\n"
                     % (s["setpoint"], s["settle_error"], s["max_error"]))
        fh.write("最差稳态误差 %.3f m（PASS 阈值 %.3f m）\n" % (worst, threshold))
        fh.write("结论: %s\n" % ("PASS" if worst <= threshold else "FAIL"))


def main():
    # launch_ros 的 Node 会追加 --ros-args / __node:= 等参数，这里容忍未知参数
    args, _unknown = build_argparser().parse_known_args()
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
