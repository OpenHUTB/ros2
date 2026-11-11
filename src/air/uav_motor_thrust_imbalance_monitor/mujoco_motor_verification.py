#!/usr/bin/env python3
"""MuJoCo 台架验证：转子摩擦退化的真实动力学被监测器检出。

与 main.py 合成遥测不同，本脚本让故障真正经过物理引擎——按行业里
电机/ESC 测试的标准做法，把机体固定在测试台架上：

  * 四个转子用铰链关节 + 速度作动器建模，`data.qvel` 里的关节角速度
    就是「ESC 实测转速」——物理状态，不是脚本赋值；
  * 速度作动器与关节阻尼构成真实一阶滞后：指令阶跃时转速按
    τ = I/(kv+c) 的惯性爬升，摩擦增大时稳态转速真的低于指令目标，
    稳态亏 = c/(kv+c)，与真实电机的「带不动」同构；
  * 虚拟拉力按 **实测** 角速度平方计算（F = KT·ω²），用于量化故障
    的推力后果：转速亏 11% → 拉力亏 ~21%；亏 29% → 拉力亏 ~50%；
  * 监测器只消费 (节流指令, 实测转速)，与线上节点完全同一套判据。

适用范围（如实声明）：本脚本验证转子动力学层面的检测能力；自由飞行
中的姿态耦合与紧急降落决策链路由 ROS 话题闭环冒烟测试与离线回放
覆盖，未在 MuJoCo 自由飞场景中验证。

场景（仿真步长 2 ms，遥测 100 Hz，先 1.5 s 预滚到稳态）：
  0–3 s     健康台架运行（四路对称小幅节流起伏）      → 基线学习 → MONITORING
  3–6 s     2 号转子关节阻尼 0 → 0.0062（稳态转速亏 ~11%）
            → IMBALANCE（worst=2）
  6–9 s     阻尼加深到 0.0204（稳态亏 ~29%）          → EMERGENCY
  9–12 s    阻尼渐变回 0.001                          → 恢复 MONITORING

运行：python mujoco_motor_verification.py [--plot DIR]
依赖：mujoco >= 3.0（pip install mujoco）
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List, Optional

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from uav_motor_thrust_imbalance_monitor import (  # noqa: E402
        Config,
        MotorImbalanceMonitor,
        MotorSample,
        MotorState,
    )
else:
    from .uav_motor_thrust_imbalance_monitor import (
        Config,
        MotorImbalanceMonitor,
        MotorSample,
        MotorState,
    )

try:
    import mujoco
except ImportError as exc:  # pragma: no cover
    print(
        f"[错误] 未安装 mujoco（{exc}）。\n"
        "       pip install mujoco 后重试；离线回放 python main.py 无需 mujoco。",
        file=sys.stderr,
    )
    raise SystemExit(2)

# --- 台架模型 ----------------------------------------------------------------

XML = """
<mujoco model="motor_test_bench">
  <option timestep="0.002" integrator="implicitfast"/>
  <worldbody>
    <geom name="bench" type="box" size="0.4 0.4 0.05" pos="0 0 -0.05"
          rgba="0.55 0.55 0.6 1"/>
    <body name="hub" pos="0 0 0.04">
      <geom type="box" size="0.12 0.12 0.04" mass="0.5" rgba="0.3 0.5 0.8 1"/>
      <body name="rotor0" pos="0.15 0.15 0.06">
        <joint name="r0" type="hinge" axis="0 0 1" damping="0.001"/>
        <geom type="cylinder" size="0.08 0.008" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
      <body name="rotor1" pos="0.15 -0.15 0.06">
        <joint name="r1" type="hinge" axis="0 0 1" damping="0.001"/>
        <geom type="cylinder" size="0.08 0.008" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
      <body name="rotor2" pos="-0.15 -0.15 0.06">
        <joint name="r2" type="hinge" axis="0 0 1" damping="0.001"/>
        <geom type="cylinder" size="0.08 0.008" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
      <body name="rotor3" pos="-0.15 0.15 0.06">
        <joint name="r3" type="hinge" axis="0 0 1" damping="0.001"/>
        <geom type="cylinder" size="0.08 0.008" mass="0.05"
              contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity joint="r0" kv="0.05"/>
    <velocity joint="r1" kv="0.05"/>
    <velocity joint="r2" kv="0.05"/>
    <velocity joint="r3" kv="0.05"/>
  </actuator>
</mujoco>
"""

N = 4
OMEGA_MAX = 500.0                     # rad/s，指令 1.0 的目标角速度
HOVER_CMD = 0.55
KT = 2.27e-5                          # 虚拟拉力系数：F = KT * ω²（kg·m/s²）
KV = 0.05                             # 速度作动器增益
TELEM_DT = 0.01                       # 遥测周期 100 Hz
PREROLL = 1.5                         # 预滚：转子到稳态，不喂监测器
END_T = 12.0
RAD2RPM = 60.0 / (2.0 * math.pi)


def friction_at(t: float) -> float:
    """2 号转子关节阻尼（退化注入）。

    速度作动器 kv 与关节阻尼 c 的稳态转速亏 = c/(kv+c)：
    c=0.0062 → 约 11%（warn 级）；c=0.0204 → 约 29%（emg 级）。
    """

    if t < 3.0:
        return 0.001
    if t < 6.0:
        return 0.001 + (0.0062 - 0.001) * (t - 3.0) / 3.0
    if t < 9.0:
        return 0.0062 + (0.0204 - 0.0062) * (t - 6.0) / 3.0
    if t < 10.0:
        return 0.0204 + (0.001 - 0.0204) * (t - 9.0) / 1.0
    return 0.001


def bench_commands(t: float) -> List[float]:
    """台架节流指令：悬停点附近四路对称的小幅起伏（模拟小幅机动）。"""

    wiggle = 0.04 * math.sin(2.0 * math.pi * 0.4 * t)
    return [min(1.0, HOVER_CMD + wiggle)] * N


def run(out_dir: Optional[Path] = None):
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    cfg = Config(rpm_max=OMEGA_MAX * RAD2RPM, hover_cmd=HOVER_CMD,
                 baseline_frames=100, ema_alpha=0.2)
    monitor = MotorImbalanceMonitor(cfg)
    dof2 = 6 + 2 - 6                                  # 台架无自由关节：转子 DOF 0..3
    dof2 = 2

    def step_once(t: float):
        m.dof_damping[dof2] = friction_at(t)
        cmds = bench_commands(t)
        d.ctrl[:] = [OMEGA_MAX * c for c in cmds]
        mujoco.mj_step(m, d)
        return cmds

    # --- 预滚：转子从零加速到稳态，不喂监测器 ---------------------------
    while d.time < PREROLL - 1e-9:
        step_once(0.0)

    rows = []
    events = []
    prev_state = monitor.state
    telem_next = PREROLL
    healthy_thrust_ref = None

    max_iters = int((END_T + PREROLL) / m.opt.timestep) + 1000
    iters = 0
    while iters < max_iters:
        iters += 1
        t = d.time - PREROLL                          # 场景时钟从 0 计
        if t > END_T + 1e-9:
            break
        cmds = step_once(t)
        if d.time + 1e-9 >= telem_next:
            tt = d.time - PREROLL
            rpm = [abs(d.qvel[i]) * RAD2RPM for i in range(N)]
            thrust = [KT * (abs(d.qvel[i])) ** 2 for i in range(N)]
            if healthy_thrust_ref is None:
                healthy_thrust_ref = thrust[2]
            out = monitor.update(MotorSample(t=tt, commands=cmds, rpm=rpm))
            if out.state != prev_state:
                events.append((tt, out.state, out.reason))
                prev_state = out.state
            rows.append({
                "t": round(tt, 3),
                "state": out.state.value,
                "reason": out.reason,
                "worst_motor": out.worst_motor,
                "emergency_land": out.emergency_land,
                "track_err2_pct": round(out.track_err_pct[2], 2)
                if out.track_err_pct[2] == out.track_err_pct[2] else None,
                "cmd": round(cmds[0], 4),
                "rpm2": round(rpm[2], 1),
                "rpm3": round(rpm[3], 1),
                "thrust2_deficit_pct": round(
                    (1.0 - thrust[2] / healthy_thrust_ref) * 100.0, 2),
            })
            telem_next += TELEM_DT

    # --- 汇总与断言 ----------------------------------------------------
    lines = [f"MuJoCo {mujoco.__version__}（台架模型），步长 "
             f"{m.opt.timestep*1000:.0f} ms，遥测 {1/TELEM_DT:.0f} Hz，"
             f"总时长 {END_T:.0f} s"]
    lines.append("")
    lines.append("状态变化事件：")
    for te, st, why in events:
        lines.append(f"  {te:6.2f} s  → {st.value:11s} ({why})")

    def first_state(name, lo, hi):
        for r in rows:
            if lo <= r["t"] <= hi and r["state"] == name:
                return r["t"]
        return None

    checks = []

    def check(name, ok, detail=""):
        checks.append((name, ok, detail))

    t_imb = first_state("IMBALANCE", 3.0, 6.0)
    check("健康段先进入 MONITORING",
          first_state("MONITORING", 0.5, 3.0) is not None)
    check("中退化段检出 IMBALANCE（worst=2，含 EMA 平滑延迟）",
          t_imb is not None and any(
              r["worst_motor"] == 2 for r in rows if r["state"] == "IMBALANCE"
          ), (f"首帧 t={t_imb}（阻尼从 t=3 渐变爬升，越过告警门限即检出）"
              if t_imb else "未检出"))
    t_emg = first_state("EMERGENCY", 6.0, 9.0)
    check("深退化段升级 EMERGENCY 并建议紧急降落",
          t_emg is not None and any(
              r["emergency_land"] for r in rows if r["state"] == "EMERGENCY"
          ), f"首帧 t={t_emg}" if t_emg else "未检出")
    healthy_rows = [r for r in rows if 1.0 <= r["t"] <= 3.0]
    mid_rows = [r for r in rows if 5.0 <= r["t"] <= 6.0]
    deep_rows = [r for r in rows if 8.0 <= r["t"] <= 9.0]
    check("健康段 2 号转速跟踪误差 < 3%",
          max(r["track_err2_pct"] for r in healthy_rows) < 3.0,
          f"峰值 {max(r['track_err2_pct'] for r in healthy_rows):.1f}%")
    check("中退化段稳态转速亏 ~11%（物理稳态 c/(kv+c)）",
          6.0 < max(r["track_err2_pct"] for r in mid_rows) < 16.0,
          f"峰值 {max(r['track_err2_pct'] for r in mid_rows):.1f}%")
    check("深退化段稳态转速亏 ~29%，拉力亏 > 40%",
          max(r["track_err2_pct"] for r in deep_rows) > 24.0
          and max(r["thrust2_deficit_pct"] for r in deep_rows) > 40.0,
          f"转速亏 {max(r['track_err2_pct'] for r in deep_rows):.1f}%，"
          f"拉力亏 {max(r['thrust2_deficit_pct'] for r in deep_rows):.1f}%")
    check("恢复段回到 MONITORING",
          first_state("MONITORING", 10.0, 12.0) is not None)
    check("全程无 NaN（仿真数值稳定）",
          all(r["rpm2"] == r["rpm2"] for r in rows))

    lines.append("")
    lines.append("检查项：")
    all_ok = True
    for name, ok, detail in checks:
        all_ok = all_ok and ok
        lines.append(f"  [{'PASS' if ok else 'FAIL'}] {name}"
                     + (f"（{detail}）" if detail else ""))

    deep_track = max(r["track_err2_pct"] for r in deep_rows)
    deep_thrust = max(r["thrust2_deficit_pct"] for r in deep_rows)
    lines.append("")
    lines.append(
        f"深退化段：2 号实测转速亏 {deep_track:.1f}%（>24% 触发 EMERGENCY），"
        f"对应拉力亏 {deep_thrust:.1f}%——摩擦退化经物理引擎传导到推力输出，"
        "再被监测器从遥测中检出。")
    lines.append("适用范围：本脚本为台架（定基座）验证，覆盖转子动力学与"
                 "转速遥测链路；自由飞姿态耦合与紧急降落决策链路由 ROS 冒烟"
                 "测试与离线回放覆盖，未在 MuJoCo 自由飞场景验证。")

    report = "\n".join(lines)
    print(report)

    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "mujoco_verification.txt").write_text(
            report + "\n", encoding="utf-8")
        with (out_dir / "mujoco_replay.csv").open(
                "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"已写出: {out_dir / 'mujoco_verification.txt'}")
        print(f"已写出: {out_dir / 'mujoco_replay.csv'}")

    return all_ok, rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plot", metavar="DIR", help="写出验证报告与 CSV")
    args = parser.parse_args()
    ok, _ = run(Path(args.plot) if args.plot else None)
    raise SystemExit(0 if ok else 1)
