#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""接触与牵引力诊断（需要 MuJoCo，不需要图形界面）。

回答三个问题：
    1. 静置后轮胎是否稳定着地（接触数够不够、车身高度是否稳定）？
    2. 车轮是否达到了指令转速？
    3. 车速与轮面速度之差（滑移率）有多大？

用法：
    python3 tools/check_traction.py
退出码：0 = 三项都正常；1 = 存在异常（同时给出可能原因）。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_car_avoidance import config                       # noqa: E402
from mujoco_car_avoidance.car_env import CarEnv, MujocoUnavailable  # noqa: E402

#: 诊断用的满指令轮速 (rad/s)，40 rad/s 对应 2.0 m/s
COMMAND_OMEGA = 40.0
DURATION = 1.5


def main() -> int:
    try:
        env = CarEnv(config.resolve_scene_path())
    except MujocoUnavailable as exc:
        print(exc)
        return 1

    radius = config.WHEEL_RADIUS_M

    # ---------------- 1) 静置 ----------------
    env.set_wheel_speeds(0.0, 0.0)
    env.step(int(round(config.SETTLE_SECONDS / env.timestep)))
    rest = env.state()
    rest_ncon = int(env.data.ncon)
    print("[静置] t={:.2f}s  z={:.4f}  接触数 ncon={}  车速 v={:+.4f} m/s".format(
        rest.t, rest.z, rest_ncon, rest.v))

    # ---------------- 2) 满指令直行 ----------------
    env.set_wheel_speeds(COMMAND_OMEGA, COMMAND_OMEGA)
    print("\n   t       z  ncon   轮ω(rad/s)  轮面速度  车速   滑移率  姿态" )
    slips = []
    omegas = []
    min_ncon = rest_ncon
    max_z = rest.z
    min_upright = rest.upright
    total_steps = int(round(DURATION / env.timestep))
    for step in range(1, total_steps + 1):
        env.step(1)
        if step % 25:
            continue            # 每 25 个物理步（0.05 s）打一行
        state = env.state()
        wheel = 0.5 * (state.wheel_left + state.wheel_right)
        surface = wheel * radius
        slip = 0.0 if abs(surface) < 1e-6 else max(0.0, (surface - state.v) / surface)
        slips.append(slip)
        omegas.append(abs(wheel))
        min_ncon = min(min_ncon, int(env.data.ncon))
        max_z = max(max_z, state.z)
        min_upright = min(min_upright, state.upright)
        print("%5.2f  %7.4f  %4d  %10.2f  %8.3f  %6.3f  %6.1f%%  %6.3f"
              % (state.t, state.z, env.data.ncon, wheel, surface, state.v, slip * 100,
                 state.upright))

    final = env.state()
    print("\n[结果] 末速 v={:.3f} m/s（指令 {:.2f} m/s），最小接触数 {}，"
          "车身高度 {:.4f}~{:.4f} m，最低姿态 {:.3f}".format(
              final.v, COMMAND_OMEGA * radius, min_ncon, rest.z, max_z, min_upright))

    # ---------------- 3) 判定 ----------------
    max_omega = max(omegas) if omegas else 0.0
    mean_slip = (sum(slips) / len(slips)) if slips else 1.0
    checks = [
        ("静置后接触数 >= 3（轮胎稳定着地）", rest_ncon >= 3,
         "静置 ncon={}（正常应为 4~5：两驱动轮各 1~2 点 + 前支撑轮 1 点）".format(rest_ncon)),
        ("车身未被弹飞（最高点 < 0.09 m）", max_z < 0.09,
         "车身最高 {:.4f} m（静置 {:.4f} m；>0.09 m 说明驱动力没有限幅，把车弹起来了）".format(
             max_z, rest.z)),
        ("车身未翻（姿态 > 0.9）", min_upright > 0.9,
         "最低姿态 {:.3f}（-1 表示车倒扣着，此时车轮悬空、滑移率必然 ~100%）".format(min_upright)),
        ("车轮能达到指令转速的 80% 以上", max_omega >= 0.8 * COMMAND_OMEGA,
         "最大轮速 {:.1f} / 指令 {:.1f} rad/s".format(max_omega, COMMAND_OMEGA)),
        ("平均滑移率 < 25%（牵引正常）", mean_slip < 0.25,
         "平均滑移率 {:.1f}%（>50% 说明驱动轮几乎没有法向力）".format(mean_slip * 100)),
    ]

    print("\n  [判定]")
    for name, passed, detail in checks:
        print("    [{}] {} —— {}".format("PASS" if passed else "FAIL", name, detail))

    ok = all(passed for _n, passed, _d in checks)
    if not ok:
        print("\n  [提示] 逐项对照：")
        if min_upright <= 0.9 or max_z >= 0.09:
            print("    - 车被弹飞/掀翻：驱动轮出力没有限幅。检查 XML 中 actuator 的 forcerange")
            print("      （应为 ±8 N；不限幅时 40 rad/s 阶跃会让伺服瞬间输出几百牛）")
        if rest_ncon < 3:
            print("    - 接触数不足：检查 car 的出生高度与 floor 是否误设了 margin")
            print("      （支撑接触不应有 margin，运行 python3 tools/check_scene.py 复核）")
        if max_omega < 0.8 * COMMAND_OMEGA:
            print("    - 轮速上不去：轮速伺服增益 kv 偏低，或车轮被卡住")
        if mean_slip >= 0.25 and min_upright > 0.9 and max_z < 0.09:
            print("    - 滑移过大且车没被弹飞：驱动轮法向力不足。确认 caster_rear 底面离地")
            print("      （静止时不承重）、且车体质心在驱动轮轴前方（check_scene.py 会打印质心 x）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
