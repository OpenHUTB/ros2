#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：只测控制律数学与状态机，**不需要 MuJoCo / ROS**。

直接运行：``python3 test/test_standalone.py``（CI 也是这么跑的）。
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco_hopper_slip_control.config as config          # noqa: E402
from mujoco_hopper_slip_control.controller import (         # noqa: E402
    HopperController,
    Phase,
    desired_liftoff_speed,
    foot_placement_angle,
    required_liftoff_speed,
)

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    return ok


def test_constants():
    check("总质量等于各部件之和",
          math.isclose(config.TOTAL_MASS,
                       config.MASS_TORSO + config.MASS_THIGH + config.MASS_SHANK
                       + config.MASS_FOOT),
          f"{config.TOTAL_MASS:.2f} kg")
    check("Z_REST = 标称腿长 + 足半径",
          math.isclose(config.Z_REST, config.LEG_NOMINAL + config.FOOT_RADIUS),
          f"{config.Z_REST:.3f} m")
    check("腿的行程范围非空", config.LEG_Q_MAX > config.LEG_Q_MIN,
          f"[{config.LEG_Q_MIN}, {config.LEG_Q_MAX}]")
    check("能量增益在允许区间内",
          config.ENERGY_GAIN_MIN <= config.ENERGY_GAIN <= config.ENERGY_GAIN_MAX,
          f"{config.ENERGY_GAIN}")


def test_liftoff_speed():
    for h in (0.45, 0.55, 0.70):
        v = desired_liftoff_speed(h)
        rise = v ** 2 / (2 * config.GRAVITY)
        check(f"顶点 {h:.2f} m 的起飞速度反解自洽",
              math.isclose(rise, h - (config.Z_REST - 0.02), rel_tol=1e-9),
              f"vz={v:.3f} m/s")
    # 效率越低，需要的起飞速度越高
    v1 = required_liftoff_speed(0.55, 0.35, efficiency=1.0)
    v2 = required_liftoff_speed(0.55, 0.35, efficiency=0.7)
    check("效率低时需要更大的起飞速度", v2 > v1, f"{v2:.3f} > {v1:.3f}")
    check("目标低于离地高度时起飞速度为 0",
          required_liftoff_speed(0.20, 0.35) == 0.0)


def test_foot_placement():
    # 比目标慢 -> 脚放身后（θ>0）以加速
    th_slow = foot_placement_angle(vx=0.0, vx_des=0.6)
    check("偏慢时落脚角为正（脚在身后）", th_slow > 0, f"θ={th_slow:+.3f} rad")
    # 比目标快 -> 脚放身前（θ<0）以减速
    th_fast = foot_placement_angle(vx=1.2, vx_des=0.0)
    check("偏快时落脚角为负（脚在身前）", th_fast < 0, f"θ={th_fast:+.3f} rad")
    # 限幅
    for vx, vd in ((-9.0, 9.0), (9.0, -9.0), (5.0, 0.0)):
        th = foot_placement_angle(vx=vx, vx_des=vd)
        check(f"落脚角限幅 (vx={vx}, vd={vd})",
              abs(th) <= config.RAIBERT_ANGLE_LIMIT + 1e-12, f"θ={th:+.3f}")
    # 增益必须运行时生效（不能是 import 时绑死的默认参数）
    config.RAIBERT_KV = 0.30
    th_big = foot_placement_angle(vx=0.2, vx_des=0.0)
    config.RAIBERT_KV = 0.02
    th_small = foot_placement_angle(vx=0.2, vx_des=0.0)
    check("落脚增益运行时生效", abs(th_big) > abs(th_small),
          f"{th_big:+.3f} vs {th_small:+.3f}")
    config.RAIBERT_KV = 0.10


def test_state_machine():
    c = HopperController(debounce=3)
    # 单步接触不切相位
    for _ in range(2):
        c.update(0.30, 0.0, 0.0, 0.05, 0.0, 0.0, contact=True)
    check("接触不足去抖步数不切相位", c.phase is Phase.FLIGHT, str(c.phase))
    c.update(0.30, 0.0, 0.0, 0.05, 0.0, 0.0, contact=True)
    check("连续接触达到去抖步数后切入支撑相", c.phase is Phase.STANCE, str(c.phase))

    # 支撑相：伸展段（q̇<0）应输出负的注能力，压缩段输出 0
    cmd_ext = c.update(0.30, -1.0, 0.0, 0.06, -1.0, 0.0, contact=True)
    check("支撑相伸展段注入负力（蹬伸）", cmd_ext.leg_force < 0, f"{cmd_ext.leg_force:+.1f} N")
    cmd_comp = c.update(0.30, -1.0, 0.0, 0.08, +1.0, 0.0, contact=True)
    check("支撑相压缩段不注能", abs(cmd_comp.leg_force) < 1e-9, f"{cmd_comp.leg_force:+.1f} N")

    # 离地去抖
    for _ in range(2):
        c.update(0.40, 1.0, 0.0, 0.0, 0.0, 0.0, contact=False)
    check("离地不足去抖步数仍算支撑相", c.phase is Phase.STANCE, str(c.phase))
    c.update(0.40, 1.0, 0.0, 0.0, 0.0, 0.0, contact=False)
    check("离地达到去抖步数后回到飞行相", c.phase is Phase.FLIGHT, str(c.phase))


def test_energy_servo():
    c = HopperController(debounce=1)
    f0 = c.f_ex
    # 触地 -> 起飞速度远低于目标 -> 注能力应上升
    c.update(0.30, 0.0, 0.0, 0.05, 0.0, 0.0, contact=False)
    c.update(0.30, -1.0, 0.0, 0.10, 1.0, 0.0, contact=True)
    c.state.vz_liftoff_prev = 0.8          # 人为压低上一跳起飞速度
    c.state.z_liftoff_prev = 0.35
    c.state.apex_max = 0.40
    f1 = c.f_ex
    check("起飞速度不足时注能力上升", f1 > f0, f"{f0:.1f} -> {f1:.1f} N")

    # 起飞速度过高 -> 注能力下降
    c2 = HopperController(debounce=1)
    f0 = c2.f_ex
    c2.state.vz_liftoff_prev = 3.6
    c2.state.z_liftoff_prev = 0.35
    c2.state.apex_max = 0.95
    c2._on_touchdown()
    check("起飞速度过高时注能力下降", c2.f_ex < f0, f"{f0:.1f} -> {c2.f_ex:.1f} N")

    # 注能力上下限
    c3 = HopperController(debounce=1)
    c3.energy_gain = 50.0
    c3.state.vz_liftoff_prev = 0.0
    c3.state.z_liftoff_prev = 0.35
    for _ in range(50):
        c3._on_touchdown()
    check("注能力不超过上限", c3.f_ex <= config.FORCE_LIMIT + 1e-9, f"{c3.f_ex:.1f} N")

    # 效率辨识在合理区间内
    c4 = HopperController(debounce=1)
    c4.state.vz_liftoff_prev = 2.0
    c4.state.z_liftoff_prev = 0.35
    c4.state.apex_max = 0.55
    c4._on_touchdown()
    check("效率辨识落在配置区间内",
          config.EFFICIENCY_MIN <= c4.state.efficiency <= config.EFFICIENCY_MAX,
          f"η={c4.state.efficiency:.3f}")


def test_gain_clamp():
    c = HopperController(energy_gain=-5.0)
    check("能量增益下限被截断", c.energy_gain >= config.ENERGY_GAIN_MIN, f"{c.energy_gain}")
    c = HopperController(energy_gain=99.0)
    check("能量增益上限被截断", c.energy_gain <= config.ENERGY_GAIN_MAX, f"{c.energy_gain}")


def test_setup_version():
    """setup.py 用正则从 config.py 抠版本号。匹配不到会让 setup.py 抛 AttributeError，
    而 colcon 会把真实错误吞掉、只报「exited with code 1」。这里用同一条正则做断言，
    把「版本号必须以字面量写在 config.py 里」这个假设钉住。"""
    import re
    cfg = Path(__file__).resolve().parent.parent / "mujoco_hopper_slip_control" / "config.py"
    m = re.search(r'^VERSION\s*=\s*"([^"]+)"', cfg.read_text(encoding="utf-8"),
                  re.MULTILINE)
    check("config.py 的 VERSION 能被 setup.py 解析出",
          m is not None and m.group(1) == config.VERSION,
          f"解析={m.group(1) if m else 'None'} 期望={config.VERSION}")


def main():
    for fn in (test_constants, test_liftoff_speed, test_foot_placement,
               test_state_machine, test_energy_servo, test_gain_clamp,
               test_setup_version):
        print(f"\n--- {fn.__name__} ---")
        fn()
    print()
    if FAILS:
        print(f"{len(FAILS)} 项失败：{', '.join(FAILS)}")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
