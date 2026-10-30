#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_standalone.py —— 平衡与抗扰恢复模块自动化测试（无 ROS 依赖）
====================================================================

测试项
------
1.  模型加载：自由度 / 执行器数量 / 关节数量；
2.  底盘 freejoint 未被速度锁定（与「靠改写速度站住」的模块区分开）；
3.  支撑多边形：脚底 8 角点凸包有效，静立时质心投影在多边形内且有裕度；
4.  ZMP 与捕获点计算有效，且捕获点随质心速度前移；
5.  稳定裕度：实现为「捕获点到支撑多边形边界的距离」，多边形外返回负值；
6.  自由站立稳定性：无扰动 8 s 不跌倒且质心漂移 < 5 mm；
7.  控制律符号自检：踝 pitch/roll、髋 pitch/roll 四个方向的正力矩作用正确
    （符号写反会形成正反馈直接推倒机器人，是控制律正确性的前提）；
8.  对照实验：关闭平衡控制器时同样扰动必然跌倒；
9.  前向抗扰恢复：120 N 推扰可恢复，最大偏移与恢复时间在上界内；
10. 侧向抗扰恢复：150 N 侧向推扰可恢复；
11. 策略对比：踝+髋策略的临界冲量不低于仅踝策略；
12. 恢复残差：恢复后质心残差 < 5 mm；
13. 二分法前置校验：边界假设不成立时抛出异常而非返回错误结果；
14. 跌倒判定：超大扰动下正确判为跌倒且根高/倾角越界；
15. 仿真性能：实时因子满足演示要求；
16. 按键映射：方向键 KEY_UP/DOWN/LEFT/RIGHT 与字母键的大小写一致性；
17. 失败传播自检：检查失败必须以 AssertionError 结束，避免测试运行器误报通过。

运行方式（包根目录）::

    python3 test/test_standalone.py       # 独立运行，失败时退出码为 1
    python3 -m pytest test/ -v            # pytest / colcon 收集，失败即用例失败
"""

from __future__ import annotations

import functools
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from humanoid_push_recovery.balance import (                              # noqa: E402
    BalanceController, capture_point, convex_hull, distance_to_boundary,
    point_in_polygon, stability_margin)
from humanoid_push_recovery.evaluate import critical_force, run_trial     # noqa: E402
from humanoid_push_recovery.sim_core import (                             # noqa: E402
    JOINT_NAMES, BalanceSim, load_params)

PARAMS = load_params()

PASS = 0
FAIL = 0
SKIP = 0

# 当前用例内未通过的检查项；由 _testcase 在用例结束时汇总成 AssertionError。
_failures = []


def check(name, ok, detail=""):
    """记录一项检查结果。

    失败时除计数与打印外，还会登记到当前用例的失败列表，由 ``_testcase``
    在用例结束时抛出 AssertionError。否则 pytest/colcon 只会看到函数正常返回，
    即使检查失败也会报告为通过（它们不执行 ``main()`` 里的退出码判断）。
    """
    global PASS, FAIL
    if ok:
        PASS += 1
        print(f"[PASS] {name} {detail}")
    else:
        FAIL += 1
        print(f"[FAIL] {name} {detail}")
        _failures.append(f"{name} {detail}".strip())


def skip(name, reason):
    global SKIP
    SKIP += 1
    print(f"[SKIP] {name} （{reason}）")


def _testcase(func):
    """用例包装器：只要用例内有任一检查未通过，就以 AssertionError 结束。"""

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        outer = list(_failures)
        _failures.clear()
        try:
            func(*args, **kwargs)
        finally:
            failed = list(_failures)
            _failures[:] = outer
        if failed:
            raise AssertionError(
                f"{func.__name__}：{len(failed)} 项检查未通过\n  - "
                + "\n  - ".join(failed)
            )

    return wrapper


def make_sim(balance=True):
    sim = BalanceSim(params=PARAMS)
    if balance:
        sim.controller = BalanceController(PARAMS)
    sim.reset()
    return sim


class _FixedTorque:
    """固定关节力矩控制器（仅用于符号自检，替代默认平衡控制器）。"""

    def __init__(self, torque_map):
        self.torque_map = dict(torque_map)

    def reset(self, sim=None):
        pass

    def compute(self, sim):
        torque = np.zeros(len(JOINT_NAMES))
        for name, value in self.torque_map.items():
            torque[JOINT_NAMES.index(name)] = value
        return torque


def _fixed_torque_trial(torque_map, steps=400):
    """施加固定关节力矩，返回质心水平位移 (dx, dy)。"""
    sim = BalanceSim(params=PARAMS)
    sim.controller = _FixedTorque(torque_map)
    sim.reset()
    p0 = sim.com_position()[:2].copy()
    for _ in range(steps):
        sim.step()
    delta = sim.com_position()[:2] - p0
    sim.close()
    return delta


# ----------------------------------------------------------------------
@_testcase
def test_model():
    """1~2. 模型与执行器。"""
    sim = make_sim()
    check("模型加载与执行器数量",
          sim.model.nq > 0 and sim.model.nu == len(JOINT_NAMES) and len(JOINT_NAMES) == 15,
          f"(nq={sim.model.nq}, nv={sim.model.nv}, nu={sim.model.nu}, 关节={len(JOINT_NAMES)})")
    check("底盘 freejoint 未被速度锁定",
          sim.model.jnt_type[sim.root_id] == 0,      # mjJNT_FREE == 0
          f"(jnt_type={int(sim.model.jnt_type[sim.root_id])} = free)")
    sim.close()


@_testcase
def test_support_polygon():
    """3. 支撑多边形与质心投影裕度。"""
    sim = make_sim()
    corners = sim.foot_corners()
    hull = sim.foot_polygon()
    com = sim.com_position()
    margin = distance_to_boundary(com[:2], hull)
    check("脚底 8 角点构成有效凸包", corners.shape == (8, 3) and len(hull) >= 3,
          f"(角点 {corners.shape}, 凸包顶点 {len(hull)})")
    check("静立时质心投影落在支撑多边形内且有裕度", margin > 0.02,
          f"(静态裕度 {margin * 1000:.1f} mm)")
    sx = corners[:, 0].max() - corners[:, 0].min()
    sy = corners[:, 1].max() - corners[:, 1].min()
    check("侧向支撑宽度大于前后支撑长度（双足站立特征）", sy > sx,
          f"(x 跨度 {sx * 1000:.0f} mm, y 跨度 {sy * 1000:.0f} mm)")
    sim.close()


@_testcase
def test_balance_criteria():
    """4. ZMP 与捕获点。"""
    sim = make_sim()
    com = sim.com_position()
    zmp = sim.zmp()
    xi = capture_point(com, sim.com_velocity(), com[2])
    check("静立时 ZMP 与质心投影接近（加速度≈0）",
          float(np.linalg.norm(zmp - com[:2])) < 0.02,
          f"(|ZMP-CoM_xy| = {float(np.linalg.norm(zmp - com[:2])) * 1000:.2f} mm)")
    check("捕获点计算有限且与质心量纲一致",
          np.all(np.isfinite(xi)) and abs(float(xi[0] - com[0])) < 0.2,
          f"(捕获点 {np.round(xi, 4)} m)")
    xi_fast = capture_point(com, np.array([0.5, 0.0, 0.0]), com[2])
    check("捕获点随质心速度前移", float(xi_fast[0]) > float(xi[0]),
          f"(v=0.5 m/s 时捕获点 x 前移 {float(xi_fast[0] - xi[0]) * 1000:.0f} mm)")
    sim.close()


@_testcase
def test_stability_margin():
    """5. 稳定裕度：必须是「捕获点到支撑多边形边界的距离」。"""
    sim = make_sim()
    state = sim.read_state(with_margin=True)
    polygon = state["support_polygon"]
    margin = state["stability_margin"]
    static = state["static_margin"]
    check("read_state 输出支撑多边形与稳定裕度",
          polygon is not None and len(polygon) >= 3 and np.isfinite(margin),
          f"(多边形 {len(polygon)} 顶点, 静态裕度 {static * 1000:.1f} mm, "
          f"动态（捕获点）裕度 {margin * 1000:.1f} mm)")
    check("静立时捕获点裕度与质心投影裕度接近（速度≈0）",
          abs(margin - static) < 0.02,
          f"(差 {abs(margin - static) * 1000:.2f} mm)")
    # 实现正确性：多边形外的点必须返回负裕度（旧实现只返回捕获点坐标，无法通过此检查）
    outside = np.array([polygon[:, 0].max() + 0.5, 0.0])
    inside = np.array([polygon[:, 0].mean(), polygon[:, 1].mean()])
    check("在多边形外返回负裕度、内部返回正裕度",
          distance_to_boundary(outside, polygon) < 0
          and distance_to_boundary(inside, polygon) > 0
          and point_in_polygon(inside, polygon) and not point_in_polygon(outside, polygon),
          f"(外部 {distance_to_boundary(outside, polygon):+.3f} m, "
          f"内部 {distance_to_boundary(inside, polygon):+.3f} m)")
    sim.close()


@_testcase
def test_free_standing():
    """6. 无扰动自由站立稳定性。"""
    sim = make_sim()
    ref = sim.com_position()
    max_drift = 0.0
    for _ in range(int(8.0 / sim.dt)):
        sim.step()
        max_drift = max(max_drift, float(np.linalg.norm(sim.com_position()[:2] - ref[:2])))
        if sim.fallen():
            break
    check("无扰动自由站立 8 s 不跌倒", not sim.fallen(),
          f"(根高 {sim.root_height():.3f} m, 倾角 {np.degrees(sim.torso_tilt()):.2f}°)")
    check("自由站立质心漂移 < 5 mm", max_drift < 0.005,
          f"(最大漂移 {max_drift * 1000:.2f} mm)")
    sim.close()


@_testcase
def test_control_sign():
    """7. 控制律符号自检（平衡控制器输出的是关节力矩，故按力矩标定）。"""
    d = _fixed_torque_trial({"ankle_pitch_l": 7.2, "ankle_pitch_r": 7.2})
    check("踝 pitch 正力矩 → 质心向后 (-x)", d[0] < -0.01,
          f"(踝 +7.2 N·m → 质心 Δx = {d[0] * 1000:+.1f} mm)")

    d = _fixed_torque_trial({"ankle_roll_l": 24.0, "ankle_roll_r": 24.0})
    check("踝 roll 正力矩 → 质心向左（+y）", d[1] > 0.005,
          f"(踝 roll +24 N·m → 质心 Δy = {d[1] * 1000:+.1f} mm)")

    d = _fixed_torque_trial({"hip_pitch_l": 7.2, "hip_pitch_r": 7.2})
    check("髋 pitch 正力矩 → 质心向前（+x）", d[0] > 0.001,
          f"(髋 +7.2 N·m → 质心 Δx = {d[0] * 1000:+.1f} mm)")

    d = _fixed_torque_trial({"hip_roll_l": 24.0, "hip_roll_r": 24.0})
    check("髋 roll 正力矩 → 质心向右 (-y)", d[1] < -0.005,
          f"(髋 roll +24 N·m → 质心 Δy = {d[1] * 1000:+.1f} mm)")


@_testcase
def test_no_balance_falls():
    """8. 对照实验：无平衡控制器必然跌倒。"""
    r = run_trial(PARAMS, force=120.0, direction_deg=0.0, balance=False)
    check("关闭平衡控制器时 120 N 推扰即跌倒（对照成立）", bool(r["fell"]),
          f"(无平衡：{'跌倒' if r['fell'] else '竟未跌倒'}, "
          f"最大偏移 {r['max_offset_m'] * 1000:.0f} mm)")


@_testcase
def test_forward_recovery():
    """9. 前向抗扰恢复。"""
    r = run_trial(PARAMS, force=120.0, direction_deg=0.0)
    check("前向 120 N 推扰可恢复", bool(r["recovered"]),
          f"(最大偏移 {r['max_offset_m'] * 1000:.0f} mm, "
          f"恢复 {r['recover_time_s']:.2f} s)" if r["recovered"] else "(未恢复)")
    check("前向 120 N 最大偏移 < 80 mm", r["max_offset_m"] < 0.080,
          f"(最大偏移 {r['max_offset_m'] * 1000:.1f} mm)")
    check("前向 120 N 恢复时间 < 2.5 s",
          bool(r["recover_time_s"] is not None and r["recover_time_s"] < 2.5),
          f"(恢复时间 {r['recover_time_s'] if r['recover_time_s'] else float('nan'):.2f} s)")


@_testcase
def test_lateral_recovery():
    """10. 侧向抗扰恢复。"""
    r = run_trial(PARAMS, force=150.0, direction_deg=90.0)
    check("侧向 150 N 推扰可恢复", bool(r["recovered"]),
          f"(最大偏移 {r['max_offset_m'] * 1000:.0f} mm, "
          f"恢复 {r['recover_time_s']:.2f} s)" if r["recovered"] else "(未恢复)")
    check("侧向 150 N 最大偏移 < 80 mm", r["max_offset_m"] < 0.080,
          f"(最大偏移 {r['max_offset_m'] * 1000:.1f} mm)")


@_testcase
def test_strategy_comparison():
    """11~12. 策略对比与恢复残差。"""
    f_ankle = critical_force(PARAMS, direction_deg=0.0, hip_enable=False,
                             iterations=8, total=6.0)
    f_hip = critical_force(PARAMS, direction_deg=0.0, hip_enable=True,
                           iterations=8, total=6.0)
    dur = float(PARAMS.get("push_duration", 0.08))
    check("仅踝策略临界冲量 > 5 N·s", f_ankle * dur > 5.0,
          f"(临界 {f_ankle:.1f} N = {f_ankle * dur:.2f} N·s)")
    check("踝+髋策略临界冲量不低于仅踝策略", f_hip >= f_ankle - 1e-6,
          f"(踝+髋 {f_hip:.1f} N = {f_hip * dur:.2f} N·s vs 仅踝 {f_ankle * dur:.2f} N·s, "
          f"提升 {(f_hip - f_ankle) / f_ankle * 100:.1f}%)")

    r = run_trial(PARAMS, force=100.0, direction_deg=0.0)
    resid = r["residual_offset_m"]
    check("恢复后质心残差 < 5 mm", bool(resid is not None and resid < 0.005),
          f"(残差 {resid * 1000:.2f} mm)" if resid is not None else "(未恢复)")


@_testcase
def test_critical_force_bounds():
    """13. 二分法前置校验。"""
    raised = None
    try:
        # lo 取一个必然跌倒的力，边界假设不成立，应抛出异常
        critical_force(PARAMS, direction_deg=0.0, hip_enable=False,
                       lo=500.0, hi=600.0, iterations=2, total=6.0)
    except ValueError as exc:
        raised = str(exc)
    check("二分法边界假设不成立时抛出 ValueError（不返回错误的临界值）",
          raised is not None,
          f"(已抛出：{raised[:36]}…)" if raised else "(未抛出异常)")

    ok = True
    try:
        f = critical_force(PARAMS, direction_deg=0.0, hip_enable=False,
                           iterations=6, total=6.0)
    except ValueError:
        ok = False
        f = 0.0
    check("二分法在边界成立时正常返回临界力", ok and f > 0,
          f"(临界力 {f:.1f} N)")


@_testcase
def test_fall_detection():
    """14. 跌倒判定。"""
    r = run_trial(PARAMS, force=500.0, direction_deg=0.0)
    check("超大扰动被正确判为跌倒", bool(r["fell"]),
          f"(500 N 推扰 → {'跌倒' if r['fell'] else '竟恢复'})")
    check("跌倒时根高或倾角越界",
          r["fell"] and (r["final_root_height"] < PARAMS.get("fall_height", 0.85)
                         or r["final_tilt_rad"] > PARAMS.get("fall_tilt", 0.55)),
          f"(根高 {r['final_root_height']:.3f} m, 倾角 {np.degrees(r['final_tilt_rad']):.1f}°)")


@_testcase
def test_performance():
    """15. 仿真吞吐。"""
    sim = make_sim()
    steps = 2000
    start = time.perf_counter()
    for _ in range(steps):
        sim.step()
    elapsed = time.perf_counter() - start
    sps = steps / max(1e-9, elapsed)
    check("仿真吞吐 > 2000 step/s（实时因子 > 4×）", sps > 2000.0,
          f"({sps:.0f} step/s, 实时因子 {sps * float(sim.model.opt.timestep):.2f}×)")
    sim.close()


@_testcase
def test_keymap():
    """16. 按键映射：方向键与字母键（含大小写），确保遥控不会静默忽略按键。"""
    from humanoid_push_recovery.keymap import (
        ARROW_KEYS, KEY_DIRECTIONS, direction_label, key_to_direction, normalize_key)

    expect = {'KEY_UP': 0.0, 'KEY_DOWN': 180.0, 'KEY_LEFT': 90.0, 'KEY_RIGHT': 270.0}
    got = {k: key_to_direction(k) for k in ARROW_KEYS}
    check("方向键四个键均能查到正确方向角", got == expect, f"({got})")

    check("方向键经 normalize_key 后保持大写",
          all(normalize_key(k) == k for k in ARROW_KEYS),
          f"({', '.join(ARROW_KEYS)})")

    # 历史上的 bug：先 key.lower() 再查只含大写键名的表，方向键必然落空。
    check("lower() 后的 KEY_* 不在映射表中（该写法会静默丢失方向键）",
          'key_up' not in KEY_DIRECTIONS and key_to_direction('KEY_UP') is not None,
          "(KEY_UP → 0.0°；'key_up' 不在表中)")

    pairs = {c: (key_to_direction(c), key_to_direction(c.upper())) for c in 'wasdqe'}
    check("字母键 W/A/S/D/Q/E 大小写等价",
          all(low is not None and low == up for low, up in pairs.values()),
          "(" + ", ".join(f"{c.upper()}→{low:.0f}°"
                          for c, (low, _) in pairs.items() if low is not None) + ")")

    unmapped = {k: key_to_direction(k) for k in ('z', 'x', '', 'KEY_HOME')}
    check("未映射按键返回 None（不会误发扰动）",
          all(v is None for v in unmapped.values()),
          f"(已测 {len(unmapped)} 个非方向键)")

    check("方向角中文说明与映射一致",
          [direction_label(key_to_direction(k)) for k in ARROW_KEYS] == ['前', '后', '左', '右'],
          "(↑前 ↓后 ←左 →右)")


@_testcase
def test_failure_propagation():
    """17. 失败传播自检：check() 判失败必须以 AssertionError 结束。

    pytest/colcon 只调用 ``test_*`` 函数，不执行 ``main()`` 的退出码判断；
    若失败只计数打印而不抛异常，测试会误报通过。这里人为构造一项失败，
    验证它确实被抛出。
    """
    global PASS, FAIL
    saved = (PASS, FAIL)
    print("[INFO] 下面这行 [FAIL] 是人为构造的，用于验证失败会真正传播")

    @_testcase
    def _deliberately_failing():
        check("人为构造的失败项", False, "(应当触发 AssertionError)")

    raised = None
    try:
        _deliberately_failing()
    except AssertionError as exc:
        raised = str(exc)
    finally:
        PASS, FAIL = saved          # 人为失败不计入统计

    check("人为构造的失败项会抛出 AssertionError（pytest 据此返回非零）",
          raised is not None,
          f"({raised.splitlines()[0]})" if raised else "(失败被吞掉，测试会误报通过)")


TESTS = (
    test_model,
    test_support_polygon,
    test_balance_criteria,
    test_stability_margin,
    test_free_standing,
    test_control_sign,
    test_no_balance_falls,
    test_forward_recovery,
    test_lateral_recovery,
    test_strategy_comparison,
    test_critical_force_bounds,
    test_fall_detection,
    test_performance,
    test_keymap,
    test_failure_propagation,
)


def main():
    print("=" * 66)
    print("  人形机器人平衡与抗扰恢复模块自动化测试")
    print("=" * 66)
    for func in TESTS:
        try:
            func()
        except AssertionError as exc:
            # 独立运行时跑完其余用例（未通过项已由 check() 计数），
            # 最终由退出码统一反映；pytest 下则直接判该用例失败。
            print(f"[FAIL] 用例 {func.__name__} 存在 {len(str(exc).splitlines()) - 1} 项未通过检查")
    print("=" * 66)
    print(f"结果：{PASS} 项通过，{FAIL} 项失败，{SKIP} 项跳过")
    print("=" * 66)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
