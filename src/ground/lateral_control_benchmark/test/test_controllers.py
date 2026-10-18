#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：不需要 ROS、不需要任何仿真器，只需 numpy。

运行::

    python3 test/test_controllers.py

覆盖内容：
1. 几何：角度归一化、投影符号、闭合路径曲率、弧长参数化
2. LQR 求解器：与解析解对比（一阶/二阶系统）
3. 三个控制器：符号约定、限幅、每类控制器的专项性质
4. 闭环仿真：四场景下三个控制器是否收敛、指标是否有限
5. 指标与排名：定义正确性、排名一致
6. 车辆模型：运动学解与解析解一致、动力学模型低速退化
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from lateral_control_benchmark import (  # noqa: E402
    DynamicBicycle, KinematicBicycle, LQR, PurePursuit, Stanley, VehicleParams,
    build_controller, build_scenario, circular_arc, double_lane_change,
    evaluate, from_waypoints, rank_controllers, sine_wave, solve_care,
    straight_line, wrap_to_pi,
)
from lateral_control_benchmark.simulation import (  # noqa: E402
    SimulationConfig, Simulator, SpeedProfile, track_speed,
)

PASS, FAIL = 0, 0


def check(cond: bool, msg: str) -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {msg}")
    else:
        FAIL += 1
        print(f"  [FAIL] {msg}")


def close(actual: float, expected: float, tol: float, msg: str) -> None:
    check(abs(actual - expected) <= tol, f"{msg}（实际 {actual:.6f}，期望 {expected:.6f} ± {tol:g}）")


# ---------------------------------------------------------------- 1. 几何
def test_geometry() -> None:
    print("\n[1] 几何与参考轨迹")
    close(float(wrap_to_pi(0.0)), 0.0, 1e-12, "wrap_to_pi(0)=0")
    close(abs(float(wrap_to_pi(3 * math.pi))), math.pi, 1e-9, "wrap_to_pi(3pi) 落在 ±pi")
    close(float(wrap_to_pi(2.5 * math.pi)), 0.5 * math.pi, 1e-9, "wrap_to_pi(2.5pi)=pi/2")

    line = straight_line(length=40.0)
    close(line.total_length, 40.0, 1e-9, "直线长度 = 40 m")
    p = line.project(np.array([10.0, 2.0]))
    close(p.signed_distance, 2.0, 1e-9, "路径左侧横向偏差为正")
    close(p.s, 10.0, 1e-9, "投影弧长 s = 10 m")
    p2 = line.project(np.array([10.0, -3.0]))
    close(p2.signed_distance, -3.0, 1e-9, "路径右侧横向偏差为负")

    arc = circular_arc(radius=20.0)
    check(arc.closed, "360° 圆弧标记为闭合")
    close(arc.vertex_curvature[0], 1.0 / 20.0, 2e-3, "闭合圆弧首点曲率 ≈ 1/R")
    close(arc.vertex_curvature[0], arc.vertex_curvature[1], 1e-9, "闭合圆弧首点与相邻点曲率一致")
    close(arc.total_length, 2 * math.pi * 20.0, 0.1, "圆弧总长 ≈ 2πR")

    way = from_waypoints([(0, 0), (3, 0), (3, 4)])
    close(way.total_length, 7.0, 1e-9, "折线航点总长正确")
    lane = double_lane_change()
    check(abs(float(lane.points[:, 1].max())) > 1.0, "双移线存在横向位移")
    sine = sine_wave(amplitude=2.0, wavelength=20.0, length=60.0)
    check(abs(float(sine.points[:, 1].max()) - 2.0) < 0.2, "正弦轨迹幅值正确")


# ------------------------------------------------------------ 2. LQR 求解器
def test_lqr_solver() -> None:
    print("\n[2] Riccati 求解器（纯 NumPy）")
    # 一阶系统 A=0, B=1, Q=1, R=1 的解为 P=1
    p = solve_care(np.array([[0.0]]), np.array([[1.0]]),
                   np.array([[1.0]]), np.array([[1.0]]))
    close(float(p[0, 0]), 1.0, 1e-9, "一阶系统的 CARE 解 P=1")

    # 二阶积分器 A=[[0,1],[0,0]], B=[[0],[1]], Q=I, R=1
    a = np.array([[0.0, 1.0], [0.0, 0.0]])
    b = np.array([[0.0], [1.0]])
    q = np.eye(2)
    r = np.array([[1.0]])
    p2 = solve_care(a, b, q, r)
    # 验证 Riccati 方程残差
    res = a.T @ p2 + p2 @ a - p2 @ b @ np.linalg.inv(r) @ b.T @ p2 + q
    check(np.max(np.abs(res)) < 1e-8, "二阶积分器的 CARE 残差接近 0")
    # 验证 P 对称半正定
    check(np.max(np.abs(p2 - p2.T)) < 1e-9, "解矩阵对称")
    check(np.all(np.linalg.eigvalsh(p2) > -1e-9), "解矩阵半正定")


# ---------------------------------------------------------------- 3. 控制器
def test_controllers() -> None:
    print("\n[3] 三个控制器的符号与限幅")
    path = straight_line(length=50.0)
    params = VehicleParams()

    # 车在路径左侧 1 m、车头对准 -> 三者都应向右打方向（delta < 0 或等效右转）
    state = np.array([0.0, 1.0, 0.0, 8.0])
    proj = path.project(state[:2])

    pp = PurePursuit()
    pp.reset()
    r = pp.compute(state, 8.0, path, proj, 0.5)
    check(r.delta < 0.0, "纯跟踪：左偏时向右修正")

    st_front = Stanley(use_rear_axle=False)
    st_front.reset()
    r_front = st_front.compute(state, 8.0, path, proj, 0.5)
    check(r_front.delta < 0.0, "Stanley：左偏时向右修正")

    lqr = LQR()
    lqr.reset()
    r_lqr = lqr.compute(state, 8.0, path, proj, 0.5)
    check(r_lqr.delta < 0.0, "LQR：左偏时向右修正")

    # 转角不得超出物理极限
    for ctrl, name in ((pp, "pure_pursuit"), (st_front, "stanley"), (lqr, "lqr")):
        ctrl.reset()
        far = np.array([0.0, 30.0, 0.0, 8.0])
        r = ctrl.compute(far, 8.0, path, path.project(far[:2]), 0.5)
        check(abs(r.delta) <= params.max_steering + 1e-12, f"{name}：转角不超过最大转角")

    # 纯跟踪前视距离随速度增长且有上下限
    check(pp.lookahead(0.0) == pp.lookahead_min, "纯跟踪：低速前视取下限")
    check(pp.lookahead(100.0) == pp.lookahead_max, "纯跟踪：高速前视取上限")

    # Stanley：直的参考路径上曲率前馈为 0
    st = Stanley()
    st.reset()
    r = st.compute(np.array([0.0, 0.0, 0.0, 8.0]), 8.0, path,
                   path.project(np.array([0.0, 0.0])), 0.2)
    close(r.info["feedforward"], 0.0, 1e-12, "Stanley：直道前馈为 0")

    # Stanley：圆弧上前馈 ≈ atan(L*kappa)
    arc = circular_arc(radius=20.0)
    st.reset()
    st_state = np.array([0.0, 0.0, 0.0, 8.0])
    r = st.compute(st_state, 8.0, arc, arc.project(np.array([0.0, 0.0])), 0.2)
    close(r.info["feedforward"], math.atan(params.wheelbase / 20.0), 5e-3,
          "Stanley：圆弧前馈 ≈ atan(L/R)")

    # LQR 增益随速度变化且均为有限值
    g8 = LQR().gain(8.0)
    g15 = LQR().gain(15.0)
    check(np.all(np.isfinite(g8)) and np.all(np.isfinite(g15)), "LQR：增益为有限值")
    check(not np.allclose(g8, g15), "LQR：增益随车速变化（按速度求解）")

    # 未知名报错
    try:
        build_controller("not_exist")
        check(False, "未知控制器应报错")
    except KeyError:
        check(True, "未知控制器抛 KeyError")


# ---------------------------------------------------------------- 4. 车辆模型
def test_vehicle_model() -> None:
    print("\n[4] 车辆模型")
    params = VehicleParams()
    kin = KinematicBicycle(params)
    # 匀速直行 1 s（20 步 × 0.05 s）：前进 v*T
    s = np.array([0.0, 0.0, 0.0, 5.0])
    for _ in range(20):
        s = kin.step(s, 0.0, 0.0, 0.05)
    close(float(s[0]), 5.0, 1e-9, "运动学模型：1 s 直行前进 5 m")
    close(float(s[1]), 0.0, 1e-12, "运动学模型：直行横向无位移")

    # 固定转角下做圆周运动，半径应等于 L/tan(delta)
    s = np.array([0.0, 0.0, 0.0, 5.0])
    delta = math.radians(5.0)
    radius_expected = params.wheelbase / math.tan(delta)
    for _ in range(2000):
        s = kin.step(s, delta, 0.0, 0.01)
    # 圆心在 (0, R)，行驶后到圆心的距离应约为 R
    dist = math.hypot(s[0] - 0.0, s[1] - radius_expected)
    check(abs(dist - radius_expected) < 0.2, "运动学模型：定转角圆周半径符合 L/tan(delta)")

    # 动力学模型在低速下退化为运动学
    dyn = DynamicBicycle(params, v_min=2.0)
    low = np.array([0.0, 0.0, 0.0, 0.5])
    new_state, mem = dyn.step(low, 0.0, 0.0, 0.1, {"vy": 0.0, "r": 0.0})
    check(abs(float(mem["r"])) < 1e-6, "动力学模型：低速时横摆角速度接近 0")

    # 动力学模型在高速下会产生侧偏（vy 非零）
    high = np.array([0.0, 0.0, 0.0, 15.0])
    _, mem = dyn.step(high, math.radians(5.0), 0.0, 0.1, {"vy": 0.0, "r": 0.0})
    check(abs(float(mem["vy"])) > 0.0, "动力学模型：高速转向产生侧向速度（侧偏）")


# ---------------------------------------------------------------- 5. 闭环仿真
def test_closed_loop() -> None:
    print("\n[5] 闭环仿真：三控制器 × 四场景")
    scenarios = ("straight", "arc", "sine", "lane_change")
    for name in ("pure_pursuit", "stanley", "lqr"):
        for scenario in scenarios:
            path = build_scenario(scenario)
            cfg = SimulationConfig(dt=0.02, speed=SpeedProfile(cruise_speed=8.0),
                                   duration=90.0 if scenario == "arc" else 60.0)
            tr = Simulator(path, cfg).run(build_controller(name), scenario)
            m = evaluate(tr)
            ok = (np.all(np.isfinite(tr.e)) and m.e_steady_abs < 0.5
                  and m.convergence_time < m.duration)
            check(ok, f"{name} / {scenario}: 稳态 {m.e_steady_abs:.4f} m, "
                      f"收敛 {m.convergence_time:.2f} s / 时长 {m.duration:.1f} s")

    # 初始偏移越大，最大横向偏差应越大（单调性检查）
    path = straight_line(length=60.0)
    maxes = []
    for off in (0.2, 1.0, 2.0):
        cfg = SimulationConfig(dt=0.02, initial_offset=off, initial_heading_error=0.0,
                               speed=SpeedProfile(cruise_speed=8.0))
        tr = Simulator(path, cfg).run(Stanley(), "mono")
        maxes.append(evaluate(tr).e_max_abs)
    check(maxes[0] <= maxes[1] <= maxes[2], f"初始偏移增大时最大偏差单调不减 {maxes}")


# ---------------------------------------------------------------- 6. 指标与排名
def test_metrics() -> None:
    print("\n[6] 指标与排名")
    path = straight_line(length=60.0)
    cfg = SimulationConfig(dt=0.02, speed=SpeedProfile(cruise_speed=8.0))
    metrics = []
    for name in ("pure_pursuit", "stanley", "lqr"):
        tr = Simulator(path, cfg).run(build_controller(name), "straight")
        metrics.append(evaluate(tr))

    for m in metrics:
        check(m.e_rms >= m.e_mean_abs - 1e-12, f"{m.controller}: RMS ≥ 平均绝对值")
        check(m.e_max_abs >= m.e_rms - 1e-12, f"{m.controller}: 最大值 ≥ RMS")
        check(0.0 <= m.off_track_ratio <= 1.0, f"{m.controller}: 离道占比在 [0,1]")
        check(m.convergence_time <= m.duration + 1e-9, f"{m.controller}: 收敛时间不超过总时长")

    ranks = rank_controllers(metrics)
    check(len(ranks) == 3, "排名包含 3 个控制器")
    check(all("rank_mean" in v for v in ranks.values()), "排名包含平均名次字段")
    best = min(ranks.items(), key=lambda kv: kv[1]["rank_mean"])
    check(best[1]["rank_mean"] <= 3.0, f"最优控制器平均名次 ≤ 3（实际 {best[1]['rank_mean']:.2f}）")

    # 速度策略：曲率越大目标速度越低
    arc = circular_arc(radius=10.0)
    prof = SpeedProfile(cruise_speed=10.0, min_speed=2.0, a_lat_max=3.0)
    v_straight = prof.target(path, 0.0)
    v_curve = prof.target(arc, 5.0)
    check(v_curve < v_straight, f"弯道目标速度低于直道（{v_curve:.2f} < {v_straight:.2f}）")
    close(track_speed(10.0, 0.0, 0.1, kp=1.0), 10.0, 1e-9, "纵向 P 控制器输出正确")


def main() -> int:
    print("=" * 72)
    print("横向控制对比实验台 —— 离线单元测试（不需要 ROS / 仿真器）")
    print("=" * 72)
    test_geometry()
    test_lqr_solver()
    test_controllers()
    test_vehicle_model()
    test_closed_loop()
    test_metrics()
    print("\n" + "=" * 72)
    print(f"通过 {PASS} 项，失败 {FAIL} 项")
    print("=" * 72)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
