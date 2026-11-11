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
from lateral_control_benchmark.benchmark import (  # noqa: E402
    metrics_to_csv, trace_to_csv,
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


# ------------------------------------------------------ 7. 速度边界（倒车）
def test_boundary_speed() -> None:
    """边界情况：负速度不再被静默钳位，而是配置阶段报错 / 状态层可观测。

    对应评审意见：「负速度（倒车）的时候直接把速度钳到 0，不报错但行为不对」。
    """

    def expect_error(fn, msg: str) -> None:
        try:
            fn()
        except ValueError:
            check(True, msg)
        else:
            check(False, msg + "（未抛 ValueError）")

    # 1) 配置阶段就拒绝非法的巡航速度
    expect_error(lambda: SpeedProfile(cruise_speed=-3.0), "cruise_speed 为负时报 ValueError")
    expect_error(lambda: SpeedProfile(cruise_speed=0.0), "cruise_speed 为 0 时报 ValueError")
    expect_error(lambda: SpeedProfile(cruise_speed=8.0, min_speed=-1.0),
                 "min_speed 为负时报 ValueError")
    expect_error(lambda: SpeedProfile(cruise_speed=2.0, min_speed=5.0),
                 "min_speed 大于 cruise_speed 时报 ValueError")
    expect_error(lambda: SpeedProfile(a_lat_max=0.0), "a_lat_max 为 0 时报 ValueError")
    expect_error(lambda: VehicleParams(max_decel=-1.0), "max_decel 为负时报 ValueError")
    expect_error(lambda: SimulationConfig(initial_speed=-2.0),
                 "initial_speed 为负时报 ValueError")
    expect_error(lambda: SimulationConfig(model="bicycle"),
                 "未知车辆模型时报 ValueError")

    # 2) 不允许倒车（默认）：速度钳在 0，且拦截步数可观测
    mdl = KinematicBicycle()
    state = np.array([0.0, 0.0, 0.0, 0.0])
    for _ in range(10):
        state = mdl.step(state, 0.0, -3.0, 0.02)
    check(state[3] >= 0.0, f"默认不允许倒车：速度不为负（v={state[3]:.6f}）")
    check(mdl.reverse_blocked_steps == 10,
          f"拦截步数被记录（{mdl.reverse_blocked_steps} 步）")
    check(abs(state[0]) < 1e-9, f"被拦截时不产生反向位移（x={state[0]:.2e}）")

    # 3) 显式允许倒车：保留负速度并向后运动
    mdl_rev = KinematicBicycle(VehicleParams(allow_reverse=True))
    st = np.array([0.0, 0.0, 0.0, 0.0])
    for _ in range(10):
        st = mdl_rev.step(st, 0.0, -3.0, 0.02)
    check(st[3] < 0.0, f"允许倒车时速度为负（v={st[3]:.4f}）")
    check(st[0] < -1e-6, f"允许倒车时产生反向位移（x={st[0]:.4f}）")
    check(mdl_rev.reverse_blocked_steps == 0, "允许倒车时不计拦截步数")

    # 4) 动力学模型行为一致
    dyn = DynamicBicycle()
    st = np.array([0.0, 0.0, 0.0, 0.0])
    for _ in range(10):
        st, _mem = dyn.step(st, 0.0, -3.0, 0.02)
    check(st[3] >= 0.0 and dyn.reverse_blocked_steps > 0,
          f"动力学模型同样不倒退（v={st[3]:.6f}, 拦截 {dyn.reverse_blocked_steps} 步）")

    # 5) 指标里能查到该字段，且正常场景下为 0
    path = straight_line(length=60.0)
    cfg = SimulationConfig(dt=0.02, speed=SpeedProfile(cruise_speed=8.0))
    m = evaluate(Simulator(path, cfg).run(Stanley(), "s"))
    check("reverse_blocked_steps" in m.as_dict(), "指标字典包含 reverse_blocked_steps")
    check(m.as_dict()["reverse_blocked_steps"] == 0,
          "正常场景不会触发倒车拦截（计数值为 0）")

    # 6) 每次 run 独立计数，不累加历史
    sim = Simulator(path, cfg)
    sim.run(Stanley(), "a")
    sim.run(Stanley(), "b")
    check(sim.model.reverse_blocked_steps == 0, "重复 run 时计数器按次重置")


# --------------------------------------------------- 8. CSV 编码（无 BOM）
def test_csv_encoding() -> None:
    """CSV 不带 BOM：Excel 打开首列表头不会出现乱码。"""
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_test_output")
    os.makedirs(out_dir, exist_ok=True)
    path = straight_line(length=40.0)
    cfg = SimulationConfig(dt=0.02, speed=SpeedProfile(cruise_speed=8.0))
    tr = Simulator(path, cfg).run(Stanley(), "straight")
    m = evaluate(tr)

    try:
        csv_path = metrics_to_csv([m], os.path.join(out_dir, "m.csv"))
        with open(csv_path, "rb") as fh:
            head = fh.read(3)
        check(head != b"\xef\xbb\xbf", f"指标 CSV 无 BOM（前三字节 {head!r}）")
        with open(csv_path, "r", encoding="utf-8") as fh:
            first = fh.readline()
        check(first.startswith("controller,"),
              f"首列表头以字段名开头（{first[:24]!r}）")

        trace_path = trace_to_csv(tr, os.path.join(out_dir, "t.csv"))
        with open(trace_path, "rb") as fh:
            thead = fh.read(3)
        check(thead != b"\xef\xbb\xbf", f"轨迹 CSV 无 BOM（前三字节 {thead!r}）")
    finally:
        for name in ("m.csv", "t.csv"):
            p = os.path.join(out_dir, name)
            if os.path.isfile(p):
                os.remove(p)


# ------------------------------------------------- 9. 动力学模型（dynamic）
def test_dynamic_model_closed_loop() -> None:
    """`--model dynamic` 的回归保护。

    文档 §5 写明可用 `--model dynamic` 切换二自由度动力学模型（含轮胎侧偏刚度），
    `simulation.py` 里也实现了该分支，但此前没有任何测试真正走 ``model="dynamic"``
    的闭环路径——只有 `DynamicBicycle` 的单步单元测试。
    也就是说：若有人改动 dynamic 分支（轮胎模型、积分顺序、v_min 退化条件），
    现有测试不会报错。这里补上闭环级别的冒烟保护（评审 #422 指出）。
    """
    path = circular_arc(radius=20.0)
    cfg = SimulationConfig(dt=0.02, model="dynamic", duration=20.0,
                           speed=SpeedProfile(cruise_speed=8.0))
    tr = Simulator(path, cfg).run(Stanley(), "arc_dynamic")

    # 1) 不发散：状态与输出全程有限
    check(bool(np.all(np.isfinite(tr.x))) and bool(np.all(np.isfinite(tr.y))),
          "dynamic: 轨迹坐标全程有限（无 NaN / inf）")
    check(bool(np.all(np.isfinite(tr.e))) and bool(np.all(np.isfinite(tr.delta))),
          "dynamic: 横向偏差与转角全程有限")
    check(bool(np.all(np.isfinite(tr.v))), "dynamic: 车速全程有限")

    # 2) 误差有界（圆弧半径 20 m，留足余量：允许 2 m）
    e_max = float(np.abs(tr.e).max())
    check(e_max < 2.0, f"dynamic: 最大横向偏差有界（{e_max:.4f} m < 2.0 m）")

    # 3) 能收敛：末段稳态偏差与收敛时间都应在合理范围
    m = evaluate(tr)
    check(m.e_steady_abs < 0.30,
          f"dynamic: 末段稳态偏差 < 0.30 m（实际 {m.e_steady_abs:.4f} m）")
    check(m.convergence_time < m.duration,
          f"dynamic: 能在时长内收敛（{m.convergence_time:.2f} s < {m.duration:.1f} s）")
    for name, value in (("e_rms", m.e_rms), ("v_mean", m.v_mean),
                        ("delta_max", m.delta_max_abs_deg)):
        check(bool(np.isfinite(value)), f"dynamic: 指标 {name} 为有限值（{value:.4f}）")

    # 4) 车速不越界：既不倒车（默认不允许），也不超过巡航速度
    check(float(tr.v.min()) >= 0.0, f"dynamic: 车速不为负（最小 {tr.v.min():.4f} m/s）")
    check(float(tr.v.max()) <= 8.0 + 1e-6, f"dynamic: 车速不超过巡航值（最大 {tr.v.max():.4f} m/s）")


def test_dynamic_low_speed_fallback() -> None:
    """dynamic 模型在低于 ``v_min`` 时退化为运动学模型，不应出现除零或 NaN。

    ``DynamicBicycle.derivative`` 在 ``v < v_min`` 时改用运动学方程（否则式中含 1/v），
    这条退化分支此前也没有闭环覆盖。
    """
    path = straight_line(length=30.0)
    cfg = SimulationConfig(dt=0.02, model="dynamic", duration=12.0,
                           initial_speed=1.0,
                           speed=SpeedProfile(cruise_speed=1.5, min_speed=1.0))
    tr = Simulator(path, cfg).run(PurePursuit(), "low_speed_dynamic")
    check(bool(np.all(np.isfinite(tr.e))),
          "dynamic 低速退化：横向偏差全程有限（无除零 NaN）")
    check(bool(np.all(np.isfinite(tr.v))), "dynamic 低速退化：车速全程有限")
    m = evaluate(tr)
    check(np.isfinite(m.e_rms) and m.e_rms < 1.0,
          f"dynamic 低速退化：误差 RMS 有界（{m.e_rms:.4f} m）")


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
    test_boundary_speed()
    test_csv_encoding()
    test_dynamic_model_closed_loop()
    test_dynamic_low_speed_fallback()
    print("\n" + "=" * 72)
    print(f"通过 {PASS} 项，失败 {FAIL} 项")
    print("=" * 72)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
