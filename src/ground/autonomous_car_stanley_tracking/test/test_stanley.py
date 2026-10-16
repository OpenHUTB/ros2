#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stanley 跟踪模块的离线单元测试。

这些测试**不需要 ROS 2、matplotlib 或任何仿真器**，
只用 numpy 校验几何、控制器定律、仿真器和指标计算。

运行::

    python3 test/test_stanley.py
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

from stanley_tracking import benchmark  # noqa: E402
from stanley_tracking.controllers import (  # noqa: E402
    AdaptivePurePursuitController,
    LookaheadPIDController,
    StanleyController,
    build_controller,
)
from stanley_tracking.geometry import ReferencePath, wrap_to_pi  # noqa: E402
from stanley_tracking.reference import (  # noqa: E402
    build_scenario,
    circular_arc,
    double_lane_change,
    from_waypoints,
    sine_wave,
    straight_line,
)
from stanley_tracking.simulator import SimulationConfig, Simulator, SpeedProfile, evaluate  # noqa: E402

PASSED = 0
FAILED = 0


def check(condition: bool, message: str) -> None:
    """极简断言：累计通过/失败计数并打印结果。"""
    global PASSED, FAILED
    if condition:
        PASSED += 1
        print(f"  [PASS] {message}")
    else:
        FAILED += 1
        print(f"  [FAIL] {message}")


def assert_close(value: float, expected: float, tolerance: float, message: str) -> None:
    check(abs(value - expected) <= tolerance,
          f"{message}（实际 {value:.6f}，期望 {expected:.6f} ± {tolerance:g}）")


# --------------------------------------------------------------------- 几何
def test_geometry() -> None:
    print("\n[1] 平面几何与参考路径")
    assert_close(float(wrap_to_pi(0.0)), 0.0, 1e-12, "wrap_to_pi(0) = 0")
    # 归一化区间是 (-pi, pi]：3pi 与 -3pi 都落在边界，返回 -pi
    assert_close(abs(float(wrap_to_pi(3.0 * math.pi))), math.pi, 1e-9, "wrap_to_pi(3pi) = ±pi")
    assert_close(abs(float(wrap_to_pi(-3.0 * math.pi))), math.pi, 1e-9, "wrap_to_pi(-3pi) = ±pi")
    assert_close(float(wrap_to_pi(2.5 * math.pi)), 0.5 * math.pi, 1e-9, "wrap_to_pi(2.5pi) = pi/2")

    line = straight_line(length=20.0, step=0.5)
    assert_close(line.total_length, 20.0, 1e-9, "直线路径总长度 = 20 m")
    projection = line.project(np.array([10.0, 2.0]))
    assert_close(projection.signed_distance, 2.0, 1e-9, "路径左侧点横向偏差为正")
    projection = line.project(np.array([10.0, -3.0]))
    assert_close(projection.signed_distance, -3.0, 1e-9, "路径右侧点横向偏差为负")
    assert_close(projection.s, 10.0, 1e-9, "投影点弧长 s = 10 m")

    arc = circular_arc(radius=15.0, sweep_deg=90.0, step_deg=0.5)
    mid = arc.project(arc.points[len(arc) // 2])
    assert_close(mid.curvature, 1.0 / 15.0, 2e-3, "圆弧中段曲率 ≈ 1/R")
    assert_close(mid.signed_distance, 0.0, 1e-9, "路径上的点横向偏差为 0")

    # 闭合圆弧的首点曲率必须与其他折点一致（曾经因为用错线段而偏大一倍）
    closed = circular_arc(radius=15.0, sweep_deg=360.0, step_deg=1.0)
    check(closed.closed, "360° 圆弧标记为闭合路径")
    assert_close(closed.vertex_curvature[0], 1.0 / 15.0, 2e-3,
                 "闭合圆弧首点曲率 ≈ 1/R（不再是两倍）")
    assert_close(closed.vertex_curvature[0], closed.vertex_curvature[1], 1e-6,
                 "闭合圆弧首点曲率与相邻折点一致")
    assert_close(closed.total_length, 2.0 * math.pi * 15.0, 0.05,
                 "闭合圆弧总长度 ≈ 2*pi*R")

    # 曲率方向：圆弧向左弯，曲率应为正
    check(arc.curvature_at_s(5.0) > 0.0, "左转弯圆弧曲率为正")


# ----------------------------------------------------------------- 控制器
def front_axle_on_path(front_xy, yaw: float, wheelbase: float = 2.7) -> np.ndarray:
    """构造后轴位姿，使**前轴中心**恰好落在给定点上。

    Stanley 律同时使用航向偏差与前轴横向偏差，测试时必须把两者解耦：
    固定前轴位置、只改变车头朝向，才能单独检验航向偏差项；
    把车头对准路径切向，才能单独检验横向偏差项。
    前轴位置 p_fa 与后轴位置 p_ra 的关系是 p_ra = p_fa - L * (cos yaw, sin yaw)。
    """
    x = front_xy[0] - wheelbase * math.cos(yaw)
    y = front_xy[1] - wheelbase * math.sin(yaw)
    return np.array([x, y, yaw])


def test_controller_law() -> None:
    print("\n[2] Stanley 控制律")
    path = straight_line(length=40.0)
    controller = StanleyController(k=1.6, k_soft=1.2, wheelbase=2.7, use_feedforward=True)

    # 情况一：车头对准路径、整体位于路径左侧 1 m。
    # 前轴中心在 (2.7, 1.0)，横向偏差 e = +1.0 m（左偏为正）。
    # 车辆在路径左侧必须**右转**（delta < 0）才能回到路径，
    # 因此横向偏差项取负号：delta = psi_e - atan(k*e/(k_soft+v))，此处 psi_e = 0。
    # 这里同样把 dt 取大一些，避免 120°/s 的转向速率限幅把结果截断：
    # 期望转角约 14.5°，dt=0.15 s 时允许的单步变化为 18°，不会触发限幅。
    state = np.array([0.0, 1.0, 0.0])
    projection = path.project(state[:2])
    command = controller.compute(state, 5.0, path, projection, 0.15)
    assert_close(command.cross_track_error, 1.0, 1e-9, "前轴横向偏差 = +1.0 m")
    check(command.heading_error == 0.0, "车头对准路径时航向偏差为 0")
    check(command.steering < 0.0, "左偏时应右转修正（转角为负）")
    assert_close(command.steering, -math.atan(1.6 * 1.0 / (1.2 + 5.0)), 1e-9,
                 "左偏 1 m 时转角 = -atan(k*e/(k_soft+v))")

    # 情况二：前轴落在路径上、车头相对路径左偏 10°。
    # 航向偏差 psi_e = path_yaw - yaw = -10°；由于默认控制点已改为后轴，
    # 后轴中心在路径右侧、横向偏差不为 0，因此转角不等于 psi_e，
    # 这里只校验航向偏差本身的定义与符号，以及前轴控制点下的偏差取值。
    controller.reset()
    yaw_error = math.radians(10.0)
    state = front_axle_on_path((0.0, 0.0), yaw_error)
    projection = path.project(state[:2])
    command = controller.compute(state, 5.0, path, projection, 0.1)
    assert_close(command.heading_error, -yaw_error, 1e-9,
                 "航向偏差 = 路径切向 - 车头朝向（车头左偏时为负）")
    check(command.steering < 0.0, "车头左偏时转角为负（右转修正）")

    # 同一状态下改用前轴控制点：前轴恰好落在路径上，横向偏差应为 0，
    # 此时转角完全由航向偏差决定
    front_controller = StanleyController(control_point="front_axle")
    front_command = front_controller.compute(state, 5.0, path, projection, 0.1)
    assert_close(front_command.cross_track_error, 0.0, 1e-9, "前轴控制点时横向偏差为 0")
    assert_close(front_command.steering, -yaw_error, 1e-6, "前轴控制点时转角 = 航向偏差")

    # 情况三：直的参考路径上，曲率前馈为 0
    assert_close(command.feedforward, 0.0, 1e-12, "直线路径的前馈项为 0")

    # 情况四：圆弧场景下前馈项应给出与曲率匹配的转角 atan(L*kappa)
    arc = circular_arc(radius=20.0)
    controller.reset()
    state = np.array([0.0, 0.0, 0.0])
    projection = arc.project(state[:2])
    command = controller.compute(state, 5.0, arc, projection, 0.02)
    assert_close(command.feedforward, math.atan(2.7 / 20.0), 2e-3,
                 "圆弧前馈转角 ≈ atan(L/R)")

    # 情况五：转角必须被最大转角限幅
    controller.reset()
    far = np.array([0.0, 50.0, 0.0])
    projection = path.project(far[:2])
    command = controller.compute(far, 5.0, path, projection, 0.02)
    check(abs(command.steering) <= controller.max_steering + 1e-12, "转角不超过最大转角")

    # 情况六：转向速率限制
    controller.reset()
    limited = StanleyController(k=5.0, k_soft=0.1, steering_rate_limit=math.radians(30.0))
    state = np.array([0.0, 10.0, 0.0])
    projection = path.project(state[:2])
    command = limited.compute(state, 5.0, path, projection, 0.5)
    check(abs(command.steering) <= math.radians(30.0) * 0.5 + 1e-9,
          "单步转角变化不超过速率限制")


def test_controller_variants() -> None:
    print("\n[3] 对照控制器")
    path = straight_line(length=40.0)
    pursuit = AdaptivePurePursuitController()
    assert_close(pursuit.lookahead_distance(0.0), pursuit.lookahead_min, 1e-12,
                 "低速时前视距离取下限")
    assert_close(pursuit.lookahead_distance(100.0), pursuit.lookahead_max, 1e-12,
                 "高速时前视距离取上限")
    state = np.array([0.0, 1.0, 0.0])
    projection = path.project(state[:2])
    command = pursuit.compute(state, 5.0, path, projection, 0.02)
    check(command.steering < 0.0, "纯跟踪在左偏时同样向右修正")

    pid = LookaheadPIDController(lookahead=6.0)
    pid.reset()
    state = np.array([0.0, 1.0, 0.0])          # 车头对准路径，整体在路径左侧
    projection = path.project(state[:2])
    command = pid.compute(state, 5.0, path, projection, 0.02)
    # 预瞄点在 (6, 0)，位于车体（yaw=0，位于 (0,1)）右侧，
    # 车体坐标系下的横向偏差为 e = (0,1)·(-1) = -1 m
    check(command.cross_track_error < 0.0, "预瞄点位于车体右侧（偏差为负）")
    check(command.steering < 0.0, "预瞄 PID 判定偏右时向右修正")
    check(abs(pid._integral) > 0.0, "PID 积分项已累积")


# ------------------------------------------------------------------- 仿真
def test_simulator_and_metrics() -> None:
    print("\n[4] 闭环仿真与指标")
    path = straight_line(length=60.0)
    config = SimulationConfig(dt=0.02, speed_profile=SpeedProfile(target_speed=5.0))
    controller = StanleyController()
    trace = Simulator(path, config).run(controller, scenario="straight")
    metric = evaluate(trace)
    check(len(trace) > 100, "仿真步数大于 100")
    check(np.all(np.isfinite(trace.x)) and np.all(np.isfinite(trace.y)), "轨迹数值有限")
    check(metric.cross_track_max_abs < 2.0, f"最大横向偏差 < 2 m（实际 {metric.cross_track_max_abs:.3f}）")
    check(metric.cross_track_steady_abs < 0.05,
          f"稳态横向偏差 < 0.05 m（实际 {metric.cross_track_steady_abs:.4f}）")
    check(metric.convergence_time < 10.0,
          f"10 s 内收敛到 0.2 m 以内（实际 {metric.convergence_time:.2f} s）")

    # 初始就对准路径时，偏差应始终很小
    aligned = SimulationConfig(dt=0.02, initial_state=np.array([0.0, 0.0, 0.0]),
                               speed_profile=SpeedProfile(target_speed=5.0))
    trace = Simulator(path, aligned).run(StanleyController(), scenario="straight_aligned")
    metric = evaluate(trace)
    check(metric.cross_track_max_abs < 0.02,
          f"初始对准时最大偏差 < 0.02 m（实际 {metric.cross_track_max_abs:.4f}）")

    # 侧向风扰（0.3 m/s）下仍应收敛：Stanley 没有积分项，
    # 稳态会留下一个与风扰相平衡的残余偏差（实测约 0.4 m），
    # 这里只要求它不发散、且比无控漂移小一个量级。
    windy = SimulationConfig(dt=0.02, lateral_wind=0.3, speed_profile=SpeedProfile(target_speed=5.0))
    trace = Simulator(path, windy).run(StanleyController(), scenario="straight_wind")
    metric = evaluate(trace)
    check(metric.cross_track_steady_abs < 0.5,
          f"侧向风扰下稳态偏差 < 0.5 m（实际 {metric.cross_track_steady_abs:.4f}）")
    check(metric.cross_track_max_abs < 2.5,
          f"侧向风扰下最大偏差有界（实际 {metric.cross_track_max_abs:.4f}）")


def test_scenarios() -> None:
    print("\n[5] 参考场景")
    arc = build_scenario("circle")
    check(arc.closed, "圆弧场景是闭合路径")
    lane = double_lane_change(length=120.0, width=3.5)
    check(abs(float(lane.points[:, 1].max())) > 1.0, "双移线场景存在横向位移")
    sine = sine_wave(amplitude=2.0, wavelength=20.0, length=60.0)
    check(abs(float(sine.points[:, 1].max()) - 2.0) < 0.2, "正弦场景幅值正确")
    waypoints = from_waypoints([(0, 0), (5, 0), (10, 5), (15, 5)])
    assert_close(waypoints.total_length, 5.0 + math.hypot(5, 5) + 5.0, 1e-6,
                 "航点路径长度累计正确")

    # 三种控制器在圆弧上都能稳定跟踪。
    # 注意阈值不同：Stanley 有曲率前馈，稳态偏差最小；
    # 预瞄 PID 没有前馈，只能靠积分项慢慢压偏差，稳态残差明显更大（实测约 0.8 m），
    # 这正是引入前馈的理由，因此给它单独的、更宽松的判据。
    config = SimulationConfig(dt=0.02, speed_profile=SpeedProfile(target_speed=4.0),
                              max_time=120.0)
    thresholds = {"stanley": 0.5, "pure_pursuit": 0.5, "lookahead_pid": 1.2}
    for name in ("stanley", "pure_pursuit", "lookahead_pid"):
        controller = build_controller(name)
        trace = Simulator(arc, config).run(controller, scenario="circle")
        metric = evaluate(trace)
        check(metric.cross_track_steady_abs < thresholds[name],
              f"{name} 在圆弧上稳态偏差 < {thresholds[name]} m"
              f"（实际 {metric.cross_track_steady_abs:.4f}）")


def test_benchmark_helpers() -> None:
    print("\n[6] 批量实验与导出")
    scenarios = ["straight"]
    controllers = {"stanley": build_controller("stanley")}
    traces, metrics = benchmark.run_matrix(controllers, scenarios, benchmark.make_config())
    check(len(traces) == 1 and len(metrics) == 1, "矩阵实验返回 1 组结果")
    table = benchmark.format_table(metrics)
    check("stanley" in table and "steady|e|" in table, "指标表格包含控制器名与列名")

    # 刻意不使用 tempfile.TemporaryDirectory：在受限环境下系统临时目录可能不可写，
    # 这里直接写到模块内的 _test_output 目录，跑完删除，保证任何机器上都能运行。
    out_dir = os.path.join(_ROOT, "_test_output")
    os.makedirs(out_dir, exist_ok=True)
    try:
        csv_path = benchmark.metrics_to_csv(metrics, os.path.join(out_dir, "m.csv"))
        json_path = benchmark.metrics_to_json(metrics, os.path.join(out_dir, "m.json"))
        trace_path = benchmark.trace_to_csv(traces[0], os.path.join(out_dir, "t.csv"))
        check(os.path.getsize(csv_path) > 0 and os.path.getsize(json_path) > 0,
              "指标 CSV/JSON 已写出且非空")
        check(os.path.getsize(trace_path) > 0, "轨迹时序 CSV 已写出且非空")
    finally:
        for name in ("m.csv", "m.json", "t.csv"):
            target = os.path.join(out_dir, name)
            if os.path.isfile(target):
                os.remove(target)

    sweep = benchmark.run_sweep("straight", "k", [0.5, 2.0])
    check(len(sweep) == 2, "参数扫描返回 2 组结果")
    check(all(m.cross_track_steady_abs < 0.2 for m in sweep), "两组增益都能收敛到 0.2 m 以内")


def test_control_point_comparison() -> None:
    """回归测试：后轴控制点的稳态偏差显著优于前轴控制点。

    这是本模块的一个实测结论（见 docs 与 README）：
    前轴比车辆实际位置超前一个轴距，前馈在车辆前方取值，弯道上会引入稳态偏差。
    """
    print("\n[7] 控制点对比（后轴 vs 前轴）")
    arc = circular_arc(radius=15.0)
    config = SimulationConfig(dt=0.02, speed_profile=SpeedProfile(target_speed=4.0),
                              max_time=120.0)
    rear = evaluate(Simulator(arc, config).run(
        StanleyController(control_point="rear_axle"), scenario="R15_rear"))
    front = evaluate(Simulator(arc, config).run(
        StanleyController(control_point="front_axle"), scenario="R15_front"))
    check(rear.cross_track_steady_abs < 0.05,
          f"后轴控制点稳态偏差 < 0.05 m（实际 {rear.cross_track_steady_abs:.4f}）")
    check(front.cross_track_steady_abs > 3.0 * rear.cross_track_steady_abs,
          f"前轴控制点稳态偏差明显更大（后轴 {rear.cross_track_steady_abs:.4f}，"
          f"前轴 {front.cross_track_steady_abs:.4f}）")


def main() -> int:
    print("=" * 68)
    print("Stanley 横向跟踪模块 离线测试（不依赖 ROS / 仿真器）")
    print("=" * 68)
    test_geometry()
    test_controller_law()
    test_controller_variants()
    test_simulator_and_metrics()
    test_scenarios()
    test_benchmark_helpers()
    test_control_point_comparison()
    print("\n" + "=" * 68)
    print(f"通过 {PASSED} 项，失败 {FAILED} 项")
    print("=" * 68)
    return 0 if FAILED == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
