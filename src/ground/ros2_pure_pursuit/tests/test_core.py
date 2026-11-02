"""核心算法单元测试。

运行：
    python3 -m pytest tests -v
或：
    python3 tests/test_core.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from ros2_pure_pursuit.arbitrate import arbitrate  # noqa: E402
from ros2_pure_pursuit.obstacle import AvoidInfo, ObstacleAvoider, synthetic_scan  # noqa: E402
from ros2_pure_pursuit.paths import circle, load_path, straight  # noqa: E402
from ros2_pure_pursuit.pure_pursuit import PurePursuitController  # noqa: E402
from ros2_pure_pursuit.robot import DiffDriveRobot  # noqa: E402
from ros2_pure_pursuit.safety import FreshnessTracker, sanitize_target_speed  # noqa: E402


def test_straight_motion_distance():
    """直行 1 秒应前进 v * t 的距离，且朝向不变（关闭加速度限幅）。"""
    r = DiffDriveRobot(dt=0.05, max_accel=100.0)
    r.reset(0.0, 0.0, 0.0)
    for _ in range(20):
        r.step(1.0, 0.0)
    assert abs(r.pose.x - 1.0) < 1e-6
    assert abs(r.pose.y) < 1e-6
    assert abs(r.pose.theta) < 1e-6


def test_acceleration_limit_delays_speed():
    """默认加速度限幅下，速度不能瞬间跳变到指令值。"""
    r = DiffDriveRobot(dt=0.05, max_accel=1.5)
    r.reset(0.0, 0.0, 0.0)
    r.step(1.0, 0.0)
    assert r.v < 1.0
    assert abs(r.v - 1.5 * 0.05) < 1e-9


def test_turn_radius_matches_kinematics():
    """纯角速度运动时转弯半径应为 v / w。

    圆弧上任一点到原点的距离为 2R sin(theta/2)，据此反解半径。
    """
    v, w = 0.8, 0.8
    r = DiffDriveRobot(dt=0.01, max_accel=100.0)
    r.reset(0.0, 0.0, 0.0)
    for _ in range(628):
        r.step(v, w)
    chord = math.hypot(r.pose.x, r.pose.y)
    radius = chord / (2.0 * math.sin(abs(r.pose.theta) / 2.0))
    assert abs(radius - v / w) < 0.05


def test_wheel_speed_inverse():
    """左右轮转速反解应满足 (vl + vr) / 2 = v。"""
    r = DiffDriveRobot(wheel_base=0.4, wheel_radius=0.1)
    r.v, r.w = 1.0, 0.5
    wl, wr = r.wheel_speeds()
    assert abs((wl + wr) / 2 * r.wheel_radius - r.v) < 1e-9


def test_pure_pursuit_converges_on_straight_path():
    """从偏离处出发，纯追踪应收敛到直线路径且横向误差小于 5 cm。"""
    path = straight(length=20.0)
    ctrl = PurePursuitController()
    r = DiffDriveRobot(dt=0.05)
    r.reset(x=0.0, y=0.6, theta=0.0)
    errors = []
    for _ in range(400):
        _, _, info = ctrl.compute(path, r.pose, 0.8)
        r.step(0.8, ctrl.compute(path, r.pose, 0.8)[1])
        errors.append(abs(info.cte))
    assert errors[-1] < 0.05
    assert float(np.mean(errors[-50:])) < 0.05


def test_pure_pursuit_tracks_circle():
    """圆形路径上跟踪误差应保持较小。"""
    path = circle(radius=8.0)
    ctrl = PurePursuitController()
    r = DiffDriveRobot(dt=0.05)
    p0, p1 = path.points[0], path.points[1]
    r.reset(p0[0], p0[1], math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    errors = []
    for _ in range(600):
        _, w, info = ctrl.compute(path, r.pose, 0.9)
        r.step(0.9, w)
        errors.append(abs(info.cte))
    assert float(np.mean(errors[100:])) < 0.15


def test_lookahead_distance_speed_adaptive():
    """前视距离应随速度增大，并被 ld_max 截断。"""
    ctrl = PurePursuitController(ld0=0.6, k=0.45, ld_min=0.4, ld_max=2.5)
    assert ctrl.lookahead_distance(0.0) == 0.6
    assert ctrl.lookahead_distance(1.0) > ctrl.lookahead_distance(0.0)
    assert ctrl.lookahead_distance(100.0) == 2.5


def test_avoider_no_obstacle_is_neutral():
    """空旷环境下避障模块不应干预速度与转向。"""
    av = ObstacleAvoider()
    ranges = np.full(360, 10.0)
    info = av.evaluate(ranges, -math.pi, 2 * math.pi / 360)
    assert info.risk == 0.0
    assert info.turn_bias == 0.0
    assert info.slow_factor == 1.0


def test_avoider_detects_front_obstacle():
    """正前方 0.5 m 处障碍应触发危险度并给出转向偏置。"""
    av = ObstacleAvoider(safe_distance=1.2, danger_distance=0.45)
    ranges = np.full(360, 10.0)
    # 车头方向为角度 0，即索引 180
    ranges[175:186] = 0.5
    info = av.evaluate(ranges, -math.pi, 2 * math.pi / 360)
    assert info.risk > 0.5
    assert info.slow_factor < 1.0
    assert abs(info.turn_bias) > 0.0


def test_synthetic_scan_range_is_correct():
    """合成激光在正前方障碍处测得的距离应接近真实切线距离。"""
    ranges, amin, ainc = synthetic_scan(0.0, 0.0, 0.0, [(3.0, 0.0, 0.5)])
    idx = int((0.0 - amin) / ainc)
    assert ranges[idx] < 3.0
    assert abs(ranges[idx] - (3.0 - 0.5)) < 0.1


def test_path_geometry_helpers():
    """路径工具函数应给出正确长度与最近点索引。"""
    c = circle(radius=8.0)
    assert abs(c.length - 2 * math.pi * 8.0) < 0.5
    assert c.closed is True
    idx = c.nearest_index(8.0, 0.0)
    assert abs(c.points[idx, 0] - 8.0) < 0.2


def test_load_path_with_and_without_header():
    """csv 路径加载应同时兼容带表头与不带表头。"""
    import tempfile

    with tempfile.TemporaryDirectory() as d:
        with_header = os.path.join(d, "with_header.csv")
        without_header = os.path.join(d, "without_header.csv")
        with open(with_header, "w", encoding="utf-8") as f:
            f.write("x,y\n0,0\n1,0\n2,0\n")
        with open(without_header, "w", encoding="utf-8") as f:
            f.write("0,0\n1,0\n2,0\n")

        p1 = load_path(with_header)
        p2 = load_path(without_header)
        assert len(p1) == 3
        assert len(p2) == 3
        assert abs(p1.points[-1, 0] - 2.0) < 1e-9


def test_arbitration_finished_wins_over_avoidance():
    """回归：到达终点必须停车，即使此刻避障正要求转向/减速。

    早期实现把终点判断写成 `if 避障... elif finished`，导致开着避障并收到
    激光时终点永远判不到，机器人会绕着终点继续跑。本测试锁定该行为。
    """
    danger = AvoidInfo(min_front=0.30, risk=1.0, turn_bias=1.6, slow_factor=0.15)
    res = arbitrate(v_pursuit=0.8, w_pursuit=0.3, finished=True, avoid=danger)
    assert res.v == 0.0
    assert res.w == 0.0
    assert res.reason == "finished"
    assert res.applied_avoidance is False


def test_arbitration_finished_without_lidar_also_stops():
    """无激光数据时到达终点同样应停车。"""
    res = arbitrate(0.8, 0.3, finished=True, avoid=None)
    assert (res.v, res.w) == (0.0, 0.0)
    assert res.reason == "finished"


def test_arbitration_pursuit_passthrough():
    """未到终点且无避障输入时，应原样采纳纯追踪输出。"""
    res = arbitrate(0.8, 0.3, finished=False, avoid=None)
    assert res.v == 0.8
    assert res.w == 0.3
    assert res.reason == "pursuit"
    assert res.applied_avoidance is False


def test_arbitration_avoidance_overrides_pursuit():
    """未到终点且有障碍时，应在纯追踪输出基础上叠加转向偏置与减速。"""
    av = AvoidInfo(min_front=0.5, risk=0.8, turn_bias=-0.9, slow_factor=0.32)
    res = arbitrate(1.0, 0.2, finished=False, avoid=av)
    assert abs(res.v - 1.0 * 0.32) < 1e-9
    assert abs(res.w - (0.2 + (-0.9))) < 1e-9
    assert res.reason == "avoidance"
    assert res.applied_avoidance is True


def test_arbitration_neutral_avoid_info_changes_nothing():
    """前方空旷（危险度为 0）时，避障层不应引入任何扰动。"""
    neutral = AvoidInfo(min_front=10.0, risk=0.0, turn_bias=0.0, slow_factor=1.0)
    res = arbitrate(0.8, 0.3, finished=False, avoid=neutral)
    assert res.v == 0.8
    assert res.w == 0.3
    assert res.reason == "pursuit"          # 未真正介入，仍记作纯追踪
    assert res.applied_avoidance is False


def test_avoider_emergency_stop_inside_danger_distance():
    """回归：障碍进入 danger_distance 内必须急停，而不是只保留 15% 速度。

    早期 slow_factor 有 0.15 下限，risk=1 时机器人仍在以 15% 速度逼近障碍。
    """
    av = ObstacleAvoider(safe_distance=1.2, danger_distance=0.45)
    ranges = np.full(360, 10.0)
    ranges[175:186] = 0.30          # 正前方 0.30 m < danger_distance
    info = av.evaluate(ranges, -math.pi, 2 * math.pi / 360)
    assert info.emergency is True
    assert info.slow_factor == 0.0
    assert abs(info.turn_bias) > 0.0   # 仍需给出转向方向以便脱离


def test_avoider_no_emergency_just_outside_danger():
    """危险距离之外（0.6 m）只减速，不急停。"""
    av = ObstacleAvoider(safe_distance=1.2, danger_distance=0.45)
    ranges = np.full(360, 10.0)
    ranges[175:186] = 0.60
    info = av.evaluate(ranges, -math.pi, 2 * math.pi / 360)
    assert info.emergency is False
    assert 0.0 < info.slow_factor < 1.0


def test_arbitration_emergency_stop_zero_speed_keeps_turn():
    """急停时线速度必须为 0，但保留转向以便原地转离障碍。"""
    danger = AvoidInfo(min_front=0.30, risk=1.0, turn_bias=-1.2, slow_factor=0.0,
                       emergency=True)
    res = arbitrate(0.8, 0.1, finished=False, avoid=danger)
    assert res.v == 0.0
    assert abs(res.w - (0.1 - 1.2)) < 1e-9
    assert res.reason == "emergency_stop"
    assert res.applied_avoidance is True


def test_arbitration_stale_odom_stops_robot():
    """回归：定位数据过期时必须停车，不许沿用最后一帧位姿继续跑。"""
    res = arbitrate(0.8, 0.3, finished=False, avoid=None, odom_stale=True)
    assert (res.v, res.w) == (0.0, 0.0)
    assert res.reason == "stale_odom"


def test_arbitration_stale_odom_wins_over_finished():
    """定位过期时连“是否到终点”都无法确认，停车优先级高于终点分支。"""
    res = arbitrate(0.8, 0.3, finished=True, avoid=None, odom_stale=True)
    assert res.reason == "stale_odom"
    assert (res.v, res.w) == (0.0, 0.0)


def test_arbitration_angular_speed_is_clamped():
    """转向偏置叠加后角速度不得超过 max_angular。"""
    av = AvoidInfo(min_front=0.6, risk=0.8, turn_bias=1.6, slow_factor=0.32)
    res = arbitrate(1.0, 1.2, finished=False, avoid=av, max_angular=1.8)
    assert res.w == 1.8
    res_neg = arbitrate(1.0, -1.2, finished=False,
                        avoid=AvoidInfo(0.6, 0.8, -1.6, 0.32), max_angular=1.8)
    assert res_neg.w == -1.8


def test_freshness_tracker_detects_stale_and_fresh():
    """超时判定：未超时为 fresh，超过阈值为 stale。"""
    ft = FreshnessTracker()
    ft.register("odom", 0.5)
    ft.register("scan", 0.5)
    assert ft.is_stale("odom", 0.0) is True       # 从未收到过
    ft.mark("odom", 10.0)
    assert ft.is_fresh("odom", 10.3) is True
    assert ft.is_stale("odom", 10.6) is True
    assert ft.is_stale("scan", 10.3) is True      # 另一个源未更新，互不影响


def test_freshness_tracker_age_is_monotonic():
    """age 应返回距上一帧的秒数，且永不为负。"""
    ft = FreshnessTracker()
    ft.register("scan", 1.0)
    ft.mark("scan", 5.0)
    assert abs(ft.age("scan", 7.5) - 2.5) < 1e-9
    assert ft.age("scan", 4.0) == 0.0


def test_sanitize_target_speed_rejects_bad_values():
    """非法期望速度应回退默认值，过大值应被截断，合法值原样通过。"""
    v, warn = sanitize_target_speed(float("nan"))
    assert v == 0.8 and warn is not None
    v, warn = sanitize_target_speed(0.0)
    assert v == 0.8 and warn is not None
    v, warn = sanitize_target_speed(-1.5)
    assert v == 0.8 and warn is not None
    v, warn = sanitize_target_speed(10.0, vmax=3.0)
    assert v == 3.0 and warn is not None
    v, warn = sanitize_target_speed(0.9)
    assert v == 0.9 and warn is None


def test_simulation_survives_emergency_stop():
    """开启避障跑直线路径：急停介入后仍不应发生碰撞。"""
    from ros2_pure_pursuit.obstacle import synthetic_scan as _scan  # noqa: F401

    path = straight(length=20.0)
    obstacles = [(6.0, 0.30, 0.50)]
    ctrl = PurePursuitController()
    avoider = ObstacleAvoider()
    robot = DiffDriveRobot(dt=0.05)
    robot.reset(0.0, 0.0, 0.0)
    for _ in range(600):
        v_cmd, w_cmd, info = ctrl.compute(path, robot.pose, 0.9)
        scan, amin, ainc = synthetic_scan(
            robot.pose.x, robot.pose.y, robot.pose.theta, obstacles
        )
        av = avoider.evaluate(scan, amin, ainc)
        res = arbitrate(v_cmd, w_cmd, info.finished, av, max_angular=robot.max_w)
        robot.step(res.v, res.w)
        for ox, oy, orad in obstacles:
            assert math.hypot(robot.pose.x - ox, robot.pose.y - oy) > orad + 0.30
        if info.finished:
            break


if __name__ == "__main__":
    import traceback

    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception:
            failed += 1
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    raise SystemExit(1 if failed else 0)
