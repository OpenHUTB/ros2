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

from ros2_pure_pursuit.obstacle import ObstacleAvoider, synthetic_scan  # noqa: E402
from ros2_pure_pursuit.paths import circle, load_path, straight  # noqa: E402
from ros2_pure_pursuit.pure_pursuit import PurePursuitController  # noqa: E402
from ros2_pure_pursuit.robot import DiffDriveRobot  # noqa: E402


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
