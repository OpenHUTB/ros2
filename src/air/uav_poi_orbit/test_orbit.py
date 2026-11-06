#!/usr/bin/env python3
"""POI 环绕航拍纯计算单元测试, 不依赖 ROS."""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'uav_poi_orbit'))

from orbit_math import (
    normalize_angle, compute_orbit_position, compute_yaw_to_poi,
    advance_angle, count_full_turns, compute_height, validate_params,
    TWO_PI,
)


def test_normalize_angle():
    assert abs(normalize_angle(0.0) - 0.0) < 1e-9
    assert abs(normalize_angle(TWO_PI) - 0.0) < 1e-9
    assert abs(normalize_angle(-0.5) - (TWO_PI - 0.5)) < 1e-9
    assert abs(normalize_angle(3 * TWO_PI + 1.0) - 1.0) < 1e-9
    print("[OK] 角度归一化到 [0, 2pi)")


def test_position_on_circle():
    # 角度 0 → 正东方向
    x, y = compute_orbit_position(0.0, 0.0, 5.0, 0.0)
    assert abs(x - 5.0) < 1e-9 and abs(y) < 1e-9
    # 角度 pi/2 → 正北方向
    x, y = compute_orbit_position(0.0, 0.0, 5.0, math.pi / 2)
    assert abs(x) < 1e-9 and abs(y - 5.0) < 1e-9
    # 半径保持不变
    for a in [0.0, 1.0, 2.0, 4.0]:
        x, y = compute_orbit_position(0.0, 0.0, 5.0, a)
        assert abs(math.hypot(x, y) - 5.0) < 1e-9
    print("[OK] 圆周位置计算")


def test_position_offset_by_poi():
    # POI 偏移后圆周整体平移
    x, y = compute_orbit_position(10.0, -3.0, 2.0, 0.0)
    assert abs(x - 12.0) < 1e-9 and abs(y + 3.0) < 1e-9
    print("[OK] POI 偏移平移")


def test_yaw_points_to_poi():
    # 无人机在东侧 → 偏航应指向西 (pi)
    yaw = compute_yaw_to_poi(0.0, 0.0, 5.0, 0.0)
    assert abs(abs(yaw) - math.pi) < 1e-9
    # 无人机在北侧 → 偏航应指向南 (-pi/2)
    yaw = compute_yaw_to_poi(0.0, 0.0, 0.0, 5.0)
    assert abs(yaw + math.pi / 2) < 1e-9
    print("[OK] 偏航角始终对准 POI")


def test_advance_angle_wraps():
    ang, total = advance_angle(0.0, 0.0, math.pi, 1.0)
    assert abs(ang - math.pi) < 1e-9
    assert abs(total - math.pi) < 1e-9
    # 跨过 2pi 后 angle 回绕, total 继续累积
    ang, total = advance_angle(ang, total, math.pi, 1.0)
    assert ang < 1e-9, f"angle 应回绕到 0, got {ang}"
    assert abs(total - TWO_PI) < 1e-9, f"total 应累积到 2pi, got {total}"
    print("[OK] 角度推进与回绕")


def test_count_full_turns():
    assert abs(count_full_turns(0.0)) < 1e-9
    assert abs(count_full_turns(TWO_PI) - 1.0) < 1e-9
    assert abs(count_full_turns(3.5 * TWO_PI) - 3.5) < 1e-9
    print("[OK] 圈数折算")


def test_height_no_spiral():
    assert abs(compute_height(8.0, TWO_PI, False, 2.0) - 8.0) < 1e-9
    print("[OK] 非螺旋模式高度恒定")


def test_height_spiral_continuous():
    # 螺旋高度随总转角连续变化, 圈边界不阶跃
    h0 = compute_height(8.0, 0.9 * TWO_PI, True, 2.0)
    h1 = compute_height(8.0, 1.0 * TWO_PI, True, 2.0)
    h2 = compute_height(8.0, 1.1 * TWO_PI, True, 2.0)
    assert abs(h0 - 9.8) < 1e-9, f"0.9圈应为9.8m, got {h0}"
    assert abs(h1 - 10.0) < 1e-9, f"1圈应为10.0m, got {h1}"
    assert abs(h2 - 10.2) < 1e-9
    # 连续: 相邻差值应远小于一整圈的增量
    assert abs((h1 - h0) - 0.2) < 1e-9
    print("[OK] 螺旋高度连续变化 (圈边界无阶跃)")


def test_height_floor():
    # 螺旋下降时不应低于最低高度
    z = compute_height(1.0, 5 * TWO_PI, True, -2.0, min_height=0.5)
    assert z == 0.5, f"应被限制到最低高度 0.5, got {z}"
    print("[OK] 高度下限保护")


def test_validate_params_ok():
    assert validate_params(5.0, 8.0, math.radians(15), 0.2) == []
    print("[OK] 合法参数通过校验")


def test_validate_params_bad():
    errs = validate_params(-1.0, 8.0, math.radians(15), 0.2)
    assert len(errs) == 1 and "orbit_radius" in errs[0]

    errs = validate_params(5.0, 0.1, math.radians(15), 0.2)
    assert len(errs) == 1 and "orbit_height" in errs[0]

    errs = validate_params(5.0, 8.0, 0.0, 0.2)
    assert len(errs) == 1 and "omega_dps" in errs[0]

    errs = validate_params(5.0, 8.0, math.radians(15), 0.0)
    assert len(errs) == 1 and "waypoint_interval_s" in errs[0]
    print("[OK] 非法参数被拦截 (半径/高度/角速度/步长)")


if __name__ == "__main__":
    test_normalize_angle()
    test_position_on_circle()
    test_position_offset_by_poi()
    test_yaw_points_to_poi()
    test_advance_angle_wraps()
    test_count_full_turns()
    test_height_no_spiral()
    test_height_spiral_continuous()
    test_height_floor()
    test_validate_params_ok()
    test_validate_params_bad()
    print("\n全部测试通过")
