"""基本单元测试：验证控制算法核心函数正确性"""

import math
import sys
import os

# 将 src 目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from r2d2_waypoint_cruise.cruise import (
    normalize_angle,
    compute_control,
    WAYPOINTS,
    ARRIVE_THRESHOLD,
)


def test_normalize_angle():
    """测试角度归一化"""
    assert abs(normalize_angle(0)) < 1e-6
    assert abs(normalize_angle(math.pi) - math.pi) < 1e-6
    assert abs(normalize_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-6
    assert abs(normalize_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-6
    print("test_normalize_angle passed")


def test_waypoints_loaded():
    """测试航点配置加载"""
    assert len(WAYPOINTS) == 5
    assert WAYPOINTS[0] == (0.0, 0.0)
    assert WAYPOINTS[2] == (3.0, 3.0)
    print("test_waypoints_loaded passed")


def test_compute_control_straight():
    """测试直线前进控制：目标在正前方"""
    # 小车在原点，yaw=0（朝向y负方向）
    # 目标在 (0, -3)，即正前方
    ls, rs, dist, herr = compute_control(0, 0, 0, 0, -3)
    # 距离应该是3
    assert abs(dist - 3.0) < 0.1
    # 航向误差应该接近0
    assert abs(herr) < 0.5
    print("test_compute_control_straight passed")


def test_arrive_threshold():
    """测试到达阈值配置"""
    assert ARRIVE_THRESHOLD > 0
    assert ARRIVE_THRESHOLD < 1.0
    print("test_arrive_threshold passed")


if __name__ == "__main__":
    test_normalize_angle()
    test_waypoints_loaded()
    test_compute_control_straight()
    test_arrive_threshold()
    print("\nAll tests passed!")
