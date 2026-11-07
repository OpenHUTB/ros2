#!/usr/bin/env python3
"""AprilTag 降落模块纯计算单元测试, 不依赖 ROS。"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'uav_apriltag_landing'))

from landing_math import (
    pixel_to_ground, decide_next_state,
    compute_align_target, compute_descend_target, timeout_target,
    ST_SEARCH, ST_ALIGN, ST_DESCEND, ST_LANDED,
)


def test_pixel_to_ground():
    gx, gy = pixel_to_ground(50.0, -20.0, 5.0, 500.0, 500.0)
    assert abs(gx - 0.5) < 1e-6
    assert abs(gy + 0.2) < 1e-6
    print("[OK] pixel_to_ground")


def test_decide_state_search_to_align():
    s = decide_next_state(ST_SEARCH, True, 10.0, 5.0, 15.0, 5.0, 0.1)
    assert s == ST_ALIGN
    print("[OK] SEARCH -> ALIGN (检测到标签)")


def test_decide_state_stay_search():
    s = decide_next_state(ST_SEARCH, False, 0.0, 0.0, 15.0, 5.0, 0.1)
    assert s == ST_SEARCH
    print("[OK] SEARCH 保持 (未检测到标签)")


def test_decide_state_align_to_descend():
    s = decide_next_state(ST_ALIGN, True, 5.0, 3.0, 15.0, 5.0, 0.1)
    assert s == ST_DESCEND
    print("[OK] ALIGN -> DESCEND (对准完成)")


def test_decide_state_align_back_search():
    s = decide_next_state(ST_ALIGN, False, 0.0, 0.0, 15.0, 5.0, 0.1)
    assert s == ST_SEARCH
    print("[OK] ALIGN -> SEARCH (标签丢失)")


def test_decide_state_descend_to_landed():
    s = decide_next_state(ST_DESCEND, True, 2.0, 1.0, 15.0, 0.1, 0.1)
    assert s == ST_LANDED
    print("[OK] DESCEND -> LANDED (接地)")


def test_compute_descend_rate_limited():
    """高度远低于目标时, 不连续下压。"""
    target = (1.0, 2.0, 4.0)
    # 高度 3.0m, 目标 z=4.0, 差距 1.0m > 0.15m, 不应继续降
    new_t = compute_descend_target(target, 0.0, 0.0, 3.0, 500.0, 500.0, 0.3, 0.1)
    assert new_t[2] == 4.0, f"高度远低于目标时应保持 z, got {new_t[2]}"
    print("[OK] DESCEND 不连续下压")


def test_timeout_hold_height():
    """超时保持当前高度, 不继续下降。"""
    target = (1.0, 2.0, 3.0)
    new_t = timeout_target(target, 2.5, 0.1)
    assert new_t[0] == 1.0 and new_t[1] == 2.0
    assert new_t[2] == 2.5, f"超时应保持当前高度 2.5m, got {new_t[2]}"
    print("[OK] 超时悬停保持当前高度")


if __name__ == "__main__":
    test_pixel_to_ground()
    test_decide_state_search_to_align()
    test_decide_state_stay_search()
    test_decide_state_align_to_descend()
    test_decide_state_align_back_search()
    test_decide_state_descend_to_landed()
    test_compute_descend_rate_limited()
    test_timeout_hold_height()
    print("\n全部测试通过")
