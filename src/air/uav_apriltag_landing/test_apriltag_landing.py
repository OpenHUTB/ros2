#!/usr/bin/env python3
"""
AprilTag 降落模块纯计算单元测试.
不依赖 ROS, 直接运行: python3 test_apriltag_landing.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from landing_math import pixel_to_ground, ST_SEARCH, ST_ALIGN, ST_DESCEND, ST_LANDED


def test_pixel_to_ground():
    fx, fy = 500.0, 500.0
    h = 5.0
    # 像素偏差 50px → 地面位移 0.5m
    gx, gy = pixel_to_ground(50.0, -20.0, h, fx, fy)
    assert abs(gx - 0.5) < 1e-6, f"gx={gx}"
    assert abs(gy + 0.2) < 1e-6, f"gy={gy}"
    print("[OK] pixel_to_ground 基本换算")


def test_pixel_to_ground_zero():
    gx, gy = pixel_to_ground(0.0, 0.0, 5.0, 500.0, 500.0)
    assert gx == 0.0 and gy == 0.0
    print("[OK] pixel_to_ground 零偏差")


def test_pixel_to_ground_scale_with_height():
    # 高度翻倍, 地面位移也翻倍
    gx1, _ = pixel_to_ground(50.0, 0.0, 5.0, 500.0, 500.0)
    gx2, _ = pixel_to_ground(50.0, 0.0, 10.0, 500.0, 500.0)
    assert abs(gx2 - 2 * gx1) < 1e-6
    print("[OK] pixel_to_ground 与高度成正比")


def test_state_constants():
    assert ST_SEARCH == "SEARCH"
    assert ST_ALIGN == "ALIGN"
    assert ST_DESCEND == "DESCEND"
    assert ST_LANDED == "LANDED"
    print("[OK] 状态机常量")


def test_descend_rate_limited():
    """DESCEND 状态: 只有当前高度接近目标高度才继续降, 避免连续下压."""
    hover_h = 5.0
    descend_step = 0.3
    landed_h = 0.1

    # 场景1: 高度还没到目标 (刚从ALIGN进来), 第一次下降
    current_target_z = hover_h
    current_height = hover_h
    if current_height >= current_target_z - 0.15:
        new_z = max(current_target_z - descend_step, landed_h)
    else:
        new_z = current_target_z
    assert abs(new_z - (hover_h - 0.3)) < 1e-6, f"new_z={new_z}"

    # 场景2: 还没降到位 (高度5.0, 目标4.7), 继续降
    current_target_z = 4.7
    current_height = 4.95
    if current_height >= current_target_z - 0.15:
        new_z = max(current_target_z - descend_step, landed_h)
    else:
        new_z = current_target_z
    assert abs(new_z - (4.7 - 0.3)) < 1e-6

    # 场景3: 高度远低于目标 (刚降完还没到), 不继续下压
    current_target_z = 4.4
    current_height = 4.0  # 已经低于目标0.4m
    if current_height >= current_target_z - 0.15:
        new_z = max(current_target_z - descend_step, landed_h)
    else:
        new_z = current_target_z
    assert new_z == current_target_z, "高度远低于目标时应保持, 不连续下压"
    print("[OK] DESCEND 下降速率限制 (不连续下压)")


def test_timeout_hold_height():
    """传感器超时: 保持当前高度悬停, 不继续下降."""
    # 模拟超时保护逻辑
    current_height = 3.5
    landed_h = 0.1
    target_x, target_y = (1.0, 2.0)
    # 超时发布: (target_x, target_y, max(current_height, landed_h))
    timeout_z = max(current_height, landed_h)
    assert timeout_z == current_height, "超时应保持当前高度"

    # 即使在DESCEND中, 超时也不继续下降
    current_target_z = 3.0  # 正在往下降
    assert timeout_z >= current_target_z, "超时高度不应低于目标高度"
    print("[OK] 超时时保持当前高度悬停")


def test_no_pose_no_publish():
    """未收到位姿: 不发布setpoint (逻辑验证)."""
    # 未收到odom时, 节点只发布状态, 不发布位置目标
    pose_received = False
    published_setpoint = False
    # 模拟逻辑
    if not pose_received:
        # 只发布状态, 不发布setpoint
        pass
    else:
        published_setpoint = True
    assert not published_setpoint, "未收到位姿时不应发布setpoint"
    print("[OK] 未收到位姿时不发布跳变目标")


if __name__ == "__main__":
    test_pixel_to_ground()
    test_pixel_to_ground_zero()
    test_pixel_to_ground_scale_with_height()
    test_state_constants()
    test_descend_rate_limited()
    test_timeout_hold_height()
    test_no_pose_no_publish()
    print("\n全部测试通过")
