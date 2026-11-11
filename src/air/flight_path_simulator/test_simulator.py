#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预设航线飞行模拟器 - 最小测试
验证基本输出：帧数、高度范围、状态序列完整性
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flight_path_simulator import run_simulation, CONTROL_RATE, TOTAL_TIME, HOVER_ALT


def test_basic_output():
    """测试基本输出是否符合预期"""
    records = run_simulation()

    # 1. 帧数约等于总时长 * 控制频率
    expected_frames = int(TOTAL_TIME * CONTROL_RATE)
    assert abs(len(records) - expected_frames) <= 2, \
        f"帧数异常: {len(records)} != {expected_frames}"
    print(f"[PASS] 帧数 = {len(records)} (期望 ~{expected_frames})")

    # 2. 最大高度应贴合设定的悬停高度 HOVER_ALT（允许一个离散步长的越过量）
    max_alt = max(r["altitude_m"] for r in records)
    assert abs(max_alt - HOVER_ALT) <= 0.1, \
        f"最大高度异常: {max_alt:.2f}m 偏离设定悬停高度 {HOVER_ALT}m 超过 0.1m"
    print(f"[PASS] 最大高度 = {max_alt:.2f}m (设定 {HOVER_ALT}m)")

    # 3. 最后一帧应该是 landed/grounded 状态
    final_state = records[-1]["state"]
    assert final_state in ("landed", "grounded"), \
        f"结束状态异常: {final_state}"
    print(f"[PASS] 结束状态 = {final_state}")

    # 4. 水平速度不超过巡航速度 + 噪声上限
    max_speed = max(r["horizontal_speed"] for r in records)
    assert max_speed <= 5.0, \
        f"水平速度过高: {max_speed:.2f} m/s"
    print(f"[PASS] 最大水平速度 = {max_speed:.2f} m/s")

    # 5. 总航程在合理范围（巡航 2.5m/s × 约 14s ≈ 35m，含转弯段）
    dist = 0.0
    for i in range(1, len(records)):
        dx = records[i]["pos_x"] - records[i - 1]["pos_x"]
        dy = records[i]["pos_y"] - records[i - 1]["pos_y"]
        dist += (dx * dx + dy * dy) ** 0.5
    assert 25.0 <= dist <= 50.0, \
        f"总航程异常: {dist:.2f}m 不在 25~50m 范围内"
    print(f"[PASS] 总航程 = {dist:.2f}m")

    # 6. 状态序列应覆盖起飞、巡航、转弯、降落四个关键阶段
    seen = {r["state"] for r in records}
    required = {"takeoff", "forward", "turn_right", "landing"}
    missing = required - seen
    assert not missing, f"状态序列缺失: {missing}"
    print(f"[PASS] 状态序列完整 = {sorted(seen)}")

    print("\n全部测试通过！")


if __name__ == '__main__':
    test_basic_output()
