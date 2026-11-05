#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全速度包络 safety.py 的本地测试（不需要 ROS / 仿真器 / 模型权重）.

覆盖：
    A. 参数校验与默认值
    B. 无遮挡时速度原样通过
    C. 近障按比例减速（沿速度方向的扇形）
    D. 过近时水平急停（垂直速度保留）
    E. 只看速度方向的扇形，不被侧向/后方障碍误伤
    F. 世界系速度方向 -> 扇形下标（0 = 正东，逆时针）
    G. 速度限幅
    H. 悬停（水平速度为 0）不触发急停
    I. 传感器有效性：无点云 / 缺时间戳 / 过期 -> 零速度（fail-safe）
    J. 工程文件：config / launch / CMakeLists / 节点接线

用法: python3 tests/test_ppo_nav_safety_local.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "uav_ppo_nav")
SCRIPTS = os.path.join(PKG, "scripts")
sys.path.insert(0, SCRIPTS)

from policy import MAX_HORIZ_SPEED, MAX_RANGE, MAX_VERT_SPEED, N_SECTORS  # noqa: E402
from safety import SafetyEnvelope  # noqa: E402

EAST = 0
NORTH = N_SECTORS // 4
FRESH = 0.0          # 刚收到的点云（接收耗时 0 s）


def hist_with(sector, value, default=1.0):
    """构造只在一个扇形内放障碍的直方图。"""
    hist = np.full(N_SECTORS, default, dtype=np.float64)
    hist[sector] = value
    return hist


# ============================================================ A. 参数校验
def test_parameters():
    print("== A. 参数校验 ==")
    envelope = SafetyEnvelope()
    check(envelope.stop_distance == 0.8 and envelope.slow_distance == 3.0, "默认 stop=0.8 / slow=3.0")
    check(envelope.sensor_timeout == 0.5, "默认点云超时 = 0.5 s")
    check(envelope.max_horiz_speed == MAX_HORIZ_SPEED and envelope.max_range == MAX_RANGE,
          "默认沿用 policy 的速度/量程上限")

    for kwargs in ({'stop_distance': -0.1}, {'stop_distance': 3.0, 'slow_distance': 3.0},
                   {'stop_distance': 2.0, 'slow_distance': 1.0},
                   {'max_horiz_speed': 0.0}, {'sensor_timeout': 0.0}):
        try:
            SafetyEnvelope(**kwargs)
            check(False, "非法参数应抛 ValueError: %s" % kwargs)
        except ValueError:
            check(True, "非法参数抛 ValueError: %s" % kwargs)


# ============================================================ B. 无遮挡
def test_clear_path():
    print("== B. 无遮挡 ==")
    envelope = SafetyEnvelope()
    vx, vy, vz, info = envelope.filter(1.5, -1.0, 0.4, np.ones(N_SECTORS), cloud_age=FRESH)
    check((vx, vy, vz) == (1.5, -1.0, 0.4), "无遮挡时速度原样输出")
    check(info['scale'] == 1.0 and not info['braking'], "无遮挡 scale=1 且不急停")
    check(info['clearance_m'] == MAX_RANGE, "无遮挡净空等于量程上限")
    check(info['reason'] == 'ok', "点云有效时 reason=ok")


# ============================================================ C. 近障减速
def test_slow_down():
    print("== C. 近障减速 ==")
    envelope = SafetyEnvelope(stop_distance=0.8, slow_distance=3.0)
    hist = hist_with(EAST, 2.0 / MAX_RANGE)          # 正东 2.0 m 有障碍
    vx, vy, vz, info = envelope.filter(3.0, 0.0, 0.0, hist, cloud_age=FRESH)
    expect = (2.0 - 0.8) / (3.0 - 0.8)
    check(math.isclose(info['clearance_m'], 2.0, rel_tol=1e-6), "净空换算为 2.0 m")
    check(math.isclose(info['scale'], expect, rel_tol=1e-3), "减速比例 = (d-stop)/(slow-stop)")
    check(math.isclose(vx, 3.0 * expect, rel_tol=1e-3), "水平速度按比例衰减")
    check(not info['braking'], "减速阶段不标记急停")

    hist_far = hist_with(EAST, 3.5 / MAX_RANGE)
    vx2, _, _, info2 = envelope.filter(3.0, 0.0, 0.0, hist_far, cloud_age=FRESH)
    check(vx2 == 3.0 and info2['scale'] == 1.0, "净空大于 slow 时不减速")


# ============================================================ D. 急停
def test_emergency_stop():
    print("== D. 过近急停 ==")
    envelope = SafetyEnvelope(stop_distance=0.8, slow_distance=3.0)
    hist = hist_with(EAST, 0.5 / MAX_RANGE)          # 正东 0.5 m
    vx, vy, vz, info = envelope.filter(2.0, 1.0, 0.6, hist, cloud_age=FRESH)
    check((vx, vy) == (0.0, 0.0), "急停时水平速度清零")
    check(vz == 0.6, "垂直速度保留（便于脱离）")
    check(info['braking'] is True and info['scale'] == 0.0, "标记 braking")

    hist_edge = hist_with(EAST, 0.8 / MAX_RANGE)      # 恰好等于 stop_distance
    _, _, _, info2 = envelope.filter(2.0, 0.0, 0.0, hist_edge, cloud_age=FRESH)
    check(info2['braking'] is True, "净空等于 stop_distance 也急停")


# ============================================================ E. 方向相关
def test_direction_matters():
    print("== E. 只看速度方向 ==")
    envelope = SafetyEnvelope(stop_distance=0.8, slow_distance=3.0)
    hist = hist_with(EAST, 0.2 / MAX_RANGE)           # 正东很近
    vx, vy, _, info = envelope.filter(0.0, 2.0, 0.0, hist, cloud_age=FRESH)   # 往北飞
    check((vx, vy) == (0.0, 2.0), "侧向障碍不影响北向速度")
    check(not info['braking'], "北向不被东侧障碍误判为急停")
    check(info['sector'] == NORTH, "北向对应扇形 4")


# ============================================================ F. 扇区映射
def test_sector_mapping():
    print("== F. 扇区映射 ==")
    envelope = SafetyEnvelope()
    cases = [((1.0, 0.0), 0), ((0.0, 1.0), 4), ((-1.0, 0.0), 8),
             ((0.0, -1.0), 12), ((1.0, 1.0), 2), ((-1.0, -1.0), 10)]
    for (vx, vy), expected in cases:
        got = envelope.sector_of(vx, vy, N_SECTORS)
        check(got == expected % N_SECTORS, "方向 (%.0f,%.0f) -> 扇区 %d" % (vx, vy, got % N_SECTORS))


# ============================================================ G. 限幅
def test_clamp():
    print("== G. 速度限幅 ==")
    envelope = SafetyEnvelope()
    vx, vy, vz, _ = envelope.filter(10.0, -10.0, 5.0, np.ones(N_SECTORS), cloud_age=FRESH)
    check(vx == MAX_HORIZ_SPEED and vy == -MAX_HORIZ_SPEED, "水平速度夹到 ±MAX_HORIZ_SPEED")
    check(vz == MAX_VERT_SPEED, "垂直速度夹到 MAX_VERT_SPEED")


# ============================================================ H. 悬停
def test_hover():
    print("== H. 悬停 ==")
    envelope = SafetyEnvelope()
    vx, vy, vz, info = envelope.filter(0.0, 0.0, 0.5, np.zeros(N_SECTORS), cloud_age=FRESH)
    check((vx, vy, vz) == (0.0, 0.0, 0.5), "悬停时零水平速度不受影响")
    check(not info['braking'] and info['scale'] == 1.0, "悬停不触发急停")
    check(info['sector'] is None, "悬停不计算方向扇区")


# ============================================================ I. 传感器有效性
def test_sensor_validity():
    print("== I. 传感器有效性（缺失/过期 -> 零速度）==")
    envelope = SafetyEnvelope(stop_distance=0.8, slow_distance=3.0, sensor_timeout=0.5)

    cases = [
        (None, 0.0, 'no_lidar', "从未收到点云"),
        (np.array([]), 0.0, 'no_lidar', "空点云"),
        (np.ones(N_SECTORS), None, 'no_timestamp', "缺少接收时间戳"),
        (np.ones(N_SECTORS), 0.8, 'stale_lidar', "点云过期（0.8 s > 0.5 s）"),
    ]
    for hist, age, reason, label in cases:
        vx, vy, vz, info = envelope.filter(2.0, 1.0, 0.6, hist, cloud_age=age)
        check((vx, vy, vz) == (0.0, 0.0, 0.0), "%s -> 零速度" % label)
        check(info['braking'] is True and info['reason'] == reason,
              "%s -> reason=%s" % (label, reason))
        check(info['clearance_m'] == 0.0, "%s -> 净空置 0，不假设无遮挡" % label)

    vx, vy, vz, info = envelope.filter(2.0, 1.0, 0.6, np.ones(N_SECTORS), cloud_age=0.5)
    check((vx, vy, vz) == (2.0, 1.0, 0.6) and info['reason'] == 'ok',
          "恰好等于超时阈值仍可用（只有 > 才判定过期）")

    ok, reason = envelope.sensor_valid(np.ones(N_SECTORS), float('nan'))
    check(not ok and reason == 'no_timestamp', "NaN 时间戳视为不可用")
    ok2, reason2 = envelope.sensor_valid(np.ones(N_SECTORS), -1.0)
    check(not ok2 and reason2 == 'no_timestamp', "负时间戳视为不可用")

    strict = SafetyEnvelope(sensor_timeout=0.2)
    _, _, _, info3 = strict.filter(1.0, 0.0, 0.0, np.ones(N_SECTORS), cloud_age=0.3)
    check(info3['reason'] == 'stale_lidar', "sensor_timeout 可配置（0.3 s > 0.2 s 即过期）")


# ============================================================ J. 工程文件
def test_project_files():
    print("== J. 工程文件 ==")
    config = open(os.path.join(PKG, "config", "ppo_nav.yaml"), encoding="utf-8").read()
    for key in ("safety", "enabled", "stop_distance", "slow_distance", "cloud_timeout"):
        check(key in config, "config/ppo_nav.yaml 含 %s" % key)

    launch = open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read()
    for arg in ("safety_enabled", "safety_stop_distance", "safety_slow_distance",
                "safety_cloud_timeout"):
        check(arg in launch, "launch 暴露参数 %s" % arg)

    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    check("scripts/safety.py" in cmake, "CMakeLists 以普通文件安装 safety.py（保证可 import）")

    node = open(os.path.join(SCRIPTS, "ppo_nav_node.py"), encoding="utf-8").read()
    check("from safety import SafetyEnvelope" in node, "节点导入 SafetyEnvelope")
    check("cloud_age=age" in node, "step_once 把点云时效传给安全包络")
    check("self.last_hist = None" in node, "无点云时不把占位直方图交给安全层")


def main():
    test_parameters()
    test_clear_path()
    test_slow_down()
    test_emergency_stop()
    test_direction_matters()
    test_sector_mapping()
    test_clamp()
    test_hover()
    test_sensor_validity()
    test_project_files()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for item in FAIL:
            print("  FAIL: %s" % item)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
