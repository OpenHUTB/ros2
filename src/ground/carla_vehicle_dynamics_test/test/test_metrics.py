#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``carla_vehicle_dynamics_test.metrics`` 的离线单元测试（不需要 CARLA）。

运行::

    python3 test/test_metrics.py

每项检查都打印"检查什么 / 期望 / 实际"，最后给出通过项数。
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_vehicle_dynamics_test.metrics import (      # noqa: E402
    braking_distance_theory, circle_fit_radius, mean_acceleration, path_length,
    radius_from_yaw_rate, steer_to_angle, stopping_deceleration, summarize_test,
    turn_radius_theory)

PASSED = []
FAILED = []


def check(name, condition, detail=''):
    if condition:
        PASSED.append(name)
        print('  [OK]   %s %s' % (name, detail))
    else:
        FAILED.append(name)
        print('  [FAIL] %s %s' % (name, detail))


def close(a, b, tol=1e-6):
    return abs(float(a) - float(b)) <= tol


def expect_error(func, name):
    """确认某个调用会抛出 ValueError（边界条件要显式拒绝，而不是返回错值）。"""
    try:
        func()
    except ValueError:
        check(name, True)
    except Exception as exc:                                    # noqa: BLE001
        check(name, False, '抛出了非 ValueError: %r' % exc)
    else:
        check(name, False, '没有抛出异常')


def main():
    print('=' * 72)
    print('carla_vehicle_dynamics_test 指标函数单元测试（不需要 CARLA）')
    print('=' * 72)

    print('\n[1] 平均加速度 a = dv/dt')
    check('0→16.67 m/s 用 10 s', close(mean_acceleration(0.0, 16.67, 10.0), 1.667))
    check('16.67→0 m/s 用 3 s（减速）',
          close(mean_acceleration(16.67, 0.0, 3.0), -5.5566666, 1e-4))
    expect_error(lambda: mean_acceleration(0.0, 1.0, 0.0), 'dt = 0 必须被拒绝')

    print('\n[2] 理论制动距离 s = v^2 / (2a)')
    check('16.67 m/s、6 m/s^2',
          close(braking_distance_theory(16.67, 6.0), 16.67 ** 2 / 12.0, 1e-9),
          '(实际 %.4f m)' % braking_distance_theory(16.67, 6.0))
    check('速度翻倍 → 距离变 4 倍',
          close(braking_distance_theory(20.0, 6.0), 4 * braking_distance_theory(10.0, 6.0)))
    expect_error(lambda: braking_distance_theory(10.0, 0.0), '减速度 0 必须被拒绝')

    print('\n[3] 由制动距离反推减速度 a = v^2 / (2s)')
    d = braking_distance_theory(16.67, 6.0)
    check('与理论制动距离互为逆运算', close(stopping_deceleration(16.67, d), 6.0, 1e-6))
    expect_error(lambda: stopping_deceleration(10.0, 0.0), '制动距离 0 必须被拒绝')

    print('\n[4] 阿克曼理论转弯半径 R = L / tan(delta)')
    check('轴距 2.8 m、前轮 0.3 rad',
          close(turn_radius_theory(2.8, 0.3), 2.8 / math.tan(0.3), 1e-9))
    check('转角越大半径越小',
          turn_radius_theory(2.8, 0.4) < turn_radius_theory(2.8, 0.2))
    check('归一化指令换算：steer 0.5 × max 0.6 rad = 0.3 rad',
          close(steer_to_angle(0.5, 0.6), 0.3, 1e-9))
    expect_error(lambda: turn_radius_theory(2.8, 0.0), '前轮转角 0 必须被拒绝')

    print('\n[5] 最小二乘圆拟合（Kasa）')
    theta = np.linspace(0.0, 2.0 * math.pi, 200, endpoint=False)
    cx, cy, r = 2.0, -3.0, 5.0
    exact = np.column_stack((cx + r * np.cos(theta), cy + r * np.sin(theta)))
    radius, center = circle_fit_radius(exact)
    check('理想圆：半径', close(radius, 5.0, 1e-6), '(实际 %.8f)' % radius)
    check('理想圆：圆心', close(center[0], 2.0, 1e-6) and close(center[1], -3.0, 1e-6),
          '(实际 %.8f, %.8f)' % center)

    rng = np.random.default_rng(0)
    noisy = exact + rng.normal(0.0, 0.02, exact.shape)
    radius_n, _ = circle_fit_radius(noisy)
    check('加 2 cm 噪声后半径仍在 ±5% 内', abs(radius_n - 5.0) / 5.0 < 0.05,
          '(实际 %.4f)' % radius_n)

    # 只取一小段圆弧（真实测试中转向半径较大，采样到的就是一段弧）
    arc = np.column_stack((cx + r * np.cos(theta[:40]), cy + r * np.sin(theta[:40])))
    radius_arc, _ = circle_fit_radius(arc)
    check('只取 1/5 段圆弧也能拟合', close(radius_arc, 5.0, 1e-3), '(实际 %.4f)' % radius_arc)
    expect_error(lambda: circle_fit_radius([[0.0, 0.0], [1.0, 1.0]]), '少于 3 点必须被拒绝')
    expect_error(lambda: circle_fit_radius([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]),
                 '三点共线必须被拒绝')

    print('\n[6] 稳态偏航角速度求半径 R = v / omega（合成圆周运动）')
    r_true, v_true = 12.0, 7.0
    ts = np.arange(0.0, 1.2, 0.05)
    omega_true = v_true / r_true
    yaws = omega_true * ts                       # 匀速圆周：偏航角随时间线性增长
    speeds = np.full_like(ts, v_true)
    radius, omega, v_mean = radius_from_yaw_rate(ts, yaws, speeds)
    check('半径 = 12 m', close(radius, r_true, 1e-6), '(实际 %.6f)' % radius)
    check('角速度 = v/R = 0.5833 rad/s', close(omega, omega_true, 1e-6))
    check('平均车速透传', close(v_mean, v_true))
    wrapped = np.where(yaws + math.pi - 0.1 > math.pi,
                       yaws + math.pi - 0.1 - 2 * math.pi, yaws + math.pi - 0.1)
    radius_w, _, _ = radius_from_yaw_rate(ts, wrapped, speeds)
    check('偏航角跨越 ±π 时仍正确（unwrap）', close(radius_w, r_true, 1e-3),
          '(实际 %.4f)' % radius_w)
    radius_line, omega_line, _ = radius_from_yaw_rate(ts, np.zeros_like(ts), speeds)
    check('直线行驶（omega=0）返回 None 而不是除零',
          radius_line is None and abs(omega_line) < 1e-9)
    expect_error(lambda: radius_from_yaw_rate([0.1, 0.2], [0.0, 0.1], [1.0, 1.0]),
                 '少于 3 点必须被拒绝')

    print('\n[7] 轨迹长度与汇总统计')
    check('直线段 3-4-5 三角形斜边', close(path_length([[0, 0], [3, 4]]), 5.0))
    check('折线累计长度', close(path_length([[0, 0], [3, 4], [3, 0]]), 5.0 + 4.0))
    check('单点轨迹长度为 0', close(path_length([[1.0, 2.0]]), 0.0))
    s = summarize_test([1.0, 2.0, 3.0], 'demo')
    check('汇总 min/mean/max', s['min'] == 1.0 and s['mean'] == 2.0 and s['max'] == 3.0)
    s2 = summarize_test([1.0, None, float('nan')], 'demo')
    check('忽略 None 与 NaN', s2['n'] == 1 and s2['mean'] == 1.0)
    s3 = summarize_test([], 'demo')
    check('空输入不报错', s3['n'] == 0 and s3['mean'] is None)

    print('\n' + '=' * 72)
    print('[结果] %d 项全部通过' % len(PASSED) if not FAILED
          else '[结果] 通过 %d 项，失败 %d 项：%s' % (len(PASSED), len(FAILED), FAILED))
    print('=' * 72)
    return 1 if FAILED else 0


if __name__ == '__main__':
    sys.exit(main())
