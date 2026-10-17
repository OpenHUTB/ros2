# -*- coding: utf-8 -*-
"""三项动力性测试的测量流程。

* :func:`measure_acceleration` —— 全油门 0→v 加速，记录用时与平均加速度；
* :func:`measure_braking`      —— 提速到 v 后全力刹车，记录制动距离与减速度；
* :func:`measure_turn`         —— 阶跃转向（先直线提速再打固定方向），用偏航角速度
  与轨迹圆拟合两种方式求转弯半径。

三个函数都遵循同一条原则：**测试开始时车辆必须处于同一初始状态**（由调用方用
``carla_utils.reset_vehicle`` 搬回起点），并且**加速/制动全程保持车道**
（``carla_utils.lane_steer``）——否则车辆会冲出车道撞上路边障碍，
测出"减速度 1.8g"这种没有意义的结果。
"""

import math

import numpy as np

from .carla_utils import (carla, lane_steer, set_spectator_follow, speed_of)
from .metrics import (braking_distance_theory, circle_fit_radius, mean_acceleration,
                      path_length, radius_from_yaw_rate, stopping_deceleration,
                      turn_radius_theory)

STOP_SPEED = 0.1            # 判定"已停稳"的速度阈值（m/s）


def _print_progress(prefix, t, speed, extra=''):
    print('  t=%5.1fs  %s=%6.2f m/s（%5.1f km/h）%s'
          % (t, prefix, speed, speed * 3.6, extra))


def _control_step(session, vehicle, carla_map, throttle, brake, steer=None,
                  keep_lane=True, follow=True):
    """执行一步控制并推进一帧；``steer=None`` 时按车道保持计算方向。"""
    if steer is None:
        steer = lane_steer(vehicle, carla_map) if keep_lane else 0.0
    vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
    session.tick()
    if follow:
        set_spectator_follow(session.world, vehicle)
    return speed_of(vehicle)


def _drive_to_speed(session, vehicle, v_target, carla_map, max_time=25.0,
                    kp=0.4, bias=0.5, max_throttle=1.0, log_every=1.0,
                    keep_lane=True, follow=True, tag='提速'):
    """P 控制器 + 前馈项把车速提到 ``v_target``（用于制动/转向测试的前置提速）。

    ``bias`` 是前馈油门，用来抵消滚动阻力与风阻。**没有它时纯 P 控制会卡在目标下方**
    （实测：目标 16.67 m/s，油门被算成 0.53、车速停在 15.7 m/s 再也上不去，
    于是制动测试判定"未能达到目标车速"而跳过）。这与仓库跟车模块里记录的振荡问题
    是同一类前馈缺失。

    返回 ``(ok, elapsed, speed)``。
    """
    t = 0.0
    next_log = 0.0
    speed = speed_of(vehicle)
    while t < max_time:
        error = v_target - speed
        throttle = max(0.0, min(max_throttle, bias + kp * error)) if error > 0 else 0.0
        brake = max(0.0, min(0.5, -kp * error)) if error < 0 else 0.0
        speed = _control_step(session, vehicle, carla_map, throttle, brake,
                              keep_lane=keep_lane, follow=follow)
        t += session.dt
        if t >= next_log:
            _print_progress('v', t, speed, '  （%s）' % tag)
            next_log += log_every
        if speed >= v_target * 0.985:
            return True, t, speed
        if t > 3.0 and speed < 0.3:
            # 车动不了：出生点被挡或撞到东西，提前退出而不是白等
            return False, t, speed
    return False, t, speed


def measure_acceleration(session, vehicle, carla_map, v_target=16.67, max_time=30.0,
                         log_every=1.0, follow=True):
    """全油门加速测试（保持车道）。

    返回 ``{'ok', 'time', 'v0', 'v1', 'accel', 'distance', 'samples'}``。
    """
    print('[测试] 全油门加速：目标 %.2f m/s（%.1f km/h）' % (v_target, v_target * 3.6))
    v0 = speed_of(vehicle)
    start = vehicle.get_transform().location
    t = 0.0
    next_log = 0.0
    samples = []
    ok = False
    speed = v0
    while t < max_time:
        speed = _control_step(session, vehicle, carla_map, 1.0, 0.0,
                              keep_lane=True, follow=follow)
        t += session.dt
        samples.append((t, speed))
        if t >= next_log:
            _print_progress('v', t, speed)
            next_log += log_every
        if speed >= v_target:
            ok = True
            break

    end = vehicle.get_transform().location
    distance = math.dist((start.x, start.y), (end.x, end.y))
    accel = mean_acceleration(v0, speed, t) if t > 0 else None
    if ok:
        print('[加速] %.2f→%.2f m/s（%.1f→%.1f km/h）用时 %.2f s，'
              '平均加速度 %.2f m/s^2（位移 %.1f m）'
              % (v0, speed, v0 * 3.6, speed * 3.6, t, accel, distance))
    else:
        print('[加速] %.1f s 内未达到 %.1f km/h（末速 %.1f km/h）'
              % (max_time, v_target * 3.6, speed * 3.6))
    return {'ok': ok, 'time': t if ok else None, 'v0': v0, 'v1': speed,
            'accel': accel, 'distance': distance, 'samples': samples}


def measure_braking(session, vehicle, carla_map, v_start=16.67, max_time=15.0,
                    log_every=1.0, follow=True):
    """制动距离测试：先提速到 ``v_start``，再全力刹车到停稳（全程保持车道）。

    返回 ``{'ok', 'v0', 'distance', 'decel', 'theory', 'duration', 'samples'}``。
    """
    ok, _, _ = _drive_to_speed(session, vehicle, v_start, carla_map,
                               log_every=log_every, follow=follow, tag='提速到目标车速')
    if not ok:
        print('[制动] 未能达到 %.1f km/h，跳过制动测试' % (v_start * 3.6))
        return {'ok': False, 'v0': None, 'distance': None, 'decel': None,
                'theory': None, 'samples': []}

    v0 = speed_of(vehicle)
    print('[测试] 全力制动：起始 %.2f m/s（%.1f km/h）' % (v0, v0 * 3.6))
    positions = [(vehicle.get_transform().location.x, vehicle.get_transform().location.y)]
    samples = []
    t = 0.0
    next_log = 0.0
    speed = v0
    while t < max_time:
        speed = _control_step(session, vehicle, carla_map, 0.0, 1.0,
                              keep_lane=True, follow=follow)
        t += session.dt
        loc = vehicle.get_transform().location
        positions.append((loc.x, loc.y))
        samples.append((t, speed))
        if t >= next_log:
            _print_progress('v', t, speed)
            next_log += log_every
        if speed <= STOP_SPEED:
            break

    distance = path_length(positions)
    decel = stopping_deceleration(v0, distance) if distance > 0.0 else None
    theory = braking_distance_theory(v0, decel) if decel else None
    print('[制动] %.1f km/h → 0：实测制动距离 %.2f m，平均减速度 %.2f m/s^2（用时 %.2f s）'
          % (v0 * 3.6, distance, decel, t))
    print('       匀减速理论值 s = v^2/(2a) = %.2f m，与实测相差 %.2f m'
          % (theory, abs(theory - distance)))
    return {'ok': True, 'v0': v0, 'distance': distance, 'decel': decel,
            'theory': theory, 'duration': t, 'samples': samples}


def measure_turn(session, vehicle, carla_map, steer_cmd=0.05, v_target=7.0,
                 step_time=1.5, spin_up=20.0, kp=0.4, bias=0.5, max_throttle=1.0,
                 log_every=0.5, follow=True):
    r"""阶跃转向测试：直线提速到目标车速，再打固定转向角，测稳态转弯半径。

    **参数为什么这么温和**（``steer_cmd=0.05`` ≈ 3.5° 前轮转角、``step_time=1.5 s``）：

    * 车道只有 3.5 m 宽，转向角一大（14° → 半径 10 m）车会在 1 秒内冲出路沿，
      实测直接撞停，速度掉到 0.07 m/s，算出来的半径毫无意义；
    * 取 3.5°（理论半径约 40 m）时侧向加速度只有 1.2 m/s²，车能留在车道内，
      1.5 s 内横向偏出不到 1 m，同时偏航角变化约 12°，足够测出角速度。

    半径用两种互相独立的方式计算，便于交叉验证：

    1. 稳态偏航角速度法 :math:`R = v / \omega`（末段平均）；
    2. 转向段轨迹的最小二乘圆拟合。

    两者都与阿克曼理论值 :math:`R = L/\tan\delta` 对照。
    """
    params = session.vehicle_params(vehicle)
    delta = steer_cmd * params['max_steer_rad']
    print('[测试] 阶跃转向：转向指令 %.2f（前轮转角约 %.2f°），目标车速 %.2f m/s'
          % (steer_cmd, math.degrees(delta), v_target))
    print('       轴距 %.2f m（来源：%s）'
          % (params['wheelbase'], params.get('wheelbase_source', '未知')))

    # ---------- 阶段一：直线提速（保持车道） ----------
    ok, _, _ = _drive_to_speed(session, vehicle, v_target, carla_map,
                               max_time=spin_up, log_every=log_every, follow=follow,
                               tag='直线提速段')
    if not ok:
        print('[转向] 提速失败，跳过转向测试')
        return {'ok': False, 'radius': None, 'theory': None, 'points': []}

    # ---------- 阶段二：打固定转向角，采集响应 ----------
    print('       进入转向段：保持约 %.1f m/s，转向指令 %.2f，持续 %.1f s'
          % (v_target, steer_cmd, step_time))
    samples = []          # (t, v, yaw_rad)
    points = []           # (x, y)
    t = 0.0
    next_log = 0.0
    speed = speed_of(vehicle)
    while t < step_time:
        error = v_target - speed
        throttle = max(0.0, min(max_throttle, bias + kp * error)) if error > 0 else 0.0
        brake = max(0.0, min(0.5, -kp * error)) if error < 0 else 0.0
        speed = _control_step(session, vehicle, carla_map, throttle, brake,
                              steer=steer_cmd, follow=follow)
        t += session.dt
        transform = vehicle.get_transform()
        samples.append((t, speed, math.radians(transform.rotation.yaw)))
        points.append((transform.location.x, transform.location.y))
        if t >= next_log:
            _print_progress('v', t, speed, '  采样点 %d' % len(points))
            next_log += log_every

    if len(samples) < 5:
        print('[转向] 采样点不足，无法计算')
        return {'ok': False, 'radius': None, 'theory': None, 'points': points}

    # ---------- 半径算法 1：稳态偏航角速度 R = v / omega ----------
    window = max(0.4, step_time * 0.4)
    tail = [s for s in samples if s[0] >= samples[-1][0] - window]
    if len(tail) >= 3:
        radius_yaw, omega, v_mean = radius_from_yaw_rate(
            [s[0] for s in tail], [s[2] for s in tail], [s[1] for s in tail])
    else:
        radius_yaw, omega, v_mean = None, 0.0, float(np.mean([s[1] for s in samples]))

    # ---------- 半径算法 2：轨迹圆拟合（仅转向段） ----------
    radius_fit = None
    try:
        radius_fit, _ = circle_fit_radius(points)
    except ValueError as exc:
        print('[转向] 轨迹圆拟合失败（%s），仅采用偏航角速度法' % exc)

    theory = turn_radius_theory(params['wheelbase'], delta)
    print('[转向] 稳态偏航角速度 %.3f rad/s（末段 %.1f s），平均车速 %.2f m/s'
          % (omega, window, v_mean))
    if radius_yaw:
        print('       半径(偏航角速度法) R = v/ω = %.2f m' % radius_yaw)
    if radius_fit:
        print('       半径(轨迹圆拟合法)      = %.2f m' % radius_fit)
    print('       阿克曼理论值 R = L/tanδ  = %.2f m（轴距 %.2f m）'
          % (theory, params['wheelbase']))

    radius = radius_yaw if radius_yaw else radius_fit
    error = abs(radius - theory) / theory * 100.0 if radius else None
    if radius:
        print('       相对理论值偏差 %.1f%%' % error)
    return {'ok': radius is not None, 'radius': radius, 'radius_yaw': radius_yaw,
            'radius_fit': radius_fit, 'omega': omega, 'v_mean': v_mean,
            'theory': theory, 'error_pct': error, 'points': points, 'params': params}
