#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：不需要 CARLA、不需要 ROS，只依赖 Python 标准库。

运行方式::

    cd src/ground/carla_nn_car_following
    python3 test/test_reference_acc.py

覆盖内容：经典 ACC 控制器的公式（期望车距、车距误差、控制律、油门/刹车映射、TTC、
跟车状态分类）以及四种前车工况的速度曲线。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_nn_car_following.reference_acc import ReferenceACC          # noqa: E402
from carla_nn_car_following import scenario as scenario_mod            # noqa: E402

PASSED = 0


def check(condition, message):
    global PASSED
    assert condition, '未通过：' + message
    PASSED += 1
    print('  [通过] %s' % message)


def main():
    acc = ReferenceACC(d0=5.0, headway=1.2, kp=0.35, kv=0.8, k_throttle=0.25, k_brake=0.35)

    print('1) 期望车距 d_des = d0 + t_h * v_ego')
    check(abs(acc.desired_gap(0.0) - 5.0) < 1e-9, '静止时期望车距 = 5.0 m')
    check(abs(acc.desired_gap(10.0) - 17.0) < 1e-9, '10 m/s 时期望车距 = 17.0 m')

    print('2) 车距误差 e = d - d_des')
    check(abs(acc.gap_error(20.0, 10.0) - 3.0) < 1e-9, 'd=20、v=10 时误差 = 3.0 m')

    print('3) 控制律 a_cmd = kp * e + kv * v_rel 的符号')
    check(acc.acceleration(40.0, 0.0, 10.0) > 0.0, '车距过大时输出正加速度（加速）')
    check(acc.acceleration(5.0, -5.0, 10.0) < 0.0, '车距过小且正在接近时输出负加速度（减速）')

    print('4) 加速度映射为油门 / 刹车（互斥且限幅到 [0, 1]）')
    far_throttle, far_brake = acc.labels(40.0, 0.0, 10.0)
    check(far_throttle > 0.0 and far_brake == 0.0, '远距离：油门 > 0 且刹车 = 0')
    near_throttle, near_brake = acc.labels(5.0, -5.0, 10.0)
    check(near_throttle == 0.0 and near_brake > 0.0, '危险：油门 = 0 且刹车 > 0')
    check(acc.labels(1000.0, 0.0, 0.0)[0] == 1.0, '油门被限幅到 1.0')
    check(acc.labels(0.0, -100.0, 0.0)[1] == 1.0, '刹车被限幅到 1.0')

    print('5) TTC = d / (-v_rel)，不在接近时为 inf')
    check(abs(acc.ttc(20.0, -2.0) - 10.0) < 1e-9, 'd=20、v_rel=-2 时 TTC = 10 s')
    check(acc.ttc(20.0, 2.0) == float('inf'), 'v_rel > 0（远离）时 TTC = inf')
    check(acc.ttc(20.0, 0.0) == float('inf'), 'v_rel = 0 时 TTC = inf')

    print('6) 跟车状态分类（感知网络的分类标签）')
    check(acc.follow_state(40.0, 10.0) == 0, '车距远 -> 0')
    check(acc.follow_state(18.0, 10.0) == 1, '车距接近期望 -> 1')
    check(acc.follow_state(10.0, 10.0) == 2, '车距过近 -> 2')

    print('7) 四种前车工况的速度曲线')
    constant = scenario_mod.build('constant')
    check(abs(constant.target_speed(0.0) - 16.67) < 0.05, '匀速工况目标速度 = 60 km/h')
    slow = scenario_mod.build('decelerate')
    check(slow.target_speed(1.0) > slow.target_speed(6.0), '缓减速工况：目标速度随时间下降')
    check(abs(slow.target_speed(100.0) - 5.0) < 1e-9, '缓减速工况速度不低于下限 5.0 m/s')
    emergency = scenario_mod.build('emergency')
    check(emergency.target_speed(1.0) > 0.0 and emergency.target_speed(5.0) == 0.0,
          '急刹工况：3 s 之前正常行驶，之后目标速度为 0')
    cut_in = scenario_mod.build('cutin')
    check(cut_in.wants_cut_in(2.0) is False, '加塞工况：3 s 之前不触发')
    check(cut_in.wants_cut_in(3.5) is True, '加塞工况：3 s 之后触发')
    cut_in.mark_spawned()
    check(cut_in.wants_cut_in(4.0) is False, '加塞车辆生成后不再重复生成')

    print('8) 工况注册表')
    check(sorted(scenario_mod.all_names()) == ['constant', 'cutin', 'decelerate', 'emergency'],
          '注册表包含四种工况')
    try:
        scenario_mod.build('not_exist')
        check(False, '未知工况应当抛出 ValueError')
    except ValueError:
        check(True, '未知工况抛出 ValueError')

    print('\n[结果] %d 项全部通过' % PASSED)
    return 0


if __name__ == '__main__':
    sys.exit(main())
